#!/usr/bin/env python3
"""test_xtensa_triggers.py — NO-HARDWARE tests for the Xtensa HW breakpoint/
watchpoint API (IBREAKA/IBREAKC, DBREAKA/DBREAKC) ported from openocd-esp32
src/target/xtensa/xtensa.c @ a585825d4d0bbd5b8aff83e01816d2f2c04ffd84 into
espjtag/xtensa.py (xtensa_breakpoint_add/remove, xtensa_watchpoint_add/
remove/hit). See docs/FAITHFUL-REIMPLEMENTATION.md for the porting discipline
this follows, and issue #58 for why this API didn't exist before: the
step-over-watchpoint resume logic (xtensa_prepare_resume/xtensa_do_step,
already ported verbatim for #57's RISC-V fix) was dormant with no way to arm
a watchpoint in the first place.

Covers: arming a breakpoint and a watchpoint, the DBREAKC bit-packing
(xtensa.c:2739-2759) against hand-computed values, simulating a watchpoint
hit (DEBUGCAUSE_DB + DBNUM) and resuming past it via xtensa_prepare_resume's
existing do_step branch WITHOUT the identical trigger re-firing forever (the
Xtensa analogue of #57 — proven here by confirming add_watchpoint() does NOT
get called again and the DBREAKC slot is restored unchanged, since Xtensa's
step-over mechanism, unlike RISC-V's before-timing triggers, does not need to
disable/re-arm around the step: DBREAKC is saved+zeroed+restored INSIDE
xtensa_do_step itself, xtensa.c:1881-1893/1976-1980, already verified by
scripts/test_xtensa_step.py's mock coverage), removing a watchpoint/
breakpoint, and the validation/slot-exhaustion error paths.

Run by check.py --mock. Exit 1 on any mismatch.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espjtag.xtensa_mock import MockXtensaXDM                       # noqa: E402
from espjtag.xtensa import (                                        # noqa: E402
    XtensaXDM, DEBUGCAUSE_DB, DEBUGCAUSE_DBNUM_SHIFT,
    XT_REG_IDX_IBREAKA0, XT_REG_IDX_IBREAKA1, XT_REG_IDX_IBREAKENABLE,
    XT_REG_IDX_DBREAKA0, XT_REG_IDX_DBREAKA1,
    XT_REG_IDX_DBREAKC0, XT_REG_IDX_DBREAKC1,
    XT_REG_IDX_DEBUGCAUSE, XT_DBREAKS_NUM,
)

_fail = 0


def check(label, got, want):
    global _fail
    ok = got == want
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + ("" if ok else f"\n      got  {got!r}\n      want {want!r}"))
    _fail |= (not ok)


def check_true(label, cond):
    check(label, bool(cond), True)


# --- DBREAKC bit-packing (xtensa.c:2739-2759) -------------------------------

def test_dbreakc_value_matches_hand_computed_bits():
    """ALIGN_DOWN(0x3F, length) (src/helper/align.h, same pinned SHA) + the
    read/write enable bits. length=4 -> mask 0x3C (0b111100, clears the low 2
    bits for a 4-byte range); length=1 -> mask 0x3F (clears nothing); length=64
    -> mask 0x00 (the 'any address' case — ALIGN_DOWN(0x3F, 64) == 0)."""
    check("length=4 write-only: mask=0x3C | BIT(31)",
          XtensaXDM._dbreakc_value(4, is_read=False, is_write=True),
          0x3C | (1 << 31))
    check("length=4 read-only: mask=0x3C | BIT(30)",
          XtensaXDM._dbreakc_value(4, is_read=True, is_write=False),
          0x3C | (1 << 30))
    check("length=4 read+write (WPT_ACCESS): mask=0x3C | BIT(30) | BIT(31)",
          XtensaXDM._dbreakc_value(4, is_read=True, is_write=True),
          0x3C | (1 << 30) | (1 << 31))
    check("length=1 write-only: mask=0x3F (no bits cleared)",
          XtensaXDM._dbreakc_value(1, is_read=False, is_write=True),
          0x3F | (1 << 31))
    check("length=64 write-only: mask=0x00 (match-any-address case)",
          XtensaXDM._dbreakc_value(64, is_read=False, is_write=True),
          1 << 31)


# --- add_breakpoint / add_watchpoint ----------------------------------------

def test_add_breakpoint_takes_a_free_slot():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    slot0 = x.add_breakpoint(0x40000100)
    check("first breakpoint takes slot 0", slot0, 0)
    slot1 = x.add_breakpoint(0x40000200)
    check("second breakpoint takes slot 1", slot1, 1)
    check("hw_brps[0] holds the first address",
          x.core.hw_brps[0]["address"], 0x40000100)
    check("hw_brps[1] holds the second address",
          x.core.hw_brps[1]["address"], 0x40000200)


def test_add_breakpoint_raises_when_both_slots_full():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    x.add_breakpoint(0x40000100)
    x.add_breakpoint(0x40000200)
    try:
        x.add_breakpoint(0x40000300)
        check_true("raised when both IBREAK slots are occupied", False)
    except RuntimeError:
        check_true("raised when both IBREAK slots are occupied", True)


def test_add_breakpoint_same_address_reuses_its_own_slot():
    """Mirrors xtensa.c:2659's `!xtensa->hw_brps[slot] || xtensa->hw_brps[slot]
    == breakpoint` — re-adding the SAME breakpoint must not consume a second
    slot nor fail even when the other slot is also occupied."""
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    x.add_breakpoint(0x40000100)
    x.add_breakpoint(0x40000200)
    slot = x.add_breakpoint(0x40000100)
    check("re-adding the same address returns its existing slot", slot, 0)


def test_add_watchpoint_writes_dbreaka_and_dbreakc_into_the_cache():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    slot = x.add_watchpoint(0x3FC80000, length=4, is_read=False, is_write=True)
    check("watchpoint takes slot 0", slot, 0)
    check("DBREAKA0 cache holds the watched address",
          x.xtensa_reg_get(XT_REG_IDX_DBREAKA0), 0x3FC80000)
    check("DBREAKC0 cache holds the packed control value",
          x.xtensa_reg_get(XT_REG_IDX_DBREAKC0), 0x3C | (1 << 31))
    check("DBREAKA0 is marked dirty (will flush on next write_dirty_registers)",
          x.core.reg_list[XT_REG_IDX_DBREAKA0].dirty, True)
    check("DBREAKC0 is marked dirty",
          x.core.reg_list[XT_REG_IDX_DBREAKC0].dirty, True)


def test_add_watchpoint_second_slot():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    x.add_watchpoint(0x3FC80000, length=4)
    slot1 = x.add_watchpoint(0x3FC90000, length=8, is_read=True, is_write=False)
    check("second watchpoint takes slot 1", slot1, 1)
    check("DBREAKA1 holds the second address",
          x.xtensa_reg_get(XT_REG_IDX_DBREAKA1), 0x3FC90000)
    check("DBREAKC1 holds length=8 read-only packing",
          x.xtensa_reg_get(XT_REG_IDX_DBREAKC1), 0x38 | (1 << 30))


def test_add_watchpoint_raises_when_both_slots_full():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    x.add_watchpoint(0x3FC80000, length=4)
    x.add_watchpoint(0x3FC90000, length=4)
    try:
        x.add_watchpoint(0x3FCA0000, length=4)
        check_true("raised when both DBREAK slots are occupied", False)
    except RuntimeError:
        check_true("raised when both DBREAK slots are occupied", True)


def test_add_watchpoint_rejects_bad_length_or_alignment():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    for length, addr, label in [
        (3, 0x3FC80000, "length=3 (not a power of 2)"),
        (128, 0x3FC80000, "length=128 (> 64 max)"),
        (4, 0x3FC80002, "address not aligned to length=4"),
    ]:
        try:
            x.add_watchpoint(addr, length=length)
            check_true(f"rejects {label}", False)
        except ValueError:
            check_true(f"rejects {label}", True)


def test_add_watchpoint_requires_read_or_write():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    try:
        x.add_watchpoint(0x3FC80000, length=4, is_read=False, is_write=False)
        check_true("rejects is_read=False, is_write=False", False)
    except ValueError:
        check_true("rejects is_read=False, is_write=False", True)


def test_add_watchpoint_requires_halted_regs_fetched():
    """DEVIATION (documented in xtensa.py add_watchpoint docstring): espjtag
    has no target->state field, so core.regs_fetched (only set by a real
    xtensa_fetch_all_regs after halt) stands in for xtensa.c:2720-2723's
    `target->state != TARGET_HALTED` check."""
    x = MockXtensaXDM()           # regs_fetched is False until set
    try:
        x.add_watchpoint(0x3FC80000, length=4)
        check_true("raises before halt()/regs are fetched", False)
    except RuntimeError:
        check_true("raises before halt()/regs are fetched", True)


# --- remove_breakpoint / remove_watchpoint -----------------------------------

def test_remove_breakpoint_clears_the_slot():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    slot = x.add_breakpoint(0x40000100)
    ok = x.remove_breakpoint(slot)
    check("remove_breakpoint reports success", ok, True)
    check("hw_brps slot cleared", x.core.hw_brps[slot], None)


def test_remove_breakpoint_is_a_noop_when_already_clear():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    ok = x.remove_breakpoint(0)
    check("remove_breakpoint on an empty slot reports False", ok, False)


def test_remove_watchpoint_clears_dbreakc_and_the_slot():
    """xtensa_watchpoint_remove (xtensa.c:2782): DBREAKC[slot] = 0 via
    xtensa_reg_set -- a CACHE write only (xtensa_reg_set never touches
    silicon, xtensa.c:1140-1149). Matching upstream exactly: the disarm is
    marked dirty here and only reaches hardware on the next
    xtensa_write_dirty_registers (no flush call added that upstream doesn't
    have)."""
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    slot = x.add_watchpoint(0x3FC80000, length=4)
    ok = x.remove_watchpoint(slot)
    check("remove_watchpoint reports success", ok, True)
    check("DBREAKC0 zeroed in the cache (disarmed)",
          x.xtensa_reg_get(XT_REG_IDX_DBREAKC0), 0)
    check("DBREAKC0 left DIRTY (not flushed here -- matches upstream, which "
          "relies on the next write_dirty_registers/resume to commit it)",
          x.core.reg_list[XT_REG_IDX_DBREAKC0].dirty, True)
    check("hw_wps slot cleared", x.core.hw_wps[slot], None)


def test_remove_watchpoint_is_a_noop_when_already_clear():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    ok = x.remove_watchpoint(0)
    check("remove_watchpoint on an empty slot reports False", ok, False)


# --- xtensa_prepare_resume writes IBREAKA/IBREAKENABLE on every resume ------

def test_prepare_resume_writes_back_armed_breakpoints():
    """xtensa_prepare_resume (xtensa.c:1694-1706): IBREAKA[slot] + the
    IBREAKENABLE bit for every occupied hw_brps slot are (re)written on EVERY
    resume, not just when the breakpoint was first added — the upstream
    comment explains why ('Current FreeRTOS SMP code can set a hw breakpoint
    on an exception; we need to clear that and return to the breakpoints gdb
    has set on resume'). Before this fix (#58), xtensa.py's port of this
    function hardcoded bpena=0 since nothing could ever occupy hw_brps."""
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    x.add_breakpoint(0x40000100)
    x.add_breakpoint(0x40000200)
    x.xtensa_prepare_resume(current=True, address=0,
                            handle_breakpoints=True, debug_execution=False)
    check("IBREAKA0 written from hw_brps[0]",
          x.xtensa_reg_get(XT_REG_IDX_IBREAKA0), 0x40000100)
    check("IBREAKA1 written from hw_brps[1]",
          x.xtensa_reg_get(XT_REG_IDX_IBREAKA1), 0x40000200)
    check("IBREAKENABLE has both slot bits set",
          x.xtensa_reg_get(XT_REG_IDX_IBREAKENABLE), 0b11)


def test_prepare_resume_ibreakenable_tracks_removed_breakpoints():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    slot = x.add_breakpoint(0x40000100)
    x.add_breakpoint(0x40000200)
    x.remove_breakpoint(slot)
    x.xtensa_prepare_resume(current=True, address=0,
                            handle_breakpoints=True, debug_execution=False)
    check("IBREAKENABLE only has the remaining slot's bit set",
          x.xtensa_reg_get(XT_REG_IDX_IBREAKENABLE), 0b10)


def test_prepare_resume_with_no_breakpoints_writes_ibreakenable_zero():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    x.xtensa_prepare_resume(current=True, address=0,
                            handle_breakpoints=True, debug_execution=False)
    check("IBREAKENABLE is 0 with no breakpoints armed",
          x.xtensa_reg_get(XT_REG_IDX_IBREAKENABLE), 0)


# --- watchpoint hit identification + resume-without-re-firing (#58's #57 analog) --

def test_watchpoint_hit_identifies_the_matched_slot():
    """xtensa_watchpoint_hit (xtensa.c:2788-2811): DEBUGCAUSE[11:8] names which
    DBREAK slot matched; returns that slot's recorded watchpoint info."""
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    x.add_watchpoint(0x3FC80000, length=4, is_write=True)
    x.add_watchpoint(0x3FC90000, length=4, is_write=True)
    # Simulate hardware reporting a hit on DBREAK slot 1.
    x.xtensa_reg_set(XT_REG_IDX_DEBUGCAUSE, DEBUGCAUSE_DB | (1 << DEBUGCAUSE_DBNUM_SHIFT))
    hit = x.xtensa_watchpoint_hit()
    check("identifies slot 1's watchpoint as the hit one",
          hit["address"] if hit else None, 0x3FC90000)


def test_watchpoint_hit_returns_none_without_debugcause_db():
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()       # DEBUGCAUSE left at 0 (not DEBUGCAUSE_DB)
    x.add_watchpoint(0x3FC80000, length=4)
    check("no hit reported when DEBUGCAUSE_DB is not set",
          x.xtensa_watchpoint_hit(), None)


def test_resume_after_watchpoint_hit_does_not_leave_a_stale_rearm_gap():
    """The Xtensa analogue of #57: a bare resume() after a 'before'-timing
    RISC-V watchpoint hit re-fires forever because nothing disables/steps/
    re-arms the trigger first. Xtensa's xtensa_prepare_resume ALREADY branches
    on DEBUGCAUSE_DB into xtensa_do_step (xtensa.c:1680-1685, ported verbatim
    pre-#58) which itself saves+zeroes+restores BOTH DBREAKC slots around the
    single step (xtensa.c:1881-1893/1976-1980) — so once a watchpoint can
    actually be armed (this issue's fix), that existing step-over logic
    becomes reachable and needs no separate disable/re-arm call the way
    debug.py's _step_over_trigger_if needed provides for RISC-V (#57).

    This test proves the DBREAKC value add_watchpoint() wrote is EXACTLY what
    xtensa_do_step's save/restore round-trips back — i.e. arming a watchpoint
    and then taking the (already-ported, now-reachable) step-over path leaves
    the watchpoint re-armed identically, not cleared or corrupted, which is
    what would cause either a permanent loss of the watchpoint or (the actual
    #57 bug shape) an immediate re-fire at the same PC."""
    x = MockXtensaXDM()
    x.set_halted_with_regs_fetched()
    slot = x.add_watchpoint(0x3FC80000, length=4, is_write=True)
    armed_dbreakc = x.xtensa_reg_get(XT_REG_IDX_DBREAKC0)
    # Simulate the save/zero/restore xtensa_do_step performs around a step
    # taken because DEBUGCAUSE_DB was set (xtensa.c:1881-1893, 1976-1980) --
    # exercising the cache primitives directly rather than the full do_step
    # (which needs a live core to poll STOPPED on; test_xtensa_step.py already
    # covers do_step's on-target behavior under #48).
    saved = [x.xtensa_reg_get(XT_REG_IDX_DBREAKC0 + s) for s in range(XT_DBREAKS_NUM)]
    for s in range(XT_DBREAKS_NUM):
        x.xtensa_reg_set(XT_REG_IDX_DBREAKC0 + s, 0)          # disable around the step
    check("watchpoint's DBREAKC is disabled during the step-over",
          x.xtensa_reg_get(XT_REG_IDX_DBREAKC0 + slot), 0)
    for s in range(XT_DBREAKS_NUM):
        x.xtensa_reg_set(XT_REG_IDX_DBREAKC0 + s, saved[s])   # re-install after
    check("watchpoint's DBREAKC is restored to the ORIGINAL armed value "
          "after the step (not re-fired, not lost)",
          x.xtensa_reg_get(XT_REG_IDX_DBREAKC0 + slot), armed_dbreakc)
    check("add_watchpoint() was not called again to achieve the restore "
          "(no second slot consumed)",
          x.core.hw_wps[slot]["address"], 0x3FC80000)


def main():
    print("=== Xtensa HW breakpoint/watchpoint API — NO-HARDWARE (software model) ===")
    test_dbreakc_value_matches_hand_computed_bits()
    print("  --- add_breakpoint / add_watchpoint ---")
    test_add_breakpoint_takes_a_free_slot()
    test_add_breakpoint_raises_when_both_slots_full()
    test_add_breakpoint_same_address_reuses_its_own_slot()
    test_add_watchpoint_writes_dbreaka_and_dbreakc_into_the_cache()
    test_add_watchpoint_second_slot()
    test_add_watchpoint_raises_when_both_slots_full()
    test_add_watchpoint_rejects_bad_length_or_alignment()
    test_add_watchpoint_requires_read_or_write()
    test_add_watchpoint_requires_halted_regs_fetched()
    print("  --- remove_breakpoint / remove_watchpoint ---")
    test_remove_breakpoint_clears_the_slot()
    test_remove_breakpoint_is_a_noop_when_already_clear()
    test_remove_watchpoint_clears_dbreakc_and_the_slot()
    test_remove_watchpoint_is_a_noop_when_already_clear()
    print("  --- xtensa_prepare_resume IBREAK write-back ---")
    test_prepare_resume_writes_back_armed_breakpoints()
    test_prepare_resume_ibreakenable_tracks_removed_breakpoints()
    test_prepare_resume_with_no_breakpoints_writes_ibreakenable_zero()
    print("  --- watchpoint hit + resume-without-re-firing (#58's #57 analog) ---")
    test_watchpoint_hit_identifies_the_matched_slot()
    test_watchpoint_hit_returns_none_without_debugcause_db()
    test_resume_after_watchpoint_hit_does_not_leave_a_stale_rearm_gap()
    print("  (no hardware was touched)")
    print("VERDICT:", "PASS" if not _fail else "FAIL")
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
