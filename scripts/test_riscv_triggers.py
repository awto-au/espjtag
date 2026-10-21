#!/usr/bin/env python3
"""test_riscv_triggers.py — NO-HARDWARE tests for the RISC-V trigger module
(watchpoints/breakpoints) ported from openocd-esp32 src/target/riscv/riscv.c
@ a585825d4d0bbd5b8aff83e01816d2f2c04ffd84 into espjtag/debug.py. See
docs/FAITHFUL-REIMPLEMENTATION.md for the porting discipline this follows.

Covers: trigger enumeration against a mock hart, find_next_free_trigger,
set_trigger's full disable/write/verify DMI sequence (accept + reject paths),
a full add_watchpoint() composing a real mcontrol6 equality-match tdata1 value
checked against a hand-computed expected value, and the #57 step-over-trigger
fix (step(), hit-bit detection, and resume()'s disable/step/re-arm sequence —
ported from riscv.c resume_prep/riscv_openocd_step_impl + riscv-013.c
set_dcsr_config/riscv013_step_or_resume_current_hart, same pinned SHA).

Run by check.py --mock. Exit 1 on any mismatch.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espjtag.mock import MockEspUsbJtag                            # noqa: E402
from espjtag.debug import EspUsbJtag, ERROR_OK                      # noqa: E402
from espjtag.constants import (                                     # noqa: E402
    DMCONTROL, DMSTATUS, DM_ALLHALTED, DM_ALLRUNNING, DM_ALLRESUMEACK,
    DM_DMACTIVE, DM_RESUMEREQ, DM_HALTREQ,
    CSR_TSELECT, CSR_TDATA1, CSR_TDATA2, CSR_TINFO, CSR_MISA, CSR_DCSR,
    CSR_TDATA1_TYPE_MCONTROL6, CSR_TDATA1_TYPE_MASK, CSR_TDATA1_TYPE_OFFSET,
    CSR_MCONTROL6_DMODE_MASK, CSR_MCONTROL6_ACTION_MASK,
    CSR_MCONTROL6_ACTION_DEBUG_MODE, CSR_MCONTROL6_M_MASK,
    CSR_MCONTROL6_EXECUTE_MASK, CSR_MCONTROL6_LOAD_MASK, CSR_MCONTROL6_STORE_MASK,
    CSR_MCONTROL6_CHAIN_MASK, CSR_MCONTROL6_CHAIN_DISABLED,
    CSR_MCONTROL6_MATCH_MASK, CSR_MCONTROL6_MATCH_EQUAL, CSR_MCONTROL6_SIZE_MASK,
    CSR_MCONTROL6_SIZE_ANY, CSR_MCONTROL6_TYPE_MASK, CSR_MCONTROL6_TYPE_OFFSET,
    CSR_MCONTROL6_HIT0_MASK, CSR_MCONTROL6_HIT1_MASK, CSR_MCONTROL6_HIT0_BEFORE,
    CSR_DCSR_STEP, CSR_DCSR_CAUSE_MASK, CSR_DCSR_CAUSE_TRIGGER,
    CSR_TINFO_VERSION_1,
)

_fail = 0


def check(label, got, want):
    global _fail
    ok = got == want
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + ("" if ok else f"\n      got  {got!r}\n      want {want!r}"))
    _fail |= (not ok)


class CsrMockJtag(MockEspUsbJtag):
    """Trigger tests need real per-CSR semantics (TSELECT/TDATA1/TDATA2/TINFO/
    MISA each holding their OWN value), which the base MockEspUsbJtag can't give
    for free: write_register/read_register both funnel through the single
    DATA0/COMMAND DM slots (correct for testing the DMI *sequence* of one
    register access, but a single shared slot can't model 4 simultaneously-live
    CSRs). This subclass overrides read_register/write_register to track a
    proper {regno: value} dict while still logging through self.ops, so trigger
    orchestration tests see realistic hart state without re-deriving the DMI
    encoding (set_trigger's own DMI-level test below exercises that directly
    against the base mock).

    `csrs`: {regno: value}. `tselect_clamp`: simulates a hart with fewer
    triggers than probed — TSELECT reads back clamped to this value once index
    exceeds it (check_if_trigger_exists's enumeration-stop condition).
    `deny_tdata1`, `deny_tdata2`: sets of (idx, tdata1[, tdata2]) or plain
    values that set_trigger's hardware MUST reject (readback != written)."""

    def __init__(self, idcode=0x0000DC25, trigger_tinfo=None, tselect_clamp=None):
        super().__init__(idcode=idcode)
        # _enumerate_triggers requires the hart halted (riscv.c:6380-6383) before
        # touching any CSR -- default every trigger-test mock to halted so tests
        # exercise the enumeration/add_trigger logic itself, not this precondition.
        self.set_dm(DMSTATUS, DM_ALLHALTED)
        self.csrs = {}
        self._tselect_clamp = tselect_clamp
        # {idx: [tinfo, ...]} - what TINFO reads back as once TSELECT=idx
        self._trigger_tinfo_by_idx = trigger_tinfo or {}
        self._cur_tselect = 0
        self.deny_tdata1_for = set()      # idx values where tdata1 readback is forced wrong
        self.deny_tdata2_for = set()      # idx values where tdata2 readback is forced wrong

    def write_register(self, regno, value):
        self.ops.append(("write_register", regno, value & 0xFFFFFFFF))
        if regno == CSR_TSELECT:
            self._cur_tselect = value
            if self._tselect_clamp is not None and value > self._tselect_clamp:
                self._cur_tselect = self._tselect_clamp
            return 0
        self.csrs[regno] = value & 0xFFFFFFFF
        return 0

    def read_register(self, regno):
        self.ops.append(("read_register", regno))
        if regno == CSR_TSELECT:
            return self._cur_tselect
        if regno == CSR_TDATA1:
            v = self.csrs.get(CSR_TDATA1, 0)
            if self._cur_tselect in self.deny_tdata1_for:
                return v ^ 0xFFFFFFFF                     # force a readback mismatch
            return v
        if regno == CSR_TDATA2:
            v = self.csrs.get(CSR_TDATA2, 0)
            if self._cur_tselect in self.deny_tdata2_for:
                return v ^ 0xFFFFFFFF
            return v
        if regno == CSR_TINFO:
            return self._trigger_tinfo_by_idx.get(self._cur_tselect, 0)
        if regno == CSR_MISA:
            return self.csrs.get(CSR_MISA, 0)              # default 0: no S/U/H (C-series)
        return self.csrs.get(regno, 0)


def test_can_use_napot_match():
    check("length=4 addr=0x1000 (aligned, pow2) -> True",
          EspUsbJtag._can_use_napot_match(0x1000, 4), True)
    check("length=4 addr=0x1002 (misaligned) -> False",
          EspUsbJtag._can_use_napot_match(0x1002, 4), False)
    check("length=3 (not pow2) -> False",
          EspUsbJtag._can_use_napot_match(0x1000, 3), False)
    check("length=1 -> False (must be > 1)",
          EspUsbJtag._can_use_napot_match(0x1000, 1), False)


def test_count_trailing_ones():
    check("count_trailing_ones(0) == 0", EspUsbJtag._count_trailing_ones(0), 0)
    check("count_trailing_ones(0b1) == 1", EspUsbJtag._count_trailing_ones(0b1), 1)
    check("count_trailing_ones(0b111) == 3", EspUsbJtag._count_trailing_ones(0b111), 3)
    check("count_trailing_ones(0xFFFFFFFF) == 32",
          EspUsbJtag._count_trailing_ones(0xFFFFFFFF), 32)
    check("count_trailing_ones(0b1011) == 2 (stops at the 0)",
          EspUsbJtag._count_trailing_ones(0b1011), 2)


def test_find_next_free_trigger():
    j = MockEspUsbJtag()
    j.trigger_count = 4
    j.trigger_unique_id = [-1, 5, -1, -1]
    j.trigger_tinfo = [1 << 6, 1 << 6, 1 << 6, 1 << 6]   # all report type 6
    ok, idx = EspUsbJtag._find_next_free_trigger(j, 6, False, 0)
    check("single: finds idx 0 (first free)", (ok, idx), (True, 0))
    ok, idx = EspUsbJtag._find_next_free_trigger(j, 6, False, 1)
    check("single: starting scan at 1 skips occupied idx 1, finds idx 2",
          (ok, idx), (True, 2))
    ok, idx = EspUsbJtag._find_next_free_trigger(j, 6, True, 0)
    check("chained: skips occupied idx 1, finds the NEXT consecutive pair (2,3)",
          (ok, idx), (True, 2))
    j.trigger_unique_id = [-1, 5, 5, -1]
    ok, idx = EspUsbJtag._find_next_free_trigger(j, 6, True, 0)
    check("chained: no 2 consecutive free anywhere -> not found", ok, False)
    j.trigger_unique_id = [-1, -1, 5, -1]
    ok, idx = EspUsbJtag._find_next_free_trigger(j, 6, True, 0)
    check("chained: finds idx 0,1 consecutive free", (ok, idx), (True, 0))
    # wrong type -> not found even though free
    j.trigger_tinfo = [1 << 2, 1 << 2, 1 << 2, 1 << 2]
    ok, idx = EspUsbJtag._find_next_free_trigger(j, 6, False, 0)
    check("type mismatch -> not found", ok, False)


def test_wp_triggers_cache():
    j = MockEspUsbJtag()
    j.trigger_count = 2
    EspUsbJtag._create_wp_trigger_cache(j)
    check("cache starts empty", EspUsbJtag._wp_triggers_cache_search(j, 0, 0x1234, 0x5678), False)
    EspUsbJtag._wp_triggers_cache_add(j, 0, 0x1234, 0x5678)
    check("cache hits after add (same idx/tdata1/tdata2)",
          EspUsbJtag._wp_triggers_cache_search(j, 0, 0x1234, 0x5678), True)
    check("cache miss on different tdata2",
          EspUsbJtag._wp_triggers_cache_search(j, 0, 0x1234, 0x9999), False)
    check("cache miss on different idx (same tdata1/tdata2)",
          EspUsbJtag._wp_triggers_cache_search(j, 1, 0x1234, 0x5678), False)
    # a second (tdata1, tdata2) on the SAME idx coexists with the first
    EspUsbJtag._wp_triggers_cache_add(j, 0, 0x1234, 0xAAAA)
    check("second tdata2 under the same tdata1 also cached",
          EspUsbJtag._wp_triggers_cache_search(j, 0, 0x1234, 0xAAAA), True)
    check("first tdata2 entry still cached after the second add",
          EspUsbJtag._wp_triggers_cache_search(j, 0, 0x1234, 0x5678), True)


def test_set_trigger_accepts_clean_write():
    """set_trigger's happy path against the base (DATA0-slot) mock: TSELECT<-idx,
    TDATA1<-0 (disable), TDATA2<-tdata2, TDATA1<-tdata1, then read back TDATA1
    and TDATA2 in that order — script DATA0 to echo exactly what was written,
    proving set_trigger accepts a clean, faithfully-echoed hardware response."""
    j = MockEspUsbJtag()
    j.trigger_count = 4
    j.reserved_triggers = [False] * 4
    tdata1 = (CSR_MCONTROL6_TYPE_MASK & (CSR_TDATA1_TYPE_MCONTROL6 << CSR_TDATA1_TYPE_OFFSET))
    tdata2 = 0x3FC80000
    # read order inside set_trigger: read TDATA1, then read TDATA2 -> script
    # DATA0's read queue with exactly those two values, in that order.
    j.set_dm_sequence(0x04, [tdata1, tdata2])     # DATA0 = 0x04 (constants.DATA0)
    ret = EspUsbJtag._set_trigger(j, 2, tdata1, tdata2)
    check("set_trigger accepts a clean readback", ret, ERROR_OK)
    # TSELECT (CSR_TSELECT) write must have carried idx=2 as the staged DATA0
    # value immediately before a COMMAND write (write_register's own sequence,
    # already covered by test_riscv_mock.py's write_register test) — here we
    # only check set_trigger's OWN ordering: TDATA1 disabled (0) before TDATA2.
    from espjtag.constants import COMMAND, DATA0
    data0_writes = [v for a, v in j.dmi_writes if a == DATA0]
    # Expect: [idx, 0 (tdata1 disable), tdata2, tdata1] staged via write_register
    check("set_trigger staged idx then 0 (disable) then tdata2 then tdata1",
          data0_writes[:4], [2, 0, tdata2 & 0xFFFFFFFF, tdata1 & 0xFFFFFFFF])


def test_set_trigger_rejects_denied_config():
    """set_trigger's reject path: hardware echoes back something OTHER than what
    was written (e.g. a read-only trigger slot) -> ERROR_TARGET_RESOURCE_NOT_
    AVAILABLE, and TDATA1 is written back to 0 (disabling the attempted config)."""
    from espjtag.debug import ERROR_TARGET_RESOURCE_NOT_AVAILABLE
    j = MockEspUsbJtag()
    j.trigger_count = 1
    j.reserved_triggers = [False]
    tdata1 = 0x12345678
    tdata2 = 0xDEADBEEF
    # hardware readback does NOT match what was written (config denied)
    j.set_dm_sequence(0x04, [0, 0])
    ret = EspUsbJtag._set_trigger(j, 0, tdata1, tdata2)
    check("set_trigger rejects a denied config", ret, ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
    from espjtag.constants import DATA0
    data0_writes = [v for a, v in j.dmi_writes if a == DATA0]
    check("set_trigger wrote TDATA1=0 again after the denial (final write is 0)",
          data0_writes[-1], 0)


def test_set_trigger_reserved_blocks_immediately():
    """set_trigger must refuse a reserved trigger WITHOUT touching the DMI at
    all (riscv.c:868-872) — #trigger-port keeps this check even with no CLI
    'reserve_trigger' feature behind it."""
    from espjtag.debug import ERROR_TARGET_RESOURCE_NOT_AVAILABLE
    j = MockEspUsbJtag()
    j.trigger_count = 1
    j.reserved_triggers = [True]
    ret = EspUsbJtag._set_trigger(j, 0, 0x1, 0x2)
    check("set_trigger refuses a reserved trigger", ret, ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
    check("set_trigger made NO dmi writes for a reserved trigger", j.dmi_writes, [])


def test_enumerate_triggers_type6():
    """_enumerate_triggers against a mock hart reporting 2 type-6 triggers,
    using the CsrMockJtag (real per-CSR state) since enumeration reads TSELECT/
    TDATA1/TINFO interleaved across multiple trigger indices."""
    j = CsrMockJtag(idcode=0x0000DC25,
                     trigger_tinfo={0: 1 << 6, 1: 1 << 6},
                     tselect_clamp=1)           # only indices 0,1 exist
    j.csrs[CSR_TDATA1] = 0       # tdata1 readback while probing (type field 0 unused;
                                  # get_trigger_types uses TINFO here, not tdata1)
    ret = EspUsbJtag._enumerate_triggers(j)
    check("enumerate_triggers returns ERROR_OK", ret, ERROR_OK)
    check("found 2 triggers", j.trigger_count, 2)
    check("trigger_tinfo recorded for both", j.trigger_tinfo, [1 << 6, 1 << 6])
    check("trigger_unique_id starts all-free (-1)", j.trigger_unique_id, [-1, -1])
    check("reserved_triggers defaults all-False", j.reserved_triggers, [False, False])
    check("idempotent: a second call is a no-op (triggers_enumerated guard)",
          EspUsbJtag._enumerate_triggers(j), ERROR_OK)


def test_add_watchpoint_equality_match_t6():
    """Full add_watchpoint() -> add_trigger -> ... -> set_trigger against a mock
    hart with a SINGLE type-6 trigger and NAPOT/GE+LT support switched off (so
    the fallback chain lands on the plain equality match), composing a real
    mcontrol6 tdata1 and checking it against a HAND-COMPUTED expected value —
    so a future reader can check the bit math without running anything."""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6}, tselect_clamp=0)
    j.csrs[CSR_TDATA1] = 0
    # _enumerate_triggers() sets these True (riscv.c:6133-6135 defaults); force
    # the fallback chain straight to plain equality match for a single
    # hand-computable expected value (NAPOT/GE+LT need 2 triggers anyway — this
    # mock only tables 1 — so they'd fail on trigger-count, not settle the test
    # on a deterministic single-trigger-count-independent value).
    EspUsbJtag._enumerate_triggers(j)
    j.wp_allow_napot_trigger = False
    j.wp_allow_ge_lt_trigger = False

    ADDR = 0x3FC80010
    handle = j.add_watchpoint(ADDR, length=4, is_read=False, is_write=True)
    check("add_watchpoint returns a handle (unique_id 0)", handle, 0)
    check("trigger 0 now owns the handle", j.trigger_unique_id[0], 0)

    # Hand-computed expected tdata1 for a type-6 equality-match STORE watchpoint,
    # misa S/U/H all 0 (CsrMockJtag.read_register(CSR_MISA) default 0):
    #   type=6<<28, dmode=1<<27, action=DEBUG_MODE(1)<<12, m=1<<6,
    #   size.any=0<<16, chain.disable=0<<11, match.eq=0<<7, store=1<<1
    expected_tdata1 = (
        (6 << 28) | (1 << 27) | (1 << 12) | (1 << 6) | (1 << 1)
    )
    got_tdata1 = j.csrs[CSR_TDATA1]
    check(f"composed tdata1 == hand-computed 0x{expected_tdata1:08x}",
          got_tdata1, expected_tdata1)
    check("composed tdata2 == watchpoint address", j.csrs[CSR_TDATA2], ADDR)

    cleared = j.remove_watchpoint(handle)
    check("remove_watchpoint clears exactly 1 trigger", cleared, 1)
    check("trigger 0 freed after remove", j.trigger_unique_id[0], -1)


def test_add_watchpoint_napot_range():
    """A range watchpoint (length=16, naturally aligned) with NAPOT allowed and
    enough triggers available takes the NAPOT path FIRST (riscv.c:1284-1294,
    tried before GE+LT and equality) — one trigger, tdata2 = addr | ((len-1)>>1)."""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6}, tselect_clamp=0)
    j.csrs[CSR_TDATA1] = 0
    EspUsbJtag._enumerate_triggers(j)
    # wp_allow_napot_trigger defaults True (riscv.c:6135) — leave it.

    ADDR = 0x3FC80000           # 16-aligned
    LEN = 16
    handle = j.add_watchpoint(ADDR, length=LEN, is_read=True, is_write=False)
    check("NAPOT path: add_watchpoint succeeds with 1 trigger", handle, 0)
    expected_tdata2 = ADDR | ((LEN - 1) >> 1)
    check("NAPOT tdata2 == addr | ((length-1)>>1)", j.csrs[CSR_TDATA2], expected_tdata2)
    match_field = (j.csrs[CSR_TDATA1] & CSR_MCONTROL6_MATCH_MASK) >> 7
    check("NAPOT path selected (match field == CSR_MCONTROL6_MATCH_NAPOT=1)",
          match_field, 1)
    check("NAPOT tdata1 LOAD bit set (is_read=True)",
          bool(j.csrs[CSR_TDATA1] & CSR_MCONTROL6_LOAD_MASK), True)


def test_add_watchpoint_chained_ge_lt_range():
    """A range watchpoint where NAPOT is disabled falls to the chained GE+LT
    pair (riscv.c:1301-1330) — needs 2 CONSECUTIVE free triggers."""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6, 1: 1 << 6},
                     tselect_clamp=1)
    j.csrs[CSR_TDATA1] = 0
    EspUsbJtag._enumerate_triggers(j)
    j.wp_allow_napot_trigger = False       # force past NAPOT

    ADDR = 0x3FC80003           # NOT napot-aligned (also irrelevant: napot is off)
    LEN = 7
    handle = j.add_watchpoint(ADDR, length=LEN, is_read=False, is_write=True)
    check("chained GE+LT: add_watchpoint succeeds", handle, 0)
    check("both triggers 0 and 1 claimed by the same handle",
          j.trigger_unique_id, [0, 0])
    # GE_1 (idx 0): tdata2 == ADDR, match field == GE(2); LT_2 (idx 1):
    # tdata2 == ADDR+LEN, match field == LT(3) -- riscv.c:1303-1312.
    # CsrMockJtag's csrs dict holds only the LAST value written per regno, so
    # read back via a fresh TSELECT walk using _set_trigger's own read path
    # is not re-derivable post-hoc here; instead assert via the ops log, which
    # records every (TSELECT-select, TDATA1, TDATA2) write in order.
    tdata_writes = [(op[1], op[2]) for op in j.ops if op[0] == "write_register"
                    and op[1] in (CSR_TDATA1, CSR_TDATA2)]
    # Per trigger: [TDATA1<-0 (disable), TDATA2<-tdata2, TDATA1<-tdata1] x2
    check("6 TDATA1/TDATA2 writes total (2 triggers x 3 writes each)",
          len(tdata_writes), 6)
    ge1_tdata2 = tdata_writes[1][1]
    ge1_tdata1 = tdata_writes[2][1]
    lt2_tdata2 = tdata_writes[4][1]
    lt2_tdata1 = tdata_writes[5][1]
    check("GE_1 tdata2 == ADDR", ge1_tdata2, ADDR)
    check("GE_1 match field == GE(2)",
          (ge1_tdata1 & CSR_MCONTROL6_MATCH_MASK) >> 7, 2)
    check("LT_2 tdata2 == ADDR+LEN", lt2_tdata2, ADDR + LEN)
    check("LT_2 match field == LT(3)",
          (lt2_tdata1 & CSR_MCONTROL6_MATCH_MASK) >> 7, 3)


def test_enumerate_triggers_refuses_when_not_halted():
    """_enumerate_triggers must refuse (ERROR_TARGET_NOT_HALTED) and touch NO
    register when the hart isn't halted (riscv.c:6380-6383) — checked BEFORE any
    CSR access, not just documented."""
    from espjtag.debug import ERROR_TARGET_NOT_HALTED
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6}, tselect_clamp=0)
    j.set_dm(DMSTATUS, 0)                      # NOT halted
    ret = EspUsbJtag._enumerate_triggers(j)
    check("enumerate_triggers refuses when not halted", ret, ERROR_TARGET_NOT_HALTED)
    check("no trigger state was populated", hasattr(j, "trigger_count"), False)
    check("no register was read/written before the halted check",
          [op for op in j.ops if op[0] in ("read_register", "write_register")], [])


def test_add_watchpoint_raises_when_no_free_trigger():
    """No triggers at all (trigger_count=0, e.g. a core with no trigger module)
    -> add_watchpoint must raise, not silently no-op."""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={}, tselect_clamp=-1)
    raised = False
    try:
        j.add_watchpoint(0x1000, length=4)
    except RuntimeError:
        raised = True
    check("add_watchpoint raises RuntimeError when no trigger is available", raised, True)


# =============================================================================
# #57 step-over-trigger: resume() after a watchpoint hit re-fires forever
# =============================================================================

def _arm_one_mcontrol6_watchpoint(j, addr=0x600A1FF0):
    """Shared setup: enumerate a single type-6 trigger and add a store
    watchpoint on it, mirroring the #57 repro (0x600A1FF0, is_write=True)."""
    j.csrs[CSR_TDATA1] = 0
    EspUsbJtag._enumerate_triggers(j)
    j.wp_allow_napot_trigger = False
    j.wp_allow_ge_lt_trigger = False
    handle = j.add_watchpoint(addr, length=4, is_read=False, is_write=True)
    return handle


def test_step_sets_and_clears_dcsr_step():
    """step() (riscv-013.c set_dcsr_config + riscv013_step_or_resume_current_
    hart, step=True): dcsr.step set before the resumereq handshake, the
    handshake waits for BOTH allresumeack and allhalted (a step halts again
    immediately), and dcsr.step is cleared again afterwards -- the hart must
    never be left in permanent single-step mode."""
    from espjtag.constants import DATA0
    j = MockEspUsbJtag()
    j.set_dm(DATA0, 0)                                    # initial dcsr read == 0
    j.set_dm(DMSTATUS, DM_ALLRESUMEACK | DM_ALLHALTED)    # step completes on try 1
    ok = j.step()
    check("step() reports success", ok, True)
    data0_writes = [v for a, v in j.dmi_writes if a == DATA0]
    check("dcsr.step was asserted at some point (a write had the STEP bit set)",
          any(v & CSR_DCSR_STEP for v in data0_writes), True)
    check("dcsr.step left clear by the FINAL write (never left stuck in step mode)",
          data0_writes[-1] & CSR_DCSR_STEP, 0)
    dmcontrol_writes = [v for a, v in j.dmi_writes if a == DMCONTROL]
    check("step() issued resumereq then dropped it (DMCONTROL sequence)",
          dmcontrol_writes, [DM_RESUMEREQ | DM_DMACTIVE, DM_DMACTIVE])


def test_step_fails_if_hart_never_rehalts():
    """step() must report failure (not hang forever) if dmstatus never shows
    allhalted after resumereq -- mirrors riscv013_step_or_resume_current_hart's
    own bounded retry loop (riscv.c:5667, `for (i = 0; i < 256; ++i)`)."""
    from espjtag.constants import DATA0
    j = MockEspUsbJtag()
    j.set_dm(DATA0, 0)
    j.set_dm(DMSTATUS, DM_ALLRESUMEACK)        # resumeack but NEVER halted again
    ok = j.step()
    check("step() reports failure when the hart never re-halts", ok, False)
    data0_writes = [v for a, v in j.dmi_writes if a == DATA0]
    check("dcsr.step still cleared on the failure path (never left stuck)",
          data0_writes[-1] & CSR_DCSR_STEP, 0)


def test_detect_hit_bits_before_timing_needs_step():
    """_riscv_trigger_detect_hit_bits (riscv.c:1904): a type-6 trigger with
    hit0/hit1 == BEFORE must report need_single_step=True and CLEAR the hit
    bits on readback -- the exact condition that makes bare resume() re-fire
    forever (#57 root cause) when nothing acts on it."""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6}, tselect_clamp=0)
    handle = _arm_one_mcontrol6_watchpoint(j)
    j.tinfo_version = CSR_TINFO_VERSION_1
    # Simulate the hardware firing: OR the BEFORE hit pattern into the live tdata1.
    j.csrs[CSR_TDATA1] |= (CSR_MCONTROL6_HIT0_BEFORE << 0x16)  # hit0 offset
    unique_id, need_step = EspUsbJtag._riscv_trigger_detect_hit_bits(j)
    check("detected the trigger that hit", unique_id, handle)
    check("BEFORE timing -> need_single_step", need_step, True)
    check("hit bits cleared after detection",
          j.csrs[CSR_TDATA1] & (CSR_MCONTROL6_HIT0_MASK | CSR_MCONTROL6_HIT1_MASK), 0)


def test_halt_then_bare_resume_would_refire_without_the_fix():
    """Demonstrates the #57 bug class directly: after a trigger-cause halt with
    a BEFORE-timing hit, the trigger's tdata1/tdata2 are UNCHANGED (still armed,
    still matching the same address) unless something disables+steps+re-arms
    it. This proves the precondition the fix must act on -- if resume() did
    nothing extra, the exact same tdata1/tdata2 that just matched would still
    be live when the hart resumes at the identical (unretired) PC, so hardware
    re-traps immediately. (We don't model instruction retirement in the mock;
    the point is the trigger STATE that forces the re-fire is reproduced, which
    is what _step_over_trigger_if_needed's disable/step/re-arm is for.)"""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6}, tselect_clamp=0)
    handle = _arm_one_mcontrol6_watchpoint(j)
    tdata1_armed, tdata2_armed = j.csrs[CSR_TDATA1], j.csrs[CSR_TDATA2]
    j.tinfo_version = CSR_TINFO_VERSION_1
    j.set_dm(DMSTATUS, DM_ALLHALTED)
    j.csrs[CSR_DCSR] = CSR_DCSR_CAUSE_TRIGGER << 6     # dcsr.cause = trigger
    j.csrs[CSR_TDATA1] |= (CSR_MCONTROL6_HIT0_BEFORE << 0x16)
    EspUsbJtag._note_trigger_hit_for_step_over(j)
    pending = j._pending_step_over
    check("halt() recorded a pending step-over for the hit trigger",
          pending[0] if pending else None, handle)
    check("the recorded trigger config is exactly what was armed (so re-arm is faithful)",
          pending[1], [(0, tdata1_armed, tdata2_armed)])


def test_resume_disables_steps_and_rearms_the_hit_trigger():
    """THE fix, end to end: resume() after a recorded step-over pending must
    (1) clear the hit trigger's tdata1 (disable_watchpoints), (2) step() once
    (riscv_step_rtos_hart), (3) re-arm the EXACT original tdata1/tdata2
    (enable_watchpoints), (4) THEN do the normal dmcontrol resumereq resume.
    Checked via the TSELECT/TDATA1 write order, not just end state, since
    ordering is the load-bearing part (OpenOCD does disable-step-rearm BEFORE
    the real resume, never after)."""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6}, tselect_clamp=0)
    handle = _arm_one_mcontrol6_watchpoint(j)
    tdata1_armed, tdata2_armed = j.csrs[CSR_TDATA1], j.csrs[CSR_TDATA2]
    j._pending_step_over = (handle, [(0, tdata1_armed, tdata2_armed)])
    # step()'s internal poll (200 tries) sees ALLRESUMEACK|ALLHALTED (the step
    # completed) for as many reads as it takes; resume()'s OWN poll afterwards
    # needs ALLRUNNING instead (the real resume actually ran the hart) — model
    # both with a long scripted sequence: plenty of "step done" reads, then
    # "running" reads for the real resume's own poll.
    j.set_dm_sequence(DMSTATUS,
                      [DM_ALLRESUMEACK | DM_ALLHALTED] * 4 + [DM_ALLRUNNING] * 4)

    ok = j.resume()
    check("resume() succeeds", ok, True)
    check("pending step-over consumed (won't re-apply on the NEXT resume)",
          j._pending_step_over, None)
    check("the hit trigger ends re-armed with its ORIGINAL tdata1",
          j.csrs[CSR_TDATA1], tdata1_armed)
    check("the hit trigger ends re-armed with its ORIGINAL tdata2",
          j.csrs[CSR_TDATA2], tdata2_armed)

    # Ordering: CsrMockJtag.write_register logs to j.ops (not dmi_writes/DATA0 —
    # see its docstring). tdata1 must be written 0 (disable) strictly BEFORE it
    # is written back to tdata1_armed (re-arm) — proves disable happens before
    # re-arm, not the other way round.
    tdata1_writes = [v for op in j.ops if op[0] == "write_register"
                     and op[1] == CSR_TDATA1 for v in (op[2],)]
    first_rearm_idx = tdata1_writes.index(tdata1_armed)
    zero_before_rearm = [i for i, v in enumerate(tdata1_writes)
                         if v == 0 and i < first_rearm_idx]
    check("a TDATA1=0 (disable) write occurred before the re-arm write",
          len(zero_before_rearm) > 0, True)
    # resume()'s own resumereq must come AFTER the step-over's resumereq (the
    # step's resumereq is issued, cleared, then the real resume issues its own).
    dmcontrol_writes = [v for a, v in j.dmi_writes if a == DMCONTROL]
    check("TWO resumereq cycles happened (one for step(), one for the real resume)",
          dmcontrol_writes.count(DM_RESUMEREQ | DM_DMACTIVE), 2)


def test_resume_is_a_noop_passthrough_without_a_pending_hit():
    """The common case (resume after halt(), ebreak, or haltreq -- no trigger
    hit pending) must NOT be touched by #57's fix: no extra TSELECT/TDATA1
    churn, straight to the normal resumereq handshake."""
    j = MockEspUsbJtag()
    j.set_dm(DMSTATUS, DM_ALLRUNNING)
    ok = j.resume()
    check("resume() with no pending step-over succeeds normally", ok, True)
    dmcontrol_writes = [v for a, v in j.dmi_writes if a == DMCONTROL]
    check("exactly ONE resumereq cycle (no step() was invoked)",
          dmcontrol_writes, [DM_RESUMEREQ | DM_DMACTIVE, DM_DMACTIVE])


def test_halt_does_not_record_pending_step_for_non_trigger_causes():
    """_note_trigger_hit_for_step_over must be a no-op for ebreak/haltreq/step
    causes (dcsr.cause != TRIGGER) -- only a genuine trigger hit schedules a
    step-over, matching set_debug_reason's own cause switch (riscv.c:2647)."""
    j = CsrMockJtag(idcode=0x0000DC25, trigger_tinfo={0: 1 << 6}, tselect_clamp=0)
    _arm_one_mcontrol6_watchpoint(j)
    j.csrs[CSR_DCSR] = 3 << 6          # dcsr.cause = haltreq, NOT trigger
    EspUsbJtag._note_trigger_hit_for_step_over(j)
    check("no pending step-over recorded for a haltreq-caused halt",
          getattr(j, "_pending_step_over", None), None)


def main():
    print("=== RISC-V trigger module — NO-HARDWARE (software model) ===")
    test_can_use_napot_match()
    test_count_trailing_ones()
    test_find_next_free_trigger()
    test_wp_triggers_cache()
    test_set_trigger_accepts_clean_write()
    test_set_trigger_rejects_denied_config()
    test_set_trigger_reserved_blocks_immediately()
    test_enumerate_triggers_type6()
    test_enumerate_triggers_refuses_when_not_halted()
    test_add_watchpoint_equality_match_t6()
    test_add_watchpoint_napot_range()
    test_add_watchpoint_chained_ge_lt_range()
    test_add_watchpoint_raises_when_no_free_trigger()
    print("  --- #57 step-over-trigger ---")
    test_step_sets_and_clears_dcsr_step()
    test_step_fails_if_hart_never_rehalts()
    test_detect_hit_bits_before_timing_needs_step()
    test_halt_then_bare_resume_would_refire_without_the_fix()
    test_resume_disables_steps_and_rearms_the_hit_trigger()
    test_resume_is_a_noop_passthrough_without_a_pending_hit()
    test_halt_does_not_record_pending_step_for_non_trigger_causes()
    print("  (no hardware was touched)")
    print("VERDICT:", "PASS" if not _fail else "FAIL")
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
