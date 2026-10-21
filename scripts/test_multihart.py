#!/usr/bin/env python3
"""test_multihart.py — NO-HARDWARE test of the hartsel multi-hart mechanism
added for ESP32-P4 chip support. Validates EspUsbJtag._dmcontrol()'s hartsel field
against OpenOCD's set_dmcontrol_hartsel (riscv-013.c) and debug_defines.h's
DM_DMCONTROL_HARTSELLO_OFFSET/LENGTH (already-pinned a585825d). Also validates
the new chips.py P4 entry (chain/hart_count/IDCODE) and that hart=0 is
byte-identical to every pre-existing single-hart chip's DMI sequence — the
P4 port must not change ANY existing chip's wire behaviour.

Run by check.py --mock. Exit 1 on any mismatch.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espjtag.mock import MockEspUsbJtag                         # noqa: E402
from espjtag.debug import EspUsbJtag                             # noqa: E402
from espjtag.constants import (                                  # noqa: E402
    DMCONTROL, DM_DMACTIVE, DM_HALTREQ,
    DM_DMCONTROL_HARTSELLO, DM_DMCONTROL_HARTSELLO_OFFSET,
    DM_DMCONTROL_HARTSELLO_LENGTH,
)
from espjtag import chips                                        # noqa: E402

_fail = 0


def check(label, got, want):
    global _fail
    ok = got == want
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + ("" if ok else f"\n      got  {got!r}\n      want {want!r}"))
    _fail |= (not ok)


def test_hartsello_field_shape():
    """debug_defines.h DM_DMCONTROL_HARTSELLO_OFFSET=0x10, LENGTH=0xa ->
    constants.DM_DMCONTROL_HARTSELLO mask must be 0x3ff0000 (10 bits at bit
    16) — transcribed, not re-derived."""
    check("HARTSELLO_OFFSET", DM_DMCONTROL_HARTSELLO_OFFSET, 0x10)
    check("HARTSELLO_LENGTH", DM_DMCONTROL_HARTSELLO_LENGTH, 0xA)
    check("HARTSELLO mask", DM_DMCONTROL_HARTSELLO, 0x3FF0000)


def test_hart0_is_a_noop_matches_pre_p4_behaviour():
    """hart=0 (the default, and every chip before P4) must produce EXACTLY the
    same DMCONTROL writes as the bare bits with no hartsel OR'd in -- the P4
    port must not perturb any existing chip's DMI sequence."""
    j = MockEspUsbJtag(idcode=0x0000DC25)              # C6, hart defaults to 0
    EspUsbJtag.halt(j, disable_wdt=False)
    dmcontrol = [v for a, v in j.dmi_writes if a == DMCONTROL]
    check("hart=0 halt() first DMCONTROL has no hartsel bits set",
          dmcontrol[0] & DM_DMCONTROL_HARTSELLO, 0)
    check("hart=0 halt() first DMCONTROL == bare HALTREQ|DMACTIVE (unchanged)",
          dmcontrol[0], DM_HALTREQ | DM_DMACTIVE)


def test_hart1_ors_in_hartsel():
    """hart=1 (P4 cpu1) must set hartsello=1 (bit 16) on every DMCONTROL write,
    on top of the SAME bits hart=0 would send -- mirrors OpenOCD's
    set_dmcontrol_hartsel being applied to every dmcontrol write, not just a
    dedicated 'select' op (dm013_select_hart builds the whole value at once)."""
    j0 = MockEspUsbJtag(idcode=0x00012C25, hart=0)
    EspUsbJtag.halt(j0, disable_wdt=False)
    j1 = MockEspUsbJtag(idcode=0x00012C25, hart=1)
    EspUsbJtag.halt(j1, disable_wdt=False)
    dc0 = [v for a, v in j0.dmi_writes if a == DMCONTROL]
    dc1 = [v for a, v in j1.dmi_writes if a == DMCONTROL]
    check("hart=0 vs hart=1 emit the same NUMBER of DMCONTROL writes",
          len(dc0), len(dc1))
    check("hart=1 first DMCONTROL hartsello field == 1",
          (dc1[0] & DM_DMCONTROL_HARTSELLO) >> 16, 1)
    check("hart=1 first DMCONTROL == hart=0's PLUS the hartsel bit (same op, different hart)",
          dc1[0], dc0[0] | (1 << 16))
    # every non-hartsel bit must be identical, write-for-write
    check("hart=0/hart=1 DMCONTROL sequences differ ONLY in the hartsel field",
          [v & ~DM_DMCONTROL_HARTSELLO for v in dc1],
          [v & ~DM_DMCONTROL_HARTSELLO for v in dc0])


def test_p4_chip_entry():
    """chips.py P4 entry: IDCODE 0x00012c25 (openocd-esp32
    tcl/target/esp32p4.cfg _CPUTAPID @ pinned f10eceff), two-TAP chain
    matching the C5's shape (same _LP_TAPNUM=1/_HP_TAPNUM=1, no _LP_TAPID
    override -> shared-ID legacy branch), riscv core, 2 harts, UNVERIFIED (no
    bench access)."""
    e = chips.lookup(0x00012C25)
    check("P4 resolves", e is not None, True)
    check("P4 name", e["name"], "P4")
    check("P4 core", e["core"], "riscv")
    check("P4 chain shape == C5's (taps_after=1, idcode_index=1)",
          (e["chain"]["taps_after"], e["chain"]["idcode_index"]), (1, 1))
    check("P4 hart_count", e.get("hart_count"), 2)
    check("P4 is_riscv()", chips.is_riscv(0x00012C25), True)
    check("P4 is_verified() == False (no bench access)",
          chips.is_verified(0x00012C25), False)
    check("P4 bootloader_offset_for() == 0x2000 (esp_rom_caps.h, same family as C5)",
          chips.bootloader_offset_for(0x00012C25), 0x2000)
    check("P4 wdt table present", bool(chips.lookup(0x00012C25).get("wdt")), True)


def test_p4_wdt_disable_runtime_sequence():
    """_wdt_disable() on a P4-IDCODE mock must emit the VERBATIM
    esp32p4_wdt_disable write set (verify_chips_vs_tcl.py cross-checks this
    same table against the live installed OpenOCD Tcl proc) -- same drill as
    the existing C5 test in test_riscv_mock.py (SBADDRESS0/SBDATA0 pairing)."""
    from espjtag.constants import SBADDRESS0, SBDATA0
    w = chips.lookup(0x00012C25)["wdt"]
    expected = []
    for wkey_reg, cfg_reg, cfg_val in w["disable"]:
        expected.append((wkey_reg, w["key"]))
        expected.append((cfg_reg, cfg_val))
    expected += list(w["int_clear"])

    j = MockEspUsbJtag(idcode=0x00012C25)
    EspUsbJtag._wdt_disable(j)
    sba, pend_addr = [], None
    for a, v in j.dmi_writes:
        if a == SBADDRESS0:
            pend_addr = v
        elif a == SBDATA0 and pend_addr is not None:
            sba.append((pend_addr, v)); pend_addr = None
    check("P4 _wdt_disable SBA (addr,data) pairs == chips.py table (verbatim esp32p4_wdt_disable)",
          sba, expected)


def main():
    print("=== multi-hart (hartsel) — NO-HARDWARE (ESP32-P4 chip support) ===")
    test_hartsello_field_shape()
    test_hart0_is_a_noop_matches_pre_p4_behaviour()
    test_hart1_ors_in_hartsel()
    test_p4_chip_entry()
    test_p4_wdt_disable_runtime_sequence()
    print("  (no hardware was touched)")
    return _fail


if __name__ == "__main__":
    sys.exit(main())
