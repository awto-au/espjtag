#!/usr/bin/env python3
"""test_s3_wdt_regs.py — NO-HARDWARE check of the S3 watchdog table in chips.py (#52).

- Every address is checked against the ESP32-S3 TRM v1.8 register map:
  RTC_CNTL (base 0x60008000) ch.10 summary, TIMG0/1 (0x6001F000/0x60020000) ch.13.
- _wdt_disable() is run against a fake memory: the super-WDT key must land in
  SWD_WPROTECT and AUTO_FEED in SWD_CONF, leaving the RTC WDT registers alone.
Run by check.py --mock.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espjtag.chips import CHIPS                                  # noqa: E402
from espjtag.xtensa import XtensaXDM                             # noqa: E402

_fail = 0

RTC_CNTL = 0x60008000
TRM = dict(
    RTC_WDTCONFIG0=RTC_CNTL + 0x0098,
    RTC_WDTWPROTECT=RTC_CNTL + 0x00B0,
    RTC_SWD_CONF=RTC_CNTL + 0x00B4,
    RTC_SWD_WPROTECT=RTC_CNTL + 0x00B8,
    TIMG0_WDTCONFIG0=0x6001F000 + 0x0048,
    TIMG0_WDTWPROTECT=0x6001F000 + 0x0064,
    TIMG1_WDTCONFIG0=0x60020000 + 0x0048,
    TIMG1_WDTWPROTECT=0x60020000 + 0x0064,
)
SWD_KEY = 0x8F1D312A
SWD_AUTO_FEED = 1 << 31


def check(label, got, want):
    global _fail
    ok = got == want
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + ("" if ok else f"\n      got  {got!r}\n      want {want!r}"))
    _fail |= (not ok)


def _s3():
    return next(c for c in CHIPS.values() if c.get("name") == "S3")


def test_table_matches_trm():
    w = _s3()["wdt"]
    check("disable table", sorted(w["disable"]), sorted([
        (TRM["TIMG0_WDTWPROTECT"], TRM["TIMG0_WDTCONFIG0"], 0),
        (TRM["TIMG1_WDTWPROTECT"], TRM["TIMG1_WDTCONFIG0"], 0),
        (TRM["RTC_WDTWPROTECT"], TRM["RTC_WDTCONFIG0"], 0),
    ]))
    check("swd.wprotect = RTC_CNTL_SWD_WPROTECT_REG", w["swd"]["wprotect"],
          TRM["RTC_SWD_WPROTECT"])
    check("swd.conf = RTC_CNTL_SWD_CONF_REG", w["swd"]["conf"], TRM["RTC_SWD_CONF"])
    check("swd.key", w["swd"]["key"], SWD_KEY)
    check("swd.auto_feed", w["swd"]["auto_feed"], SWD_AUTO_FEED)


class _FakeXdm(XtensaXDM):
    def __init__(self, chip):                                    # no transport
        self._chip_dict = chip
        self.mem = {}
        self.writes = []

    def read_mem32(self, addr):
        return self.mem.get(addr, 0)

    def write_mem32(self, addr, value):
        self.writes.append((addr, value))
        self.mem[addr] = value


def test_wdt_disable_writes():
    x = _FakeXdm(_s3())
    x.mem[TRM["RTC_SWD_CONF"]] = 300 << 18             # TRM reset value (SWD_SIGNAL_WIDTH=300)
    x._wdt_disable()
    swd = [(a, v) for a, v in x.writes
           if a in (TRM["RTC_SWD_CONF"], TRM["RTC_SWD_WPROTECT"])]
    check("super-WDT: unlock then auto-feed", swd, [
        (TRM["RTC_SWD_WPROTECT"], SWD_KEY),
        (TRM["RTC_SWD_CONF"], 300 << 18 | SWD_AUTO_FEED),
    ])
    check("RTC WDT write-protect not clobbered by the SWD step",
          [v for a, v in x.writes if a == TRM["RTC_WDTWPROTECT"]],
          [0x50D83AA1])


def main():
    print("=== S3 watchdog register table — NO HARDWARE ===")
    test_table_matches_trm()
    test_wdt_disable_writes()
    print("  (no hardware was touched)")
    print("VERDICT:", "PASS" if not _fail else "FAIL")
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
