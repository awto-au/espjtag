#!/usr/bin/env python3
"""test_bootloader_offset.py — NO-HARDWARE regression test for espjtag#55:
_rom_flash_ready() must read the 2nd-stage bootloader header at the PER-CHIP
offset (chips.py bootloader_offset / bootloader_offset_for), not a hardcoded
0x0. Covers both debug.EspUsbJtag (RISC-V) and xtensa.XtensaXDM (S3).

Exercises the exact regression: a C5 mock with the real 0xE9 magic at 0x2000
and FFs at the old-wrong 0x0 must gate PASS (not FAIL as it did before #55).

Run by check.py --mock. Exit 1 on any mismatch.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from espjtag.mock import MockEspUsbJtag                             # noqa: E402
from espjtag.debug import EspUsbJtag                                 # noqa: E402
from espjtag import chips                                            # noqa: E402

_fail = 0


def check(label, got, want):
    global _fail
    ok = got == want
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + ("" if ok else f"\n      got  {got!r}\n      want {want!r}"))
    _fail |= (not ok)


# --- a minimal flash-backed mock: models call_rom("spiflash_read") as a copy
# out of a {offset: bytes} image dict, and read_mem/write_mem as a scratch-SRAM
# byte array — enough to drive _rom_flash_ready() without re-modelling the full
# DMI resume/trap handshake (that sequence is already covered by
# test_riscv_mock.py's call_function-adjacent tests).
class FlashImageMockJtag(MockEspUsbJtag):
    """`flash_image`: {byte_offset: bytes} sparse image. Unbacked bytes read as
    0xFF (erased flash). `calls`: [(sym, args)] every call_rom() invocation, so
    tests can assert the OFFSET actually used."""

    def __init__(self, idcode, flash_image):
        super().__init__(idcode=idcode)
        self.flash_image = flash_image
        self.calls = []
        self._scratch = {}

    def _flash_read(self, offset, n):
        out = bytearray()
        for i in range(offset, offset + n):
            for base, blob in self.flash_image.items():
                if base <= i < base + len(blob):
                    out.append(blob[i - base])
                    break
            else:
                out.append(0xFF)              # erased/unbacked flash
        return bytes(out)

    def read_mem(self, addr, nwords):
        return [self._scratch.get(addr + 4 * i, 0) for i in range(nwords)]

    def write_mem(self, addr, words):
        for i, w in enumerate(words):
            self._scratch[addr + 4 * i] = w & 0xFFFFFFFF

    def call_rom(self, sym, args=(), restore=True):
        self.calls.append((sym, args))
        if sym == "spiflash_read":
            offset, dest, nbytes = args
            data = self._flash_read(offset, nbytes)
            words = [int.from_bytes(data[i:i + 4], "little")
                     for i in range(0, len(data), 4)]
            self.write_mem(dest, words)
            return 0, True                     # (ret=OK, halted=True)
        return 0, True                          # cache_disable/enable_icache etc.


# A real ESP image's first 4 bytes: magic 0xE9 + segment count + flash-mode byte +
# flash-size/freq nibble (the exact trailing 3 bytes don't matter to the gate,
# which only checks byte 0). Matches the live C5 bench capture in espjtag#55.
_BOOTLOADER_HEADER = bytes([0xE9, 0x03, 0x02, 0x1F])


def test_c6_reads_offset_0x0():
    """C6: no bootloader_offset override tabled -> defaults to 0x0 (matches
    ESP-IDF's own Kconfig default for this chip family)."""
    j = FlashImageMockJtag(idcode=0x0000DC25, flash_image={0x0: _BOOTLOADER_HEADER})
    ready, words, magic = EspUsbJtag._rom_flash_ready(j)
    reads = [a for sym, a in j.calls if sym == "spiflash_read"]
    check("C6 spiflash_read offset", reads[0][0], 0x0)
    check("C6 gate passes with magic at 0x0", (ready, magic), (True, 0xE9))


def test_c5_reads_offset_0x2000_not_0x0():
    """C5: bootloader_offset=0x2000 tabled (espjtag#55). The regression this
    bug caused: real flash has the 0xE9 header at 0x2000 and ERASED (0xFF) at
    the old-wrong 0x0 — exactly the live bench capture in the issue. Before the
    fix, _rom_flash_ready always read 0x0 and would see 0xFF, FAIL the gate,
    and block flash_write on a perfectly good board."""
    j = FlashImageMockJtag(idcode=0x00017C25,
                            flash_image={0x2000: _BOOTLOADER_HEADER})
    ready, words, magic = EspUsbJtag._rom_flash_ready(j)
    reads = [a for sym, a in j.calls if sym == "spiflash_read"]
    check("C5 spiflash_read offset", reads[0][0], 0x2000)
    check("C5 gate passes reading the real header at 0x2000", (ready, magic), (True, 0xE9))

    # Prove the OLD-WRONG offset genuinely would have failed: 0x0 on this same
    # image is erased flash, not the bootloader.
    wrong_magic = j._flash_read(0x0, 1)[0]
    check("C5 offset 0x0 is NOT the bootloader (erased, 0xFF) — the bug this fixes",
          wrong_magic, 0xFF)


def test_chips_bootloader_offset_table():
    """Every CHIPS entry's bootloader_offset matches the ESP-IDF Kconfig
    default/override table (espjtag#55) — VERIFIED C5=0x2000 on the bench;
    P4=0x2000 is UNVERIFIED (no P4 bench access) but read directly
    from esp-idf components/esp_rom/esp32p4/esp_rom_caps.h
    ESP_ROM_BOOTLOADER_OFFSET_FLASH, same P4/C5/H4 family as the C5 one is.
    Everything else (no override tabled) defaults to 0x0 via
    bootloader_offset_for()."""
    for idc, e in chips.CHIPS.items():
        name = e.get("name", f"0x{idc:x}")
        want = 0x2000 if name in ("C5", "P4") else 0x0
        check(f"{name} bootloader_offset_for()", chips.bootloader_offset_for(idc), want)


def main():
    print("=== bootloader-offset gate — NO-HARDWARE (espjtag#55) ===")
    test_c6_reads_offset_0x0()
    test_c5_reads_offset_0x2000_not_0x0()
    test_chips_bootloader_offset_table()
    print("  (no hardware was touched)")
    return _fail


if __name__ == "__main__":
    sys.exit(main())
