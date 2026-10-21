"""espjtag.debug — the full RISC-V debugger built on the transport: halt/resume,
GPR/CSR read+write (abstract command), memory read/write (System Bus Access),
diag, the reset_run() method, and the trigger module (watchpoints/breakpoints,
#trigger-port). EspUsbJtag is the public class most users want.

DM register addresses + field offsets used here are ported from the RISC-V debug
spec via openocd-esp32 src/target/riscv/debug_defines.h (BSD-2-Clause OR
CC-BY-4.0); the reset_run() DMI sequence was captured from OpenOCD's `reset run`
-d3 log. The trigger module is ported function-by-function from openocd-esp32
src/target/riscv/riscv.c @ a585825d4d0bbd5b8aff83e01816d2f2c04ffd84 (GPL-2.0-or-
later) per docs/FAITHFUL-REIMPLEMENTATION.md. Pinned upstream: ../upstream.lock.
Provenance: ../ACKNOWLEDGEMENTS.md.
"""

import os
import time
import zlib

import usb.util

from .constants import (
    VID, PID,
    DMCONTROL, DMSTATUS, HARTINFO, ABSTRACTCS, COMMAND, DATA0,
    SBCS, SBADDRESS0, SBDATA0, DMI_READ, DMI_WRITE, DMI_NOP,
    SB_SBERROR, SB_SBBUSYERROR,
    DM_DMACTIVE, DM_NDMRESET, DM_ACKHAVERESET, DM_RESUMEREQ, DM_HALTREQ,
    DM_DMCONTROL_HARTSELLO, DM_DMCONTROL_HARTSELLO_LENGTH,
    DM_ALLHALTED, DM_ALLRUNNING, DM_ALLHAVERESET,
    DM_ANYRUNNING, DM_ALLRESUMEACK, DM_ALLUNAVAIL,
    ABS_BUSY, ABS_CMDERR,
    CMD_ACCESS_REGISTER, AC_TRANSFER, AC_WRITE, AC_AARSIZE32, AC_POSTEXEC,
    CSR_DCSR, CSR_DPC, CSR_MSTATUS, MSTATUS_MIE, REG_GPR_BASE, PROGBUF0,
    DCSR_EBREAK_BITS, EBREAK,
    CSR_DCSR_STEP, CSR_DCSR_CAUSE_MASK, CSR_DCSR_CAUSE_TRIGGER,
    CSR_TINFO_VERSION_0, CSR_TINFO_VERSION_1,
    CSR_MCONTROL_HIT_MASK, CSR_MCONTROL6_HIT0_MASK, CSR_MCONTROL6_HIT1_MASK,
    CSR_MCONTROL6_HIT0_FALSE, CSR_MCONTROL6_HIT0_BEFORE,
    CSR_MCONTROL6_HIT0_AFTER, CSR_MCONTROL6_HIT0_IMMEDIATELY_AFTER,
    CSR_MISA, MISA_EXT_S, MISA_EXT_U, MISA_EXT_H, XLEN,
    CSR_TSELECT, CSR_TDATA1, CSR_TDATA2, CSR_TINFO,
    CSR_TDATA1_TYPE_MASK,
    CSR_TDATA1_TYPE_LEGACY, CSR_TDATA1_TYPE_MCONTROL,
    CSR_TDATA1_TYPE_ICOUNT, CSR_TDATA1_TYPE_ITRIGGER, CSR_TDATA1_TYPE_ETRIGGER,
    CSR_TDATA1_TYPE_MCONTROL6,
    CSR_TINFO_VERSION_MASK,
    CSR_MCONTROL_DMODE_MASK,
    CSR_MCONTROL_MASKMAX_MASK,
    CSR_MCONTROL_SIZELO_ANY, CSR_MCONTROL_ACTION_DEBUG_MODE, CSR_MCONTROL_ACTION_MASK,
    CSR_MCONTROL_CHAIN_DISABLED, CSR_MCONTROL_CHAIN_ENABLED, CSR_MCONTROL_CHAIN_MASK,
    CSR_MCONTROL_MATCH_MASK, CSR_MCONTROL_MATCH_EQUAL, CSR_MCONTROL_MATCH_NAPOT,
    CSR_MCONTROL_MATCH_GE, CSR_MCONTROL_MATCH_LT,
    CSR_MCONTROL_M_MASK, CSR_MCONTROL_S_MASK, CSR_MCONTROL_U_MASK,
    CSR_MCONTROL_EXECUTE_MASK, CSR_MCONTROL_LOAD_MASK, CSR_MCONTROL_STORE_MASK,
    CSR_MCONTROL_TYPE_MASK, CSR_MCONTROL_SIZELO_MASK, CSR_MCONTROL_SIZEHI_MASK,
    CSR_MCONTROL6_DMODE_MASK, CSR_MCONTROL6_VS_MASK, CSR_MCONTROL6_VU_MASK,
    CSR_MCONTROL6_SIZE_ANY, CSR_MCONTROL6_ACTION_DEBUG_MODE, CSR_MCONTROL6_ACTION_MASK,
    CSR_MCONTROL6_CHAIN_DISABLED, CSR_MCONTROL6_CHAIN_ENABLED, CSR_MCONTROL6_CHAIN_MASK,
    CSR_MCONTROL6_MATCH_MASK, CSR_MCONTROL6_MATCH_EQUAL, CSR_MCONTROL6_MATCH_NAPOT,
    CSR_MCONTROL6_MATCH_GE, CSR_MCONTROL6_MATCH_LT,
    CSR_MCONTROL6_M_MASK, CSR_MCONTROL6_S_MASK, CSR_MCONTROL6_U_MASK,
    CSR_MCONTROL6_EXECUTE_MASK, CSR_MCONTROL6_LOAD_MASK, CSR_MCONTROL6_STORE_MASK,
    CSR_MCONTROL6_TYPE_MASK, CSR_MCONTROL6_SIZE_MASK,
)
from . import chips
from .transport import EspUsbJtagTransport

# --- trigger-module return "codes" ------------------------------------------
# OpenOCD's riscv.c trigger-setup functions (set_trigger, try_setup_*,
# maybe_add_trigger_*, add_trigger) return a 3-way C int: ERROR_OK (0),
# ERROR_TARGET_RESOURCE_NOT_AVAILABLE (this trigger/config doesn't work, try the
# next one), or ERROR_FAIL (hard failure, abort). The actual numeric values live
# in OpenOCD's jtag/error.h / target/target.h, which are NOT in our pinned
# source bundle (only riscv.c + debug_defines.h were fetched) — porting the
# exact integers would be re-deriving from memory, not transcribing from a
# source we have. The 3-way branching the algorithm depends on IS upstream
# logic and is kept exactly; only the representation changes (Python sentinels
# instead of OpenOCD's int codes) — FAITHFUL-REIMPLEMENTATION.md's "port the
# behavior, the mechanism may differ when the host language differs".
ERROR_OK = None
ERROR_TARGET_RESOURCE_NOT_AVAILABLE = "resource_not_available"
ERROR_FAIL = "fail"
ERROR_TARGET_NOT_HALTED = "target_not_halted"

# get_field/set_field/field_value (target/register.h in upstream OpenOCD, not in
# our pinned source bundle — these are generic bitfield macros, not RISC-V-
# specific facts, so transcribing their trivial definitions here is not "porting
# from memory": `get_field(reg, mask) = (reg & mask) >> trailing_zeros(mask)`.
def _get_field(reg, mask):
    return (reg & mask) >> ((mask & -mask).bit_length() - 1) if mask else 0


def _set_field(reg, mask, value):
    shift = (mask & -mask).bit_length() - 1 if mask else 0
    return (reg & ~mask) | ((value << shift) & mask)


def _field_value(mask, value):
    shift = (mask & -mask).bit_length() - 1 if mask else 0
    return (value << shift) & mask


def _crc32_le_raw(data, crc=0):
    """Reflected CRC-32 (poly 0xEDB88320), raw accumulation, no pre/post inversion — a
    host mirror of the ESP ROM crc32_le primitive, used to calibrate the host digest
    convention against the live ROM (see EspUsbJtag._crc_host)."""
    crc &= 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if (crc & 1) else 0)
    return crc & 0xFFFFFFFF


class EspUsbJtag(EspUsbJtagTransport):
    # === RISC-V Debug Module: halt / resume / examine =====================
    def dm_read(self, addr):
        d, _ = self.dmi_read(addr)
        return d

    def _dmcontrol(self, bits):
        """Write `bits` to DMCONTROL with this instance's hart OR'd into the
        hartsel field — OpenOCD's set_dmcontrol_hartsel (riscv-013.c), applied
        at every single dmcontrol write, not just a dedicated "select hart" op
        (dm013_select_hart does the same: build the whole value, hartsel
        included, in one write). self.hart defaults to 0 (transport.py
        __init__), under which this is `bits | 0` — IDENTICAL to the bare
        self.dmi_write(DMCONTROL, bits) every single-hart chip (C3/C5/C6/H2)
        used before ESP32-P4 chip support was added, so no existing behaviour
        changes. hartsello alone (10 bits, values 0-1023) covers the P4's 2
        harts; hartselhi (also in debug_defines.h) is for >1024 harts and
        unused by any currently-tabled chip."""
        hartsel = (self.hart & ((1 << DM_DMCONTROL_HARTSELLO_LENGTH) - 1)) << (
            (DM_DMCONTROL_HARTSELLO & -DM_DMCONTROL_HARTSELLO).bit_length() - 1)
        self.dmi_write(DMCONTROL, bits | hartsel)

    def halt(self, timeout=200, disable_wdt=True):
        """Request halt and wait for allhalted. Returns True if halted.

        On success, by default disables the chip's watchdogs (`disable_wdt`) so a
        WDT can't reset the chip out from under the debugger while it's held halted
        — the C6 halt-flakiness fix (probe-rs + OpenOCD both do this). Pass
        disable_wdt=False to leave them running."""
        self._dmcontrol(DM_HALTREQ | DM_DMACTIVE)
        for _ in range(timeout):
            if self.dm_read(DMSTATUS) & DM_ALLHALTED:
                self._dmcontrol(DM_DMACTIVE)     # drop haltreq
                if disable_wdt:
                    self._wdt_disable()
                # #57 step-over-trigger: record whether this halt was a
                # "before"-timing watchpoint hit, so the next resume() knows to
                # step past it first (set_debug_reason, riscv.c:2642).
                self._note_trigger_hit_for_step_over()
                return True
        return False

    def _wdt_disable(self):
        """Disable this chip's watchdogs (TG0/TG1/LP-RTC/super-WDT) so none resets
        the chip while it's halted. Data-driven from chips.py `wdt`; a no-op on a
        chip without a wdt table. Writes the memory-mapped WDT regs via SBA — the
        VERBATIM esp32c6.cfg esp32c6_wdt_disable sequence. Core must be halted."""
        w = self._chip().get("wdt")
        if not w:
            return                                 # untabled chip — leave WDTs alone
        for wkey_reg, cfg_reg, cfg_val in w["disable"]:
            self.write_mem32(wkey_reg, w["key"])   # unlock
            self.write_mem32(cfg_reg, cfg_val)     # zero/feed the config
        for reg, val in w.get("int_clear", ()):
            self.write_mem32(reg, val)             # clear pending WDT int state

    def resume(self, timeout=200):
        # #57 step-over-trigger (resume_prep, riscv.c:2916): if the last halt
        # was a "before"-timing watchpoint hit, disable it, step past the
        # watched instruction, and re-arm it before the real resume below —
        # otherwise this resume just re-fires at the identical PC forever.
        self._step_over_trigger_if_needed()
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)
        for _ in range(timeout):
            if self.dm_read(DMSTATUS) & DM_ALLRUNNING:
                self._dmcontrol(DM_DMACTIVE)
                return True
        return False

    def examine(self):
        """Bring the DM up: dmactive, clear any stale havereset, read dmstatus."""
        self._dmcontrol(DM_DMACTIVE)
        st = self.dm_read(DMSTATUS)
        if st & DM_ALLHAVERESET:
            self._dmcontrol(DM_ACKHAVERESET | DM_DMACTIVE)
        return st

    # === Abstract command: read/write GPRs and CSRs =======================
    def _abstract_wait(self, timeout=200):
        for _ in range(timeout):
            cs = self.dm_read(ABSTRACTCS)
            if not (cs & ABS_BUSY):
                err = (cs & ABS_CMDERR) >> 8
                if err:
                    # clear the error (W1C) for the next command
                    self.dmi_write(ABSTRACTCS, ABS_CMDERR)
                return err
        return -1

    def read_register(self, regno):
        """Read a hart register (GPR x1.. = 0x1000+n, CSR = its csr number).
        The hart must be halted. RAISES on an abstract-command error — returning
        stale DATA0 as if it were the register poisoned everything downstream
        when a hart was unexpectedly running (#33 diagnosis)."""
        cmd = (CMD_ACCESS_REGISTER | AC_AARSIZE32 | AC_TRANSFER | (regno & 0xFFFF))
        self.dmi_write(COMMAND, cmd)
        err = self._abstract_wait()
        if err:
            raise RuntimeError(f"read_register 0x{regno:x}: abstract cmderr={err} "
                               "(hart running? unsupported reg?)")
        return self.dm_read(DATA0)

    def write_register(self, regno, value):
        self.dmi_write(DATA0, value & 0xFFFFFFFF)
        cmd = (CMD_ACCESS_REGISTER | AC_AARSIZE32 | AC_TRANSFER | AC_WRITE
               | (regno & 0xFFFF))
        self.dmi_write(COMMAND, cmd)
        err = self._abstract_wait()
        if err:
            raise RuntimeError(f"write_register 0x{regno:x}: abstract cmderr={err} "
                               "(hart running? unsupported reg?)")
        return err

    # --- batched register access: one USB stream instead of ~1 ms per access.
    # Register-transfer abstract commands complete within the DTM-advertised idle
    # cycles _dmi_batch already inserts per scan (the same trust OpenOCD's batch
    # path places in dtmcs.idle); any overrun shows up as DTM busy in the batch
    # statuses or cmderr in the trailing ABSTRACTCS check, and we redo the whole
    # thing through the slow per-register path — never trust a dirty batch.
    def write_registers(self, pairs):
        """Write [(regno, value), ...] in ONE batched DMI stream."""
        reqs = []
        for regno, value in pairs:
            reqs.append((DATA0, value & 0xFFFFFFFF, DMI_WRITE))
            reqs.append((COMMAND, CMD_ACCESS_REGISTER | AC_AARSIZE32 | AC_TRANSFER
                         | AC_WRITE | (regno & 0xFFFF), DMI_WRITE))
        res = self._dmi_batch(reqs)
        if any(r is not None and r[1] != 0 for r in res) or self._abstract_wait():
            for regno, value in pairs:
                self.write_register(regno, value)

    def read_registers(self, regnos):
        """Read [regno, ...] in ONE batched DMI stream. Per register the stream is
        [COMMAND=transfer-read, READ(DATA0)]; the DTM read pipeline returns scan
        k's data in scan k+1's capture, so reg k's value lands in slot 2k+2 (the
        next COMMAND write / the trailing NOP)."""
        reqs = []
        for regno in regnos:
            reqs.append((COMMAND, CMD_ACCESS_REGISTER | AC_AARSIZE32 | AC_TRANSFER
                         | (regno & 0xFFFF), DMI_WRITE))
            reqs.append((DATA0, 0, DMI_READ))
        reqs.append((DATA0, 0, DMI_NOP))                    # flush the last read
        res = self._dmi_batch(reqs)
        if any(r is not None and r[1] != 0 for r in res) or self._abstract_wait():
            return [self.read_register(r) for r in regnos]
        return [res[2 * k + 2][0] for k in range(len(regnos))]

    # === System Bus Access: memory read/write (no running hart needed) =====
    @staticmethod
    def _sbcs_value(size_access=2, autoincrement=False, readondata=False,
                    readonaddr=False):
        # sbcs: sbaccess at [19:17] (2 = 32-bit), sbautoincrement[16],
        # sbreadondata[15], sbreadonaddr[20]
        cs = (size_access << 17)
        if autoincrement:
            cs |= (1 << 16)
        if readondata:
            cs |= (1 << 15)
        if readonaddr:
            cs |= (1 << 20)
        return cs

    def _sb_setup(self, size_access=2, autoincrement=False, readondata=False,
                  readonaddr=False):
        self.dmi_write(SBCS, self._sbcs_value(size_access, autoincrement,
                                              readondata, readonaddr))

    def read_mem32(self, addr):
        """Read one 32-bit word from `addr` via System Bus Access."""
        self._sb_setup(readonaddr=True)
        self.dmi_write(SBADDRESS0, addr & 0xFFFFFFFF)
        return self.dm_read(SBDATA0)

    def write_mem32(self, addr, value):
        """Write one 32-bit word to `addr` via System Bus Access."""
        self._sb_setup()
        self.dmi_write(SBADDRESS0, addr & 0xFFFFFFFF)
        self.dmi_write(SBDATA0, value & 0xFFFFFFFF)

    def _read_mem_slow(self, addr, nwords):
        """The original per-word read_mem: one USB round-trip per word. Kept as a
        proven fallback for when the batched path detects an SBA bus error."""
        self._sb_setup(autoincrement=True, readondata=True, readonaddr=True)
        self.dmi_write(SBADDRESS0, addr & 0xFFFFFFFF)
        out = [self.dm_read(SBDATA0) for _ in range(nwords)]
        self.dmi_write(SBCS, 0)
        return out

    def read_mem(self, addr, nwords):
        """Read nwords 32-bit words from addr via System Bus Access, BATCHED.

        SBA with sbreadonaddr+sbreadondata+sbautoincrement makes the hardware do
        the walking: writing SBADDRESS0 fetches word0 (readonaddr); each READ of
        SBDATA0 returns the current word AND auto-triggers the next bus read +
        bumps the address (readondata+autoincrement). So instead of N separate
        round-trips we issue ONE address write + a PIPELINE of READ(SBDATA0)
        scans in one batched scan stream (chunked at the IN-FIFO limit).

        Two pipelines stack and BOTH must be accounted for:
          * the RISC-V DTM read pipeline — a DMI READ scan returns the PREVIOUS
            DMI access's data, so the data for read #k appears in scan #k+1;
          * the SBA readondata pipeline — reading SBDATA0 yields the current word
            and arms the next.
        Net effect (verified on the bench against the per-word path and OpenOCD
        `mdw`): issue nwords+1 READ(SBDATA0) scans after the address write; the
        word stream lands in capture slots [2 .. 2+nwords). We then read SBCS and
        fall back to the slow per-word path if the burst hit an SBA bus error."""
        if nwords <= 0:
            return []
        self._ensure_dtmcs()
        # ONE batch = ONE USB round-trip for the whole read, including the SBA
        # setup, the closing SBCS error-check read AND the SBCS clear. The old
        # shape spent 4 round-trips (setup / burst / SBCS read / SBCS write) —
        # ~600 µs for a single-word read where probe-rs pays ~240.
        reqs = [(SBCS, self._sbcs_value(autoincrement=True, readondata=True,
                                        readonaddr=True), DMI_WRITE)]
        reqs += [(SBADDRESS0, addr & 0xFFFFFFFF, DMI_WRITE)]  # arms word0 fetch
        reqs += [(SBDATA0, 0, DMI_READ)] * (nwords + 1)       # pipeline of reads
        reqs += [(SBDATA0, 0, DMI_NOP)]                       # flush last DTM read
        reqs += [(SBCS, 0, DMI_READ), (SBCS, 0, DMI_NOP)]     # error check
        reqs += [(SBCS, 0, DMI_WRITE)]                        # clear/disarm SBA
        # single-word reads ride the memoized one-chunk batch (OUT template
        # cached per address): ~150 us instead of rebuilding every scan.
        if nwords == 1:
            res = self.dmi_batch_cached(("rm1", addr), reqs)
        else:
            res = self._dmi_batch(reqs)
        # DTM pipeline: a scan's capture is the PREVIOUS op's result. Word i's
        # READ is issued at slot 2+i, so its data lands in slot 3+i; the SBCS
        # read issued at slot nwords+4 lands in slot nwords+5.
        out = [res[3 + i][0] for i in range(nwords)]
        sbcs = res[nwords + 5][0]
        # Guard: any DTM op-status 3 (busy) in the burst = pipeline stall.
        if any(res[i][1] == 3 for i in range(1, len(res))):
            self.dmi_write(SBCS, 0)
            return self._read_mem_slow(addr, nwords)
        # Guard: SBA bus error (sberror / sbbusyerror) -> the burst raced the bus.
        if sbcs & (SB_SBERROR | SB_SBBUSYERROR):
            self.dmi_write(SBCS, SB_SBERROR | SB_SBBUSYERROR)  # W1C clear
            self.dmi_write(SBCS, 0)
            return self._read_mem_slow(addr, nwords)
        return out

    def write_mem(self, addr, words):
        """Write a list of 32-bit `words` starting at `addr` via System Bus Access,
        BATCHED. Mirror of read_mem: arm autoincrement, write SBADDRESS0 once, then
        stream SBDATA0 writes — the hardware bumps the address per write. One handful
        of USB exchanges for the whole block instead of N round-trips. This is the
        data-staging primitive for the flash loader (#3): fill a RAM buffer fast.
        Returns the number of words written. Raises on an SBA bus error (don't
        silently half-write a flash buffer)."""
        words = list(words)
        if not words:
            return 0
        self._ensure_dtmcs()
        # SBA setup rides the stream head (it's just another DMI write); the
        # whole write costs the stream + its sticky-status read + ONE closing
        # batch for the SBCS error-check-and-clear.
        reqs = [(SBCS, self._sbcs_value(autoincrement=True), DMI_WRITE)]
        reqs += [(SBADDRESS0, addr & 0xFFFFFFFF, DMI_WRITE)]
        reqs += [(SBDATA0, w & 0xFFFFFFFF, DMI_WRITE) for w in words]
        # OUT-only stream (no capture, no FIFO chunking) + ONE sticky-status read.
        # sticky dmistat nonzero = some op was dropped -> redo slowly, correctly.
        if self._dmi_stream_writes(reqs):
            self._sb_setup(autoincrement=True)
            self.dmi_write(SBADDRESS0, addr & 0xFFFFFFFF)
            for w in words:
                self.dmi_write(SBDATA0, w & 0xFFFFFFFF)
        fin = self._dmi_batch([(SBCS, 0, DMI_READ), (SBCS, 0, DMI_NOP),
                               (SBCS, 0, DMI_WRITE)])
        sbcs = fin[1][0]
        if sbcs & (SB_SBERROR | SB_SBBUSYERROR):
            self.dmi_write(SBCS, SB_SBERROR | SB_SBBUSYERROR)   # W1C clear
            self.dmi_write(SBCS, 0)
            raise RuntimeError(f"write_mem SBA bus error at 0x{addr:08x} "
                               f"(sbcs=0x{sbcs:08x})")
        return len(words)

    # === "call a function on the target" — the flash-loader call primitive (#3) ==
    # A halted RISC-V hart can be made to run an arbitrary on-chip routine (a ROM
    # entry point, or code we staged in RAM) by the standard debug recipe:
    #   * stage args in a0..a7 (GPR x10..x17), sp in a scratch stack, ra at a
    #     scratch SRAM word holding `ebreak` (the return trap);
    #   * set dpc to the entry, make sure dcsr.ebreak* is set so the ebreak at `ra`
    #     re-enters debug mode, resume, and poll for the halt;
    #   * read a0 for the return value.
    # We SAVE and RESTORE every register we clobber (a0..aN, sp=x2, ra=x1, dpc,
    # dcsr) so the interrupted app is left byte-identical and can resume cleanly.
    # The hart MUST already be halted (call halt() first).
    _ABI_ARG_GPR = (10, 11, 12, 13, 14, 15, 16, 17)        # a0..a7 = x10..x17

    def call_function(self, entry, args=(), stack=None, trap=None,
                      timeout=4000, restore=True, wait=True):
        """Call on-target code at `entry` with up to 8 integer `args` (placed in
        a0..a7), returning a0. `stack` = an SP value (a scratch SRAM top, grows
        down); `trap` = address of an SRAM word we set to `ebreak` and point ra at,
        so the callee's `ret` traps back into debug mode. The hart must be halted.

        Saves/restores ra, sp, dpc, dcsr and the clobbered arg GPRs (restore=True)
        so the live app is undisturbed. Returns (a0, halted) — halted=False means
        the callee did not trap within `timeout` polls (left halted; inspect)."""
        if len(args) > len(self._ABI_ARG_GPR):
            raise ValueError("call_function: at most 8 integer args")
        # --- save state we're about to clobber (ONE batched read stream) ---
        # dcsr/mstatus are read regardless: the setup decisions below need them.
        save_regs = [CSR_DPC, CSR_DCSR, CSR_MSTATUS,
                     REG_GPR_BASE + 1, REG_GPR_BASE + 2]
        save_regs += [REG_GPR_BASE + self._ABI_ARG_GPR[i] for i in range(len(args))]
        vals = self.read_registers(save_regs)
        saved = dict(zip(save_regs, vals)) if restore else {}
        dcsr, mstatus = vals[1], vals[2]
        # --- ebreak return-trap + args + dpc (ONE batched write stream) ---
        self.write_mem32(trap, EBREAK)
        wr = [(REG_GPR_BASE + 1, trap)]                        # ra = trap
        if stack is not None:
            wr.append((REG_GPR_BASE + 2, stack))               # sp = scratch top
        wr += [(REG_GPR_BASE + self._ABI_ARG_GPR[i], a & 0xFFFFFFFF)
               for i, a in enumerate(args)]
        # ebreak -> debug mode, AND force prv=3: dcsr.prv is the privilege the hart
        # RESUMES into. If the app was halted in U-mode (dcsr.prv=0 — e.g. a Zephyr
        # user thread), the ROM callee would run unprivileged and fault/reset the
        # chip on the spot (#33 cycle-deterministic failure, dcsr.cause=5). The
        # saved dcsr restores the app's own privilege afterwards.
        want_dcsr = dcsr | DCSR_EBREAK_BITS | 0x3
        if want_dcsr != dcsr:
            wr.append((CSR_DCSR, want_dcsr))
        # MASK INTERRUPTS for the callee (what OpenOCD/probe-rs do for algorithm
        # runs): once the app is past early boot, a pending timer/radio interrupt
        # otherwise hijacks the resume at the first instruction — the callee never
        # runs, the trap never fires, the hart is left running (#33 on the C5).
        # The saved mstatus restores the app's interrupt state afterwards.
        if mstatus & MSTATUS_MIE:
            wr.append((CSR_MSTATUS, mstatus & ~MSTATUS_MIE))
        wr.append((CSR_DPC, entry))                            # jump target
        self.write_registers(wr)
        # resume handshake (mirrors _resume_go): set resumereq, wait for the hart
        # to ack it is running, THEN clear resumereq — otherwise the request can
        # race and never take. Then poll for the ebreak re-halt.
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)
        for _ in range(256):
            if self.dm_read(DMSTATUS) & (DM_ALLRESUMEACK | DM_ALLRUNNING):
                break
        self._dmcontrol(DM_DMACTIVE)                 # resumereq <- 0
        if not wait:
            return saved          # pipeline ctx: finish with _call_finish(ctx)
        return self._call_finish(saved, timeout, restore)

    def _call_finish(self, saved, timeout=4000, restore=True):
        """Second half of call_function: wait for the callee's trap, collect a0,
        restore the app's registers. Split out so a caller can do useful SBA
        work (e.g. stage the NEXT flash sector) while the callee runs — SBA is
        hart-independent; abstract register access is NOT, so between the resume
        and this call only memory (SBA) operations are legal."""
        halted = False
        for _ in range(timeout):
            if self.dm_read(DMSTATUS) & DM_ALLHALTED:
                halted = True
                break
        if not halted:
            # The callee never trapped (e.g. ROM spiflash_* spinning on a wedged
            # SPI1 when the app was halted mid-flash-IO — #33). Previously we left
            # the hart RUNNING the stuck callee, which poisoned every subsequent
            # abstract command in the session. Re-halt, restore the app's
            # registers, and fail CLEANLY so the caller can attempt recovery
            # (flash_init re-attaches the controller).
            self._dmcontrol(DM_HALTREQ | DM_DMACTIVE)
            for _ in range(256):
                if self.dm_read(DMSTATUS) & DM_ALLHALTED:
                    break
            self._dmcontrol(DM_DMACTIVE)
            if restore and (self.dm_read(DMSTATUS) & DM_ALLHALTED):
                self.write_registers(list(saved.items()))
            return None, False
        ret = self.read_register(REG_GPR_BASE + 10)                       # a0
        # --- restore the app's registers so resume() continues it cleanly ---
        if restore:
            self.write_registers(list(saved.items()))          # one batched stream
        return ret, halted

    # === flash over JTAG — Option A: call the ROM esp_rom_spiflash_* (#3) ========
    # Build on call_function + the per-chip ROM/SRAM table (chips.py). The whole
    # path is GATED behind a read-back self-test (_rom_flash_ready): we will not
    # erase or program unless a ROM read of a known region matches the XIP window,
    # because the legacy ROM g_rom_spiflash_chip state is NOT guaranteed to be
    # configured in a running app (measured: in the Zephyr app it is NOT — ROM
    # reads return constant garbage), and on the C6 ECO0 the ROM read/write path
    # needs revision-specific workarounds (see bootloader_flash.c rom_read_api_
    # workaround). Erasing/programming through a misconfigured ROM state could
    # corrupt the wrong flash region. So flash_write() refuses on a board that
    # doesn't pass the gate rather than risk a brick.

    def _chip(self):
        return chips.lookup(self.read_idcode()) or {}

    # === RISC-V trigger module: watchpoints/breakpoints (hardware triggers) ===
    # Ported from openocd-esp32 src/target/riscv/riscv.c @
    # a585825d4d0bbd5b8aff83e01816d2f2c04ffd84 (2026-10-21), function-by-function,
    # per docs/FAITHFUL-REIMPLEMENTATION.md. Types 3/4/5 (icount/itrigger/etrigger)
    # and the CLI `reserve_trigger` feature are OUT OF SCOPE (#trigger-port) — only
    # a `self.reserved_triggers` list (default all-False) is kept, since
    # set_trigger's own reserved-trigger check is real upstream logic.
    #
    # Per-instance trigger state (OpenOCD's `struct riscv_info *r` fields),
    # lazily populated by _enumerate_triggers() — mirrors how self._chip_dict /
    # self.abits etc. are set lazily elsewhere in this file, not in __init__
    # (the DM isn't up yet at construction time).
    #   self.trigger_count            r->trigger_count
    #   self.trigger_tinfo            r->trigger_tinfo[]   (list, len trigger_count)
    #   self.trigger_unique_id        r->trigger_unique_id[] (list, -1 == free)
    #   self.reserved_triggers        r->reserved_triggers[] (list, all False — #trigger-port scope)
    #   self.triggers_enumerated      r->triggers_enumerated
    #   self.tinfo_version            r->tinfo_version
    #   self.wp_allow_napot_trigger   r->wp_allow_napot_trigger     (riscv.c:6135, default True)
    #   self.wp_allow_ge_lt_trigger   r->wp_allow_ge_lt_trigger     (riscv.c:6134, default True)
    #   self.wp_allow_equality_match_trigger  (riscv.c:6133, default True)
    #   self._wp_triggers_negative_cache     r->wp_triggers_negative_cache[idx] -> {tdata1: {tdata2, ...}}
    #   self._next_unique_id           espjtag addition: OpenOCD's trigger->unique_id
    #                                   comes from the breakpoint/watchpoint object;
    #                                   espjtag has no such object, so add_watchpoint()
    #                                   hands out a monotonically increasing id itself.

    @staticmethod
    def _can_use_napot_match(address, length):
        """can_use_napot_match (riscv.c:797). NAPOT needs a power-of-2 length > 1
        and an address aligned to that length."""
        size_power_of_2 = (length & (length - 1)) == 0
        addr_aligned = (address & (length - 1)) == 0
        return length > 1 and size_power_of_2 and addr_aligned

    def _find_next_free_trigger(self, type_, chained, idx):
        """find_next_free_trigger (riscv.c:807). Pure scan over
        trigger_unique_id[]/trigger_tinfo[], no target I/O. Returns (ok, idx) —
        the C signature returns the found index via an in/out pointer starting
        the scan at *idx; we return it instead since Python has no out-param."""
        num_found = 0
        num_required = 2 if chained else 1
        i = idx
        while i < self.trigger_count:
            if self.trigger_unique_id[i] == -1:
                if self.trigger_tinfo[i] & (1 << type_):
                    num_found += 1
                    if num_required == num_found:
                        return True, i - (num_required - 1)
                    i += 1
                    continue
            # Trigger already occupied or incompatible type. Reset the counter.
            num_found = 0
            i += 1
        return False, idx

    def _find_first_trigger_by_id(self, unique_id):
        """find_first_trigger_by_id (riscv.c:842). Pure scan."""
        for i in range(self.trigger_count):
            if self.trigger_unique_id[i] == unique_id:
                return i
        return -1

    @staticmethod
    def _count_trailing_ones(reg):
        """count_trailing_ones (riscv.c:853). riscv_reg_t is 64-bit upstream
        (XLEN<=64 always fits); espjtag is RV32-only so reg is masked to 32
        bits by every caller — walk 32 bits, same as the C loop would for a
        32-bit value held in a wider register."""
        for i in range(XLEN):
            if (1 & (reg >> i)) == 0:
                return i
        return XLEN

    def _set_trigger(self, idx, tdata1, tdata2):
        """set_trigger (riscv.c:863). THE core DMI sequence: select the trigger
        via TSELECT, disable it (TDATA1=0), write TDATA2, write the real TDATA1,
        read back TDATA1+TDATA2, and verify the hardware actually accepted the
        config. Returns ERROR_OK (0) or an error string (truthy) — see the
        ERROR_* constants below; the caller inspects both forms exactly like the
        C caller inspects the int return.

        DEVIATION (documented, not a port gap): the upstream
        r->trigger_match_result_fixup hook (riscv.c:906-907) is an Espressif
        per-chip-variant function pointer registered by target files NOT in our
        pinned source bundle (only riscv.c + debug_defines.h were fetched) — it
        is NULL on vanilla targets and espjtag has no equivalent mechanism, so
        it is omitted rather than guessed at. See the trigger-support commit."""
        assert idx < self.trigger_count
        if self.reserved_triggers[idx]:
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE

        # Select which trigger to use
        self.write_register(CSR_TSELECT, idx)

        # Disable the trigger by writing 0 to it
        self.write_register(CSR_TDATA1, 0)

        # Set trigger data for tdata2 (tdata3 is out of scope — no type 3/4/5)
        self.write_register(CSR_TDATA2, tdata2 & 0xFFFFFFFF)

        # Set trigger data for tdata1
        self.write_register(CSR_TDATA1, tdata1 & 0xFFFFFFFF)

        # Read back tdata1, tdata2, and check if the configuration is supported
        tdata1_rb = self.read_register(CSR_TDATA1)
        tdata2_rb = self.read_register(CSR_TDATA2)

        type_ = _get_field(tdata1, CSR_TDATA1_TYPE_MASK)
        is_mcontrol = type_ == CSR_TDATA1_TYPE_MCONTROL

        # Determine if tdata1 supports what we need. For mcontrol triggers, we
        # don't care about the value in the read-only "maskmax" field.
        tdata1_ignore_mask = CSR_MCONTROL_MASKMAX_MASK if is_mcontrol else 0
        tdata1_config_denied = (tdata1 & ~tdata1_ignore_mask & 0xFFFFFFFF) != \
            (tdata1_rb & ~tdata1_ignore_mask & 0xFFFFFFFF)

        # Determine if tdata1.maskmax is sufficient (only relevant for mcontrol
        # triggers and NAPOT match type)
        unsupported_napot_range = False
        maskmax_value = 0
        if not tdata1_config_denied:
            is_napot_match = _get_field(tdata1_rb, CSR_MCONTROL_MATCH_MASK) == \
                CSR_MCONTROL_MATCH_NAPOT
            if is_mcontrol and is_napot_match:
                maskmax_value = _get_field(tdata1_rb, CSR_MCONTROL_MASKMAX_MASK)
                napot_size = self._count_trailing_ones(tdata2) + 1
                if maskmax_value < napot_size:
                    unsupported_napot_range = True

        tdata2_config_denied = (tdata2 & 0xFFFFFFFF) != tdata2_rb
        if tdata1_config_denied or tdata2_config_denied or unsupported_napot_range:
            self.write_register(CSR_TDATA1, 0)
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE

        return ERROR_OK

    def _maybe_add_trigger_t1(self, address, length, is_read, is_write,
                               is_execute, unique_id):
        """maybe_add_trigger_t1 (riscv.c:952). Legacy type-1 trigger (`bpcontrol`,
        pre-dates mcontrol). Not used by C3/C5/C6 (they report type 6) but small
        enough to port for completeness, as instructed. `misa` is read live via
        CSR_MISA (riscv_supports_extension equivalent — see the module docstring
        on self.wp_allow_* for why espjtag reads this instead of caching r->misa
        the way OpenOCD does)."""
        bpcontrol_x = 1 << 0
        bpcontrol_w = 1 << 1
        bpcontrol_r = 1 << 2
        bpcontrol_u = 1 << 3
        bpcontrol_s = 1 << 4
        bpcontrol_h = 1 << 5
        bpcontrol_m = 1 << 6
        bpcontrol_bpmatch = 0xF << 7
        bpcontrol_bpaction = 0xFF << 11

        idx = 0
        ok, idx = self._find_next_free_trigger(CSR_TDATA1_TYPE_LEGACY, False, idx)
        if not ok:
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE

        try:
            tdata1 = self.read_register(CSR_TDATA1)
        except RuntimeError:
            return ERROR_FAIL
        if tdata1 & (bpcontrol_r | bpcontrol_w | bpcontrol_x):
            # Trigger is already in use, presumably by user code.
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE

        # DEVIATION: upstream uses the cached r->misa (populated during target
        # examine, can't fail here); espjtag has no misa cache, so this is a live
        # read that CAN raise — a new failure mode vs. upstream, treated the same
        # as the TDATA1 read above (ERROR_FAIL) since there's no better upstream
        # precedent for "misa unreadable" to match against.
        try:
            misa = self.read_register(CSR_MISA)
        except RuntimeError:
            return ERROR_FAIL
        tdata1 = 0
        tdata1 = _set_field(tdata1, bpcontrol_r, is_read)
        tdata1 = _set_field(tdata1, bpcontrol_w, is_write)
        tdata1 = _set_field(tdata1, bpcontrol_x, is_execute)
        tdata1 = _set_field(tdata1, bpcontrol_u, bool(misa & MISA_EXT_U))
        tdata1 = _set_field(tdata1, bpcontrol_s, bool(misa & MISA_EXT_S))
        tdata1 = _set_field(tdata1, bpcontrol_h, bool(misa & MISA_EXT_H))
        tdata1 = _set_field(tdata1, bpcontrol_m, 1)
        tdata1 = _set_field(tdata1, bpcontrol_bpaction, 0)    # cause bp exception
        tdata1 = _set_field(tdata1, bpcontrol_bpmatch, 0)     # exact match
        tdata2 = address
        ret = self._set_trigger(idx, tdata1, tdata2)
        if ret != ERROR_OK:
            return ret
        self.trigger_unique_id[idx] = unique_id
        return ERROR_OK

    # --- trigger_request_info + the negative-result cache (riscv.c:999-1100) --
    # The C version is a {tdata1, tdata2} struct + an intrusive linked-list cache
    # keyed by trigger idx -> tdata1 -> {tdata2, ...} (tdata1_cache/tdata2_cache
    # list_head chains, wp_triggers_negative_cache[idx]). Ported as PYTHON DICTS,
    # not linked lists — FAITHFUL-REIMPLEMENTATION.md: "port the behavior, the
    # mechanism may differ when the host language differs". The DATA STRUCTURE
    # (nested map idx -> tdata1 -> set-of-tdata2) and its semantics (remember a
    # (idx, tdata1, tdata2) combo that set_trigger rejected, so a repeat attempt
    # skips re-probing hardware) are preserved exactly; the C list-splice-to-
    # front "most recently used" reordering (wp_triggers_cache_add's
    # list_move-to-tail on a hit) has no behavioral effect for a Python set
    # membership test, so it is dropped (not ported) rather than faked with an
    # OrderedDict for no observable benefit.

    def _create_wp_trigger_cache(self):
        """create_wp_trigger_cache (riscv.c:1045). calloc'd array of empty lists
        in C; a dict of dicts here, one slot per trigger index."""
        self._wp_triggers_negative_cache = [dict() for _ in range(self.trigger_count)]

    def _wp_triggers_cache_add(self, idx, tdata1, tdata2):
        """wp_triggers_cache_add (riscv.c:1055). Record that (tdata1, tdata2) is
        NOT supported on trigger `idx`."""
        tdata1_cache = self._wp_triggers_negative_cache[idx]
        tdata1_cache.setdefault(tdata1, set()).add(tdata2)

    def _wp_triggers_cache_search(self, idx, tdata1, tdata2):
        """wp_triggers_cache_search (riscv.c:1073). True if (tdata1, tdata2) is
        already known to be unsupported on trigger `idx`."""
        tdata2_cache = self._wp_triggers_negative_cache[idx].get(tdata1)
        if tdata2_cache is None:
            return False
        return tdata2 in tdata2_cache

    def _try_use_trigger_and_cache_result(self, idx, tdata1, tdata2):
        """try_use_trigger_and_cache_result (riscv.c:1088). Skip the hardware
        probe if the cache already knows this (idx, tdata1, tdata2) fails;
        otherwise probe via _set_trigger and memoize a negative result."""
        if self._wp_triggers_cache_search(idx, tdata1, tdata2):
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE
        ret = self._set_trigger(idx, tdata1, tdata2)
        if ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
            self._wp_triggers_cache_add(idx, tdata1, tdata2)
        return ret

    def _try_setup_single_match_trigger(self, unique_id, tdata1, tdata2, log=None):
        """try_setup_single_match_trigger (riscv.c:1102). Walk free triggers of
        the right type, trying each until one accepts."""
        if log:
            log(f"  trying to set up a match trigger: tdata1={tdata1:#x} "
                f"tdata2={tdata2:#x}")
        trigger_type = _get_field(tdata1, CSR_MCONTROL_TYPE_MASK)
        ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE
        idx = 0
        while True:
            ok, idx = self._find_next_free_trigger(trigger_type, False, idx)
            if not ok:
                break
            ret = self._try_use_trigger_and_cache_result(idx, tdata1, tdata2)
            if ret == ERROR_OK:
                self.trigger_unique_id[idx] = unique_id
                return ERROR_OK
            if ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                return ret
            idx += 1
        return ret

    def _try_setup_chained_match_triggers(self, unique_id, t1, t2, log=None):
        """try_setup_chained_match_triggers (riscv.c:1129). Pair-of-consecutive-
        triggers variant for range watchpoints: tries idx/idx+1 together, undoing
        idx if idx+1 fails but idx succeeded."""
        if log:
            log(f"  trying to set up a chain of match triggers: "
                f"t1=({t1[0]:#x},{t1[1]:#x}) t2=({t2[0]:#x},{t2[1]:#x})")
        trigger_type = _get_field(t1[0], CSR_MCONTROL_TYPE_MASK)
        ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE
        idx = 0
        while True:
            ok, idx = self._find_next_free_trigger(trigger_type, True, idx)
            if not ok:
                break
            ret = self._try_use_trigger_and_cache_result(idx, t1[0], t1[1])
            if ret == ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                idx += 1
                continue
            if ret != ERROR_OK:
                return ret

            ret = self._try_use_trigger_and_cache_result(idx + 1, t2[0], t2[1])
            if ret == ERROR_OK:
                self.trigger_unique_id[idx] = unique_id
                self.trigger_unique_id[idx + 1] = unique_id
                return ERROR_OK
            # Undo the setting of the previous trigger
            ret_undo = self._set_trigger(idx, 0, 0)
            if ret_undo != ERROR_OK:
                return ret_undo
            if ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                return ret
            idx += 1
        return ret

    # --- match_triggers_tdata1_fields (riscv.c:1170-1274) ----------------------
    # The C struct groups the tdata1 bits that are common to every match variant
    # (common), the "any size" encoding (size.any), the chain enable/disable
    # encodings (chain.enable/disable), and the four match-mode encodings
    # (match.napot/lt/ge/eq) — all pre-OR'd field_value()s, assembled once per
    # add_trigger() call and then combined by maybe_add_trigger_t2_t6_for_wp/bp.
    # Ported as a dict with the same key names (common/size/chain/match), not a
    # dataclass, to keep the field access (`fields["match"]["eq"]`) a direct
    # transliteration of the C (`fields.match.eq`) — easy to diff against source.
    #
    # DEVIATION (documented): the upstream r->trigger_match_result_fixup hook
    # (riscv.c:1227-1228, 1270-1271) is omitted here for the same reason as in
    # _set_trigger — an Espressif per-chip extension point with no implementation
    # in our pinned source bundle. tdata1_ignore_mask is therefore always 0 (its
    # C default before the hook would run) — unused by the wp/bp callers below,
    # which only reference .common/.size/.chain/.match, so this has no observable
    # effect on the port; recorded for completeness per riscv.c's own struct shape.

    def _fill_match_triggers_tdata1_fields_t2(self, is_execute, is_read, is_write):
        """fill_match_triggers_tdata1_fields_t2 (riscv.c:1193). Type-2 (legacy
        mcontrol) field composition. misa read live (see _maybe_add_trigger_t1)."""
        misa = self.read_register(CSR_MISA)
        return dict(
            common=(
                _field_value(CSR_MCONTROL_TYPE_MASK, CSR_TDATA1_TYPE_MCONTROL) |
                _field_value(CSR_MCONTROL_DMODE_MASK, 1) |
                _field_value(CSR_MCONTROL_ACTION_MASK, CSR_MCONTROL_ACTION_DEBUG_MODE) |
                _field_value(CSR_MCONTROL_M_MASK, 1) |
                _field_value(CSR_MCONTROL_S_MASK, bool(misa & MISA_EXT_S)) |
                _field_value(CSR_MCONTROL_U_MASK, bool(misa & MISA_EXT_U)) |
                _field_value(CSR_MCONTROL_EXECUTE_MASK, is_execute) |
                _field_value(CSR_MCONTROL_LOAD_MASK, is_read) |
                _field_value(CSR_MCONTROL_STORE_MASK, is_write)
            ),
            size=dict(any=_field_value(CSR_MCONTROL_SIZELO_MASK, CSR_MCONTROL_SIZELO_ANY & 3) |
                           _field_value(CSR_MCONTROL_SIZEHI_MASK, (CSR_MCONTROL_SIZELO_ANY >> 2) & 3)),
            chain=dict(enable=_field_value(CSR_MCONTROL_CHAIN_MASK, CSR_MCONTROL_CHAIN_ENABLED),
                       disable=_field_value(CSR_MCONTROL_CHAIN_MASK, CSR_MCONTROL_CHAIN_DISABLED)),
            match=dict(napot=_field_value(CSR_MCONTROL_MATCH_MASK, CSR_MCONTROL_MATCH_NAPOT),
                       lt=_field_value(CSR_MCONTROL_MATCH_MASK, CSR_MCONTROL_MATCH_LT),
                       ge=_field_value(CSR_MCONTROL_MATCH_MASK, CSR_MCONTROL_MATCH_GE),
                       eq=_field_value(CSR_MCONTROL_MATCH_MASK, CSR_MCONTROL_MATCH_EQUAL)),
            tdata1_ignore_mask=0,
        )

    def _fill_match_triggers_tdata1_fields_t6(self, is_execute, is_read, is_write):
        """fill_match_triggers_tdata1_fields_t6 (riscv.c:1233). Type-6 (mcontrol6)
        field composition — what C3/C5/C6 actually report. misa_h is read but, per
        the C source, only used to gate VS/VU (requires both H and S/U — these
        chips have no H extension, so VS/VU are always 0, same as upstream would
        compute from their real misa)."""
        misa = self.read_register(CSR_MISA)
        misa_s = bool(misa & MISA_EXT_S)
        misa_u = bool(misa & MISA_EXT_U)
        misa_h = bool(misa & MISA_EXT_H)
        return dict(
            common=(
                _field_value(CSR_MCONTROL6_TYPE_MASK, CSR_TDATA1_TYPE_MCONTROL6) |
                _field_value(CSR_MCONTROL6_DMODE_MASK, 1) |
                _field_value(CSR_MCONTROL6_ACTION_MASK, CSR_MCONTROL6_ACTION_DEBUG_MODE) |
                _field_value(CSR_MCONTROL6_M_MASK, 1) |
                _field_value(CSR_MCONTROL6_S_MASK, misa_s) |
                _field_value(CSR_MCONTROL6_U_MASK, misa_u) |
                _field_value(CSR_MCONTROL6_VS_MASK, misa_h and misa_s) |
                _field_value(CSR_MCONTROL6_VU_MASK, misa_h and misa_u) |
                _field_value(CSR_MCONTROL6_EXECUTE_MASK, is_execute) |
                _field_value(CSR_MCONTROL6_LOAD_MASK, is_read) |
                _field_value(CSR_MCONTROL6_STORE_MASK, is_write)
            ),
            size=dict(any=_field_value(CSR_MCONTROL6_SIZE_MASK, CSR_MCONTROL6_SIZE_ANY)),
            chain=dict(enable=_field_value(CSR_MCONTROL6_CHAIN_MASK, CSR_MCONTROL6_CHAIN_ENABLED),
                       disable=_field_value(CSR_MCONTROL6_CHAIN_MASK, CSR_MCONTROL6_CHAIN_DISABLED)),
            match=dict(napot=_field_value(CSR_MCONTROL6_MATCH_MASK, CSR_MCONTROL6_MATCH_NAPOT),
                       lt=_field_value(CSR_MCONTROL6_MATCH_MASK, CSR_MCONTROL6_MATCH_LT),
                       ge=_field_value(CSR_MCONTROL6_MATCH_MASK, CSR_MCONTROL6_MATCH_GE),
                       eq=_field_value(CSR_MCONTROL6_MATCH_MASK, CSR_MCONTROL6_MATCH_EQUAL)),
            tdata1_ignore_mask=0,
        )

    def _maybe_add_trigger_t2_t6_for_wp(self, unique_id, address, length, fields, log=None):
        """maybe_add_trigger_t2_t6_for_wp (riscv.c:1276). The real watchpoint
        setup: NAPOT range match (if wp_allow_napot_trigger), then chained GE+LT
        and LT+GE pairs (if wp_allow_ge_lt_trigger), then plain equality match
        (if wp_allow_equality_match_trigger) — exact fallback order/conditions."""
        ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE

        if length > 0:
            # Setting a load/store trigger ("watchpoint") on a range of addresses
            if self._can_use_napot_match(address, length):
                if self.wp_allow_napot_trigger:
                    if log:
                        log("  trying to setup NAPOT match trigger")
                    napot_tdata1 = (fields["common"] | fields["size"]["any"] |
                                     fields["chain"]["disable"] | fields["match"]["napot"])
                    napot_tdata2 = address | ((length - 1) >> 1)
                    ret = self._try_setup_single_match_trigger(
                        unique_id, napot_tdata1, napot_tdata2, log=log)
                    if ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                        return ret
                elif log:
                    log("  NAPOT match triggers are disabled for watchpoints.")

            if self.wp_allow_ge_lt_trigger:
                if log:
                    log("  trying to setup GE+LT chained match trigger pair")
                ge_1 = (fields["common"] | fields["size"]["any"] | fields["chain"]["enable"] |
                        fields["match"]["ge"], address)
                lt_2 = (fields["common"] | fields["size"]["any"] | fields["chain"]["disable"] |
                        fields["match"]["lt"], address + length)
                ret = self._try_setup_chained_match_triggers(unique_id, ge_1, lt_2, log=log)
                if ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                    return ret

                if log:
                    log("  trying to setup LT+GE chained match trigger pair")
                lt_1 = (fields["common"] | fields["size"]["any"] | fields["chain"]["enable"] |
                        fields["match"]["lt"], address + length)
                ge_2 = (fields["common"] | fields["size"]["any"] | fields["chain"]["disable"] |
                        fields["match"]["ge"], address)
                ret = self._try_setup_chained_match_triggers(unique_id, lt_1, ge_2, log=log)
                if ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                    return ret
            elif log:
                log("  LT+GE chained match triggers are disabled for watchpoints.")

        if self.wp_allow_equality_match_trigger:
            if log:
                log("  trying to setup equality match trigger")
            eq_tdata1 = (fields["common"] | fields["size"]["any"] |
                         fields["chain"]["disable"] | fields["match"]["eq"])
            ret = self._try_setup_single_match_trigger(unique_id, eq_tdata1, address, log=log)
            if ret != ERROR_OK:
                return ret
        elif log:
            log("  equality match triggers are disabled for watchpoints.")

        if ret == ERROR_OK and length > 1:
            if not self.range_trigger_fallback_encountered:
                # This message is displayed only once per target to avoid
                # overwhelming the user with such messages on resume.
                if log:
                    log("  WARNING: could not set a trigger that will match a "
                        "whole address range. As a fallback, this trigger (and "
                        "maybe others) will only match against the first "
                        "address of the range.")
            self.range_trigger_fallback_encountered = True

        return ret

    def _maybe_add_trigger_t2_t6_for_bp(self, unique_id, address, fields, log=None):
        """maybe_add_trigger_t2_t6_for_bp (riscv.c:1372). Breakpoint variant:
        equality match only."""
        if log:
            log("  trying to setup equality match trigger")
        eq_tdata1 = (fields["common"] | fields["size"]["any"] |
                     fields["chain"]["disable"] | fields["match"]["eq"])
        return self._try_setup_single_match_trigger(unique_id, eq_tdata1, address, log=log)

    def _maybe_add_trigger_t2_t6(self, unique_id, address, length, is_read,
                                  is_write, is_execute, fields, log=None):
        """maybe_add_trigger_t2_t6 (riscv.c:1385). Dispatches bp vs wp based on
        is_execute."""
        if is_execute:
            assert not is_read and not is_write
            return self._maybe_add_trigger_t2_t6_for_bp(unique_id, address, fields, log=log)
        assert is_read or is_write
        return self._maybe_add_trigger_t2_t6_for_wp(unique_id, address, length, fields, log=log)

    # --- trigger enumeration (riscv.c:6294-6447) --------------------------
    # MUST run (and populate trigger_count/trigger_tinfo) before
    # find_next_free_trigger is usable. Not in the "add a watchpoint" call path
    # directly, but add_trigger() calls riscv_enumerate_triggers() first thing,
    # exactly as upstream does — ported below as _enumerate_triggers().

    def _check_if_trigger_exists(self, index):
        """check_if_trigger_exists (riscv.c:6294). Write TSELECT=index, read it
        back; if the hart doesn't support that many triggers, hardware clamps
        tselect to a lower value and the readback won't match. A write failure
        (can't select -> no trigger module) is RESOURCE_NOT_AVAILABLE; a READ
        failure is the harder ERROR_FAIL (riscv.c:6300-6301) — distinct from a
        clean "index out of range", so the caller must not treat them the same."""
        try:
            self.write_register(CSR_TSELECT, index)
        except RuntimeError:
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE
        try:
            tselect_rb = self.read_register(CSR_TSELECT)
        except RuntimeError:
            return ERROR_FAIL
        # Mask off the top bit, used as tdrmode in legacy RISC-V Debug Spec (old
        # v0.11 revisions). espjtag is RV32-only: top bit is bit 31.
        tselect_rb &= ~(1 << (XLEN - 1)) & 0xFFFFFFFF
        if tselect_rb != index:
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE
        return ERROR_OK

    def _get_trigger_types(self, tdata1):
        """get_trigger_types (riscv.c:6315). Reads tinfo to determine supported
        trigger types for the currently-selected trigger; if reading tinfo FAILS
        (abstract cmderr -> our read_register raises, mirroring riscv_reg_get
        returning != ERROR_OK), falls back to deriving a single-type bitmask from
        tdata1's own type field. Returns (ret, trigger_tinfo)."""
        try:
            tinfo = self.read_register(CSR_TINFO)
        except RuntimeError:
            type_ = _get_field(tdata1, CSR_TDATA1_TYPE_MASK)
            if type_ == 0:
                return ERROR_TARGET_RESOURCE_NOT_AVAILABLE, None
            return ERROR_OK, 1 << type_
        # tinfo.INFO == 1: trigger doesn't exist. tinfo == 0 or LSB set: invalid.
        if tinfo == 0 or tinfo & 0x1:
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE, None
        return ERROR_OK, tinfo

    def _disable_trigger_if_dmode(self, tdata1):
        """disable_trigger_if_dmode (riscv.c:6335). If the currently-selected
        trigger (already selected by the caller) has dmode set, it was left
        armed by a previous/killed debug session — disable it (write TDATA1=0)."""
        type_ = _get_field(tdata1, CSR_TDATA1_TYPE_MASK)
        dmode_is_set = False
        if type_ == CSR_TDATA1_TYPE_LEGACY:
            # On these older cores we don't support software using triggers.
            dmode_is_set = True
        elif type_ == CSR_TDATA1_TYPE_MCONTROL:
            dmode_is_set = bool(tdata1 & CSR_MCONTROL_DMODE_MASK)
        elif type_ == CSR_TDATA1_TYPE_MCONTROL6:
            dmode_is_set = bool(tdata1 & CSR_MCONTROL6_DMODE_MASK)
        elif type_ in (CSR_TDATA1_TYPE_ICOUNT, CSR_TDATA1_TYPE_ITRIGGER,
                       CSR_TDATA1_TYPE_ETRIGGER):
            # Types 3/4/5 (icount/itrigger/etrigger) are out of scope
            # (#trigger-port) — espjtag never SETS these, but if a hart reports
            # one already dmode-armed (leftover from another debugger), it must
            # still be disabled like upstream does. dmode bit is at the same
            # DMODE_OFFSET=(XLEN-5) for every trigger type per the debug spec,
            # so reuse CSR_MCONTROL_DMODE_MASK rather than add unused per-type
            # dmode constants for types this port never constructs.
            dmode_is_set = bool(tdata1 & CSR_MCONTROL_DMODE_MASK)
        if not dmode_is_set:
            return ERROR_OK
        self.write_register(CSR_TDATA1, 0)
        return ERROR_OK

    def _enumerate_triggers(self, log=None):
        """riscv_enumerate_triggers (riscv.c:6373). Count triggers and populate
        trigger_tinfo for each; idempotent (triggers_enumerated guard). Disables
        any hardware trigger left with dmode set (maybe from a killed session).
        The hart MUST be halted — enforced below (riscv.c:6380-6383), not just
        documented: CSR access on a running hart is undefined/may silently read
        garbage rather than cleanly erroring, so this is checked BEFORE any
        register I/O, exactly where upstream checks it."""
        if getattr(self, "triggers_enumerated", False):
            return ERROR_OK

        if not (self.dm_read(DMSTATUS) & DM_ALLHALTED):
            if log:
                log("  unable to enumerate triggers: target not halted")
            return ERROR_TARGET_NOT_HALTED

        chips.require_riscv(self.read_idcode(), op="trigger enumeration")

        # trigger-module instance state, lazily created here (see the class-level
        # doc block above _can_use_napot_match for the full field list).
        self.wp_allow_napot_trigger = True             # riscv.c:6135
        self.wp_allow_ge_lt_trigger = True              # riscv.c:6134
        self.wp_allow_equality_match_trigger = True     # riscv.c:6133
        self.range_trigger_fallback_encountered = False

        try:
            orig_tselect = self.read_register(CSR_TSELECT)
        except RuntimeError:
            # If tselect is not readable, the trigger module is likely not
            # implemented.
            if log:
                log("  cannot access tselect register; assuming triggers are "
                    "not implemented")
            self.triggers_enumerated = True
            self.trigger_count = 0
            self.trigger_tinfo = []
            self.trigger_unique_id = []
            self.reserved_triggers = []
            return ERROR_OK

        # Obtaining tinfo.version value once. No need to enumerate per-trigger.
        try:
            tinfo = self.read_register(CSR_TINFO)
            self.tinfo_version = _get_field(tinfo, CSR_TINFO_VERSION_MASK)
        except RuntimeError:
            self.tinfo_version = None       # RISCV_TINFO_VERSION_UNKNOWN

        trigger_tinfo = []
        # ARRAY_SIZE(r->trigger_tinfo) upstream is a fixed-size C array
        # (RISCV_MAX_TRIGGERS); Python has no such static bound, so the loop
        # just runs until check_if_trigger_exists/get_trigger_types says stop
        # (#trigger-port: "Python not having C's static-array size limit").
        t = 0
        while True:
            result = self._check_if_trigger_exists(t)
            if result == ERROR_FAIL:
                return ERROR_FAIL
            if result == ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                break

            try:
                tdata1 = self.read_register(CSR_TDATA1)
            except RuntimeError:
                return ERROR_FAIL

            result, tinfo_t = self._get_trigger_types(tdata1)
            if result == ERROR_FAIL:
                return ERROR_FAIL
            if result == ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
                break
            trigger_tinfo.append(tinfo_t)

            if log:
                log(f"  trigger {t}: supported types (mask) = {tinfo_t:#010x}")

            if self._disable_trigger_if_dmode(tdata1) != ERROR_OK:
                return ERROR_FAIL
            t += 1

        try:
            self.write_register(CSR_TSELECT, orig_tselect)
        except RuntimeError:
            return ERROR_FAIL

        self.triggers_enumerated = True
        self.trigger_count = t
        self.trigger_tinfo = trigger_tinfo
        self.trigger_unique_id = [-1] * t
        if log:
            log(f"  found {t} triggers")
        self.reserved_triggers = [False] * t         # #trigger-port: no CLI reserve feature
        self._create_wp_trigger_cache()
        return ERROR_OK

    # --- add_trigger (riscv.c:1494) — the top-level entry point ----------------

    def _add_trigger(self, address, length, is_read, is_write, is_execute,
                      unique_id, log=None):
        """add_trigger (riscv.c:1494). Enumerate triggers, try type 1 then type 2
        then type 6 (in that order, matching upstream's `do { } while(0)` chain),
        restore tselect afterwards regardless of outcome."""
        ret = self._enumerate_triggers(log=log)
        if ret != ERROR_OK:
            return ret

        try:
            tselect = self.read_register(CSR_TSELECT)
        except RuntimeError:
            return ERROR_FAIL

        ret = self._maybe_add_trigger_t1(address, length, is_read, is_write,
                                          is_execute, unique_id)
        if ret != ERROR_OK:
            fields_t2 = self._fill_match_triggers_tdata1_fields_t2(
                is_execute, is_read, is_write)
            ret = self._maybe_add_trigger_t2_t6(
                unique_id, address, length, is_read, is_write, is_execute,
                fields_t2, log=log)
            if ret != ERROR_OK:
                fields_t6 = self._fill_match_triggers_tdata1_fields_t6(
                    is_execute, is_read, is_write)
                ret = self._maybe_add_trigger_t2_t6(
                    unique_id, address, length, is_read, is_write, is_execute,
                    fields_t6, log=log)

        # riscv.c:1521-1523: a TSELECT-restore failure only overrides `ret` when
        # the attempt above had otherwise succeeded — a genuine setup failure
        # keeps its own error code even if the restore also fails.
        try:
            self.write_register(CSR_TSELECT, tselect)
        except RuntimeError:
            if ret == ERROR_OK:
                return ERROR_FAIL
        return ret

    # --- public API --------------------------------------------------------

    def add_watchpoint(self, address, length=4, is_read=False, is_write=True, log=None):
        """Set a hardware watchpoint on [address, address+length) — a load/store
        trigger via the RISC-V trigger module (riscv.c add_trigger, ported
        above). The hart MUST be halted first (call halt()). Tries, in order:
        a NAPOT range match, a chained GE+LT/LT+GE trigger pair, then a plain
        equality match on `address` alone (watch_allow_* flags, all default
        True) — the exact upstream fallback chain. Returns a watchpoint handle
        (opaque int) to pass to remove_watchpoint(), or raises RuntimeError if
        no free/compatible trigger was found."""
        if not is_read and not is_write:
            raise ValueError("add_watchpoint: at least one of is_read/is_write required")
        unique_id = getattr(self, "_next_unique_id", 0)
        self._next_unique_id = unique_id + 1
        ret = self._add_trigger(address, length, is_read, is_write, False,
                                 unique_id, log=log)
        if ret != ERROR_OK:
            raise RuntimeError(
                f"add_watchpoint: no trigger available for addr=0x{address:08x} "
                f"length={length} read={is_read} write={is_write} (ret={ret})")
        return unique_id

    def add_breakpoint(self, address, log=None):
        """Set a hardware (trigger-module) breakpoint at `address` — an execute
        trigger via add_trigger. The hart MUST be halted first. Returns a handle
        for remove_watchpoint(), or raises RuntimeError if unavailable."""
        unique_id = getattr(self, "_next_unique_id", 0)
        self._next_unique_id = unique_id + 1
        ret = self._add_trigger(address, 0, False, False, True, unique_id, log=log)
        if ret != ERROR_OK:
            raise RuntimeError(
                f"add_breakpoint: no trigger available for addr=0x{address:08x} "
                f"(ret={ret})")
        return unique_id

    def remove_watchpoint(self, handle):
        """Clear every trigger carrying `handle` (the id returned by
        add_watchpoint()/add_breakpoint() — a chained range watchpoint occupies
        two). Not a literal upstream function (OpenOCD's remove path is spread
        across target.c breakpoint/watchpoint bookkeeping this port doesn't
        carry); same effect expressed directly against trigger_unique_id[]."""
        if not getattr(self, "triggers_enumerated", False):
            return 0
        cleared = 0
        for idx in range(self.trigger_count):
            if self.trigger_unique_id[idx] == handle:
                self._set_trigger(idx, 0, 0)
                self.trigger_unique_id[idx] = -1
                cleared += 1
        return cleared

    # --- step-over-trigger (#57): resume() after a watchpoint hit ----------
    # RISC-V triggers default to "before" timing: the trigger fires BEFORE the
    # matched instruction commits, so bare resume() re-fires at the identical
    # PC forever. OpenOCD's fix (riscv.c resume_prep + riscv_openocd_step_impl,
    # ported below) is: on halt, inspect the hit trigger's timing
    # (riscv_trigger_detect_hit_bits); if it needs a step, resume() first
    # disables that one trigger, single-steps past the instruction (dcsr.step),
    # re-arms the trigger, THEN does the real resume. Scoped to mcontrol/
    # mcontrol6 (types 2/6, the only ones this port's add_trigger ever sets,
    # per the class-level scope note above _can_use_napot_match) — icount/i/e
    # trigger (types 3/4/5) hit-bit handling (riscv.c:1956-1964) is out of
    # scope, matching the rest of #trigger-port.

    def _check_mcontrol6_hit_status(self, tdata1, hit_mask):
        """check_mcontrol6_hit_status (riscv.c:1855). hit_mask is HIT0|HIT1.
        Returns one of M6_HIT_{NOT_SUPPORTED,NOT_HIT,BEFORE,AFTER,IMM_AFTER}
        (module-level string sentinels — see ERROR_OK et al above for why
        espjtag uses Python sentinels instead of OpenOCD's C enum ints)."""
        hit0 = _get_field(tdata1, CSR_MCONTROL6_HIT0_MASK)
        hit1 = _get_field(tdata1, CSR_MCONTROL6_HIT1_MASK)
        hit_info = (hit1 << 1) | hit0
        if hit_info == CSR_MCONTROL6_HIT0_BEFORE:
            return "M6_HIT_BEFORE"
        if hit_info == CSR_MCONTROL6_HIT0_AFTER:
            return "M6_HIT_AFTER"
        if hit_info == CSR_MCONTROL6_HIT0_IMMEDIATELY_AFTER:
            return "M6_HIT_IMM_AFTER"
        if hit_info == CSR_MCONTROL6_HIT0_FALSE:
            # hit[1..0] == 0: either "hit bits supported, not fired" or "hit
            # bits not implemented". Probe by writing every non-zero pattern
            # to hit[1:0] and reading back; if none sticks, hit bits aren't
            # implemented (riscv.c:1876-1894).
            tdata1_tests = [
                _set_field(tdata1, CSR_MCONTROL6_HIT0_MASK, 1),
                _set_field(tdata1, CSR_MCONTROL6_HIT1_MASK, 1),
                _set_field(tdata1, CSR_MCONTROL6_HIT0_MASK, 1) |
                _field_value(CSR_MCONTROL6_HIT1_MASK, 1),
            ]
            for test in tdata1_tests:
                try:
                    self.write_register(CSR_TDATA1, test)
                    test_rb = self.read_register(CSR_TDATA1)
                except RuntimeError:
                    return "M6_HIT_ERROR"
                if test_rb == test:
                    try:
                        self.write_register(CSR_TDATA1, test_rb & ~hit_mask)
                    except RuntimeError:
                        return "M6_HIT_ERROR"
                    return "M6_NOT_HIT"
            return "M6_HIT_NOT_SUPPORTED"
        return "M6_HIT_NOT_SUPPORTED"

    def _riscv_trigger_detect_hit_bits(self):
        """riscv_trigger_detect_hit_bits (riscv.c:1904). Scan every ARMED
        trigger (trigger_unique_id[i] != -1) for its hit bit; clear the hit bit
        (W1C-by-convention: write back tdata1 with it cleared) on the one that
        fired. Returns (unique_id, need_single_step) — unique_id is None if no
        hit bit was found (RISCV_TRIGGER_HIT_NOT_FOUND).

        DEVIATION (documented): type 3/4/5 (icount/itrigger/etrigger) hit-bit
        handling (riscv.c:1956-1964) is omitted — out of #trigger-port's scope,
        same as everywhere else those types are skipped in this file. A hart
        reporting one of those types here just falls through with hit_mask=0,
        same as the LEGACY (type 1) case upstream."""
        if not getattr(self, "triggers_enumerated", False):
            return None, False
        tselect = self.read_register(CSR_TSELECT)

        unique_id = None
        need_single_step = False
        for i in range(self.trigger_count):
            if self.trigger_unique_id[i] == -1:
                continue
            self.write_register(CSR_TSELECT, i)
            tdata1 = self.read_register(CSR_TDATA1)
            type_ = _get_field(tdata1, CSR_TDATA1_TYPE_MASK)

            hit_mask = 0
            if type_ == CSR_TDATA1_TYPE_LEGACY:
                pass                                    # doesn't support hit bit
            elif type_ == CSR_TDATA1_TYPE_MCONTROL:
                hit_mask = CSR_MCONTROL_HIT_MASK
                need_single_step = True
            elif type_ == CSR_TDATA1_TYPE_MCONTROL6:
                hit_mask = CSR_MCONTROL6_HIT0_MASK | CSR_MCONTROL6_HIT1_MASK
                if self.tinfo_version == CSR_TINFO_VERSION_0:
                    need_single_step = True
                elif self.tinfo_version in (None, CSR_TINFO_VERSION_1):
                    status = self._check_mcontrol6_hit_status(tdata1, hit_mask)
                    if status == "M6_HIT_ERROR":
                        raise RuntimeError(
                            "_riscv_trigger_detect_hit_bits: mcontrol6 hit-bit "
                            "probe failed")
                    if status in ("M6_HIT_BEFORE", "M6_HIT_NOT_SUPPORTED"):
                        need_single_step = True

            if tdata1 & hit_mask:
                self.write_register(CSR_TDATA1, tdata1 & ~hit_mask)
                unique_id = self.trigger_unique_id[i]
                break

        self.write_register(CSR_TSELECT, tselect)
        return unique_id, need_single_step

    def step(self):
        """Single-step the current hart one instruction (dcsr.step). Ported
        from openocd-esp32 src/target/riscv/riscv-013.c @
        a585825d4d0bbd5b8aff83e01816d2f2c04ffd84 set_dcsr_config (:1756, step
        bit only — the ebreak*/cetrig fields it also writes are set once at
        reset-run time, not per-step, see DCSR_EBREAK_BITS/call_function above)
        + riscv013_step_or_resume_current_hart(step=True) (:5638): set
        dcsr.step=1, issue the SAME resumereq handshake as resume(), but wait
        for ALLHALTED too (a step halts again immediately) before dropping
        resumereq. The hart MUST be halted first. Returns True on success.

        DEVIATION (documented): upstream clears dcsr.step at the START of the
        NEXT resume/step (riscv.c resume_prep:2949 -> riscv013_resume_prep:5486
        -> riscv013_on_step_or_resume(step=False):5624 -> set_dcsr_config,
        the SAME function this step() ports FROM, called with step=False) —
        never by the step operation itself. espjtag's step() clears it
        immediately after its own resumereq handshake instead, since
        espjtag's resume() has no separate prep/go split to hang a pre-resume
        dcsr-clear off of. Net effect on the wire is identical either way
        (dcsr.step=0 before the hart is ever let run freely); this only moves
        WHEN the clearing write happens, not whether it happens."""
        dcsr = self.read_register(CSR_DCSR)
        self.write_register(CSR_DCSR, dcsr | CSR_DCSR_STEP)
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)
        ok = False
        for _ in range(200):
            st = self.dm_read(DMSTATUS)
            if not (st & DM_ALLRESUMEACK):
                continue
            if st & DM_ALLHALTED:
                ok = True
                break
        self._dmcontrol(DM_DMACTIVE)             # resumereq <- 0
        # restore dcsr.step regardless of outcome — never leave the hart
        # permanently in single-step mode.
        self.write_register(CSR_DCSR, dcsr & ~CSR_DCSR_STEP)
        return ok

    def _step_over_trigger_if_needed(self):
        """resume_prep's watchpoint-timing check (riscv.c:2916, the
        `target->debug_reason == DBG_REASON_WATCHPOINT && r->need_single_step`
        branch) + riscv_openocd_step_impl's disable/step/re-enable sequence
        (riscv.c:4283, disable_watchpoints/riscv_step_rtos_hart/
        enable_watchpoints — scoped here to the SINGLE trigger that hit,
        mirroring OpenOCD's per-watchpoint disable_watchpoints loop since
        espjtag has no watchpoint linked-list to walk wholesale).

        Call this at the START of resume(). No-op unless the previous halt was
        a trigger hit that needs single-step (self._pending_step_over set by
        halt()'s hit-bit scan below)."""
        pending = getattr(self, "_pending_step_over", None)
        self._pending_step_over = None
        if pending is None:
            return
        handle, saved = pending
        # disable_watchpoints: clear just this trigger's tdata1/tdata2 (riscv.c:2867)
        for idx, tdata1, tdata2 in saved:
            self._set_trigger(idx, 0, 0)
        # riscv_step_rtos_hart (riscv.c:6185): single-step past the instruction
        if not self.step():
            raise RuntimeError(
                f"_step_over_trigger_if_needed: step past watchpoint handle "
                f"{handle} failed (hart did not re-halt)")
        # enable_watchpoints: re-arm with the EXACT tdata1/tdata2 that was
        # cleared (riscv.c:2894) -- not re-running add_trigger's search, since
        # the trigger index + config are already known good.
        for idx, tdata1, tdata2 in saved:
            ret = self._set_trigger(idx, tdata1, tdata2)
            if ret != ERROR_OK:
                raise RuntimeError(
                    f"_step_over_trigger_if_needed: failed to re-arm trigger "
                    f"{idx} for handle {handle} after step (ret={ret})")

    def _note_trigger_hit_for_step_over(self):
        """Call this right after a successful halt() while the hit trigger's
        tdata1/tdata2 are still live, so the NEXT resume() knows whether to
        step over it first. Not a literal upstream function — OpenOCD does
        this inline in set_debug_reason() at every poll; espjtag has no poll
        loop, so halt() calls this once, synchronously, right after it
        confirms ALLHALTED (riscv.c:2642-2690 set_debug_reason, scoped to the
        trigger-cause/watchpoint path only — ebreak/haltreq/step causes need no
        step-over and are left alone, same as upstream's DBG_REASON_BREAKPOINT/
        other branches)."""
        if not getattr(self, "triggers_enumerated", False):
            return
        dcsr = self.read_register(CSR_DCSR)
        if _get_field(dcsr, CSR_DCSR_CAUSE_MASK) != CSR_DCSR_CAUSE_TRIGGER:
            return
        unique_id, need_single_step = self._riscv_trigger_detect_hit_bits()
        if unique_id is None or not need_single_step:
            return
        saved = []
        for idx in range(self.trigger_count):
            if self.trigger_unique_id[idx] == unique_id:
                self.write_register(CSR_TSELECT, idx)
                tdata1 = self.read_register(CSR_TDATA1)
                tdata2 = self.read_register(CSR_TDATA2)
                saved.append((idx, tdata1, tdata2))
        self._pending_step_over = (unique_id, saved)

    def call_rom(self, sym, args=(), restore=True):
        """Call a named ROM function from this chip's chips.rom table (e.g.
        'spiflash_erase_sector') with integer args. The hart must be halted and a
        scratch SRAM window must be tabled. Returns (a0, halted)."""
        c = self._chip()
        rom, sram = c.get("rom"), c.get("sram")
        if not rom or sym not in rom:
            raise RuntimeError(f"call_rom: no ROM symbol {sym!r} tabled for "
                               f"{c.get('name', '?')}")
        if not sram:
            raise RuntimeError("call_rom: no scratch SRAM window tabled")
        return self.call_function(rom[sym], args=args, stack=sram["stack"],
                                  trap=sram["trap"], restore=restore)

    def call_rom_begin(self, sym, args=()):
        """Start a ROM call WITHOUT waiting: returns a ctx for _call_finish.
        While the callee runs, only SBA memory ops are legal (no abstract
        register access) — used to stage the next flash sector under the
        current sector's erase/program time."""
        c = self._chip()
        rom, sram = c.get("rom"), c.get("sram")
        if not rom or sym not in rom:
            raise RuntimeError(f"call_rom_begin: no ROM symbol {sym!r} tabled")
        return self.call_function(rom[sym], args=args, stack=sram["stack"],
                                  trap=sram["trap"], wait=False)

    def flash_read_xip(self, off, nwords):
        """Read nwords of flash via the cache-mapped XIP window (chips flash_xip +
        off). This is the WORKING read path (verified on the bench); the ROM
        esp_rom_spiflash_read needs init the running app may not have done. Note the
        XIP window maps the app's active flash mapping, so `off` is an offset INTO
        that mapping, not necessarily a raw flash byte address. Hart should be
        halted with icache enabled."""
        c = self._chip()
        xip = c.get("flash_xip")
        if xip is None:
            raise RuntimeError("flash_read_xip: no flash_xip window tabled")
        return self.read_mem(xip + off, nwords)

    _FLASH_VENDORS = {0x20: "XMC", 0xEF: "Winbond", 0xC8: "GigaDevice",
                      0x85: "Puya", 0x68: "Boya", 0xA1: "Fudan", 0x0B: "XTX",
                      # 0x46: JEP106 says Silicon Spice (defunct telecom) — the
                      # real holder is an unregistered fab; identified by SFDP
                      # fingerprint + behaviour only (docs/FLASH-DIE-SURVEY.md)
                      0x46: "unregistered-0x46"}
    # SPI1 (the non-cache flash controller) user-command registers — identical
    # layout + base on C5/C6 (DR_REG_SPI1_BASE, spi_mem_reg.h).
    _SPI1 = 0x60003000
    _SPI_CMD, _SPI_USER, _SPI_USER2, _SPI_MISO_DLEN, _SPI_W0 = \
        _SPI1 + 0x0, _SPI1 + 0x18, _SPI1 + 0x20, _SPI1 + 0x28, _SPI1 + 0x58
    _SPI_USR = 1 << 18                       # CMD.usr: start user transaction
    _SPI_USR_COMMAND, _SPI_USR_MISO = 1 << 31, 1 << 28

    _SPI_ADDR, _SPI_USER1 = _SPI1 + 0x4, _SPI1 + 0x1C
    _SPI_USR_ADDR, _SPI_USR_DUMMY = 1 << 30, 1 << 29

    def flash_sfdp(self, addr, nbytes):
        """Read `nbytes` of the SFDP space (JESD216, cmd 0x5A + 24-bit address +
        8 dummy cycles) over JTAG via SPI1 registers. Returns bytes."""
        out = bytearray()
        for off in range(0, nbytes, 4):
            n = min(4, nbytes - off)
            save = {r: self.read_mem32(r) for r in
                    (self._SPI_USER, self._SPI_USER1, self._SPI_USER2,
                     self._SPI_MISO_DLEN, self._SPI_ADDR)}
            try:
                self.write_mem32(self._SPI_USER,
                                 self._SPI_USR_COMMAND | self._SPI_USR_MISO
                                 | self._SPI_USR_ADDR | self._SPI_USR_DUMMY)
                self.write_mem32(self._SPI_USER1,
                                 (23 << 26) | 7)            # 24 addr bits, 8 dummies
                self.write_mem32(self._SPI_USER2, (7 << 28) | 0x5A)
                self.write_mem32(self._SPI_ADDR, addr + off)   # right-aligned (24-bit)
                self.write_mem32(self._SPI_MISO_DLEN, n * 8 - 1)
                self.write_mem32(self._SPI_CMD, self._SPI_USR)
                for _ in range(1000):
                    if not (self.read_mem32(self._SPI_CMD) & self._SPI_USR):
                        break
                else:
                    raise RuntimeError("flash_sfdp: transaction never completed")
                out += self.read_mem32(self._SPI_W0).to_bytes(4, "little")[:n]
            finally:
                for r, v in save.items():
                    self.write_mem32(r, v)
        return bytes(out)

    def _spi1_user_read(self, cmd, nbytes):
        """Execute a bare SPI flash command on SPI1 via direct register access
        (SBA) and read back `nbytes` (<=4): command phase only + MISO phase —
        no stub, no ROM call, no serial. Touched registers are saved/restored.
        Flash must be idle (run after the gate); hart halted not required but
        we only call this from halted contexts."""
        save = {r: self.read_mem32(r) for r in
                (self._SPI_USER, self._SPI_USER2, self._SPI_MISO_DLEN)}
        try:
            self.write_mem32(self._SPI_USER, self._SPI_USR_COMMAND | self._SPI_USR_MISO)
            self.write_mem32(self._SPI_USER2, (7 << 28) | (cmd & 0xFF))
            self.write_mem32(self._SPI_MISO_DLEN, nbytes * 8 - 1)
            self.write_mem32(self._SPI_CMD, self._SPI_USR)
            for _ in range(1000):            # poll usr-clear: a cmd+4B read is ~µs
                if not (self.read_mem32(self._SPI_CMD) & self._SPI_USR):
                    break
            else:
                raise RuntimeError(f"_spi1_user_read: cmd 0x{cmd:02x} never completed")
            return self.read_mem32(self._SPI_W0) & ((1 << (8 * nbytes)) - 1)
        finally:
            for r, v in save.items():
                self.write_mem32(r, v)

    def flash_info(self):
        """Identify the flash die over JTAG alone: JEDEC RDID (cmd 0x9F) on SPI1.
        Returns dict(rdid, mfg, device, size_mb, vendor). Hart halted; runs the
        gate ladder first so SPI1 is attached/idle. Capacity is the JEDEC
        convention 2^cap bytes (e.g. 0x16 -> 4 MB; bytes arrive mfg, type, cap)."""
        self._ensure_flash_ready(lambda m: None, "flash_info")
        raw = self._spi1_user_read(0x9F, 3)
        mfg = raw & 0xFF                         # byte0 = manufacturer
        dev = ((raw >> 8) & 0xFF) << 8 | ((raw >> 16) & 0xFF)   # type:cap, flash-id style
        cap = (raw >> 16) & 0xFF
        size_mb = (1 << cap) // (1024 * 1024) if 0x10 <= cap <= 0x20 else None
        return dict(rdid=raw & 0xFFFFFF, mfg=mfg, device=dev, size_mb=size_mb,
                    vendor=self._FLASH_VENDORS.get(mfg, f"unknown(0x{mfg:02x})"))

    def flash_read_rom(self, addr, nwords):
        """Read nwords from RAW flash byte-offset `addr` via the ROM
        esp_rom_spiflash_read (needs flash_init done / the gate passing). Unlike
        flash_read_xip this is a TRUE raw flash offset (XIP maps the app's mapping,
        not raw flash), so it's the correct read for verify + inspection. Hart
        halted; icache toggled. Reuses the scratch staging buffer."""
        c = self._chip()
        buf = c["sram"]["data"]
        has_cache = "cache_disable_icache" in c.get("rom", {})
        if has_cache:                                   # newer chips (C5) lack the
            self.call_rom("cache_disable_icache")       # simple icache toggle; when
        self.call_rom("spiflash_read", args=(addr, buf, nwords * 4))
        words = self.read_mem(buf, nwords)
        if has_cache:                                   # halted there's no prefetch
            self.call_rom("cache_enable_icache")        # racing the ROM read anyway
        return words

    def flash_init(self, chip_size=0x1000000):
        """Repopulate the legacy ROM spiflash geometry (g_rom_spiflash_chip) so the
        esp_rom_spiflash_* helpers work on a board whose running app left it unset.
        The flash is already pin/mode-attached (XIP is live), so we only restore the
        geometry — esp_rom_spiflash_config_param(devid=0, chip_size, 64 KiB block,
        4 KiB sector, 256 B page, 0xFFFF status mask). Hart must be halted. Returns
        the ROM result (0 = OK). This is the un-gate for flash_write (#30)."""
        rom = self._chip().get("rom", {})
        if "spiflash_config_param" not in rom:
            raise RuntimeError("flash_init: spiflash_config_param not tabled for this chip")
        # clk first when tabled — empirically REQUIRED on the C5 when the app was
        # halted mid-flash-IO: attach+config_param alone left the gate failing in
        # 4/5 repro hits; config_clk(div=1) + attach + param recovered every one
        # (#33, scripts/c5_rom_read_repro.py bisect).
        if "spiflash_config_clk" in rom:
            self.call_rom("spiflash_config_clk", args=(1, 1))
        # attach — sets the legacy funcs' dummy cycles/read mode (without it the
        # ROM read returns garbage on a running app). ishspi=0 (default GPIO), legacy=0.
        if "spi_flash_attach" in rom:
            self.call_rom("spi_flash_attach", args=(0, 0))
        r, _ = self.call_rom("spiflash_config_param",
                             args=(0, chip_size, 0x10000, 0x1000, 0x100, 0xFFFF))
        return r

    def _ensure_flash_ready(self, _log, who, attempts=3):
        """Gate + recovery ladder: re-run flash_init (clk + attach + config_param)
        up to `attempts-1` times. Bench-measured on a C5 with a live flash-IO app
        (#33, c5_rom_read_repro 30-cycle soak): 18/18 gate failures recovered, 14
        on the first flash_init and 4 needing a second — one retry is not enough.
        Returns the final (ready, rom_words, magic)."""
        ready, rw, xw = self._rom_flash_ready()
        can_init = "spiflash_config_param" in self._chip().get("rom", {})
        for attempt in range(attempts - 1):
            if ready or not can_init:
                break
            _log(f"  {who}: gate failed — flash_init recovery "
                 f"(attempt {attempt + 1}/{attempts - 1})")
            self.flash_init()
            ready, rw, xw = self._rom_flash_ready()
        return ready, rw, xw

    def _rom_flash_ready(self, nwords=4):
        """SAFETY GATE for any ROM erase/program. Read the 2nd-stage bootloader
        header via the ROM esp_rom_spiflash_read and require the first byte to be
        the ESP image magic 0xE9. The header's flash offset is PER CHIP, not
        always 0 (espjtag#55) — chips.bootloader_offset_for() gives the right
        value: 0x0 for C6/C3/H2/S3/etc, 0x2000 for C5/P4/H4 (ESP-IDF
        components/bootloader/Kconfig.projbuild BOOTLOADER_OFFSET_IN_FLASH).
        A correct 0xE9 proves the ROM read path is configured (attach dummy cycles +
        config_param geometry); garbage (the running app left the controller in
        fast-XIP mode) means erase/program would target a misconfigured chip, so we
        refuse (brick risk). (Earlier this compared against flash_read_xip, but the
        XIP window maps the app's region, not raw flash — never a valid reference.)
        Read-only; icache toggled around the ROM read. Returns (ready, rom_words, magic)."""
        c = self._chip()
        rom, sram = c.get("rom"), c.get("sram")
        if not rom or not sram or "spiflash_read" not in rom:
            return False, None, None
        offset = c.get("bootloader_offset", 0x0)
        dest = sram["data"]
        saved = self.read_mem(dest, nwords)
        has_cache = "cache_disable_icache" in rom
        try:
            if has_cache:
                self.call_rom("cache_disable_icache")
            ret, halted = self.call_rom("spiflash_read", args=(offset, dest, nwords * 4))
            rom_words = self.read_mem(dest, nwords)
            if has_cache:
                self.call_rom("cache_enable_icache")
            magic = (rom_words[0] & 0xFF) if rom_words else None
            ready = bool(halted) and ret == 0 and magic == 0xE9
            return ready, rom_words, magic
        finally:
            self.write_mem(dest, saved)         # leave scratch byte-identical

    def flash_write(self, addr, data, log=None, verify=True):
        """Program `data` (bytes) to flash byte-offset `addr` via the C6 ROM
        esp_rom_spiflash_* functions (Option A, #3): unlock, erase the covering
        4 KiB sectors, stage each chunk into scratch SRAM (batched write_mem), and
        esp_rom_spiflash_write it, then (verify) read back.

        *** SAFETY: refuses to run unless _rom_flash_ready() passes. *** The legacy
        ROM flash state is not guaranteed configured in a running app, so this gate
        prevents erasing/programming a misconfigured chip (brick risk). On a board
        that hasn't set up the ROM spiflash globals (e.g. the Zephyr app, measured),
        this raises WITHOUT touching flash. The hart must be halted first.

        NOTE: addr and len(data) must be 4-byte aligned (ROM write requirement);
        erase is sector-granular so `addr` should be 4 KiB-aligned for a clean
        program. UNVERIFIED end-to-end on the bench — see docs/JTAG-FLASH-WRITES.md.
        """
        def _log(m):
            if log:
                log(m)
        if len(data) % 4 or addr % 4:
            raise ValueError("flash_write: addr and len(data) must be 4-byte aligned")
        ready, rw, xw = self._ensure_flash_ready(_log, "flash_write")
        if not ready:
            raise RuntimeError(
                "flash_write: ROM spiflash read-back self-test FAILED — the legacy "
                "ROM flash read path is not configured on this target (ROM read "
                f"{rw}, first-byte magic {xw}, want 0xE9). Refusing to erase/program "
                "(brick risk). flash_init (clk+attach+config_param) was retried; if "
                "the magic is still wrong this chip/revision needs an ECO workaround "
                "(see #33), or flash it with esptool.")
        c = self._chip()
        sram = c["sram"]
        buf = sram["data"]
        words = [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]
        _log(f"  flash_write: 0x{addr:08x} +{len(data)}B ({len(words)} words)")
        # unlock once
        self.call_rom("spiflash_unlock")
        # erase covering sectors — use 64 KiB BLOCK erase for fully-covered aligned
        # blocks: measured 4.4-5.7x faster than 16 sector erases on every die on
        # the bench (Winbond 987->211 ms, Puya 741->130, 0x46 391->88;
        # tmp/block_erase_bench.log). INVARIANT: a block is erased ONLY when the
        # write range covers all 16 of its sectors — partially-covered ends fall
        # back to sector erase (never erase what we won't rewrite; the pyOCD
        # fast_program bug class, scripts/incremental_invariant_test.py).
        first = addr & ~0xFFF
        last = (addr + len(data) - 1) & ~0xFFF
        rom = self._chip().get("rom", {})
        sa = first
        while sa <= last:
            if ("spiflash_erase_block" in rom and sa % 0x10000 == 0
                    and sa + 0x10000 - 0x1000 <= last):
                r, _ = self.call_rom("spiflash_erase_block", args=(sa >> 16,))
                if r != 0:
                    raise RuntimeError(f"flash erase block {sa >> 16} -> {r}")
                sa += 0x10000
            else:
                r, _ = self.call_rom("spiflash_erase_sector", args=(sa >> 12,))
                if r != 0:
                    raise RuntimeError(f"flash erase sector {sa >> 12} -> {r}")
                sa += 0x1000
        # stage + program in chunks bounded by the scratch buffer headroom
        chunk_words = 0x400                       # 4 KiB per call (well within scratch)
        for w0 in range(0, len(words), chunk_words):
            wchunk = words[w0:w0 + chunk_words]
            self.write_mem(buf, wchunk)
            dest = addr + w0 * 4
            r, _ = self.call_rom("spiflash_write", args=(dest, buf, len(wchunk) * 4))
            if r != 0:
                raise RuntimeError(f"flash write @0x{dest:08x} -> {r}")
        if verify:
            # read back in scratch-sized chunks — a single flash_read_rom of the
            # whole image overflows the scratch window for anything > ~4 KiB
            # (it clobbered the trap word and wedged the session; found when
            # 64 KiB verify=True was first exercised, 2026-06-12).
            for w0 in range(0, len(words), chunk_words):
                want = words[w0:w0 + chunk_words]
                got = self.flash_read_rom(addr + w0 * 4, len(want))
                if got != want:
                    raise RuntimeError(
                        f"flash_write: verify mismatch @0x{addr + w0 * 4:08x}")
            _log("  flash_write: verify OK")
        return True

    # === incremental flash — on-chip CRC-32 diff + write-only-changed + verify (#34) ===
    # The digest is a REAL CRC-32 (poly 0xEDB88320) computed ON-CHIP via the ROM
    # esp_rom_crc32_le over a scratch-staged sector read — only the 4-byte digest crosses
    # JTAG, never the sector data. This is the digest pyOCD/J-Flash and everyone who does
    # incremental RIGHT uses; STM32CubeProgrammer's 32-bit ADDITIVE sum (proven on silicon
    # to silently drop sum-preserving sector changes — docs/CUBEPROGRAMMER-BUGS.md) is the
    # anti-pattern we deliberately avoid.
    _CRC_INIT = 0xFFFFFFFF        # init passed to ROM crc32_le; host digest calibrated to match

    def _crc_host(self):
        """Return host_crc(bytes)->u32 reproducing the on-chip ROM crc32_le(_CRC_INIT, ·),
        CALIBRATED against the live ROM. The host and on-chip digest MUST agree bit-for-bit
        (the #1 incremental correctness trap, §9) — so we MEASURE the ROM's CRC convention
        from a known pattern rather than assume it. Cached per object."""
        if getattr(self, "_crc_host_fn", None):
            return self._crc_host_fn
        if "crc32_le" not in self._chip().get("rom", {}):
            raise RuntimeError("_crc_host: no crc32_le ROM symbol tabled for this chip")
        buf = self._chip()["sram"]["data"]
        pat = bytes((i * 73 + 19) & 0xFF for i in range(256))
        saved = self.read_mem(buf, 64)
        self.write_mem(buf, [int.from_bytes(pat[i:i + 4], "little") for i in range(0, 256, 4)])
        onchip, _ = self.call_rom("crc32_le", args=(self._CRC_INIT, buf, 256))
        self.write_mem(buf, saved)                              # leave scratch byte-identical
        onchip &= 0xFFFFFFFF
        cands = {
            "zlib^": lambda d: zlib.crc32(d) ^ 0xFFFFFFFF,
            "zlib": lambda d: zlib.crc32(d) & 0xFFFFFFFF,
            "raw(init)": lambda d: _crc32_le_raw(d, self._CRC_INIT),
            "raw(0)": lambda d: _crc32_le_raw(d, 0),
            "raw(0)^": lambda d: _crc32_le_raw(d, 0) ^ 0xFFFFFFFF,
        }
        for name, fn in cands.items():
            if fn(pat) == onchip:
                self._crc_host_fn, self._crc_host_name = fn, name
                return fn
        raise RuntimeError(f"_crc_host: no host candidate matched ROM crc32_le "
                           f"(onchip=0x{onchip:08x}) — convention unknown, refusing to diff")

    def flash_crc_region(self, addr, size):
        """On-chip CRC-32 of `size` bytes of RAW flash at byte-offset `addr`: ROM-read the
        region into scratch (stays on-chip) then ROM crc32_le over scratch — only the
        4-byte digest crosses JTAG. `size` must fit the scratch headroom (a 4 KiB sector
        does). Matches _crc_host() (same ROM, same _CRC_INIT). Hart halted; the ROM flash
        read path must be ready (flash_init / the gate, as flash_read_rom)."""
        return self._flash_crc_many([(addr, size)])[0]

    def _flash_crc_many(self, regions):
        """On-chip CRC-32 of several `(addr, size)` flash regions in one pass.
        The icache toggle is hoisted around the WHOLE loop (it exists to keep the
        cache off the SPI1 bus during ROM spiflash_read; crc32_le runs from ROM
        over SRAM scratch and doesn't care) — per-sector it was 4 ROM calls at
        ~13 ms of DMI round-trips each, 69% of flash_incremental's wall clock."""
        rom = self._chip().get("rom", {})
        buf = self._chip()["sram"]["data"]
        has_cache = "cache_disable_icache" in rom
        out = []
        if has_cache:
            self.call_rom("cache_disable_icache")
        try:
            for addr, size in regions:
                self.call_rom("spiflash_read", args=(addr, buf, size))
                crc, _ = self.call_rom("crc32_le", args=(self._CRC_INIT, buf, size))
                out.append(crc & 0xFFFFFFFF)
        finally:
            if has_cache:
                self.call_rom("cache_enable_icache")
        return out

    # === #27: resident RAM flasher stub — a mailbox loop fed over SBA =========
    # Each ROM call costs ~5 ms of halt/resume/register traffic; the stub runs
    # continuously and takes commands via memory writes (µs). Scratch layout:
    #   data+0x0000  buf A (4 KiB)      staging, double-buffered
    #   data+0x1000  buf B (4 KiB)
    #   data+0x2000  mailbox (16 words) cmd/status/args + per-chip ROM fn table
    #   data+0x2040  crc_out (64 words) CRC_RANGE results, 64 sectors per cmd
    #   data+0x2200  stub code (~230 B position-independent blob, ships built)
    #   sram[stack]  stub SP            sram[trap] ebreak word (QUIT returns here)
    _STUB_SYMS = ("spiflash_read", "crc32_le", "spiflash_unlock",
                  "spiflash_erase_sector", "spiflash_write")
    _STUB_CRC_MAX = 64                 # crc_out capacity per CRC_RANGE command
    _stub_blob_cache = None

    @classmethod
    def _stub_blob(cls):
        if cls._stub_blob_cache is None:
            p = os.path.join(os.path.dirname(__file__), "blobs", "stub_rv32.bin")
            cls._stub_blob_cache = open(p, "rb").read() if os.path.exists(p) else b""
        return cls._stub_blob_cache

    def _stub_layout(self):
        sram = self._chip()["sram"]
        d = sram["data"]
        return dict(bufA=d, bufB=d + 0x1000, mail=d + 0x2000, crc=d + 0x2040,
                    code=d + 0x2200, sp=sram["stack"], trap=sram["trap"])

    def stub_begin(self):
        """Upload the stub + ROM fn table and start it. Returns (ctx, layout).
        While the stub runs only SBA ops are legal (it IS the running hart)."""
        lay = self._stub_layout()
        rom = self._chip()["rom"]
        blob = self._stub_blob()
        bw = blob + b"\x00" * (-len(blob) % 4)
        self.write_mem(lay["code"], [int.from_bytes(bw[i:i + 4], "little")
                                     for i in range(0, len(bw), 4)])
        mail = [0] * 16
        for i, sym in enumerate(self._STUB_SYMS):
            mail[7 + i] = rom[sym]
        self.write_mem(lay["mail"], mail)
        ctx = self.call_function(lay["code"], args=(lay["mail"],),
                                 stack=lay["sp"], trap=lay["trap"], wait=False)
        return ctx, lay

    def _stub_post(self, lay, cmd, m2=0, m3=0, m4=0, m5=0, m6=0):
        """Queue one command: args first (mail+4..), the cmd word LAST (the stub
        triggers on m[0], so ordering is the correctness here)."""
        self.write_mem(lay["mail"] + 4, [0, m2, m3, m4, m5, m6])  # status=0, args
        self.write_mem32(lay["mail"], cmd)

    def _stub_wait(self, lay, polls=20000):
        """Poll the status word until the stub reports done/error. 20k polls
        ≈ 5 s — far beyond the slowest single command (a ~60 ms sector erase);
        expiry means the stub wedged and raising beats spinning forever."""
        for _ in range(polls):
            st = self.read_mem32(lay["mail"] + 4)
            if st:
                if st != 1:
                    err = self.read_mem32(lay["mail"] + 8)
                    raise RuntimeError(f"flash stub error: status={st} err={err}")
                return
        raise RuntimeError("flash stub: command never completed")

    def stub_end(self, ctx, lay):
        """QUIT the stub (it returns into the ebreak trap) and restore the app."""
        self._stub_post(lay, 3)
        ret, halted = self._call_finish(ctx)
        if not halted:
            raise RuntimeError("flash stub: QUIT did not trap back to debug mode")

    def _flash_incremental_stub(self, addr, data, _log, verify):
        """flash_incremental's fast path: one resident stub, commands over SBA.
        The CRC diff is ONE command per 64 sectors (vs 2 ROM calls per sector),
        and programming pipelines: sector k+1 stages into the spare buffer over
        SBA while the stub erases+writes sector k."""
        host = self._crc_host()
        lay = self._stub_layout()
        nsec = len(data) // 0x1000
        ctx, lay = self.stub_begin()
        try:
            def crc_range(base, cnt):
                self._stub_post(lay, 1, m2=self._CRC_INIT, m3=addr + base * 0x1000,
                                m4=cnt, m5=lay["bufA"], m6=lay["crc"])
                self._stub_wait(lay)
                return self.read_mem(lay["crc"], cnt)

            crcs = []
            for base in range(0, nsec, self._STUB_CRC_MAX):
                crcs += crc_range(base, min(self._STUB_CRC_MAX, nsec - base))
            changed = [s for s in range(nsec)
                       if crcs[s] != host(data[s * 0x1000:(s + 1) * 0x1000])]
            _log(f"  flash_incremental[stub]: {nsec} sectors, {len(changed)} differ")
            busy = False
            for i, s in enumerate(changed):
                buf = lay["bufA"] if i % 2 == 0 else lay["bufB"]
                img = data[s * 0x1000:(s + 1) * 0x1000]
                self.write_mem(buf, [int.from_bytes(img[k:k + 4], "little")
                                     for k in range(0, 0x1000, 4)])
                if busy:
                    self._stub_wait(lay)             # finish sector i-1
                self._stub_post(lay, 2, m3=addr + s * 0x1000, m5=buf)
                busy = True
            if busy:
                self._stub_wait(lay)
            verified = 0
            if verify and changed:
                vcrcs = []
                for base in range(0, nsec, self._STUB_CRC_MAX):
                    vcrcs += crc_range(base, min(self._STUB_CRC_MAX, nsec - base))
                for s in changed:
                    if vcrcs[s] != host(data[s * 0x1000:(s + 1) * 0x1000]):
                        raise RuntimeError(
                            f"flash_incremental[stub]: verify FAILED "
                            f"@0x{addr + s * 0x1000:08x}")
                    verified += 1
        finally:
            self.stub_end(ctx, lay)
        _log(f"  flash_incremental[stub]: programmed {len(changed)}/{nsec}, "
             f"verified {verified}")
        return dict(sectors=nsec, changed=len(changed), written=len(changed),
                    overwritten=0, verified=verified)

    def _stage_sector(self, buf, img):
        """Stage a 4 KiB sector into scratch `buf`. (ROM-tinfl compressed
        staging was tried for #36 and is UNUSABLE: tinfl_decompress_mem_to_mem
        puts its ~11 KB decompressor struct ON THE CALLER STACK, blowing through
        the whole 16 KiB scratch carve-out — measured as a corrupted staging
        buffer + failed spiflash_write. Staging cost is hidden by pipelined
        staging instead — see flash_incremental.)"""
        self.write_mem(buf, [int.from_bytes(img[i:i + 4], "little")
                             for i in range(0, 0x1000, 4)])

    def flash_incremental(self, addr, data, log=None, verify=True, erase="always"):
        """Incrementally program `data` (bytes) to flash byte-offset `addr`: CRC each 4 KiB
        sector ON-CHIP, erase+program ONLY the sectors whose CRC differs from the image,
        then verify-after-write by re-CRC. Same brick-safety gate + ROM path as flash_write.
        `addr` 4 KiB-aligned; `data` 0xFF-padded to a sector multiple. An all-identical
        image writes nothing (the root early-out falls out of the loop).

        erase="auto": read each changed sector back and SKIP the erase when every bit
        transition is 1->0 (NOR programming can only clear bits), programming only the
        differing word-run in place. Big win for append-like / bit-clearing deltas
        (logs, counters, NVS-style); a net loss (~+50% per sector) on random changes
        because the read-back is paid and the erase happens anyway — hence opt-in.
        The verify re-CRC covers both paths identically.

        Returns dict(sectors, changed, written, overwritten, verified). Hart halted."""
        def _log(m):
            if log:
                log(m)
        if addr % 0x1000:
            raise ValueError("flash_incremental: addr must be 4 KiB-aligned")
        if len(data) % 0x1000:
            data = data + b"\xFF" * (0x1000 - len(data) % 0x1000)
        ready, rw, xw = self._ensure_flash_ready(_log, "flash_incremental")
        if not ready:
            raise RuntimeError("flash_incremental: ROM flash read self-test FAILED — "
                               "refusing to erase/program (brick risk)")
        # #27 fast path: resident RAM stub (one mailbox command per 64-sector CRC
        # pass, pipelined erase+program). erase='auto' keeps the ROM-call path
        # (its read-back/overwrite logic lives host-side).
        rom = self._chip().get("rom", {})
        if (erase == "always" and self._stub_blob()
                and all(s in rom for s in self._STUB_SYMS)):
            return self._flash_incremental_stub(addr, data, _log, verify)
        host = self._crc_host()
        buf = self._chip()["sram"]["data"]
        nsec = len(data) // 0x1000
        crcs = self._flash_crc_many([(addr + s * 0x1000, 0x1000) for s in range(nsec)])
        changed = [s for s in range(nsec)
                   if crcs[s] != host(data[s * 0x1000:(s + 1) * 0x1000])]
        _log(f"  flash_incremental: {nsec} sectors, {len(changed)} differ "
             f"(digest {self._crc_host_name}) -> programming")
        # INVARIANT: every byte of every erased unit must be re-written. Today diff =
        # erase = write = 4 KiB so this holds by construction; if the erase unit ever
        # grows (e.g. 64 KiB spiflash_erase_block for speed), the whole block must be
        # programmed, not just the changed sectors — skipping "unchanged" data inside
        # an erased unit is pyOCD's fast_program data-loss bug
        # (docs/PYOCD-INCREMENTAL-PROOF.md §3; scripts/incremental_invariant_test.py).
        overwritten = 0
        for s in changed:
            sa = addr + s * 0x1000
            img = data[s * 0x1000:(s + 1) * 0x1000]
            if erase == "auto":
                # NOR programming only clears bits (1->0). Read the sector back:
                # if no bit needs 0->1, skip the erase and program ONLY the
                # differing word-run. Wins on append/bit-clearing deltas; the
                # read-back makes it a net LOSS on random changes — opt-in.
                cur = b"".join(w.to_bytes(4, "little")
                               for w in self.flash_read_rom(sa, 0x400))
                if all(c & n == n for c, n in zip(cur, img)):
                    w0 = next(i for i in range(0, 0x1000, 4)
                              if cur[i:i + 4] != img[i:i + 4])
                    w1 = next(i for i in range(0xFFC, -4, -4)
                              if cur[i:i + 4] != img[i:i + 4]) + 4
                    self.call_rom("spiflash_unlock")
                    self.write_mem(buf, [int.from_bytes(img[i:i + 4], "little")
                                         for i in range(w0, w1, 4)])
                    r, _ = self.call_rom("spiflash_write", args=(sa + w0, buf, w1 - w0))
                    if r:
                        raise RuntimeError(
                            f"flash_incremental: overwrite @0x{sa + w0:08x} -> {r}")
                    overwritten += 1
                    _log(f"  flash_incremental: sector {s} overwritten in place "
                         f"({w1 - w0} bytes, no erase)")
                    continue
            self.call_rom("spiflash_unlock")
            # PIPELINE: stage the sector over JTAG (SBA, hart-independent)
            # WHILE the ROM erases it — staging (~35 ms) hides almost entirely
            # under the erase (24-62 ms per die). The #36 goal (make staging
            # free) without the ROM inflater's impossible stack demands.
            ctx = self.call_rom_begin("spiflash_erase_sector", args=(sa >> 12,))
            self._stage_sector(buf, img)
            r, _ = self._call_finish(ctx)
            if r:
                raise RuntimeError(f"flash_incremental: erase sector {sa >> 12} -> {r}")
            r, _ = self.call_rom("spiflash_write", args=(sa, buf, 0x1000))
            if r:
                raise RuntimeError(f"flash_incremental: write @0x{sa:08x} -> {r}")
        verified = 0
        if verify and changed:
            vcrcs = self._flash_crc_many([(addr + s * 0x1000, 0x1000) for s in changed])
            for s, crc in zip(changed, vcrcs):
                if crc != host(data[s * 0x1000:(s + 1) * 0x1000]):
                    raise RuntimeError(
                        f"flash_incremental: verify FAILED @0x{addr + s * 0x1000:08x}")
                verified += 1
        _log(f"  flash_incremental: programmed {len(changed)}/{nsec} "
             f"({overwritten} in place), verified {verified}")
        return dict(sectors=nsec, changed=len(changed), written=len(changed),
                    overwritten=overwritten, verified=verified)

    def diag(self, log=print):
        """Verbose read-only dump of the debug module — useful BEFORE resetting."""
        idcode = self.read_idcode()
        log(f"  IDCODE        = 0x{idcode:08x}")
        dmcontrol, st1 = self.dmi_read(DMCONTROL)
        log(f"  dmcontrol@0x10= 0x{dmcontrol:08x} (op={st1}) "
            f"dmactive={dmcontrol & 1} ndmreset={(dmcontrol >> 1) & 1}")
        dmstatus, st2 = self.dmi_read(DMSTATUS)
        log(f"  dmstatus@0x11 = 0x{dmstatus:08x} (op={st2}) "
            f"version={dmstatus & 0xF} allhalted={(dmstatus >> 9) & 1} "
            f"allrunning={(dmstatus >> 11) & 1}")
        hartinfo, _ = self.dmi_read(HARTINFO)
        log(f"  hartinfo@0x12 = 0x{hartinfo:08x}")
        return idcode, dmcontrol, dmstatus


    def reset_run(self, log=None):
        """Full-system reset that re-samples the BOOT strap, then RUN the app —
        mirrors OpenOCD's `reset run` DMI sequence captured from its -d3 log:
            dmi_write 0x10 0x40000001   (resumereq | dmactive — pre-reset)
            dmi_write 0x10 0x80000003   (haltreq | ndmreset | dmactive — assert)
        then deassert ndmreset and resume. The ndmreset is the full-system reset
        the C6 USJ core-reset can't do; haltreq holds the hart so the reset is
        clean; resumereq runs the freshly-strapped app. Verified vs OpenOCD."""
        chips.require_riscv(self.read_idcode(), op="reset_run over JTAG")
        if log:
            log("  -- pre-reset diag --")
            self.diag(log)
        self._dmcontrol(DM_DMACTIVE)                          # claim DM
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)           # 0x40000001
        self._dmcontrol(DM_HALTREQ | DM_NDMRESET | DM_DMACTIVE)  # 0x80000003
        time.sleep(0.05)
        self._dmcontrol(DM_HALTREQ | DM_DMACTIVE)             # deassert ndmreset, keep halted
        self._dmcontrol(DM_ACKHAVERESET | DM_DMACTIVE)        # ack the reset
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)           # RUN
        self._dmcontrol(0)                                    # release DM
        usb.util.dispose_resources(self.dev)

    # === Faithful OpenOCD `reset run` building blocks (riscv-013.c) =========
    # These mirror the riscv013_* helpers OpenOCD runs around its reset, which the
    # captured 12-write list omitted (it logged only the dmi_write *commands*, not
    # the stateful halt/dcsr/resume polls). Ported by LOGIC, not literal C.

    def _halt_go(self, timeout=256):
        """riscv013_halt_go: set haltreq, poll dmstatus until no hart is running,
        then drop haltreq. Returns True if halted."""
        self._dmcontrol(DM_HALTREQ | DM_DMACTIVE)
        ok = False
        for _ in range(timeout):
            if not (self.dm_read(DMSTATUS) & DM_ANYRUNNING):
                ok = True
                break
        self._dmcontrol(DM_DMACTIVE)            # haltreq <- 0
        return ok

    def _set_dcsr_ebreak(self):
        """set_dcsr_ebreak(step=false): read dcsr, OR in ebreakm|ebreaku, write it
        back (only if changed). On the C6 this turns 0xc3 into 0x90c3 — exactly
        what OpenOCD's -d3 trace shows (dcsr <- 0x90c3). The hart must be halted."""
        dcsr = self.read_register(CSR_DCSR)
        new = dcsr | DCSR_EBREAK_BITS
        if new != dcsr:
            self.write_register(CSR_DCSR, new)
        return dcsr, new

    def _resume_go(self, timeout=256):
        """riscv013_step_or_resume_current_hart(step=false): set resumereq, poll
        dmstatus until allresumeack (fail on allunavail), then clear resumereq.
        Returns True on a clean resume. The hart must be halted first."""
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)
        for _ in range(timeout):
            st = self.dm_read(DMSTATUS)
            if st & DM_ALLUNAVAIL:
                self._dmcontrol(DM_DMACTIVE)
                return False
            if st & DM_ALLRESUMEACK:
                self._dmcontrol(DM_DMACTIVE)    # resumereq <- 0
                return True
        self._dmcontrol(DM_DMACTIVE)
        return False

    def _c6_soc_reset(self):
        """esp32c6_soc_reset (openocd-esp32 tcl/target/esp32c6.cfg, run from the
        reset-assert-post event): the ESP32-C6-specific assert. Halt the hart,
        then via System Bus Access poke the two LP_AON software-reset registers,
        clearing dmactive between them to drop SBA's sbbusy (or the DM wedges).
        Ends by re-asserting haltreq|ndmreset (0x80000003) to hold the hart in
        reset — exactly the captured 12 writes, in order.

        The two LP_AON register addresses come from the per-chip table
        (espjtag.chips, #4), keyed by the live target IDCODE — not hardcoded here.
        Raises if the connected part has no tabled reset regs (don't blindly poke
        another chip's address space)."""
        rst = chips.reset_for(self.read_idcode())
        if not rst:
            raise RuntimeError(
                "_c6_soc_reset: no tabled SoC reset registers for this IDCODE "
                f"0x{self.read_idcode():08x} — refusing to guess reset addresses")
        self._dmcontrol(DM_HALTREQ | DM_DMACTIVE)              # 0x80000001
        # LP_AON_SYS_CFG = HPSYS_SW_RESET  (write 0x80000000 to 0x600b1034 via SBA)
        self.write_mem32(rst["hpsys_cfg"], rst["hpsys_sw_reset"])
        self._dmcontrol(0)                                     # clear sbbusy
        # LP_AON_CPUCORE0_CFG = CPU_CORE0_SW_RESET (0x10000000 to 0x600b1038)
        self.write_mem32(rst["cpucore0_cfg"], rst["cpucore0_sw_reset"])
        self._dmcontrol(0)                                     # clear sbbusy
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)            # 0x40000001
        time.sleep(0.01)                                                 # OpenOCD `sleep 10` (ms): let the SW reset propagate before re-asserting
        self._dmcontrol(DM_RESUMEREQ | DM_DMACTIVE)            # 0x40000001 (clear allhalted)
        self._dmcontrol(DM_HALTREQ | DM_NDMRESET | DM_DMACTIVE)  # 0x80000003 hold in reset

    def _deassert_reset(self, timeout=256):
        """riscv_deassert_reset: clear ndmreset (write dmactive, haltreq=0 for a
        `run`), poll dmstatus until the hart has left reset (NOT allunavail-without-
        havereset), then ackhavereset."""
        self._dmcontrol(DM_DMACTIVE)            # ndmreset <- 0, haltreq <- 0
        for _ in range(timeout):
            st = self.dm_read(DMSTATUS)
            if not (st & DM_ALLUNAVAIL) or (st & DM_ALLHAVERESET):
                break
        self._dmcontrol(DM_ACKHAVERESET | DM_DMACTIVE)

    def reset_run_from_rom(self, log=None):
        """Boot a freshly-flashed ESP32-C6 OUT of post-flash USB-Serial/JTAG ROM
        download mode and into its app — the case the plain reset_run() can't do.

        WHY reset_run() isn't enough (verified on the bench): when esptool leaves
        the chip with `--after no-reset`, the C6 is held in download mode by the
        BOOT-strap being *sampled LOW*. Per Espressif's USB-Serial/JTAG console
        guide, the USJ can only trigger a CORE reset, which does NOT re-sample the
        strap — so a bare ndmreset (or even OpenOCD's pure-JTAG `reset run`)
        re-enters the ROM downloader. Measured: ndmreset-only and OpenOCD `reset
        run` both boot 0/3 from this state; the core just lands back at the ROM
        reset vector with download still latched.

        What DOES clear the strap latch is a USB *bus* reset, which re-enumerates
        the USJ peripheral. But a USB reset ALONE is also 0/3 (the core is still
        parked in esptool's download stub). The reliable boot, proven 3/3 on
        xiao-c6-b, is the COMBINATION, in this order:

            1. USB bus reset      -> re-enumerate USJ, clear the download latch
            2. ndmreset + resume  -> restart the core so the just-re-strapped ROM
                                     boots from flash instead of the stub

        The USB bus reset goes through the cross-platform helper
        espjtag.usbreset.reset_device (pyusb dev.reset() == libusb_reset_device;
        on Linux that is the SAME USBDEVFS_RESET ioctl this flow was proven with —
        see usbreset.py and docs/CROSS-PLATFORM-USB.md for the per-OS behaviour).

        == REQUIRED BENCH RE-VERIFICATION (espjtag #13) =======================
        The 3/3 ROM-boot result was measured with the OLD code path on Linux.
        Routing the reset through usbreset.reset_device does NOT change the
        underlying call on Linux (still USBDEVFS_RESET via libusb), so the strap-
        clear SHOULD be unchanged — but this MUST be re-verified on the bench
        (flash a C6 with esptool --after no-reset, then reset_run_from_rom, and
        confirm it boots the app, repeated for confidence) before this path is
        trusted. The USB reset is load-bearing for the strap clear; do NOT assume
        it boots untested. macOS is expected NOT to work (libusb_reset_device is a
        documented silent no-op there); Windows is untested.
        ======================================================================

        Around the ndmreset we run OpenOCD's faithful reset-run handshake (ported
        from openocd-esp32 riscv-013.c + esp32c6.cfg): the esp32c6_soc_reset SBA
        writes (assert), riscv_deassert_reset's poll+ackhavereset, then the
        halt_set_dcsr_ebreak step OpenOCD does whenever a hart spontaneously
        resets — halt_go, dcsr <- 0x90c3 (ebreakm|ebreaku), and the resume
        handshake (resumereq, poll allresumeack, clear resumereq).

        Returns True on a clean resume. Leaves the app running. The USB reset
        invalidates this object's handle, so a NEW transport is opened internally
        for the JTAG phase; the caller's `self.dev` is disposed."""
        from .usbreset import reset_device, platform_reset_note

        # Guard BEFORE the USB bus reset — no point re-enumerating an Xtensa part
        # for a JTAG reset phase that can't run on it (espjtag#51).
        chips.require_riscv(self.read_idcode(), op="reset_run_from_rom over JTAG")

        def _log(m):
            if log:
                log(m)

        # --- phase 1: USB bus reset to clear the BOOT-strap-LOW download latch ---
        # Remember how to re-find this exact unit (the reset re-enumerates it).
        # (bus, port_numbers) is the stable identity that SURVIVES a bus reset —
        # the bus address changes, the physical port path does not.
        bus = self.dev.bus
        ports = tuple(self.dev.port_numbers or ())
        _log(f"  reset_run_from_rom: USB bus reset (clears C6 strap latch) "
             f"on bus {bus} ports {ports}")
        _log(f"    {platform_reset_note()}")
        reset_device(self.dev, log=_log)             # cross-platform; re-enumerates
        usb.util.dispose_resources(self.dev)

        # --- phase 2: reopen JTAG on the same unit and run the reset handshake ---
        # Re-enumeration takes a moment; retry the open until the unit is back.
        # 100 tries x ~the open's own latency covers the USJ re-enumeration with
        # margin (measured back in well under a second); each failed open is the
        # device simply not on the bus yet, so we just try again.
        j = None
        last = None
        for _ in range(100):
            try:
                j = EspUsbJtag._reopen(bus, ports)
                break
            except Exception as e:                   # noqa: BLE001
                last = e
        if j is None:
            raise RuntimeError(f"reset_run_from_rom: USJ did not re-enumerate "
                               f"after USB reset (last: {last})")

        _log("  reset_run_from_rom: JTAG back; running OpenOCD-faithful reset run")
        j.examine()
        j._c6_soc_reset()                            # assert (esp32c6_soc_reset)
        j._deassert_reset()                          # leave reset, ackhavereset
        # halt_set_dcsr_ebreak: the hart just reset -> halt, set dcsr.ebreak, resume
        j._halt_go()
        old, new = j._set_dcsr_ebreak()
        _log(f"    dcsr 0x{old:08x} -> 0x{new:08x}")
        ran = j._resume_go()
        _log(f"    resume {'ACKed (app running)' if ran else 'did NOT ack'}")
        j.dmi_write(DMCONTROL, 0)                    # release DM
        usb.util.dispose_resources(j.dev)
        return ran

    @classmethod
    def _reopen(cls, bus, ports):
        """Open the EspUsbJtag for the unit at (bus, port_numbers) — used to
        re-acquire THIS device after a USB bus reset re-enumerated it. Builds a
        usb_path string the transport's matcher understands ("bus-p.p.p")."""
        usb_path = f"{bus}-" + ".".join(str(p) for p in ports) if ports else None
        return cls(usb_path)


def diag(usb_path=None, log=print):
    """Read-only RISC-V Debug Module dump (IDCODE + dmcontrol/dmstatus/hartinfo)
    over the built-in USB-JTAG — no reset, no side effects on the running app."""
    return EspUsbJtag(usb_path).diag(log=log)


def selftest(usb_path=None, rounds=3):
    """Verify the JTAG stack against a live ESP RISC-V part (C5/C6/...): IDCODE,
    DTMCS, and a dmstatus DMI read must read sane, deterministic values `rounds`
    times. Chip-agnostic — recognises the target TAP from its IDCODE via the
    per-chip table (espjtag.chips, #4), then checks DTMCS version==1 + the table's
    expected abits and a valid dmstatus (version 2/3, op 0). Returns (passed,
    total). Read-only — safe against a board running the app."""
    passed = 0
    for r in range(rounds):
        j = EspUsbJtag(usb_path)
        ic = j.read_idcode()
        dt = j.read_dtmcs()
        ds, st = j.dmi_read(DMSTATUS)
        if (ds & 0xF) == 0:
            # dmstatus reads as all-zero (version=0) until something sets
            # dmcontrol.dmactive after a chip reset — the long-standing C5
            # "dmstatus FLAKY" rows were exactly first-touch-after-reset.
            # Activating the DM is benign (no halt, no run-state change).
            j.dmi_write(DMCONTROL, DM_DMACTIVE)
            ds, st = j.dmi_read(DMSTATUS)
        exp_abits = chips.abits_for(ic)
        dmver = ds & 0xF
        ok = (exp_abits is not None and (dt & 0xF) == 1 and j.abits == exp_abits
              and st == 0 and dmver in (2, 3))
        passed += ok
        usb.util.dispose_resources(j.dev)
        chip = chips.name_for(ic) or "??"
        print(f"  round {r}: [{chip}] IDCODE=0x{ic:08x} DTMCS=0x{dt:08x} "
              f"dmstatus=0x{ds:08x}(st{st} v{dmver})  {'PASS' if ok else 'FAIL'}")
    print(f"  selftest: {passed}/{rounds} passed")
    return passed, rounds

