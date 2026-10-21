"""espjtag.constants — esp_usb_jtag protocol + RISC-V Debug Module register/field
definitions. Shared by the transport, reset, and debug layers.

Ported (hand-transcribed numeric facts, not copied code) from:
  - openocd-esp32  src/jtag/drivers/esp_usb_jtag.c   (GPL-2.0-or-later)
        the CMD_* nibble layout, VEND_JTAG_SETDIV, caps descriptor wValue
  - openocd-esp32  src/target/riscv/debug_defines.h  (BSD-2-Clause OR CC-BY-4.0)
        the RISC-V Debug Module register addresses + field bit-offsets
Pinned upstream commit + the exact symbols we depend on: ../upstream.lock.
Provenance + licensing analysis: ../ACKNOWLEDGEMENTS.md. Drift check:
scripts/check_upstream.py. Not affiliated with Espressif / OpenOCD / RISC-V.
"""

VID, PID = 0x303A, 0x1001
VENDOR_CLASS = 0xFF                        # the JTAG interface is vendor-spec;
                                           # iface 0/1 are the CDC console — NOT it
CAPS_DESCRIPTOR = 0x2000                   # GET_DESCRIPTOR wValue

# 4-bit OUT command nibbles (esp_usb_jtag.c)
def _clk(tms, tdi, cap=0):
    return (cap << 2) | (tms << 1) | tdi   # 0b0 cap tms tdi
CMD_FLUSH = 0xA


def _bits_to_int(bits):
    return sum(b << i for i, b in enumerate(bits))

# JTAG IR opcodes (5-bit on these RISC-V cores). NOTE: this client is RISC-V only
# (C3/C5/C6/H2). The Xtensa parts (S2/S3) use the SAME esp_usb_jtag USB transport
# but a DIFFERENT debug module (Xtensa OCD/TRAX, not the RISC-V Debug Module) — no
# DMI/dmcontrol/ndmreset — so reset_run() here does not apply to them (and the S3
# boots fine via esptool anyway).
IR_DTMCS = 0x10
IR_DMI = 0x11
IR_LEN = 5
# DMI op field: 0=nop, 1=read, 2=write. dmi scan = (address<<34)|(data<<2)|op
DMI_NOP = 0
DMI_READ = 1
DMI_WRITE = 2
# RISC-V Debug Module register addresses (DMI address space) — debug_defines.h
DMCONTROL = 0x10
DMSTATUS = 0x11
HARTINFO = 0x12
ABSTRACTCS = 0x16     # abstract command status (busy, cmderr)
COMMAND = 0x17        # abstract command (access register etc.)
ABSTRACTAUTO = 0x18
DATA0 = 0x04          # abstract data 0..N
PROGBUF0 = 0x20       # program buffer 0..N
# System Bus Access registers (memory r/w without a running hart)
SBCS = 0x38
SBADDRESS0 = 0x39
SBDATA0 = 0x3C
# sbcs status fields (RISC-V debug spec): sberror[14:12] (0 = no error, W1C),
# sbbusy[21], sbbusyerror[22] (a read/write was attempted while busy — W1C).
SB_SBERROR = 0x7 << 12
SB_SBBUSY = 1 << 21
SB_SBBUSYERROR = 1 << 22
# dmcontrol bits
DM_DMACTIVE = 1 << 0
DM_NDMRESET = 1 << 1
DM_ACKHAVERESET = 1 << 28          # 0x10000000
DM_HARTRESET = 1 << 29             # 0x20000000
DM_RESUMEREQ = 1 << 30            # 0x40000000
DM_HALTREQ = 1 << 31             # 0x80000000
# dmstatus bits
DM_ANYRUNNING = 1 << 10
DM_ALLHALTED = 1 << 9
DM_ALLRUNNING = 1 << 11
DM_ANYRESUMEACK = 1 << 16
DM_ALLRESUMEACK = 1 << 17
DM_ANYHAVERESET = 1 << 18
DM_ALLHAVERESET = 1 << 19
DM_ANYUNAVAIL = 1 << 12
DM_ALLUNAVAIL = 1 << 13
# abstractcs
ABS_BUSY = 1 << 12
ABS_CMDERR = 0x7 << 8
# command: access register
CMD_ACCESS_REGISTER = 0 << 24
AC_TRANSFER = 1 << 17
AC_WRITE = 1 << 16
AC_AARSIZE32 = 2 << 20
AC_POSTEXEC = 1 << 18
# CSR numbers
CSR_DCSR = 0x7B0
CSR_DPC = 0x7B1
CSR_MSTATUS = 0x300
MSTATUS_MIE = 1 << 3      # global M-mode interrupt enable
# GPR register index in abstract-command regno space (0x1000 + gpr)
REG_GPR_BASE = 0x1000

# NOTE: the ESP32-C6 LP_AON software-reset registers (used by esp32c6_soc_reset)
# moved to the per-chip table — espjtag.chips CHIPS[0x0000dc25]["reset"] is now
# the single source of truth (#4). 0x600b1034 bit31 = LP_AON_HPSYS_SW_RESET,
# 0x600b1038 bit28 = LP_AON_CPU_CORE0_SW_RESET; source openocd-esp32
# tcl/target/esp32c6.cfg esp32c6_soc_reset. debug.py reads them via chips.reset_for().

# RISC-V `ebreak` instruction word. Used as a return-trap: point a halted hart's
# return address (ra) at a scratch SRAM word holding this, so when a called
# function returns it re-enters debug mode (the "call a function on the target"
# mechanism, #3). Also what OpenOCD's progbuf-call recipe lands on.
EBREAK = 0x00100073

# dcsr value OpenOCD writes after a reset (set_dcsr_ebreak): ebreakm|ebreaku set
# so an EBREAK in any privilege mode enters debug mode rather than trapping. Read
# (0xc3 on the C6 at reset), OR in the ebreak bits, write back -> 0x90c3. This is
# part of OpenOCD's reset-run handshake the bare-write list omitted.
DCSR_EBREAKU = 1 << 12                   # 0x1000
DCSR_EBREAKM = 1 << 15                   # 0x8000
DCSR_EBREAK_BITS = DCSR_EBREAKM | DCSR_EBREAKU   # -> turns 0xc3 into 0x90c3

# dcsr.step / dcsr.cause (debug_defines.h:513-470, step-over-trigger port #57).
# CAUSE is read-only; hardware reports WHY the hart is in debug mode (ebreak=1,
# trigger=2, haltreq=3, step=4, resethaltreq=5, group=6, other=7).
CSR_DCSR_STEP_OFFSET = 2
CSR_DCSR_STEP = 1 << CSR_DCSR_STEP_OFFSET                     # 0x4
CSR_DCSR_CAUSE_OFFSET = 6
CSR_DCSR_CAUSE_MASK = 0x7 << CSR_DCSR_CAUSE_OFFSET            # 0x1c0
CSR_DCSR_CAUSE_EBREAK = 1
CSR_DCSR_CAUSE_TRIGGER = 2
CSR_DCSR_CAUSE_HALTREQ = 3
CSR_DCSR_CAUSE_STEP = 4
CSR_DCSR_CAUSE_RESETHALTREQ = 5

# Standard RISC-V privileged-spec CSR (not Debug Module); not in debug_defines.h
# (that file only has the Debug Module's own registers). Same provenance class
# as CSR_MSTATUS above: a fixed, well-known CSR number, not sourced from
# debug_defines.h. Used to read misa.{S,U,H} the way OpenOCD's
# riscv_supports_extension() does (riscv.c, see fill_match_triggers_tdata1_fields_t2/t6).
CSR_MISA = 0x301
MISA_EXT_S = 1 << (ord('S') - ord('A'))
MISA_EXT_U = 1 << (ord('U') - ord('A'))
MISA_EXT_H = 1 << (ord('H') - ord('A'))

# === RISC-V trigger module (watchpoints/breakpoints) ===================
# openocd-esp32 src/target/riscv/debug_defines.h @ a585825d4d0bbd5b8aff83e01816d2f2c04ffd84
# (2026-10-21; the already-pinned f10eceff.. SHA's debug_defines.h is confirmed
# byte-identical for this file, so these values hold at either SHA — see
# scripts/check_upstream.py run notes in the trigger-support commit).
# Pinned upstream: ../upstream.lock. Provenance: ../ACKNOWLEDGEMENTS.md.
#
# espjtag targets RV32 parts only (C3/C5/C6 — RV32IMAC/RV32IMAFC); the upstream
# macros are parameterised on XLEN, so every mask below is that macro evaluated
# at XLEN=32 — transcribed, not re-derived.
XLEN = 32

# Trigger CSR numbers (debug_defines.h:540,544,637,645)
CSR_TSELECT = 0x7A0
CSR_TDATA1 = 0x7A1
CSR_TDATA2 = 0x7A2
CSR_TINFO = 0x7A4

# tdata1.type field — common to every trigger type (debug_defines.h:545-547,
# CSR_TDATA1_TYPE_OFFSET(XLEN)=(XLEN-4), CSR_TDATA1_TYPE(XLEN)=0xf<<(XLEN-4))
CSR_TDATA1_TYPE_OFFSET = XLEN - 4                      # 28
CSR_TDATA1_TYPE_MASK = 0xF << CSR_TDATA1_TYPE_OFFSET   # 0xf0000000
# tdata1.type values (debug_defines.h:551-601)
CSR_TDATA1_TYPE_NONE = 0
CSR_TDATA1_TYPE_LEGACY = 1
CSR_TDATA1_TYPE_MCONTROL = 2
CSR_TDATA1_TYPE_ICOUNT = 3
CSR_TDATA1_TYPE_ITRIGGER = 4
CSR_TDATA1_TYPE_ETRIGGER = 5
CSR_TDATA1_TYPE_MCONTROL6 = 6
CSR_TDATA1_TYPE_TMEXTTRIGGER = 7
CSR_TDATA1_TYPE_DISABLED = 15

# tinfo (debug_defines.h:649,651,679)
CSR_TINFO_VERSION_OFFSET = 0x18
CSR_TINFO_VERSION_MASK = 0xFF000000
CSR_TINFO_INFO_MASK = 0xFFFF            # trigger-type-support bitmask, bit N = type N
# tinfo.version values (debug_defines.h:663,668) -- step-over-trigger port (#57):
# version 0 means "hit bits exist but their before/after timing is unknown",
# version 1 means "hit[1:0] encodes before/after/immediately-after" (debug_defines.h
# CSR_MCONTROL6_HIT0_{BEFORE,AFTER,IMMEDIATELY_AFTER} below).
CSR_TINFO_VERSION_0 = 0
CSR_TINFO_VERSION_1 = 1

# mcontrol (type 2, legacy match trigger) fields at XLEN=32 (debug_defines.h:742-1100)
CSR_MCONTROL_TYPE_OFFSET = XLEN - 4                          # 28 (:743)
CSR_MCONTROL_TYPE_MASK = 0xF << CSR_MCONTROL_TYPE_OFFSET
CSR_MCONTROL_DMODE_OFFSET = XLEN - 5                         # 27 (:746)
CSR_MCONTROL_DMODE_MASK = 1 << CSR_MCONTROL_DMODE_OFFSET
CSR_MCONTROL_MASKMAX_OFFSET = XLEN - 11                      # 21 (:757)
CSR_MCONTROL_MASKMAX_MASK = 0x3F << CSR_MCONTROL_MASKMAX_OFFSET
CSR_MCONTROL_SIZELO_OFFSET = 0x10                            # (:854-856)
CSR_MCONTROL_SIZELO_MASK = 0x30000
CSR_MCONTROL_SIZELO_ANY = 0                                  # (:862)
CSR_MCONTROL_SIZEHI_OFFSET = 0x15                            # (:766-768, XLEN>=64 only)
CSR_MCONTROL_SIZEHI_MASK = 0x600000
CSR_MCONTROL_ACTION_OFFSET = 0xC                             # (:925-927)
CSR_MCONTROL_ACTION_MASK = 0xF000
CSR_MCONTROL_ACTION_DEBUG_MODE = 1                           # (:935)
CSR_MCONTROL_CHAIN_OFFSET = 0xB                              # (:956-958)
CSR_MCONTROL_CHAIN_MASK = 0x800
CSR_MCONTROL_CHAIN_DISABLED = 0                              # (:962)
CSR_MCONTROL_CHAIN_ENABLED = 1                               # (:967)
CSR_MCONTROL_MATCH_OFFSET = 0x7                              # (:992-994)
CSR_MCONTROL_MATCH_MASK = 0x780
CSR_MCONTROL_MATCH_EQUAL = 0                                 # (:998)
CSR_MCONTROL_MATCH_NAPOT = 1                                 # (:1008)
CSR_MCONTROL_MATCH_GE = 2                                    # (:1013)
CSR_MCONTROL_MATCH_LT = 3                                    # (:1018)
CSR_MCONTROL_M_OFFSET = 6                                    # (:1061-1063)
CSR_MCONTROL_M_MASK = 0x40
CSR_MCONTROL_S_OFFSET = 4                                    # (:1069-1071)
CSR_MCONTROL_S_MASK = 0x10
CSR_MCONTROL_U_OFFSET = 3                                    # (:1077-1079)
CSR_MCONTROL_U_MASK = 0x8
CSR_MCONTROL_EXECUTE_OFFSET = 2                              # (:1084-1086)
CSR_MCONTROL_EXECUTE_MASK = 0x4
CSR_MCONTROL_STORE_OFFSET = 1                                # (:1091-1093)
CSR_MCONTROL_STORE_MASK = 0x2
CSR_MCONTROL_LOAD_OFFSET = 0                                 # (:1098-1100)
CSR_MCONTROL_LOAD_MASK = 0x1
# mcontrol.hit (debug_defines.h:779) -- step-over-trigger port (#57): set by
# hardware when this trigger fired; always "before" timing (no hit0/hit1 pair
# like mcontrol6), so a type-2 hit always needs the step-over-and-rearm dance.
CSR_MCONTROL_HIT_OFFSET = 0x14
CSR_MCONTROL_HIT_MASK = 0x100000

# mcontrol6 (type 6, what C3/C5/C6 actually report) fields at XLEN=32
# (debug_defines.h:1101-1462). NOTE (#trigger-port): MATCH_OFFSET=7 (4 bits,
# bits 7-10) and M_OFFSET=6 — previously mis-remembered as MATCH=10/M=9, which
# overlapped; transcribed directly from the header this time, not from memory.
CSR_MCONTROL6_TYPE_OFFSET = XLEN - 4                         # 28 (:1102)
CSR_MCONTROL6_TYPE_MASK = 0xF << CSR_MCONTROL6_TYPE_OFFSET
CSR_MCONTROL6_DMODE_OFFSET = XLEN - 5                        # 27 (:1105)
CSR_MCONTROL6_DMODE_MASK = 1 << CSR_MCONTROL6_DMODE_OFFSET
CSR_MCONTROL6_VS_OFFSET = 0x18                               # (:1134)
CSR_MCONTROL6_VS_MASK = 0x1000000
CSR_MCONTROL6_VU_OFFSET = 0x17                               # (:1142)
CSR_MCONTROL6_VU_MASK = 0x800000
CSR_MCONTROL6_SIZE_OFFSET = 0x10                             # (:1212)
CSR_MCONTROL6_SIZE_MASK = 0x70000
CSR_MCONTROL6_SIZE_ANY = 0                                   # (:1220)
CSR_MCONTROL6_ACTION_OFFSET = 0xC                            # (:1271)
CSR_MCONTROL6_ACTION_MASK = 0xF000
CSR_MCONTROL6_ACTION_DEBUG_MODE = 1                          # (:1281)
CSR_MCONTROL6_CHAIN_OFFSET = 0xB                             # (:1302)
CSR_MCONTROL6_CHAIN_MASK = 0x800
CSR_MCONTROL6_CHAIN_DISABLED = 0                             # (:1308)
CSR_MCONTROL6_CHAIN_ENABLED = 1                              # (:1313)
CSR_MCONTROL6_MATCH_OFFSET = 7                               # (:1338)
CSR_MCONTROL6_MATCH_MASK = 0x780
CSR_MCONTROL6_MATCH_EQUAL = 0                                # (:1344)
CSR_MCONTROL6_MATCH_NAPOT = 1                                # (:1356)
CSR_MCONTROL6_MATCH_GE = 2                                   # (:1361)
CSR_MCONTROL6_MATCH_LT = 3                                   # (:1366)
CSR_MCONTROL6_M_OFFSET = 6                                   # (:1409-1411)
CSR_MCONTROL6_M_MASK = 0x40
CSR_MCONTROL6_S_OFFSET = 4                                   # (:1431-1433)
CSR_MCONTROL6_S_MASK = 0x10
CSR_MCONTROL6_U_OFFSET = 3                                   # (:1439-1441)
CSR_MCONTROL6_U_MASK = 0x8
CSR_MCONTROL6_EXECUTE_OFFSET = 2                             # (:1446-1448)
CSR_MCONTROL6_EXECUTE_MASK = 0x4
CSR_MCONTROL6_STORE_OFFSET = 1                               # (:1453-1455)
CSR_MCONTROL6_STORE_MASK = 0x2
CSR_MCONTROL6_LOAD_OFFSET = 0                                # (:1460-1462)
CSR_MCONTROL6_LOAD_MASK = 0x1
# mcontrol6.hit0/hit1 (debug_defines.h:1126-1190) -- step-over-trigger port
# (#57): combine into a 2-bit field (hit1<<1|hit0) reporting WHEN the trigger
# fired relative to the matched instruction. BEFORE means the watched memory
# access has NOT committed yet -- resuming without stepping past it re-fires
# at the identical PC forever (riscv.c check_mcontrol6_hit_status/
# riscv_trigger_detect_hit_bits, ported below as _check_mcontrol6_hit_status/
# _riscv_trigger_detect_hit_bits).
CSR_MCONTROL6_HIT0_OFFSET = 0x16
CSR_MCONTROL6_HIT0_MASK = 0x400000
CSR_MCONTROL6_HIT1_OFFSET = 0x19
CSR_MCONTROL6_HIT1_MASK = 0x2000000
CSR_MCONTROL6_HIT0_FALSE = 0
CSR_MCONTROL6_HIT0_BEFORE = 1
CSR_MCONTROL6_HIT0_AFTER = 2
CSR_MCONTROL6_HIT0_IMMEDIATELY_AFTER = 3
