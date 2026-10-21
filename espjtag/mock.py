"""mock.py — a no-hardware MOCK of the RISC-V transport (EspUsbJtag), the
companion to xtensa_mock.MockXtensaXDM. Records every DMI / Debug-Module
operation and serves reads from a register model — so RISC-V debug sequences
(reset_run, halt, register access) are validated for the RIGHT OPERATION
SEQUENCE with ZERO JTAG.

Like the Xtensa mock, it is NOT a core emulator: it models the Debug Module
registers as a dict and lets you script reads (`set_dm`). It validates "does
espjtag issue the right DMI writes?" (e.g. the golden OpenOCD reset_run
sequence) — not "does the silicon respond". OpenOCD's `dummy` adapter / probe-rs
`FakeProbe` analogue at the DMI altitude.

    j = MockEspUsbJtag()
    EspUsbJtag.reset_run(j)         # or via a subclass — see the test
    j.dmi_writes                    # [(address, value), ...] every DMI write
"""


class MockEspUsbJtag:
    """Records DMI ops, serves Debug-Module reads from a dict model. Implements
    the transport surface debug.py uses: dmi_write / dmi_read / dm_read /
    read_idcode / read_dtmcs. usb.util.dispose_resources(self.dev) is a no-op
    here (self.dev is a stub)."""

    def __init__(self, idcode=0x0000DC25):     # default: an ESP32-C6 IDCODE
        self.dm = {}                  # Debug-Module register model {address: u32}
        self.dm_seq = {}              # {address: [v0, v1, ...]} scripted read queue
        self.dmi_writes = []          # [(address, value)] every dmi_write, in order
        self.dmi_reads = []           # [address]
        self.ops = []                 # full op log
        self.idcode = idcode
        self.dev = _StubDev()         # so usb.util.dispose_resources(j.dev) works

    # --- the transport surface debug.py calls ------------------------------
    def dmi_write(self, address, data, retries=8):
        self.dmi_writes.append((address, data & 0xFFFFFFFF))
        self.ops.append(("dmi_write", address, data & 0xFFFFFFFF))
        self.dm[address] = data & 0xFFFFFFFF

    def dmi_read(self, address, retries=8):
        self.dmi_reads.append(address)
        self.ops.append(("dmi_read", address))
        # a scripted read sequence (e.g. ABSTRACTCS busy, busy, ready) takes
        # priority and yields one value per read; its last value sticks.
        seq = self.dm_seq.get(address)
        if seq:
            return (seq.pop(0) if len(seq) > 1 else seq[0]), 0
        return self.dm.get(address, 0), 0          # (data, op-status)

    def dm_read(self, address):
        return self.dmi_read(address)[0]

    def read_idcode(self):
        self.ops.append(("read_idcode",))
        return self.idcode

    def read_dtmcs(self):
        self.ops.append(("read_dtmcs",))
        return self.dm.get("dtmcs", 0)

    # --- test controls -----------------------------------------------------
    def set_dm(self, address, value):
        """Script a Debug-Module register read (e.g. DMSTATUS = allhalted)."""
        self.dm[address] = value & 0xFFFFFFFF

    def set_dm_sequence(self, address, values):
        """Script SUCCESSIVE reads of `address` (e.g. ABSTRACTCS = [busy, busy,
        ready]); one value is consumed per read, the last sticks. Used to drive
        retry/poll paths like _abstract_wait."""
        self.dm_seq[address] = [v & 0xFFFFFFFF for v in values]

    # The real debug ops are GENERIC over the transport surface above
    # (dmi_write / dm_read), so we reuse EspUsbJtag's verbatim — the mock is just a
    # different backend, exactly like MockXtensaXDM borrows XtensaXDM. This lets a
    # test call j.halt() / j.read_register() and assert the DMI sequence with no
    # hardware. (Methods that touch USB endpoints directly are NOT borrowed.)
    from .debug import EspUsbJtag as _RealDM
    halt = _RealDM.halt
    resume = _RealDM.resume
    examine = _RealDM.examine
    _abstract_wait = _RealDM._abstract_wait
    read_register = _RealDM.read_register
    write_register = _RealDM.write_register
    _wdt_disable = _RealDM._wdt_disable
    _chip = _RealDM._chip                  # chips.lookup(read_idcode()) -> chip dict
    _sb_setup = _RealDM._sb_setup
    _sbcs_value = staticmethod(_RealDM._sbcs_value)   # keep it static (see debug.py)
    read_mem32 = _RealDM.read_mem32
    write_mem32 = _RealDM.write_mem32
    # --- RISC-V trigger module (watchpoints/breakpoints, see debug.py) ---------
    _can_use_napot_match = staticmethod(_RealDM._can_use_napot_match)
    _find_next_free_trigger = _RealDM._find_next_free_trigger
    _find_first_trigger_by_id = _RealDM._find_first_trigger_by_id
    _count_trailing_ones = staticmethod(_RealDM._count_trailing_ones)
    _set_trigger = _RealDM._set_trigger
    _maybe_add_trigger_t1 = _RealDM._maybe_add_trigger_t1
    _create_wp_trigger_cache = _RealDM._create_wp_trigger_cache
    _wp_triggers_cache_add = _RealDM._wp_triggers_cache_add
    _wp_triggers_cache_search = _RealDM._wp_triggers_cache_search
    _try_use_trigger_and_cache_result = _RealDM._try_use_trigger_and_cache_result
    _try_setup_single_match_trigger = _RealDM._try_setup_single_match_trigger
    _try_setup_chained_match_triggers = _RealDM._try_setup_chained_match_triggers
    _fill_match_triggers_tdata1_fields_t2 = _RealDM._fill_match_triggers_tdata1_fields_t2
    _fill_match_triggers_tdata1_fields_t6 = _RealDM._fill_match_triggers_tdata1_fields_t6
    _maybe_add_trigger_t2_t6_for_wp = _RealDM._maybe_add_trigger_t2_t6_for_wp
    _maybe_add_trigger_t2_t6_for_bp = _RealDM._maybe_add_trigger_t2_t6_for_bp
    _maybe_add_trigger_t2_t6 = _RealDM._maybe_add_trigger_t2_t6
    _check_if_trigger_exists = _RealDM._check_if_trigger_exists
    _get_trigger_types = _RealDM._get_trigger_types
    _disable_trigger_if_dmode = _RealDM._disable_trigger_if_dmode
    _enumerate_triggers = _RealDM._enumerate_triggers
    _add_trigger = _RealDM._add_trigger
    add_watchpoint = _RealDM.add_watchpoint
    add_breakpoint = _RealDM.add_breakpoint
    remove_watchpoint = _RealDM.remove_watchpoint
    del _RealDM


class _StubCtx:
    def dispose(self, device):
        pass


class _StubDev:
    """Stub so usb.util.dispose_resources(j.dev) is a harmless no-op in tests —
    it calls device._ctx.dispose(device), which we satisfy."""
    _ctx = _StubCtx()
