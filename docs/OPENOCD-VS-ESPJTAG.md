# OpenOCD vs espjtag — why this project exists, and what each is for

## OpenOCD friction was ours, not OpenOCD's

Three issues hit during this project's development, all missing/wrong config
on our side, not OpenOCD bugs:

- **Wrong board grabbed** when multiple ESP32s are on the bus — fixed with
  `adapter serial "<serial>"` in the OpenOCD config. NOT `espusbjtag
  serial_string`, which does not exist in the version used (confirmed via the
  driver's own help text).
- **GDB run/stop desync** ("Cannot execute this command while the target is
  running" despite a real halt), caused by RTOS-awareness — fixed with `set
  ESP_RTOS none` to disable FreeRTOS task tracking.
- **Flash-stub fault on GDB connect** (`Error: Failed to run flasher stub
  (-302)!`), caused by GDB's default flash-memory-map query triggering
  OpenOCD's flash probe — fixed with `gdb_memory_map disable` +
  `gdb_breakpoint_override hard`.

Minimal config (observed working, `esp32c5_c5board.cfg`, esp-emulator repo —
a scratch `tmp/` file there, not a tracked reference, so the lines are given
here directly):

```
adapter driver esp_usb_jtag
espusbjtag vid_pid 0x303a 0x1001
espusbjtag caps_descriptor 0x2000
adapter serial "10:BD:A3:CA:28:A4"
adapter speed 40000
transport select jtag
set ESP_RTOS none
source [find target/esp32c5.cfg]
gdb_memory_map disable
gdb_breakpoint_override hard
```

## Recommendation

- **espjtag is the default for interactive register-level work** on these
  chips: purpose-built, every line traces to a cited upstream source
  (`docs/FAITHFUL-REIMPLEMENTATION.md`), bugs get found and fixed same-day.
  Example: issue #57 (watchpoint resume re-firing forever) — found via real
  hardware testing, root-caused against the real upstream source, fixed and
  verified on hardware in the same session.
- **OpenOCD still has its place**: GDB-remote-protocol serving for IDE
  debugging, broader RTOS-awareness, wider adapter/chip coverage than espjtag
  targets. Also useful as an occasional cross-check if espjtag and OpenOCD
  ever disagree on a register read.
- The relationship is not "espjtag exists only to be verified against
  OpenOCD" — at this point it is the reverse: espjtag is the trusted
  register-level tool, OpenOCD is the occasional second opinion.
