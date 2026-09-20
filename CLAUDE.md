# AlphaPaint

Guidance for Claude Code (and humans) working on this repository.

AlphaPaint is a pen plotter: a Raspberry Pi 5 ("davinci") runs a daemon that
connects a hand-built control console (ESP32) to a FluidNC motion controller
(ESP32). Drawings are produced by external programs that talk to the daemon
over a JSON protocol.

**The machine moves.** Homing, jogging and drawing are physical motions that can
crash the gantry into the pen holder. Never start motion (`$H`, `$J`, G-code,
pressing buttons for the user) without the user explicitly asking for it.
Stopping the daemon takes the machine out of service - say so.

## Hardware

| Part | Details |
|---|---|
| Machine | CoreXY, roughly 1 m x 1 m, `max_travel_mm: 955` on X and Y |
| Motion controller | ESP32 running FluidNC v3.0.x, built from `~/FluidNC` (a fork, not upstream) |
| Motors | 2x TMC2209 over UART2, 20 steps/mm, 4 microsteps, 1.5 A run / 0.3 A hold |
| Pen (Z) | RC servo on gpio.12, `max_travel_mm: 60`; Z=60 is up, Z=0 is down |
| Homing | Physical switches: X on gpio.22, Y on gpio.21, both active high with pull-down. Home is X=0, Y=0, Z=60; X and Y home towards 0 |
| Pen changer | 5 slots at X = 700 + 34*index, Y = 12, Z = 13. **Stands on the right; the gantry can crash into it and no endstop is hit there** |
| Console | ESP32, TM1637 displays, PCF-style I2C LED expander, handwheel via PCNT, buttons A-G + X/Y/Z |

Both USB adapters are CP2102 chips **with the same serial number**, so
`/dev/serial/by-id` holds only one of them and `ttyUSB0`/`ttyUSB1` swap between
boots. Always identify by probing (see "Talking to the boards directly").

## Layout

```
~/AlphaPaint/Software/
  Daemon/           Python daemon (runs in place from this checkout)
    daemon.py       Entry point: port scan, device identification, reconnects
    config.yaml     Live config (config.dev.yaml takes precedence if present)
    venv/           Its virtualenv (git-ignored); the systemd unit uses it
    lib/
      fluidnc.py          Serial protocol, flow control, homing, status, alarms
      console.py          Console protocol (TYPE:ARG:ARG lines)
      state_machine.py    States, buttons, canvas, external programs
      external_program.py JSON API for drawing programs; coordinate transforms
      position_tracer.py  Expected vs reported position, CSV trace, warnings
      drawing.py          Line/ellipse helpers (legacy, buttons now run programs)
    tools/analyze_position_trace.py   Post-mortem analysis of the CSV trace
  Console/          ESP-IDF project for the console firmware (main/console.c)
  Programs/         Drawing programs + the client library
    alphapaint.py   Client library: canvas/machine moves, pen, pen changer
    alphapreview.py Offline preview backend (no machine)
    pentest.py      Button F: grid of crosses/squares/circles, uses pen changer
    ekster.py       Button G: scribble drawing from ekster.png, uses pen changer
    logo_runner.py  Button E: draws a .logo file (Examples/ROBOT.logo)
~/FluidNC/          FluidNC fork actually flashed on the controller
```

## Architecture

**Threads in the daemon** (this is where most bugs live):

- main thread: port scanning, reconnect loop
- console read thread: parses console lines, calls state-machine callbacks
- FluidNC read thread: parses status reports, delivers `ok`/`error:`, calls the
  status/alarm callbacks
- homing thread: started per homing cycle
- external-program threads: stdout reader (API calls) and stderr logger

State changes go through `StateMachine._lock`. Callbacks that arrive on the
FluidNC read thread must not call back into FluidNC synchronously (that would
deadlock); `_position_lost()` spawns a thread for that reason.

**States**: `STARTUP -> NOT_HOMED -> HOMING -> CANVAS_SETUP -> READY ->
EXTERNAL_PROGRAM`, plus `ERROR`. Buttons: A homes (long press re-homes),
B and C set the canvas corners, D sets pen-down Z, E/F/G start programs.

**Console protocol** (line based, both directions):
`BTN:A:SHORT`, `POS:X:12.34`, `AXIS:X:SELECT`, `STATUS:HOMED`, `ERROR:...` from
the console; `MODE:ACTIVE|PASSIVE`, `LED:A:ON|OFF|BLINK|FAST_BLINK`,
`POS:X:12.34`, `LIMIT:X:0.00:955.00`, `ID?` to the console. In ACTIVE mode the
console is the position master: the handwheel sends absolute targets, which the
daemon forwards as `$J=` jogs (clamped to the machine limits).

**FluidNC protocol**: character-counted streaming (128-byte RX buffer), `ok` per
line, auto status reports at 10 Hz (`$Report/Interval=100`, `$10=3` for MPos and
buffer). `ok` means "planned", **not** "executed" - use `G4 P0` to synchronize.

**External program API** (JSON lines on stdin/stdout, see `external_program.py`):
`query_machine`, `query_canvas`, `query_position`, `pen_up`, `pen_up_fast`,
`pen_down`, `move_to`, `draw_to`, `draw_arc`, the `canvas_*` and `normalized_*`
variants, `set_feedrate`, `rehome_y`, `flush`, `done`. Programs are killed with
an `interrupted` event; a rejected move raises in the program.

## Remote control API (software instead of the console)

`Daemon/lib/remote_api.py` serves a JSON API (default `http://127.0.0.1:8080`)
that hands control to external software: the console then only shows the
position on its displays, while its buttons do nothing (a long press on A stays
as an abort unless `remote_api.abort_button` is false). This is the route for a
program - or a language model wrapping these calls as tools - to drive the
machine and the gantry camera.

```
GET  /api/status        POST /api/control {"mode":"remote"|"console"}
POST /api/home          POST /api/stop      POST /api/sync
POST /api/move {"x","y","z","feed","draw","wait"}     machine coordinates
POST /api/pen {"action":"up"|"down","z"}   POST /api/pen/z {"z"}
POST /api/pen/pickup {"index"}   POST /api/pen/return {"index"}
POST /api/photo {"name"}         GET  /api/photo/last
```

`Programs/plotter_api.py` is a small client (standard library only, so it also
runs under the system python3 which has OpenCV), and `Programs/explore.py` uses
it for the calibration experiments: `scan` (diagonal sweep with photos),
`paper` (find the sheet in those photos), `pen-depth` (a ladder of strokes at
decreasing Z to see where a pen starts marking) and `camera` (draw a cross and
look at it to get the camera-to-pen offset and mm per pixel).

`MachineController` (`lib/machine_control.py`) is the layer underneath: limits
checked, a rejected move raises, a move counts as done only once FluidNC really
executed it, and the pen changer verifies the position before entering a slot.
Photos go through `ffmpeg` so the daemon needs no image libraries; the camera
is found automatically among the USB video devices.

### Gantry camera, measured 2026-09-20

The camera is mounted **rotated 90 degrees**: machine +X runs along image +y,
machine +Y along image +x. Scale 12.2 px/mm across X and 12.4 px/mm across Y
(about 0.079 mm per pixel), so one 1280x720 frame covers roughly 103 x 57 mm.

It looks **41.4 mm in +Y** ahead of the pen tip (X offset -0.2 mm, i.e. none):
to photograph the point (X, Y), put the gantry at (X - 0.2, Y - 41.4). Measured
by drawing a 20 mm cross and centring it in the frame; the check shot landed
within 0.3 mm. Values live in `~/alphapaint-exploration/camera-calibration.json`.

Check the **sign** against the machine, not against a phase correlation alone:
the shift that comes out of `cv2.phaseCorrelate` is the scene moving, which is
the opposite of the camera moving, and reading it the wrong way round put the
X axis upside down in the first version of this note.

### The pens, measured 2026-09-20 (`Programs/pen_survey.py`)

| slot | ink | contact Z | thin line | fat line |
|---|---|---|---|---|
| 0 | dark blue-teal, rgb(15,64,112) | below 2.5 | 1.0 mm at Z 2.2 (pale) | 1.8 mm at Z 0.6 |
| 1 | purple, rgb(73,58,111) | below 2.2 | 0.27 mm at Z 0.4 | 0.68 mm at Z 1.6 |
| 2, 3, 4 | **nothing** - empty slots or dried-out pens | - | - | - |

Pen 0 is a broad marker: it lays about **1.7 mm**, not the half millimetre a
drawing program tends to assume. Planning a scribble drawing at a finer line
than the pen really is fills every dark area into a solid blob - that is what
happened to the first magpie. Match the planning pixel to the measured width.

Below Z 0.2 pen 0 draws wider but **paler** (rgb 47,86,116 instead of 15,64,112):
pressing harder splays the tip and starves the ink. Z 0.6 to 1.2 is its best
range. Pen 1 stops marking altogether below Z 0.4.

Auto exposure blows a white sheet out completely (mean grey 255); the
`camera.controls` in `config.yaml` fix the exposure and white balance and are
applied before every shot.

Security: the API binds to localhost. Opening it to the network lets anything
on that network move the machine, so set `remote_api.token` as well.

## Operating

```bash
systemctl status alphapaint-daemon          # state
sudo systemctl restart alphapaint-daemon    # restart (machine goes to NOT_HOMED)
sudo systemctl stop alphapaint-daemon       # required before touching the ports
journalctl -u alphapaint-daemon -f          # live log
tail -f /var/log/alphapaint-daemon.log      # same log, file copy
```

The unit is `/etc/systemd/system/alphapaint-daemon.service` and runs the daemon
**in place** from this checkout with `Software/Daemon/venv/bin/python`.
`Software/Daemon/install.sh` re-creates that setup; it deliberately does not
copy files anywhere (an old copy in `~/alphapaint` used to be the live code).

Logs: `/var/log/alphapaint-daemon.log` and the position trace
`/var/log/alphapaint-position-trace.csv`, both rotated by
`/etc/logrotate.d/alphapaint` with `copytruncate` (the daemon keeps them open).
The `max_size`/`backup_count` keys in `config.yaml` are not implemented.

Analyze a trace: `Software/Daemon/venv/bin/python
Software/Daemon/tools/analyze_position_trace.py /var/log/alphapaint-position-trace.csv`.

## Talking to the boards directly

Stop the daemon first, then identify the ports (they swap):

```python
# $I -> "[VER:3.0 FluidNC ...]" means FluidNC; "ERROR:UNKNOWN_CMD" or
# "CONSOLE:ALPHAPAINT:V1.2" (answer to ID?) means the console.
import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
s.write(b'$I\n'); time.sleep(1); print(s.read(4096).decode(errors='replace'))
```

Useful FluidNC commands: `$I` version, `?` status (realtime, **no newline**),
`$CD` dump the running config, `$LocalFS/Show=config.yaml`, `$SS` startup log,
`$Limits` (interactive, leave with `!`). Do not send `$X`: it declares all axes
homed again, which is exactly the safety net we want to keep.

Reading a status line while the board reboots produces garbage - re-read before
concluding anything is wrong.

## Flashing

**FluidNC** (`~/FluidNC`, PlatformIO in `~/.pio-venv`):

```bash
sudo systemctl stop alphapaint-daemon
cd ~/FluidNC
~/.pio-venv/bin/pio run -e wifi                                      # build
~/.pio-venv/bin/pio run -e wifi -t upload   --upload-port /dev/ttyUSB0   # firmware
~/.pio-venv/bin/pio run -e wifi -t uploadfs --upload-port /dev/ttyUSB0   # config.yaml
sudo systemctl start alphapaint-daemon
```

- `platformio_override.ini` (git-ignored) pins `upload_speed = 230400`. The
  CP2102 adapters fail at 460800 and 921600 ("Packet content transfer stopped").
- `uploadfs` writes `FluidNC/data/` (config.yaml, favicon.ico, index.html.gz) -
  the same three files the controller holds, so nothing else is lost.
- Run flashing in the background (`run_in_background`); an interrupted write
  leaves the controller without working firmware until it is redone.
- The machine config lives in `FluidNC/data/config.yaml` **in git**; keep it in
  sync with the controller (`$CD` shows what is actually loaded).

**Console** (ESP-IDF 5.2 in `~/esp/esp-idf`):

```bash
cd ~/AlphaPaint/Software/Console
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB1 -b 230400 flash
```

The FluidNC repo has no git identity configured; commit there with
`git -c user.name=Tom-Michiels -c user.email=96994937+Tom-Michiels@users.noreply.github.com`.

## Testing without the machine

Most of the serial layer can be tested against a fake FluidNC on a pty: open
`pty.openpty()`, answer `?` with `<Idle|MPos:...|Bf:15,128>`, `ok` per line,
a `Grbl 3.0 [...]` banner on `\x18`, and point `FluidNCHandler` at
`os.ttyname(slave)`. That is how the homing verification, the feed-hold stop,
concurrent `send_gcode` calls and the alarm/reboot detection were verified.
`alphapreview.py` renders a program's output without a machine.

## The open problem: sudden loss of position

**Symptom** (from the user): the plotter is accurate for a long time, then in
one go it draws completely beside the paper, after roughly five minutes of
drawing. It also drives far into Y-negative and crashes into the pen holder on
the right, where no endstop is hit. It is *not* gradual step loss, and the
plotter lies flat (gravity plays no role).

Because every move is absolute, a single wrong command cannot explain it: the
next move would come back to the right place. So the link between FluidNC's
computed position and the physical machine breaks in one event (mechanical slip
or a driver dropping out), or a move in a sequence silently did not happen.

Evidence found in the logs (January - June), all addressed:

- 448x `error:33`: arcs rejected because coordinates were rounded to 2 decimals;
  one rejected arc shifts the start of the next one, so they cascade.
- 9710 orphaned `ok`s, hundreds of "recovered" (invented) `ok`s and stale
  timeouts: the ok/command bookkeeping desynchronized while the machine was busy.
- Six homing cycles ran the full 1050.5 mm seek (1.1 x 955) without seeing the X
  switch, grinding into the frame, and the daemon still reported "Homing
  complete". Always preceded by a Ctrl-X sent during motion.
- Rejected moves returned `success: False`, which the programs ignored and
  continued as if the move had happened.

What is in place now (2026-09-19): verified homing, feed hold before every
reset, alarm/reboot detection, arc validation, clamped jogs, a position check
before entering a pen slot, and SpreadCycle instead of StealthChop on X/Y
(`run_mode: CoolStep`). Driver fault logging was tried and reverted, see
"Mistakes made here". Whether spreadCycle is really active is still unverified:
check that both drivers report `GCONF: 0x1c4` at the next homing. `hold_amps` stays 0.3 (the user confirmed holding torque is
not the issue) and automatic Y re-homing during programs is available but
**disabled** (`REHOME_Y_EVERY_N_PEN_CHANGES = 0` in `alphapaint.py`) because it
moves Y to 0 and the pen holder's clearance has not been confirmed.

**Root cause found (2026-09-20).** `tools/motor_stall_test.py` reproduced it in
90 seconds: at `run_amps: 1.5` the X driver reaches its 143 C thermal shutdown
(`temp_shutdown:Y` in the `$MS` output) and that motor stops turning. On CoreXY
a Y move then runs diagonally - Tom saw the gantry go (+1,-1) instead of (0,-1),
which means the X+Y motor stood still - and the machine loses its position in
one go. At 1.0 A there is no thermal event but the motors lose steps from lack
of torque (about 10 mm per two minutes of sweeping). So the drivers need
cooling; `tools/current_sweep.py` measures where the usable window is.

Older notes, now superseded: Next time it happens,
collect `journalctl -u alphapaint-daemon` and the CSV trace and look for
`driver fault` (over-temperature or short - electrical), `FluidNC alarm`, or
neither (then suspect belts/pulley grub screws).

## Mistakes made here - do not repeat them

**Never read or write TMC registers from another task.** Both TMC2209s share
one UART and TMCStepper is not thread safe. A "driver health poll" added on
2026-09-19 read `DRV_STATUS` once a second from the stallguard timer task; it
raced with register writes from the protocol task and a write was lost. One
driver ended up with `GCONF 0x0`, so it took its microstep setting from the
MS1/MS2 pins (which also select the UART address, and differ per driver)
instead of from the register. The two motors then had different steps/mm and
the CoreXY machine homed **diagonally**. Registers may only be touched from the
code paths FluidNC already uses (config, `set_registers`, `set_disable`).

How to check: FluidNC dumps the registers at each homing. Both drivers must
show the same `GCONF`; `0x1c0` is StealthChop configured correctly, `0x1c4`
adds spreadCycle (`run_mode: CoolStep`). `GCONF: 0x0` means the driver is
unconfigured - stop and fix that first.

```bash
journalctl -u alphapaint-daemon --since "-10min" | grep -E 'GCONF|Homed'
```

**`ok` is not "executed", and the cached position lags.** A toolchanger safety
check compared the expected position with `_get_current_position()`, which
returns the cached auto-report (up to one 100 ms report old, several mm while
moving). It aborted a pen pickup at X=701.05 while the machine reached exactly
X=700.00 64 ms later. Synchronize with `G4 P0` **and** query a fresh status
(`_get_current_position(fresh=True)`) before judging a position.

**Verify a regression against the historical logs before blaming the machine.**
The `GCONF 0x0` was obvious once compared with `/var/log/alphapaint-daemon.log.1`,
where every historical reading was `0x1c0`.

## Conventions

- Comments and commit messages in English; the user writes Dutch, answer in Dutch.
- Do not change `hold_amps`, feed rates or accelerations without asking: the
  user knows this machine's mechanics better than the logs show.
- Position consumers must never see an invented position; `_parse_status`
  returns `None` for a truncated report rather than `(0,0,0)`.
- Prefer `feed hold -> wait for standstill -> Ctrl-X` over a bare soft reset:
  a reset during motion makes FluidNC lose its position.
