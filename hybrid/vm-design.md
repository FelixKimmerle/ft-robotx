# ROBO TX resident runtime: bytecode VM + Bluetooth link via a relay TX

Design notes for a program that runs **on** the fischertechnik ROBO TX Controller (TX, *not* TXT) as a local
(download) program, runs small bytecode programs at the controller's own rate, and talks to the PC through a
second TX acting as Bluetooth relay (called gateway in earlier notes).

Status (2026-11-10): the link works. The online-mode commands for Bluetooth messages are reverse engineered, the
relay protocol carries single commands (opcode + arguments) to `RelayModel` on the model TX, and the XCP bridge uses
it (see [`relay/README.md`](relay/README.md) and [`../xcp/README.md`](../xcp/README.md)). The VM with control flow
is not implemented yet; its instructions will extend the relay protocol's opcodes.

## Why

- Online mode (already reverse engineered in `online/`) puts every decision on the PC. Over USB a round trip takes
  ~1 ms, but the TX refreshes its I/O only every 10 ms; over Bluetooth only one round trip every ~80 ms. Through
  the relay, a command takes ~40 ms. A line follower moves too far in 40-160 ms.
- Online mode cannot draw on the display (framebuffer), only show text messages.
- Goal: the PC (Linux) uploads programs, parameters and commands; the TX executes logic like
  `if I1 then M1 = 512 else M1 = 0` locally every cycle and reports back at a lower rate.

## Facts about the platform

Sources: `PC_Programming_RoboTXC` part 2 (ftMscLib, V1.5), part 3 (C compiler, V1.2), `offline/sdk/include`.

| Fact | Source / status |
|---|---|
| ARM9 at 200 MHz (likely Atmel AT91SAM9, ARM926EJ-S), no FPU, 8 MB RAM | product data; verify chip on board |
| Local program runs in **1 ms slots, ~0.5 ms each** for the program (~100k cycles per tic) | C compiler doc 2.3; `CALL_CYCLE_MS 1` in `ROBO_TX_PRG.h` |
| Transfer area is synced with the hardware every **10 ms** | C compiler doc 2.2; matches 100 packets/s over USB |
| **I/O commands from USB/BT interrupt a running local program** (switch to online mode; restart needed) | C compiler doc 2.1 |
| Local programs have **no USB/serial API**; only BT messages to other TX, I2C, display, timers | hook table in `ROBO_TX_FW.h` |
| BT message API is **TX <-> TX only** (connecting a non-TX fails with `BT_NOT_ROBOTX`), 8 channels (1..8) | ftMscLib doc 6 |
| `BtSend` takes up to 255 bytes, but **the receiver keeps only the first 16 bytes** of a message (cut, not split) | measured, loopback and two TX |
| ftMscLib `SendBtMessage` / `BtReadMsgOn` work from the PC in **online mode**: PC -> local TX -> BT -> remote TX; telecodes 10..18 | reverse engineered, see `relay/README.md` |
| Round trip PC -> relay -> model program -> relay -> PC ~23 ms; bursts of 220-240 messages/s | measured with `BtEcho` |
| The relay drops its BT link if the PC polls it nonstop (~1000 cycles/s); fine with >= 5 ms between cycles | measured |
| The firmware needs one online request at a time; a packet right after a BT command gets garbled | measured |
| Loopback mode: connect to own BT address, messages come back on the same channel | ftMscLib doc 6.3 |
| Online-mode I/O payload per controller: **48 bytes outputs, 52 bytes inputs**, +27 bytes framing per packet | `online/cpp/src/Packets.cpp` |
| Full transfer area is 1024 bytes per controller (incl. config, display, hook table) | `offline/sdk/runtime/dispatcher.c` |
| Toolchain: generic multi-target arm gcc + own linker script/startup | `offline/sdk/runtime/robotx.ld`, `entry.S` |

**Measurements:** 1. message size: 16 bytes (done). 2. messages per second: 220-240 (done). 3. still open: VM ops
per tic (tight loop, report count at the 0.5 ms deadline) and `PrgTic` jitter.

## Topology

```
CANape --XCP/UDP--> bridge (PC)                     see xcp/README.md
                      |  USB, online mode (own lib)
                      v
                    relay TX  (no program; stock firmware relays BT messages)
                      |  also the handheld remote: potentiometer + buttons read via its transfer area,
                      |  state shown via display text messages; remote logic lives in the bridge
                      |  BT messages (TX <-> TX)
                      v
                    model TX(s)  (local program: native runtime + VM)
                      e.g. line follower car; later the stationary cell (master + extension over RS485)
```

The relay can't run its own program (online commands would interrupt it), which is fine: everything it needs
to do (relay messages, read pot/buttons, show text) works in online mode.

The online-mode telecodes for the BT message functions were found in ftMscLib's static debug library and are in
`FtConnection` (C++ and C#). Later, `hybrid/direct/` may try to skip the relay: the PC speaking the TX-to-TX protocol
itself (the firmware rejects non-TX devices; how it tells is unknown).

## Model TX: native runtime + VM

```
native runtime (C)                                 VM (C)
  - BT link: listen, receive callback, send queue    - register-based interpreter
  - protocol parser, version handshake               - cooperative tasks, instruction budget per tic
  - loader + verifier, arena allocator               - IO ops on the transfer area, display ops
  - mailboxes, watchdog, subscriptions, ring buffer
```

Rule: **all radio/protocol handling is native C**, never VM code. A buggy VM task must not be able to break the
link, so the PC can always stop or replace it. VM tasks only see mailboxes (`send(ctrl, msg)`, `recv()`).

## The VM

- **Register-based**, 3-address ops with 8-bit register indices (`ADD r3, r1, r2`), Lua 5 style. Registers are a
  per-frame array in RAM, unrelated to the ARM's physical registers.
- Types: **int32** default; **Q16.16 fixed point** as the "real" type (mul via 32x32->64 `SMULL` + shift);
  soft float only as an explicit opt-in type later (no FPU, ~50-150 cycles per op).
- Dispatch: gcc computed goto (`goto *table[op]`), no bounds check, no back-branch. Keep the interpreter loop and
  hot handlers small (I-cache is tiny). Add superinstructions for common pairs (e.g. `LOAD_IN + JZ`).
- Budget: ~100k cycles per 1 ms tic (0.5 ms) -> very roughly 3k-10k simple VM ops per tic. Plenty for control
  logic; measure.
- **Budget/yield:** a task gets N instructions per tic. Check the budget only at backward jumps and calls
  (straight-line code is bounded). Out of budget or explicit `YIELD`/`WAIT` -> save IP, return from `PrgTic`,
  resume next tic. Whole VM state lives in our structs, so suspending is just returning.
- **Input consistency:** the transfer area changes every 10 ms; snapshot inputs at the start of each task *run*,
  so a task that yields mid-way still sees consistent values. Outputs are written when the task yields.

## Tasks (ROBO Pro style parallelism)

- Each "green traffic light" start block becomes a task: own IP, own register frame. Round-robin per tic.
- `WAIT n` / `WAIT_UNTIL cond` yield. End block or `HALT` ends the task.
- Several programs at once (e.g. one handles I1/M1, another I2/M2, another handles messages).

## Memory

- One static arena in `.bss`; its size is the only build-time number.
- Program header declares code size, max registers per frame, max call depth. **No recursion**, so the verifier
  computes worst-case stack/frame usage from the acyclic call graph; programs that do not fit are rejected at
  upload with an error to the PC.
- Bump allocator at load time; nothing is allocated while running. Code uses base-relative offsets, so unloading
  compacts with `memmove` and updates base pointers (between tics).
- The verifier also checks jump targets, register indices and op validity, so handlers need no runtime checks.

## Protocol over BT messages (PC <-> relay <-> model)

Messages are at most 16 bytes, so every frame fits one message: `[op][seq][arguments]`. Larger things (programs)
need a fragmentation layer (sequence number, fragment index, CRC) under the same frame types. Implemented so far
(`relay/protocol/relay_protocol.h`):

- **Flow control:** one command in flight; the model acknowledges each with `ACK(seq, status)`, the PC resends
  after 200 ms. The firmware's send confirmation alone isn't enough: it sometimes gets lost. The model sends one
  frame at a time, the next after the confirmation (or 50 ms).
- **Handshake** on connect (`HELLO`): protocol version, controllers present. Later: VM version, arena size,
  budgets, tic rate.
- **Remote online mode as single commands** instead of the 48-byte output block (doesn't fit 16 bytes): `SET_PWM`,
  `SET_MOTOR`, `MOTOR_DISTANCE`, `MOTOR_SYNC`, `STOP_MOTOR`, `STOP_ALL`, `RESET_COUNTER`, `INPUT_MODE`, `MOTOR_MODE`,
  `COUNTER_MODE`, `TEXT`, each naming the controller (0 = model, 1..8 = its extensions). These opcodes become VM
  instructions; a direct command is a one-instruction program.
- **Values:** `SUBSCRIBE` (signal mask, interval) -> `VALUES` frames with changed signals, everything once a second
  (the keyframe idea below), plus `READ` for one value.
- **Program management:** upload (fragmented if needed) / start / stop / remove, list programs and state.
  Direct commands ("M1 = 512") are just one-shot programs: one code path.
- **Parameters:** write/read VM constants by index (what CANape calibrates).
- **Watchdog:** no message from the PC/remote for 1 s -> all outputs 0 (implemented; later: unless a program owns
  them). The PC pings when idle. The remote's stop button sends an immediate stop.
- **Telemetry, XCP-on-CAN style:** when a measurement starts, the PC sends a layout once (which signals, in which
  order, at which rate: like XCP DAQ lists/ODTs). Afterwards messages carry only a header byte, a tic delta and
  raw values; no IDs. Per-signal rates: fast signals every tic or every 10 ms, slow ones every N, digital ones on
  change with timestamp. The model buffers samples in a **ring buffer (~32 samples)** and sends batches, so
  short pulses are not lost and the PC gets real TX timestamps.
- **Keyframes:** a full state snapshot about once per second, changes in between; a lost message is healed by the
  next keyframe.
- **Overload is visible:** if the link can't keep up, the model drops samples and counts them; the drop counter is
  a measurement in CANape.
- **Multi-controller:** the same runtime on every model TX; models can also exchange messages directly
  (e.g. car <-> stationary cell "disc ready", "slot free").

## Display

- Drawing as VM ops: `TEXT`, `NUMBER`, `LINE`, `RECT`, `CIRCLE`, `CLEAR`, `FLUSH` (my graphics API already has
  rect/circle/text: `offline/sdk/gfx`). A script can show live sensor values with no BT traffic.
- Raw framebuffer (128x64 mono = 1 KB) only for images: dirty rectangles + RLE.
- Run drawing in a low-priority slot every few tics; the LCD does not need more than ~10-20 fps.

## Compiler (PC side)

- Simple language: variables, arithmetic, if/while, IO commands, wait, start blocks (tasks), send/recv, drawing.
- Register allocation: each variable gets a fixed register; expression temporaries take the next free registers
  going down the tree and are freed going back up (Lua-style). No graph colouring needed.
- Later: this is also a second backend for my own language project and its ImGui visualization tooling.

## Demo roadmap

Goal: a small "factory" running on the VM, monitored and calibrated live in CANape (see `xcp/README.md`).
I'm a software engineer, not a mechanical engineer: build from original instructions, keep custom mechanics tiny.
Each step is a complete demo on its own.

1. **XCP bridge in online mode**: stationary or slow models over USB/BT. Works (tested with pyxcp), also through
   the relay.
2. **Spurensucher (line follower)**, Training Lab instructions, wheels. First over online mode (slow), then:
3. **Relay + model TX:** BT message telecodes and "remote online mode" over BT messages are done; next the VM on
   the car. Classic CANape use case: speed and steering correction as calibration parameters,
   sensor values recorded with TX timestamps.
4. **Remote control:** the relay TX (working display) with a potentiometer for steering and a stop button;
   logic in the bridge. Colored rectangles beside the line become waypoints ("red = dock, blue = delivery").
5. **Stationary cell** (master TX + extension TX over RS485 = one VM with double IO): 3-axis robot (Automation
   Robots) with the vacuum gripper head from the ElectroPneumatic "ball course with vacuum gripper" model,
   colored discs, flat storage grid, color check over a sensor, chain conveyor to a fixed handover point.
6. **Connect them:** the car becomes an AGV with a tray and a docking funnel at the conveyor end. Request
   "a red disc" from CANape -> cell picks, checks color, loads AGV -> AGV delivers and returns.

Controllers: three TX (two with broken screens): one relay/remote (needs the display), one car, one for the
cell; the cell's extension needs a fourth TX, or the cell runs on one TX at first. All set parts are available.
Fallback demo if the rest stalls: the high-bay warehouse alone, tuned in CANape over online mode.

### Building instructions (ROBO TX sets)

| Set | Building instructions | Notes |
|---|---|---|
| ROBO TX Training Lab (505286) | [136892](https://docs.fischertechnikclub.nl/digital/136892.pdf), older [136892a](https://docs.fischertechnikclub.nl/digital/136892a.pdf) (not verified) | Spurensucher, basic model; [activity booklet DE](https://docs.fischertechnikclub.nl/digital/505286D.pdf) |
| ROBO TX Automation Robots (511933) | [143387](https://docs.fischertechnikclub.nl/digital/143387.pdf), older [143387a](https://docs.fischertechnikclub.nl/digital/143387a.pdf) | gripper robot, high-bay warehouse, swivel gripper, 3-axis robot; [activity booklet DE](https://www.fischertechnik.de/-/media/fischertechnik/rebrush/spielzeug/e-learning/alte-materialien/robo_tx_automation_robots_de.pdf) |
| ROBO TX ElectroPneumatic (516186) | [121936](https://docs.fischertechnikclub.nl/digital/121936.pdf) | air motor, color sorting robot, ball course with vacuum gripper, pinball |
| ROBO TX Explorer (508778) | [139281](https://docs.fischertechnikclub.nl/digital/139281.pdf) (scan, not verified) | |
| All documents | [fischertechnikclub.nl archive](https://docs.fischertechnikclub.nl/digital/digital.html) | also the TX Controller manual |

Spare parts: fischertechnik sells single parts on fischertechnik.de ("Einzelteile"); used bulk lots on eBay /
Kleinanzeigen; ftcommunity forum. Knobloch seems to be winding down.

## Open questions

- Answered: message size 16 bytes; 220-240 messages/s; telecodes 10..18. The relay does relay while the PC polls
  its inputs, but not at full rate: keep >= 5 ms between cycles (the libraries use 10 ms).
- VM ops per tic and `PrgTic` jitter.
- How the firmware tells a ROBO TX from other Bluetooth devices (for a direct mode without relay).
- Encoder motor as a jog wheel: counter inputs count without direction; potentiometer is the clean choice.
