# XCP bridge: ROBO TX in CANape via XCPlite

Exposes fischertechnik ROBO TX Controllers as an XCP-on-Ethernet "ECU", so CANape (or any XCP client) can measure
their inputs and set outputs and IO configuration as calibration parameters. Just for fun, and a nice demo.

Uses Vector's open-source [XCPlite](https://github.com/vectorgrp/XCPlite) (MIT, Linux/macOS/Windows, XCP on Ethernet
UDP/TCP, A2L generated at runtime). C++ only.

## Two ways to reach the controllers

| Mode | Command | Controllers in CANape (prefix) |
| ---- | ------- | ------------------------------ |
| Online | `ftrobotx_xcp_bridge -p /dev/ttyACM0` | `tx`: the controller on that port (USB or Bluetooth) |
| Relay (hybrid) | `ftrobotx_xcp_bridge -p /dev/ttyACM0 -a 00:13:7B:48:B2:E2` | `relay`: the relay TX on the port (its own inputs and outputs, e.g. a remote control: pedals, steering wheel); `model`: the model TX at the address, running `RelayModel` (see [`hybrid/relay`](../hybrid/relay/README.md)) |

`-p` is always the port of the controller connected to the PC; `-a` is the Bluetooth address of the **model** TX
(not the relay's). The model TX needs `RelayModel` built for the relay's address; see the relay README's setup.

In the code, [`src/TxIo.hpp`](src/TxIo.hpp) hides the difference: a `TxLink` (`OnlineLink`, `RelayLink`, later a
direct link without relay) runs the cycle and provides named controllers, each a `TxIo` with the same inputs and
outputs. The bridge only sees `TxIo`.

## Build and run

```bash
cd xcp && cmake -S . -B build -G Ninja && ninja -C build
```

The first `cmake` fetches XCPlite (tag `V2.3.2`) from GitHub. Use a local checkout instead with
`-DFT_XCPLITE_DIR=<path>`, another version with `-DFT_XCPLITE_TAG=<tag>`.

```bash
./build/ftrobotx_xcp_bridge -p /dev/ttyACM0                          # online
```

```bash
./build/ftrobotx_xcp_bridge -p /dev/ttyACM0 -a 00:13:7B:48:B2:E2     # relay (-p) and model (-a)
```

Then in CANape: XCP on Ethernet, UDP, port 5555, host = the PC running the bridge (from another machine: allow UDP
5555 in its firewall). CANape can upload the A2L via XCP; the bridge also writes `ftrobotx.a2l` into its working
directory when a client connects.

## What CANape sees

Per controller `<name>` (`tx`, or `relay` and `model`):

| Name | Kind | Meaning |
| ---- | ---- | ------- |
| `<name>.input[8]` | measurement | I1..I8, raw value in the unit of the input's mode (digital 0/1, mV, Ω, cm) |
| `<name>.counter[4]` | measurement | C1..C4 counter values |
| `<name>.button_left/right` | measurement | display buttons (time pressed) |
| `<name>_outputs.m1..m4`, `.o1..o8` | parameter | duties: motor −512..512, single output 0..512 |
| `<name>_config.i1..i8.mode/.digital` | parameter | input mode (dropdown: voltage, resistance, ultrasonic) and digital/analog |
| `<name>_config.p1..p4.motor_mode/.counter_edge` | parameter | motor or two single outputs, counter edge |

And for the link:

| Name | Meaning |
| ---- | ------- |
| `link_cycle_count`, `link_cycles_per_s` | cycles since start, measured rate (~100/s) |
| `link_config_generation` | +1 per configuration change sent; marks mode switches in recordings |
| `link_round_trip_ms` | relay: last command, PC → relay → model → back (~40 ms) |
| `link_resends`, `link_lost_frames` | relay: commands sent again, value frames lost |

All measurements come with the event `tx_cycle`, once per cycle (every 10 ms); DAQ works. Timestamps are the PC's
clock: the relay delivers model values about 20–40 ms late.

Changing a pair between motor and single outputs stops it first. When the model connection drops, the bridge keeps
running (the relay's values keep coming), reconnects every 3 s and then sends CANape's configuration and outputs
again.

## How it works

Loop, once per cycle (10 ms):

1. `applyConfig()` per controller: read calibration segment `<name>_config`; on change, set input modes,
   motor/output mode, counter edge.
2. `applyOutputs()` per controller: read `<name>_outputs`; send only changed values, depending on the pair's mode.
3. `link->update()`: one cycle (online exchange; with the relay also the relay commands, see `RelayConnection`).
4. `mirrorInputs()`: copy the inputs into the measurement structs (fixed addresses, absolute addressing).
5. `XcpEvent(tx_cycle)`: DAQ copies the measurements.

A2L: generated at runtime (`A2L_MODE_WRITE_ALWAYS | A2L_MODE_FINALIZE_ON_CONNECT | A2L_MODE_AUTO_GROUPS`), with enum
conversions for the dropdowns. All measurements are registered in one block: every addressing-mode block makes its
own group, and the group names would repeat.

## Tested (Linux, 2026-11-10)

With [pyxcp](https://github.com/christoph2/pyxcp) 0.30 as client instead of CANape: connect, A2L generation,
reading measurements, DAQ (298 samples in 3 s, none missing), calibration writes in both modes (relay: lamp on the
model's O1, ~40 ms per command; online and relay: input mode switched to ultrasonic / NTC from the client), reconnect
after the model restarted. Not yet tested with CANape itself, nor on Windows.

## Windows (CANape on the work laptop)

The libraries have `SerialPortWin32.cpp`, so the online and relay side run on Windows; build with MSVC (C++20). A
paired TX shows up as an outgoing Bluetooth COM port. Without building on Windows: run the bridge on the Linux PC
and connect CANape over the network (UDP 5555). WSL2 is not worth it (USB needs usbipd-win, no Bluetooth, UDP in NAT
mode is flaky).

## Next steps

1. Test with CANape (Windows build, or over the network).
2. Physical values: NTC in °C and ultrasonic in cm as computed measurements (the NTC formula has a logarithm, which
   A2L conversions can't express).
3. Extensions of a controller: one set of segments and measurements per extension.
4. With the VM on the model: model samples with the model's own time (`XcpEventAt`), VM constants as calibration
   parameters (line follower speed and steering correction, tuned while it drives), CANape's DAQ lists deciding
   what the model sends.
5. Remote control logic: the relay's inputs (pedals, steering wheel) to commands for the model.

## Things to keep in mind

- Keep `CalSeg::lock()` scopes short: XCP writes wait for readers.
- Measurement structs are written by the loop thread and read by XCP DAQ in the same `XcpEvent` call, so no extra
  locking is needed while only the loop thread writes them.
- Connecting the bridge sends the default configuration and zero outputs: it resets what the controllers were doing.
