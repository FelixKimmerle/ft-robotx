# ft-robotx-online

<p align="center">
  <img src="images/robo-tx-controller.jpg" alt="Rendered image of a ROBO TX Controller" width="280">
</p>

Control the **fischertechnik ROBO TX Controller** from a PC in **online mode**: your program runs on the PC, and
the TX acts as its I/O module. It reads the inputs and counters and drives the motors and outputs of the master
and up to eight extension controllers, over **USB or Bluetooth**.

This is for the ROBO TX Controller only. The ROBO TXT and other fischertechnik controllers use different
protocols.

The repository contains two implementations of the same protocol:

| Folder | What | Language |
| ------ | ---- | -------- |
| [`cs/`](cs) | Library and example programs | C# / .NET 10 |
| [`cpp/`](cpp) | Library, example programs, protocol tests and a graphical test tool | C++20, CMake |

## Background and thanks

This project would not exist without **Christoph Niessen**. His analysis of the TX's RS-485 extension bus
documents the packet format that the TX also speaks over USB and Bluetooth: the frame, the fields, the
checksum, the telecodes and the info structure. Thank you!

- Christoph Niessen: [Die RS-485-Schnittstelle des ROBO TX Controllers](https://web.archive.org/web/20261019230724/http://chris.cnie.de/ft/txc-rs485.html)
  (archived copy, German)

The meaning of the individual fields (input modes, motor/output configuration, counter reset, distance and
synchronization commands, version info) comes from fischertechnik's *PC Programming ROBO TX Controller*
package (`ROBO_TX_FW.h` and its documentation), which is not part of this repository.

## Contents

- [Access to the controller (Linux)](#access-to-the-controller-linux)
- [Connecting via USB](#connecting-via-usb)
- [Connecting via Bluetooth](#connecting-via-bluetooth)
- [Building](#building)
- [Programs and examples](#programs-and-examples)
- [The test tool](#the-test-tool-ftrobotx_gui)
- [Using the library](#using-the-library)
- [Troubleshooting](#troubleshooting)

## Access to the controller (Linux)

Serial devices (`/dev/ttyACM*`, `/dev/rfcomm*`) belong to the group **`uucp`** on Arch Linux, or **`dialout`**
on Debian/Ubuntu. Add yourself to it once:

```bash
sudo usermod -aG uucp $USER       # Debian/Ubuntu: dialout instead of uucp
```

Group changes only apply to new logins, so **log out and back in** afterwards. Until then, `newgrp uucp`
gives just the current terminal the new group.

Direct Bluetooth connections (`bt:…`, see below) don't need the group.

## Connecting via USB

Plug in the controller; it appears as `/dev/ttyACM0` on Linux, or as a `COMx` port on Windows. The C++ programs
and the test tool label it as *ROBO TX Controller (USB)* (USB ID `221d:1000`).

To use **extensions**, connect them to the master with the EXT cable and set each one's role in its menu to
*Extension 1*, *Extension 2*, …. The programs address them as extension 1, 2, … in exactly that order.

## Connecting via Bluetooth

First pair the controller once (its Bluetooth must be switched on):

```bash
bluetoothctl
```

```text
scan on              # wait until "ROBO TX-…" appears, note its address
pair 00:13:7B:53:DB:5E
trust 00:13:7B:53:DB:5E
quit
```

`bluetoothctl connect` is not needed. It fails with `ProfileUnavailable` because the TX uses the serial port
profile (SPP), which BlueZ doesn't connect itself. That's expected.

There are two ways to connect after pairing.

### Option 1: direct connection (C++ and the test tool, Linux)

Pass `bt:` plus the controller's address as the port:

```bash
./build/ftrobotx_gui bt:00:13:7B:53:DB:5E
```

This needs no root, no `rfcomm` binding and no group membership. The test tool lists paired ROBO TX controllers
in its port menu automatically.

### Option 2: rfcomm device (C#, other tools)

This binds the controller to a serial device `/dev/rfcomm0`, which every serial program can open. On Arch, the
`rfcomm` tool is in `bluez-deprecated-tools`:

```bash
sudo pacman -S bluez-deprecated-tools
```

The TX offers its serial port on RFCOMM channel 1 (`sdptool browse 00:13:7B:53:DB:5E` shows it):

```bash
sudo rfcomm bind 0 00:13:7B:53:DB:5E 1
```

`/dev/rfcomm0` then belongs to the `uucp` group like the USB port (see [Access](#access-to-the-controller-linux)).
The binding lasts until the next reboot; `sudo rfcomm release 0` removes it.

### Notes

- The Bluetooth link is only established with the first packet, which takes about **3.5 seconds**. The
  libraries allow up to 10 seconds for the first reply and 1 second for all later ones.
- Bluetooth manages about 25–30 request/reply cycles per second, USB about 100.
- Extensions behind the master on the EXT cable are reached through the master as usual.
- On **Windows**, pairing creates a `COMx` port; use that one.

## Building

### C++

You need CMake 3.28 or newer and a C++20 compiler (GCC, Clang or MSVC); Ninja is optional. The test tool also
needs what SDL3 needs to open a window (on Linux, X11 and/or Wayland development files, which desktop systems
usually have). CMake downloads SDL3 and Dear ImGui itself, pinned by version and checksum, so the first
configure takes a moment.

```bash
cd cpp
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

| CMake option | Default | Meaning |
| ------------ | ------- | ------- |
| `FT_BUILD_GUI` | on | Build the test tool (downloads SDL3 and Dear ImGui) |
| `FT_BUILD_EXAMPLES` | on | Build the example programs |
| `FT_BUILD_TESTS` | on | Build the protocol tests |
| `FT_WARNINGS_AS_ERRORS` | off | Treat compiler warnings as errors |

When the project is used as a subproject, the GUI, examples and tests are off by default.

### C#

You need the .NET 10 SDK.

```bash
cd cs
dotnet build FtRoboTxOnline.slnx
```

## Programs and examples

All programs take the port as their first argument: `/dev/ttyACM0`, `/dev/rfcomm0`, `COM3`, or (C++ on Linux)
`bt:00:13:7B:53:DB:5E`. Ctrl+C stops them, and they switch all outputs off before exiting.

| Program | C++ (`cpp/build/…`) | C# (`cs/…`) | What it does |
| ------- | ------------------- | ----------- | ------------ |
| Test tool | [`ftrobotx_gui`](cpp/gui) | – | Shows and controls everything; see below |
| Button and lamp | [`ftrobotx_button_lamp`](cpp/examples/button_lamp.cpp) | [`examples/ButtonLamp`](cs/examples/ButtonLamp/Program.cs) | Lamp on O1 lights while the button on I1 is pressed |
| Extension test | [`ftrobotx_extension_test`](cpp/examples/extension_test.cpp) | [`examples/ExtensionTest`](cs/examples/ExtensionTest/Program.cs) | Master and extension 1 with button on I1 and lamp on O1 each, crosswise: each button switches the *other* controller's lamp |
| Demo | [`ftrobotx_demo`](cpp/examples/main.cpp) | [`src/Program.cs`](cs/src/Program.cs) | The original demo: motor control with master and one extension |

Running them:

```bash
cd cpp && ./build/ftrobotx_button_lamp /dev/ttyACM0
```

```bash
cd cs/examples/ButtonLamp && dotnet run -- /dev/ttyACM0
```

## The test tool (`ftrobotx_gui`)

A test tool in the spirit of fischertechnik's *RoboTxTest*, for checking wiring, sensors and motors quickly.

<p align="center">
  <img src="images/ftrobotx_gui_screenshot.png" alt="Screenshot of ftrobotx_gui connected to a ROBO TX Controller: universal inputs and counters on the left, motors and outputs on the right" width="800">
</p>

```bash
./build/ftrobotx_gui                            # choose port and number of extensions, then Connect
./build/ftrobotx_gui /dev/ttyACM0 1             # connect right away: master and 1 extension
./build/ftrobotx_gui bt:00:13:7B:53:DB:5E 1     # the same via Bluetooth
```

- The **port menu** lists ROBO TX controllers on USB, paired ROBO TX controllers for Bluetooth, and bound rfcomm
  devices. You can also type any port.
- Each controller has its own window. They start as tabs; drag a tab to dock it beside or below another, or use
  **Layout** → *Side by side* / *Stacked* / *Tabs*. The arrangement is remembered (on Linux in
  `~/.local/share/ft-robotx-online/gui/imgui.ini`).
- Each window shows:
  - **Universal inputs I1–I8**: mode (voltage in mV, resistance in Ω, ultrasonic distance in cm), analog or
    digital, and the current value.
  - **Counters C1–C4**: level and value, the counting edge, and a reset button.
  - **Display buttons**: how long each has been held, in ms.
  - **M1–M4**: each pair either as a motor (speed −512…512; distance, synchronization and *Start* for distance
    commands) or as two separate outputs O1–O8 (0…512).
- Configuration changes are sent to the controller immediately.
- **STOP OUTPUTS** (or Esc) stops every motor and output, including running distance commands. Disconnecting or
  closing the tool does the same. The connection stays open.

## Using the library

C++, with CMake:

```cmake
add_subdirectory(ft-robotx-online/cpp)
target_link_libraries(my_app PRIVATE ftrobotx::ftrobotx)
```

```cpp
#include <ftrobotx/FtConnection.hpp>

using namespace ftrobotx;

FtConnection tx("/dev/ttyACM0", 2); // master + extension 1; or "bt:00:13:7B:53:DB:5E"
while (running)
{
    tx.update(); // sends outputs (and changed configuration), reads inputs
    bool pressed = tx.digitalValue(ExtensionIndex::Local, UniversalInput::I1);
    tx.setMotorOutput(ExtensionIndex::Slave1, Motor::M1, pressed ? 512 : 0);
}
tx.stopAllOutputs();
tx.update();
```

C#:

```csharp
using FtRoboTxOnline;
using FtRoboTxOnline.Extensions;

using FtConnection tx = new("/dev/ttyACM0", numExtensions: 2);
tx.Update();
bool pressed = tx.GetDigitalValue(ExtensionIndex.Local, UniversalInput.I1);
tx.SetMotorOutput(ExtensionIndex.Slave1, Motor.M1, pressed ? (short)512 : (short)0);
```

Every change takes effect with the next `update()` / `Update()`. Call it regularly: the TX refreshes its I/O
every 10 ms.

## Troubleshooting

| Problem | Cause and fix |
| ------- | ------------- |
| `Permission denied` when opening the port | You're not in the `uucp`/`dialout` group yet, or haven't logged in again since joining it. See [Access](#access-to-the-controller-linux). |
| `No response from controller (got 0 of … bytes)` | Wrong port, the controller is off, or (Bluetooth) the link didn't come up. Check the port; for Bluetooth, check pairing and that the controller's Bluetooth is on. |
| `Config failed!`, or an extension shows no name or MAC | The extension isn't reachable: check the EXT cable and that its role is set to the right *Extension n*. The test tool shows a warning in that extension's window. |
| `bluetoothctl connect` fails with `ProfileUnavailable` | Expected; see [Bluetooth](#connecting-via-bluetooth). |
| Anything else | Set `FT_TRACE=1` to log every packet sent and received to stderr (C++ and C#). A timeout error also shows how much of the reply arrived. |

## Status

- Tested on Linux with a master and one extension, via USB and Bluetooth.
- The Windows serial code (C++) is written but not yet tested.
- Not implemented: messages on the controller's display, the remote shell, and firmware updates.
