# YDLIDAR X4 Rozeta driver + Python demo (Windows and macOS/Linux)

The reusable hardware driver lives in
[Rozeta](https://github.com/michal-kukucka/rozeta): a native C++17
`rozeta::lidar::YdLidarScanner` with an opaque C ABI. This project is its Python
demo: `ctypes` loads the built Rozeta library (`librozeta.dll` on Windows,
`librozeta.dylib` on macOS, `librozeta.so` on Linux), asks for complete X4
revolutions, and draws the selected detection sector. The official SDK is no
longer in the live data path.

## Hardware wiring

The X4 adapter has two USB connections with different jobs:

1. Connect the **power USB** to a stable 5 V supply. It powers the LiDAR/motor.
   Without it the board still answers commands over the data link, but the
   motor never spins and every scan read ends in `YDLIDAR X4 scan timeout`.
2. Connect the **data USB** from the adapter to the computer. On Windows it
   must appear in Device Manager under **Ports (COM & LPT)** as a Silicon Labs
   CP210x port; on macOS it appears as `/dev/cu.usbserial-*` (or
   `/dev/cu.SLAB_USBtoUART`), on Linux as `/dev/ttyUSB*`.
3. Do not connect the LiDAR's 3.3 V UART pins directly to a 5 V UART adapter.

If no port appears, install the official
[Silicon Labs CP210x VCP driver](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads),
try a known data-capable USB cable, and reconnect the data USB. A charging-only
cable will power hardware but never create a serial port.

macOS notes:

* Recent macOS versions bring their own CP210x support, so `/dev/cu.usbserial-*`
  usually appears with no driver install. Confirm the adapter with
  `system_profiler SPUSBDataType | grep -A3 CP210`.
* Always use the `/dev/cu.*` callout device, never `/dev/tty.*`: the `tty`
  device blocks on carrier detect and will hang the open call.
* The X4 runs at 128000 baud, which macOS termios has no constant for. Rozeta's
  POSIX serial backend asks the driver for the exact rate with the `IOSSIOSPEED`
  ioctl after `tcsetattr`, so no extra configuration is needed.
* The adapter starts its motor on a DTR **low-to-high edge**, not on the level.
  A POSIX tty opens with DTR already asserted (Windows opens it deasserted), so
  Rozeta's X4 backend deasserts DTR when it opens the port and pulses it on
  `start()`. Without that pulse the board still answers `0xA5 0x90` device-info
  queries and acknowledges the scan command with `a5 5a 05 00 00 40 81`, but
  streams zero sample bytes and every read ends in `YDLIDAR X4 scan timeout`.

## One-time setup (Windows)

Open PowerShell in this directory and run:

```powershell
.\scripts\setup.ps1
```

The script downloads a pinned portable GCC/CMake/Ninja toolchain, verifies its
SHA-256, builds the sibling `michal-kukucka/rozeta` checkout with
`ROZETA_WITH_YDLIDAR=ON`, creates `.venv`, and runs Rozeta's C++ and Python
adapter tests. Clone Rozeta next to this tester or set `ROZETA_DIR` to its
source directory first. The resulting driver is `rozeta\build-x4\librozeta.dll`.
The GUI uses Python's built-in Tk toolkit, so no PyPI packages or global
installs are needed. Extraction uses the public-domain standalone `7zr.exe`
from [7-Zip](https://www.7-zip.org/).

## One-time setup (macOS and Linux)

Requirements: CMake, a C++17 compiler (`xcode-select --install` on macOS), and
a Python 3 with `tkinter` (python.org builds ship it; Homebrew needs
`brew install python-tk`). Ninja is used when present.

```bash
./scripts/setup.sh
```

The script builds the sibling `michal-kukucka/rozeta` checkout with
`ROZETA_WITH_YDLIDAR=ON` into `build-x4`, runs Rozeta's C++ test suite with
`ctest`, creates `.venv`, and runs the Python adapter tests. It looks for the
Rozeta source at `../rozeta-x4`, then `../rozeta`, and honours `ROZETA_DIR`;
the checkout must contain the X4 C ABI (`rozeta_ydlidar_x4_create` in
`src/c_api.cpp`), which lives on Rozeta's `main` branch. Use Rozeta `bfb7968`
("pulse DTR low-to-high so the X4 motor starts on POSIX") or newer: earlier
revisions never spin the motor on macOS or Linux. The result is
`build-x4/librozeta.dylib` (`.so` on Linux). No PyPI packages are used.

## Run it (macOS and Linux)

```bash
./scripts/run_demo.sh --simulate                      # no hardware needed
./scripts/run_demo.sh --list-ports                    # show serial ports
./scripts/run_demo.sh --port /dev/cu.usbserial-0001   # live X4
./scripts/run_demo.sh --port /dev/cu.usbserial-0001 --headless --frames 100 \
    --record ./recordings/scan.csv
./scripts/run_demo.sh --port /dev/cu.usbserial-0001 --forward-angle -125 \
    --field-of-view 70
```

`run_demo.sh` takes the same options as the PowerShell wrapper, spelled as long
flags, and passes anything it does not recognize straight to
`demo/x4_visualizer.py`. `ROZETA_LIBRARY` overrides the library it loads.

## Run it (Windows)

Test the complete visualization without hardware first:

```powershell
.\scripts\run_demo.ps1 -Simulate
```

List COM ports visible to Windows:

```powershell
.\.venv\Scripts\python.exe .\demo\x4_visualizer.py --list-ports
```

Start the real X4 (replace `COM4`):

```powershell
.\scripts\run_demo.ps1 -Port COM4
```

The GUI shows synchronized polar and Cartesian scans, live scan frequency and
point count, and the distance/bearing of the nearest detection. The dashed red
circle is the configurable obstacle-warning distance.

By default the demo shows a 70-degree forward region centered at -125 degrees,
matching this installation's physical forward direction. The green wedge is
the active region: detections outside it are excluded from the plots, point
count, nearest-obstacle result, and warning. Click anywhere on the polar plot
to aim the region, or adjust **Direction** and **FOV width** while scanning.

The same selection can be supplied on the command line:

```powershell
.\scripts\run_demo.ps1 -Port COM4 -ForwardAngle -125 -FieldOfView 70
```

Use `-ShowAll` to restore the full 360-degree view.

Record raw points and change the warning distance:

```powershell
.\scripts\run_demo.ps1 -Port COM4 -Record .\recordings\room.csv `
  --danger-distance 0.8
```

Run without a GUI for logging or integration:

```powershell
.\scripts\run_demo.ps1 -Port COM4 -Headless -Frames 100 `
  -Record .\recordings\scan.csv
```

Use `--help` for range cropping, orientation, the forward sector, warning
distance, CSV recording and the `--rozeta-lib` override (`--rozeta-dll` still
works). Some legacy SDK-only
noise/filter options remain accepted for command-line compatibility but are not
applied by Rozeta's X4 backend.

## X4 profile used by the driver

| Property | Value |
| --- | --- |
| Serial baud rate | 128000 |
| Protocol | X4 triangle packet stream, native Rozeta parser |
| Range | 0.12-10.0 m |
| Scan assembly | One complete revolution per `read_scan` call |
| Timing | 700 ms DTR motor spin-up, 1500 ms scan timeout |
| Intensity | Not supported by X4 |
| Motor control | USB adapter DTR, pulsed low-to-high on start |
| Angle correction | X4 triangular-head correction enabled |

Close the visualization window or press Ctrl+C in headless mode to stop scanning;
the wrapper always stops the motor, releases DTR, closes the serial port and
destroys the Rozeta scanner handle.
