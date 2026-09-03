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
revisions never spin the motor on macOS or Linux. Camera detection additionally
needs `rozeta_rgb_obstacle_tracker_create` in the same file, added in Rozeta
`061bf77` ("expose the RGB obstacle tracker over the C ABI"); without it the
twin still runs, with the LiDAR alone. The result is
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

## Camera twin and LiDAR calibration

`demo/twin_capture.py` runs the X4 next to a USB webcam and records both in
step, so the LiDAR's bearing scale can be tied to something a human can read.
Capture needs `ffmpeg` on `PATH` and no PyPI packages: frames arrive as raw
`rgb24` over a pipe (no encoder latency), Tk shows them, and each stored frame
is written as PNG by a small encoder in `demo/camera_stream.py`.

```bash
.venv/bin/python demo/twin_capture.py --list-cameras
.venv/bin/python demo/twin_capture.py --port /dev/cu.usbserial-0001 --rate 2 \
    --session recordings/train_01
.venv/bin/python demo/twin_capture.py --port /dev/cu.usbserial-0001 --headless \
    --rate 2 --duration 60 --session recordings/train_02
```

The window shows the camera on the left and the polar scan on the right; the
text box plus **Mark** writes an operator label against the current sample.

Recording is bounded, because a session left running unattended will otherwise
fill the disk: it stops at `--max-session-mb` (512 by default), stops again if
the volume drops below `--min-free-mb` free, and skips samples in which neither
the camera nor the LiDAR changed, storing an idle one every `--keepalive`
seconds so the quiet stretch is still on record. `--record-idle` stores
everything, and `--motion-luma` / `--motion-cells` / `--motion-bins` set how
much change counts as motion. Skipped samples still appear live and still count
towards `--samples`; only the frame and its scan go unwritten. A write that
fails stops the recording and says so instead of freezing the window. A
session directory holds `session.json`, `frames/*.png`, `samples.jsonl` (full
scan, 15-degree sector minima, camera/LiDAR timestamp difference) and
`labels.jsonl`. Timestamp difference stays within about +/-50 ms because the
LiDAR runs in its own reader thread and the newest camera frame is paired with
the revolution as it completes.

### Deriving the calibration

Walk in front of the pair, or move an obstacle across it, then fit:

```bash
.venv/bin/python demo/calibrate_twin.py recordings/train_01
```

The tool takes a per-pixel median as the static background, does the same per
5-degree LiDAR bin, and matches what moved in the image against what came
closer in the scan. A person is often not the nearest return (the LiDAR may
stand against a wall or a mount) and is invisible whenever they walk behind the
camera, so every sample offers several candidate clusters and a RANSAC-style
vote picks the mapping that explains the most samples. It reports

```
camera axis  = +179.2 deg LiDAR bearing
scale        = +0.0784 deg/px (horizontal FOV 50.2 deg, same handedness)
```

and writes `<session>/calibration.json` with the camera axis, degrees per
pixel, and the **blind sectors** - bearings where a return is always present at
close range, meaning the mount or cabling, not an obstacle.

### Using the calibration

`x4_visualizer.py` and `twin_capture.py` load the newest
`recordings/*/calibration.json` unless `--calibration PATH` or
`--no-calibration` says otherwise, and then:

* aim the detection sector along the camera axis, unless `--forward-angle` /
  `--field-of-view` were given explicitly;
* drop returns inside the blind sectors (drawn dark red in the twin window);
* report the nearest **cluster** rather than the nearest single return, so one
  stray sample no longer raises a warning. `--cluster-points` sets how many
  returns must agree (default 3).

```bash
.venv/bin/python demo/x4_visualizer.py --port /dev/cu.usbserial-0001
# calibration recordings/train_01: camera axis +179.2 deg, FOV 50 deg, blind ...
# nearest=2.820m at -163.8deg width=12deg n=23
```

Re-run a training session whenever the LiDAR or camera is remounted: the axis
and the blind sectors describe one physical installation.

### Camera detection fused with the LiDAR

Once the calibration exists, the twin window does more than record: the camera
looks for **new objects** and the LiDAR ranges them. The detector is Rozeta's
own `rozeta::perception::RgbObstacleTracker`, reached through the C ABI added
for it (`rozeta_rgb_obstacle_tracker_*`) and bound in `demo/rgb_obstacle.py`.
Nothing about the detection is reimplemented in Python, so a threshold tuned on
the robot behaves the same here.

It runs by default whenever a camera is open:

```bash
.venv/bin/python demo/twin_capture.py --port /dev/cu.usbserial-0001
```

Two seconds after start the first settled frame becomes the **reference
background**; every later frame is compared with it. When enough of the region
of interest differs for `--trigger-streak` frames in a row (5 by default) the
tracker triggers, and it clears again after `--clear-streak` quiet frames (3).
The **Reference** button re-takes the background; press it after the scene
settles, or whenever the camera is moved. `--no-reference` falls back to the
tracker's reference-free mode, which sees only dark blobs.

The tracker is fed every decoded frame rather than every recorded sample,
because its hysteresis counts frames: at the default 2 samples per second a
five-frame trigger would otherwise take two and a half seconds.

`demo/fusion.py` then turns the detection into a bearing with
`Calibration.pixel_to_angle` and looks for the LiDAR cluster that explains it,
within `--match-tolerance` degrees (12 by default, widened by half the
cluster's own width). Four outcomes are reported, live and in `samples.jsonl`:

| Agreement | Meaning |
| --- | --- |
| `both` | camera and LiDAR agree; this is the only state with a distance. `bearing_source` says whether the direction came from the camera box or the LiDAR cluster |
| `camera_only` | something new is visible with no return behind it — glass, a shadow, or an object above or below the scan plane |
| `lidar_only` | a return in the sector the camera watches, but nothing new in the image |
| `clear` | neither sensor has anything |

In the window the camera pane draws the detection box and the polar plot draws
a dashed ray along the camera bearing, an arc for its angular width, and a ring
around the matched cluster with its distance.

Two limits are worth knowing before trusting a bearing:

* **In a lit room the bearing is the LiDAR's, not the camera's.** Only the
  tracker's dark-blob pass localizes anything; the difference pass reports one
  coverage number for the whole region. A lit person is not a dark blob, so the
  detection usually arrives without a box and the demo falls back to the
  nearest cluster inside the camera's own field. Measured over a 69-sample
  session with the room lit: 36 triggered samples, 2 of them with a camera box.
  Each record says which sensor supplied the bearing in `bearing_source`, and
  `match_error_deg` is filled only when the two were genuinely compared. Read
  the normal `both` verdict as *the camera says something is new, the LiDAR
  says where and how far*. `--dark-max-value` sets what counts as dark and is
  what shapes the box.
* **Dark-pixel triggering is off while a reference is in use.** A dim room is
  more than half dark, and the tracker's dark and difference tests are combined
  with *or*, so the dark half alone would latch it on permanently.
  `--dark-coverage` restores it; `--no-reference` uses the library default.

Useful knobs:

```bash
# more sensitive to change, and quicker to react
.venv/bin/python demo/twin_capture.py --port /dev/cu.usbserial-0001 \
    --diff-threshold 20 --diff-coverage 0.05 --trigger-streak 3

# LiDAR only, no camera detection at all
.venv/bin/python demo/twin_capture.py --port /dev/cu.usbserial-0001 \
    --no-camera-detection
```

Detection needs the Rozeta build to carry the RGB obstacle C ABI. An older
library still runs the twin: the demo prints `camera detection disabled` and
falls back to the LiDAR alone. Rebuild the sibling checkout to get it back.

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
