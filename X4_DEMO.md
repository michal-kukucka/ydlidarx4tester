# YDLIDAR X4 Rozeta driver + Python demo (Windows)

The reusable hardware driver lives in
[Rozeta](https://github.com/michal-kukucka/rozeta): a native C++17
`rozeta::lidar::YdLidarScanner` with an opaque C ABI. This project is its Python
demo: `ctypes` loads `librozeta.dll`, asks for complete X4 revolutions, and
draws the selected detection sector. The official SDK is no longer in the live
data path.

## Hardware wiring

The X4 adapter has two USB connections with different jobs:

1. Connect the **power USB** to a stable 5 V supply. It powers the LiDAR/motor.
2. Connect the **data USB** from the adapter to the Windows PC. It must appear
   in Device Manager under **Ports (COM & LPT)** as a Silicon Labs CP210x port.
3. Do not connect the LiDAR's 3.3 V UART pins directly to a 5 V UART adapter.

If no COM port appears, install the official
[Silicon Labs CP210x VCP driver](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads),
try a known data-capable USB cable, and reconnect the data USB. A charging-only
cable will power hardware but never create a COM port.

## One-time setup

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

## Run it

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
distance, CSV recording and the `--rozeta-dll` override. Some legacy SDK-only
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
| Motor control | USB adapter DTR |
| Angle correction | X4 triangular-head correction enabled |

Close the visualization window or press Ctrl+C in headless mode to stop scanning;
the wrapper always stops the motor, releases DTR, closes the serial port and
destroys the Rozeta scanner handle.
