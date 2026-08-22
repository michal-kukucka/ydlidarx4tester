# YDLIDAR X4 native driver + Python demo (Windows)

This project uses the official YDLIDAR C/C++ SDK as the hardware driver and a
small `ctypes` layer for Python. The Python code talks directly to the compiled
`ydlidar_sdk.dll`; there is no reimplementation of the serial protocol.

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
SHA-256, builds the official SDK as `build-x4\ydlidar_sdk.dll`, creates `.venv`,
and runs offline tests. The GUI uses Python's built-in Tk toolkit, so no PyPI
packages or global installs are needed. Extraction uses the public-domain
standalone `7zr.exe` from [7-Zip](https://www.7-zip.org/).

## Run it

Test the complete visualization without hardware first:

```powershell
.\scripts\run_demo.ps1 -Simulate
```

List COM ports visible to the native SDK:

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

Record raw points and enable SDK filters:

```powershell
.\scripts\run_demo.ps1 -Port COM4 -Record .\recordings\room.csv `
  --sun-filter --glass-filter --danger-distance 0.8
```

Run without a GUI for logging or integration:

```powershell
.\scripts\run_demo.ps1 -Port COM4 -Headless -Frames 100 `
  -Record .\recordings\scan.csv
```

Use `--help` for angle/range cropping, fixed resolution, orientation, ignored
angle windows, reconnect behavior, scan frequency (5-12 Hz), filtering, debug
logging, CSV recording, and DLL selection.

## X4 profile used by the driver

| Property | Value |
| --- | --- |
| Serial baud rate | 128000 |
| Protocol | Triangle |
| Sample rate | 5 kHz |
| Range | 0.12-10.0 m |
| Scan frequency | 5-12 Hz, default 8 Hz |
| Communication | Dual channel |
| Intensity | Not supported by X4 |
| Motor control | USB adapter DTR |
| Auto reconnect | Enabled |

Close the visualization window or press Ctrl+C in headless mode to stop scanning;
the wrapper always calls `turnOff`, disconnects the serial port, and releases
the SDK object.
