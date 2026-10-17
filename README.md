# Wheelchair Odometry – ESP32 firmware + Path Viewer

Odometry for a wheelchair using **two wheel encoders** (distance) and a **GY-521 / MPU-6050 gyro**
(heading). The ESP32 fuses both into a position and heading, streams them over USB, and records
them to its internal flash. A small Python/Tk program draws the path live, replays recordings from
the board, and exports the path as an SVG for printing on A3.

Why a gyro? The encoder rollers press against the tyres and can slip. With encoders only, the
heading is computed from the *difference* of the two wheels, so a little slip on one side turns
into a large heading error. The gyro measures the rotation of the chair body itself and does not
care about slip. Distance still comes from the encoders.

```
 left  encoder ─┐
                ├─►┐
 right encoder ─┘  │   ESP32 (fusion, 20 Hz) ──USB serial──►  path_viewer.py
                   ├──►     │                                  (live path, replay,
 GY-521 gyro ─────►┘        ▼                                   delete, SVG export)
 (I2C, 200 Hz)     recording to flash files /rec_001.csv, /rec_002.csv, …
 button (GPIO13) ──► start / pause / replay
 LED  (GPIO2)   ◄── blinks while recording (fast while the gyro calibrates)
```

## Contents

1. [Repository layout](#1-repository-layout)
2. [Hardware](#2-hardware)
3. [Installation](#3-installation)
4. [Firmware usage](#4-firmware-usage)
5. [Data format](#5-data-format)
6. [Viewer usage](#6-viewer-usage)
7. [Configuration reference](#7-configuration-reference)
8. [Commissioning the encoders](#8-commissioning-the-encoders)
9. [Calibration](#9-calibration)
10. [How the sensor fusion works](#10-how-the-sensor-fusion-works)
11. [Troubleshooting](#11-troubleshooting)
12. [Known limitations](#12-known-limitations)
13. [Quick reference](#13-quick-reference)

---

## 1. Repository layout

```
├── platformio.ini        PlatformIO project (ESP32 DevKit, Arduino framework, LittleFS)
├── src/
│   ├── main.cpp          Setup, 20 Hz odometry task, serial commands
│   ├── config.h          ALL settings (pins, mechanics, filter constants)
│   ├── sample.h          One odometry sample + its CSV line
│   ├── encoders.h/.cpp   Quadrature decoding of the two wheel encoders
│   ├── imu.h/.cpp        MPU-6050 driver, offset calibration, yaw integration (own 200 Hz task)
│   ├── odometry.h/.cpp   Fusion: position + heading, standstill and slip detection (no hardware access)
│   ├── recorder.h/.cpp   Live output, recording to flash, replay, delete (own logger task)
│   └── ui.h/.cpp         Push button and LED
├── pythonViewer/
│   └── path_viewer.py    PC tool: live path, replay, delete files, gyro commands, SVG export
└── README.md
```

No additional library is needed: the I2C driver (`Wire`) is part of the Arduino core, and
PlatformIO compiles every file in `src/` automatically.

---

## 2. Hardware

### 2.1 Board

[ESP-32 Dev Board 30 PIN](https://www.robotics.org.za/ESP32-DEV) – ESP-WROOM-32, 4 MB flash,
CH340 USB-serial chip, USB-C connector, no PSRAM. Any standard ESP32 DevKit with the same pins
works. The internal LED is assumed to be on GPIO2 (the usual location).

### 2.2 Pin assignment

| Function                            | GPIO    | Notes                                                      |
|-------------------------------------|---------|------------------------------------------------------------|
| Left encoder channel A / B          | 25 / 26 | Use external pull-ups to 3V3 (see 2.3)                     |
| Right encoder channel A / B         | 14 / 27 | Use external pull-ups to 3V3 (see 2.3)                     |
| GY-521 SDA                          | **21**  | I2C data                                                   |
| GY-521 SCL                          | **22**  | I2C clock                                                  |
| Record / replay button              | **13**  | Button between GPIO13 and GND (internal pull-up is enabled)|
| Internal LED                        | **2**   | Blinks while recording                                     |

### 2.3 Wiring the encoders

Quadrature encoders (A/B channels, **3.3 V logic**, common ground) are connected to the pins in the
table above. The inputs are configured as plain inputs, so add **external pull-ups (e.g. 4.7 kΩ to 3V3)**
if your encoders have open-collector outputs. The encoders press against the wheelchair tyres, so the
distance per count is set by the diameter of the **measuring roller** on the encoder shaft
(see [section 8](#8-commissioning-the-encoders)).

### 2.4 Wiring the GY-521 (MPU-6050)

```
 GY-521            ESP32
 VCC  ───────────  3V3
 GND  ───────────  GND
 SDA  ───────────  GPIO21
 SCL  ───────────  GPIO22
 AD0  ───────────  GND      (I2C address 0x68; to 3V3 = 0x69, then set IMU_ADDR in config.h)
 INT, XDA, XCL     not connected
```

The GY-521 board already has I2C pull-up resistors. Keep the wires short (< 20 cm) and away from
motor cables.

**Mounting**

- Fix the board **rigidly** to the chair frame (not on a flexible plate or a loose cable), away from
  strong vibration sources.
- The **orientation does not matter**: the firmware finds "up" from the accelerometer and measures the
  rotation about that axis. You can mount the board flat, on its side, upside down, … Just do not move
  it relative to the chair after the calibration.
- The position on the chair does not matter either (yaw rate is the same everywhere on a rigid body).

---

## 3. Installation

### 3.1 Firmware (PlatformIO)

**Option A – VS Code**

1. Install [Visual Studio Code](https://code.visualstudio.com/) and its **PlatformIO IDE** extension.
2. `File → Open Folder…` and select the project folder (the one with `platformio.ini`).
3. Connect the board with a USB data cable.
4. Click the **Upload** (→) button in the blue PlatformIO status bar.

**Option B – command line**

```bash
pip install platformio
cd <project folder>
pio run -t upload          # build and flash
pio device monitor         # optional: serial terminal at 115200 baud
```

Notes:

- The first build downloads the ESP32 toolchain and Arduino core and needs internet access.
- If the upload cannot connect, hold the **BOOT** button on the board while the upload starts.
- On Windows you may need the **CH340 driver**; the board then appears as a COM port.
- The flash file system (LittleFS) is formatted automatically on the first boot. You do **not**
  need to upload a file-system image. A normal firmware upload does not touch the recordings.

### 3.2 Viewer (Python)

Requirements: **Python 3.8 or newer**, `pyserial`, and `tkinter`.

```bash
pip install pyserial
python pythonViewer/path_viewer.py
```

`tkinter` is part of the standard Python installer on Windows and macOS. On Linux install it
with e.g. `sudo apt install python3-tk`.

Close any other program that has the COM port open (PlatformIO monitor, Arduino serial
monitor, …). Only one program can use the port at a time.

---

## 4. Firmware usage

### 4.1 What the firmware does

Every 50 ms (20 Hz) it reads the two wheels and the gyro and updates the platform pose:

```
ds  = (dL + dR) / 2                         distance travelled by the platform centre   (encoders)
dθ  = gyro yaw change since the last sample                                              (gyro)
      or, without a working gyro: (dL − dR) / track width                                (encoders)
x  += ds · sin(θ + dθ/2)
y  += ds · cos(θ + dθ/2)
θ  += dθ
```

- The start position is (0, 0) and the start heading points along **+Y ("up")**.
- Heading `θ` is measured **clockwise from +Y** and reported in degrees (−180 … +180).
- All distances are in **millimetres**.
- If the gyro is missing or stops answering, the firmware falls back to encoder heading
  automatically and prints a message. When the gyro is back, it switches again.

The gyro itself runs in a separate task at 200 Hz (see [section 10](#10-how-the-sensor-fusion-works)).
Each sample is printed on the serial port and, while recording, appended to a file in flash.

### 4.2 Boot behaviour

At power-up the firmware **measures the gyro offset for 2 seconds** (see
[9.1](#91-gyro-offset-calibration)). **Keep the chair still during that time**, with the person sitting in
it if the chair is normally used that way. The LED blinks fast while this runs. Afterwards the live
serial output is **on** and recording is **off**, so the viewer shows the path immediately. Nothing is
written to flash until you start a recording.

If the chair moves during the calibration, the firmware prints a message and simply measures again.

### 4.3 Recording

| Action                                    | Result                                                          |
|-------------------------------------------|-----------------------------------------------------------------|
| **Short press** of the button, or `p`     | Start recording. Press again to pause.                          |
| **Long press** (≥ 1 s) of the button, `d` | Replay the next recording (see below)                           |
| `w`                                       | Delete all recordings                                           |

- Every time a recording **starts**, a **new file** is created: `/rec_001.csv`, `/rec_002.csv`, …
  The number is the highest existing number plus one.
- The **internal LED blinks** (every 250 ms) while recording, and is off otherwise.
  It stays on while a file is being replayed.
- `p` and the short press control the recording **and** the live serial output together:
  starting turns both on, pausing turns both off. Press once more to start a new recording.
- Pausing waits until all buffered samples are written, then prints the file name and size.
- A recording stops by itself when the file reaches about 1 MB or the flash is full, and a
  message is printed. A new recording is not started when less than 64 KB are free.
- Data is written in blocks of about 4 KB or every 2 s. After a power loss at most the last
  2 s of the recording are lost.

### 4.4 Replay and delete

- **Replay** (`d` or long press) sends the next recording over the serial port, one file per
  request. After the last file it wraps around to the first. Progress is printed as
  `# replay 2/3`, followed by the file between `# ---- BEGIN LOG rec_002.csv ----` and
  `# ---- END LOG ----`. The viewer draws it automatically.
- At 115200 baud a 1 MB file takes about **90 seconds** to transfer.
- **Delete** (`w`) removes all recordings.
- Replay and delete are **refused while recording** (the firmware prints a message). Pause the
  recording first.

### 4.5 Serial commands

Open the port at **115200 baud, 8N1** and send single characters (no newline needed).

| Key | Function                                                                                         |
|-----|--------------------------------------------------------------------------------------------------|
| `p` | Start / pause recording and live output (same as short button press)                             |
| `d` | Replay next recording (same as long button press)                                                |
| `w` | Delete all recordings                                                                            |
| `r` | Reset position and heading to 0 (heading 0 = "up" again). Also zeroes `enc_hdg` / `gyro_hdg`.    |
| `g` | **Calibrate the gyro offset** now (chair must stand still for 2 s)                               |
| `h` | Toggle the heading source: gyro (default) ↔ encoders only. Useful to compare both in one run.    |

### 4.6 Messages from the firmware

Lines starting with `#` are comments. Besides the confirmations of the commands, a status
line is printed twice a second:

```
# status L=12034 R=11890 v=410 mm/s hdg=12.4 deg | gyro 15.2 dps, enc_hdg=13.1 gyro_hdg=12.4 | imu=ok bias=0.412 dps 31.2 C slip=3 src=gyro still=0 rec=off
```

| Field        | Meaning                                                                                       |
|--------------|-----------------------------------------------------------------------------------------------|
| `L`, `R`     | Raw accumulated encoder counts                                                                |
| `v`          | Platform speed                                                                                |
| `hdg`        | Heading actually used (the one in the data stream)                                            |
| `gyro … dps` | Current turn rate from the gyro, clockwise positive                                           |
| `enc_hdg`    | Heading computed from the encoders **only**, continuous (not wrapped) since the last `r`      |
| `gyro_hdg`   | Heading computed from the gyro **only**, continuous since the last `r`                        |
| `imu`        | `ok`, `calibrating`, `lost` (no answers) or `off` (not found at boot)                          |
| `bias`       | The zero-rate offset that is currently subtracted from the gyro                               |
| temperature  | Sensor temperature (the offset changes with it)                                               |
| `slip`       | Number of 50 ms samples in which encoders and gyro disagreed about the turn rate while driving |
| `src`        | `gyro` or `enc`: where the heading comes from                                                 |
| `still`      | 1 = wheels have not moved for 300 ms                                                          |

`enc_hdg` and `gyro_hdg` are what you use for calibration ([9.3](#93-track-width)). When the two drift
apart while you drive, the encoders slipped or are mis-calibrated. The status line is only sent while
the live output is on (not after pausing with `p`).

---

## 5. Data format

### 5.1 Serial stream and recording files

One line per sample, comma separated, the same format on the serial port and in the files:

```
t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg
12350,3.214,48.870,-120.52,2210.40,2,-4.18
```

| Column        | Meaning                                                              |
|---------------|----------------------------------------------------------------------|
| `t_ms`        | Time since boot in milliseconds                                      |
| `dx_mm, dy_mm`| Platform displacement since the previous sample (world frame)        |
| `x_mm, y_mm`  | Accumulated position since boot or the last `r`                      |
| `n_valid`     | Number of valid wheels (always 2, kept for compatibility)            |
| `heading_deg` | Heading, 0 = +Y, positive clockwise, −180 … +180                     |

World frame: **x to the right, y up** (forward at the start).

The first line of every file is a comment: `# recording; t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg`.
The viewer also accepts the older 6-column format without heading; the heading is then
estimated from the direction of travel.

### 5.2 Storage

- Files are stored in the **LittleFS** partition (about 1.4 MB with the default partition table).
- A line is about 45 bytes, so at 20 Hz a recording grows by roughly **0.9 KB/s**:
  about 18 minutes per MB, roughly 25 minutes in total for the whole partition.

---

## 6. Viewer usage

Start it with `python pythonViewer/path_viewer.py`.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ Port: [COM5 - USB-SERIAL CH340 ▾] [⟳] Baud: [115200] [Start][Stop]           │
│       [Clear path / center] [Export]                                         │
│ [ ] Invert X  [ ] Invert Y   Scale: [1.0]   Chair size x: [1.0]              │
│ Recordings on the ESP32: [Replay next] [Delete all files]                    │
│ Gyro: [Calibrate gyro] [Heading source]                                      │
├──────────────────────────────────────────────────────────────────────────────┤
│  x = … y = … heading = …°  samples = …                                       │
│                         (autoscaled path with the wheelchair at its end)     │
│  ▬▬▬ scale bar                                                               │
├──────────────────────────────────────────────────────────────────────────────┤
│ status bar                                                                   │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 6.1 Connecting

1. Select the **port** (press ⟳ if it is not listed). The baud rate is 115200.
2. Press **Start**. The viewer opens the port without resetting the board (DTR/RTS are kept
   low) and starts drawing.
3. **Stop** closes the port. The path stays on screen.

### 6.2 The display

- **Blue line** – the path. **Green dot** – start. **Grey cross** – origin.
- The **wheelchair** is drawn at the current position, rotated to the current heading:
  two dark wheels, a grey body, and a red triangle at the front. It is drawn **to scale**
  (560 mm wheel spacing, 600 mm wheels, about 950 mm long) so you can judge turning radii.
- The view **autoscales** so that the whole path (plus free space around the chair) always
  fits. It never zooms in further than about 2.5 m width. Axes are scaled equally.
- The **scale bar** at the bottom left shows a round length in mm.
- The **top left text** shows position, heading, number of samples and the extent of the path.
- When the picture is zoomed far out the chair is not drawn smaller than about 22 px so it stays visible.

### 6.3 Buttons and fields

| Control                 | Function                                                                                   |
|-------------------------|--------------------------------------------------------------------------------------------|
| **Start / Stop**        | Open / close the serial port                                                               |
| **Clear path / center** | Clears the drawing and makes the current position the origin. Also sends `r` so that the firmware position and heading restart at 0. |
| **Export**              | Saves the path as SVG (see [6.6](#66-exporting-the-path-as-svg))                           |
| **Invert X / Invert Y** | Mirror the drawing. The heading is mirrored consistently. Both on = rotated by 180°.       |
| **Scale**               | Multiplier applied to incoming distances. 1.0 = millimetres. Applies to **new** samples only; the chair icon and export follow it. |
| **Chair size x**        | Manual size factor for the icon (1.0 = true scale)                                         |
| **Replay next**         | Sends `d`: the board sends the next recording, which is drawn (see below)                  |
| **Delete all files**    | Asks for confirmation, then sends `w`: deletes all recordings in the ESP32                 |
| **Calibrate gyro**      | Sends `g`: the board measures the gyro offset (**chair must stand still for 2 s**)         |
| **Heading source**      | Sends `h`: switches the heading between gyro and encoders only                             |

The **status bar** shows the connection state, the periodic status line from the firmware (without the
word `status`), and messages such as `recording started: /rec_003.csv` or `3 recording(s) deleted` for a
few seconds.

### 6.4 Recording from the viewer's point of view

The viewer does not start or stop recordings. Use the **button on the board** (short press) or
send `p` from a terminal. While it records, the LED blinks. Pausing also pauses the live data,
so the viewer stops moving until you start again.

### 6.5 Replaying recordings

1. Pause the recording if one is running (short press).
2. Press **Replay next** in the viewer (or long-press the board button).
3. The viewer clears the display and draws the file while it arrives. A red label at the top
   shows `REPLAY rec_002.csv (2/3) – receiving …`, later `– finished`. The status bar then
   shows the number of samples.
4. Press **Replay next** again for the next file. After the last one it wraps around.
5. The replayed path stays on screen. When live data arrives again, the live path starts fresh.
   If the output is paused, the replay stays visible.
6. A replay starts at (0, 0), because the viewer integrates the per-sample displacements itself.
   You can **export** it as SVG just like a live path.

### 6.6 Exporting the path as SVG

Press **Export** and choose a file name. The file contains **only the path**: no wheelchair,
no captions, no background (transparent).

- **Paper:** exactly **A3** (420 × 297 mm). Landscape if the path is wider than tall, otherwise
  portrait. The path is scaled uniformly to fit with a 15 mm border and centred.
- **Units:** 1 user unit = 1 mm on the sheet. The status bar shows the drawing scale
  (e.g. `scale 1:13`, real mm per mm on paper; it assumes Scale = 1.0 = mm).
- **Line:** black, 0.5 mm wide.
- **Nodes:** points that deviate less than 2 mm from the path are dropped
  (Ramer–Douglas–Peucker), so the file has far fewer nodes than samples. The status bar shows
  how many points were kept.

**Editing in Inkscape**

- Select the path and press `N` (Node tool) to see and move the nodes. Double-click a segment
  to add a node, select a node and press `Delete` to remove it (`Ctrl+Delete` keeps the shape).
- Smooth curves: `N`, `Ctrl+A` (select all nodes), `Shift+U` (make segments curves),
  `Shift+S` (make nodes smooth) – you can then drag Bézier handles.
- Automatic smoothing: `Path → Simplify` (`Ctrl+L`), repeat for stronger simplification.
- `Path → Stroke to Path` creates an outline you can fill.

---

## 7. Configuration reference

### 7.1 Firmware (`src/config.h`)

| Constant                         | Default      | Meaning                                                              |
|----------------------------------|--------------|----------------------------------------------------------------------|
| `SAMPLE_HZ`                      | 20           | Odometry and output rate                                             |
| `WHEEL_DIAMETER_MM`              | 56.5         | Diameter of the **encoder roller** (not the wheelchair wheel)        |
| `TRACK_WIDTH_MM`                 | 560          | Distance between the two wheelchair wheel contact points             |
| `ENCODER_PPR`                    | 600          | Pulses per revolution and channel (counted 4× internally)            |
| `ENC_L_A/B, ENC_R_A/B`           | 25/26, 14/27 | Encoder pins                                                         |
| `ENC_LEFT_SIGN / ENC_RIGHT_SIGN` | 1 / 1        | Set to −1 if a wheel counts backwards when rolling forward           |
| `RIGHT_WHEEL_COMP`               | 1.020        | Right wheel distance multiplier (straight-line compensation, see 9.4)|
| `IMU_SDA / IMU_SCL`              | 21 / 22      | I2C pins of the GY-521                                               |
| `IMU_ADDR`                       | 0x68         | I2C address (0x69 if AD0 is tied to 3V3)                             |
| `IMU_HZ`                         | 200          | Gyro sampling and integration rate                                   |
| `IMU_INVERT_YAW`                 | false        | `true` if a clockwise turn gives a negative heading change           |
| `IMU_YAW_SCALE`                  | 1.0          | Gyro scale correction (see 9.2)                                      |
| `IMU_CAL_MS`                     | 2000         | Length of the offset calibration window                              |
| `IMU_CAL_MAX_SPREAD_DPS`         | 2.0          | Max. gyro noise (peak-to-peak) accepted during calibration           |
| `IMU_CAL_MAX_ACC_SPREAD_G`       | 0.05         | Max. accelerometer variation accepted during calibration             |
| `IMU_GRAVITY_TAU_S`              | 1.0          | Smoothing of the "which way is up" estimate                          |
| `IMU_BIAS_TAU_S`                 | 3.0          | Speed of the automatic offset re-learning while standing still       |
| `IMU_BIAS_GATE_DPS`              | 3.0          | Offset is only re-learned while the rotation is below this           |
| `IMU_FREEZE_DPS`                 | 0.5          | While standing still and below this rotation, the heading is frozen  |
| `STILL_MS`                       | 300          | No wheel motion for this long = standing still                       |
| `STILL_WHEEL_MM`                 | 0.25         | Wheel travel per sample below this counts as "no motion"             |
| `SLIP_MIN_DPS / SLIP_REL`        | 5 / 0.10     | Slip is flagged when turn rates differ by more than max(5 dps, 10 % of the gyro rate) |
| `BTN_PIN`                        | 13           | Button                                                               |
| `LED_PIN`                        | 2            | Status LED                                                           |
| `DEBOUNCE_MS`                    | 40           | Button debounce time                                                 |
| `LONG_PRESS_MS`                  | 1000         | Press duration that counts as "long"                                 |
| `BLINK_MS` / `CAL_BLINK_MS`      | 250 / 100    | LED blink interval while recording / while calibrating the gyro      |
| `LOG_MAX_BYTES`                  | 1000000      | Maximum size of one recording                                        |
| `MIN_FREE_BYTES`                 | 65536        | Minimum free flash required to start a recording                     |

### 7.2 Viewer (`pythonViewer/path_viewer.py`, top of the file)

| Constant                | Default        | Meaning                                                      |
|-------------------------|----------------|--------------------------------------------------------------|
| `DEFAULT_BAUD`          | 115200         | Initial baud rate in the GUI                                 |
| `CHAIR_TRACK_MM`        | 560            | Wheel spacing of the icon – **keep equal to `TRACK_WIDTH_MM`** |
| `CHAIR_WHEEL_W_MM`, `CHAIR_WHEEL_D_MM`, `CHAIR_LENGTH_MM` | 50, 600, 950 | Icon dimensions    |
| `CHAIR_PAD_MM`          | 700            | Free space kept around the path                              |
| `MIN_VIEW_MM`           | 2500           | The view is never zoomed in further than this                |
| `MIN_ICON_PX_LEN`       | 22             | Smallest icon length in pixels when zoomed far out           |
| `MAX_DRAW_POINTS`       | 4000           | The on-screen polyline is thinned above this many points     |
| `EXPORT_PAGE_MM`        | (420, 297)     | Sheet size for the SVG export (A3)                           |
| `EXPORT_MARGIN_MM`      | 15             | Border on the sheet                                          |
| `EXPORT_STROKE_MM`      | 0.5            | Line width on the sheet                                      |
| `EXPORT_TOLERANCE_MM`   | 2.0            | Point thinning in the export (0 = keep all points)           |

---

## 8. Commissioning the encoders

1. Mount the encoders so their rollers press against the tyres without slipping (spring-loaded arm,
   rubber roller, constant preload).
2. Wire A/B of the left encoder to GPIO25/26 and the right encoder to GPIO14/27 (3.3 V logic,
   common ground, external pull-ups).
3. Enter the **diameter of the measuring roller** in `WHEEL_DIAMETER_MM`, the **pulses per
   revolution** in `ENCODER_PPR`, and the **wheel spacing** in `TRACK_WIDTH_MM`
   (and the same spacing in `CHAIR_TRACK_MM` of the viewer).
4. Roll the chair forward: both wheels must count positive (`L` and `R` in the status line go up).
   If one counts negative, set its `ENC_*_SIGN` to −1 (the right encoder is usually mirrored).
5. Wire the GY-521 ([2.4](#24-wiring-the-gy-521-mpu-6050)), flash, keep the chair still during the first
   2 seconds. The serial output must show `gyro calibrated: …`.
6. Check the gyro direction: turn the chair **clockwise** on the spot. `gyro` in the status line and the
   heading must become **positive**. If not, set `IMU_INVERT_YAW` to `true`.
7. Re-flash and calibrate (next section).

Distance per count = π · roller diameter / (4 · `ENCODER_PPR`).

---

## 9. Calibration

Do the steps in this order on the real chair. 9.1 is repeated regularly, the others only once
(and again if you change rollers, tyres or the gyro).

### 9.1 Gyro offset calibration

**Why it is needed.** A gyro never reads exactly zero when it stands still. This *zero-rate offset*
(also called bias) is typically between a few tenths of a degree per second and a few degrees per
second for an MPU-6050, differs from board to board, and changes with temperature. Uncorrected, an
offset of 1 °/s makes the heading wrong by 60° per minute. Everything else in the fusion depends on
this value being small, so it is measured in three ways:

1. **At every power-up** the firmware averages the gyro (all three axes) for 2 s while the chair
   stands still. It also averages the accelerometer to learn where "up" is and prints, for example:

   ```
   # gyro calibrated: offset x=1.302 y=-0.798 z=0.594 dps, |g|=1.000, board tilt 71.7 deg, 25.0 C
   ```

   `|g|` must be close to 1.0 (otherwise the sensor is faulty). "Board tilt" is how far the board's
   Z axis is from vertical; any value is fine. While it runs, the LED blinks fast.
   The measurement is **rejected and repeated** if the gyro noise exceeds `IMU_CAL_MAX_SPREAD_DPS`
   or the accelerometer varies by more than `IMU_CAL_MAX_ACC_SPREAD_G` – that means the chair was
   moving. In that case the firmware prints `chair is moving … retrying`; the heading comes from the
   encoders until a calibration succeeds.
2. **On demand**: press **Calibrate gyro** in the viewer or send `g`. Use it after the chair has warmed
   up, after you changed the load, or whenever the heading seems to creep while standing still.
3. **Automatically while standing still.** Whenever the wheels have not moved for 300 ms and the gyro
   shows almost no rotation, the offset is re-learned continuously (time constant 3 s). This follows
   the temperature drift. At the same time the heading is **frozen** while standing still, so
   standing for hours does not make the heading wander.

**Tips for a good calibration**

- Calibrate with the chair **as it is used**: with the person sitting in it, the brakes on, no one
  pushing. Even a person shifting in the seat is usually fine, a push is not.
- Let the electronics **warm up** for a minute or two (the sensor heats itself by a few °C). The
  automatic re-learning in step 3 covers the rest.
- You do not have to calibrate the accelerometer. Only the *direction* of gravity is used, so
  accelerometer offsets and scale errors are irrelevant.
- A quick quality check: after the calibration leave the chair still for 30 s. The `gyro … dps` value in
  the status line should be within about ±0.1 and `hdg` must not change at all.
- A drive test: press `r`, drive a loop of a few minutes and come back to the start mark, stop. The
  heading shown should be close to the start heading. Compare with encoders only (`h`).

### 9.2 Gyro scale (optional but recommended once)

The MPU-6050's sensitivity is only specified to about ±3 % from the factory, so a 360° turn can show
350°…370°. To correct it:

1. Put a mark on the floor and a mark on the chair (or align the chair along a floor line).
2. Press `r`, rotate the chair on the spot clockwise by **exactly 5 turns** (1800°) and stop at the mark.
3. Read `gyro_hdg` from the status line (it is continuous, not wrapped to ±180°), then:

   ```
   new IMU_YAW_SCALE = old IMU_YAW_SCALE × true angle / gyro_hdg      (e.g. 1800 / 1824 → ×0.987)
   ```

Rotate the chair at a moderate speed (one turn in about 10–20 s) and about the middle of the chair.

### 9.3 Track width

With a working gyro the track width can be calibrated from the encoders' own turn error, without
having to turn the chair by an exact angle:

1. Press `r`, turn the chair on the spot (one or more full turns, any angle). The sign does not matter.
2. Read `enc_hdg` and `gyro_hdg` from the status line (both continuous). Then:

   ```
   new TRACK_WIDTH_MM = old TRACK_WIDTH_MM × enc_hdg / gyro_hdg
   ```

   Example: the encoders say 1830°, the gyro says 1800° → track width × 1.0167.
3. Use the same value for `CHAIR_TRACK_MM` in the viewer.

Do 9.2 first, otherwise the gyro scale error ends up in the track width. The track width calibrated
this way is the *effective* value for your rollers; it does not have to equal the measured distance
between the wheels.

### 9.4 Distance and straight-line compensation

**Distance** – push the chair straight for a known distance (e.g. 5 m) and read `y` in the viewer. Then:

```
new WHEEL_DIAMETER_MM = old WHEEL_DIAMETER_MM × (true distance / measured distance)
```

**Left/right balance** (`RIGHT_WHEEL_COMP`) – the two rollers never have exactly the same effective
diameter. This does not disturb the heading any more (it comes from the gyro), but it distorts the
distance on curves, and it makes `enc_hdg` drift on straight lines. To balance them: press `r`, push the
chair straight for 5 m or more, and read `enc_hdg` and `gyro_hdg`. Then:

```
Δθ  = (enc_hdg − gyro_hdg) in radians          (degrees × π / 180)
D   = driven distance (y in the viewer, in mm)
new RIGHT_WHEEL_COMP = old RIGHT_WHEEL_COMP × (2·D + TRACK_WIDTH_MM·Δθ) / (2·D − TRACK_WIDTH_MM·Δθ)
```

The chair does not have to drive perfectly straight: the gyro reports how much it really turned.

---

## 10. How the sensor fusion works

**Gyro task (200 Hz).** The MPU-6050 delivers turn rates about its three axes. The firmware subtracts
the calibrated offset, then projects the rotation onto the vertical axis, which is determined by a
slowly smoothed accelerometer reading (τ = 1 s). That is why the board can be mounted in any orientation
and why the yaw rate stays correct when the chair stands on a slope. The yaw rate is multiplied by
`IMU_YAW_SCALE` and integrated with the real elapsed time between readings. The sensor's digital low-pass
filter is set to 42 Hz to suppress vibration.

**Odometry task (20 Hz).** The heading change since the last sample is taken from the gyro. Distance and
position are computed from the encoders. Two additional checks run:

- **Standstill detection** – both wheels below `STILL_WHEEL_MM` per sample for `STILL_MS`. It is reported to
  the gyro task, which then re-learns the offset and freezes the heading when the measured rotation is
  below `IMU_FREEZE_DPS`. A real turn immediately exceeds that threshold, so nothing is lost when the
  chair starts to move.
- **Slip detection** – while driving, the turn rate according to the encoders is compared with the gyro.
  If they differ by more than `SLIP_MIN_DPS` or `SLIP_REL` (10 %), the sample is counted in `slip=`. The
  heading is not changed by this check, because the gyro is already the heading source; it is a
  diagnostic that tells you how often and where the rollers slipped.

**Fallback.** Without a working gyro (not found at boot, I2C errors, calibration not finished) the heading
comes from the encoders. The state is shown as `src=enc` in the status line.

**What the gyro does *not* fix.** Slip that affects the distance. If both rollers slip forward together,
or one roller slips while the chair turns, the travelled distance is wrong, although the heading stays
right. This is a limit of the encoder-on-tyre principle; see [section 12](#12-known-limitations).

---

## 11. Troubleshooting

| Problem                                         | What to check                                                                                  |
|-------------------------------------------------|------------------------------------------------------------------------------------------------|
| Upload fails / "Failed to connect"              | Hold **BOOT** while the upload starts; use a data-capable USB cable; install the CH340 driver  |
| Port is not listed in the viewer                | Press ⟳; check the driver and the cable                                                        |
| "Error: could not open port" / access denied    | Another program (serial monitor) is using the port – close it                                  |
| The board resets when a terminal connects       | Some terminals toggle DTR/RTS. The viewer does not. Disable DTR/RTS in the terminal. After a reset the gyro is calibrated again – keep the chair still |
| Viewer connects but nothing moves               | Live output may be paused (`p` again). Check the status line: do `L` / `R` change when you roll the chair? |
| `IMU not found at 0x68`                         | Check VCC/GND/SDA/SCL (GPIO21/22), swap SDA/SCL, check the AD0 pin (0x69 if tied to 3V3), try shorter wires |
| `IMU WHO_AM_I = 0x…` warning                    | Clone chip. Normally it works anyway. If the readings look wrong, replace the board            |
| `chair is moving … retrying` at every start     | Vibration or people moving. Wait until everything is quiet, or raise `IMU_CAL_MAX_SPREAD_DPS` slightly |
| Gravity value in the calibration message far from 1.000 | Sensor or wiring problem                                                        |
| Heading turns the wrong way                     | Set `IMU_INVERT_YAW` to `true`                                                                 |
| Heading creeps while standing still             | Calibrate again (`g`) after warm-up. Check `still=1` in the status line; if it stays 0 while standing, the encoder inputs are noisy: raise `STILL_WHEEL_MM` or add pull-ups/shielding |
| Heading drifts slowly while driving             | Offset has changed (temperature): press `g`. Check `bias=`: it should settle after warm-up      |
| 360° turn shows 355° or 365°                    | Gyro scale: calibrate `IMU_YAW_SCALE` (9.2)                                                    |
| `slip=` counts up although nothing slips        | Track width or left/right balance is off (9.3, 9.4), or the gyro scale is off (9.2)            |
| Path is curved when driving straight            | Distance error between the wheels: calibrate `RIGHT_WHEEL_COMP` (9.4). Heading itself comes from the gyro |
| `imu=lost` in the status line                   | I2C errors: wires, power, interference from motors                                             |
| Mirrored left/right or up/down                  | Use **Invert X / Invert Y**; check `ENC_*_SIGN`                                                |
| "stop the recording first …"                    | Replay and delete are refused while recording. Pause with a short press or `p`                 |
| "flash full – cannot start"                     | Press **Delete all files** (or send `w`)                                                       |
| Replay does not appear                          | Check that a `BEGIN LOG` line arrives (terminal). Check the baud rate. Large files take ~90 s  |
| LED does not blink                              | It blinks only while recording or calibrating. Some boards have the LED on a different GPIO – change `LED_PIN` |
| `ModuleNotFoundError: serial`                   | `pip install pyserial` (not the package named `serial`)                                        |
| `ModuleNotFoundError: tkinter`                  | Linux: `sudo apt install python3-tk`                                                           |

---

## 12. Known limitations

- **Distance still drifts with roller slip.** The gyro makes the heading independent of slip, but the
  distance comes from the encoders. Possible further improvements: spring-loaded rollers with better
  grip, encoders on the wheel axle, or unpowered "dead wheels" or an optical floor sensor.
- **No absolute reference.** There is no magnetometer on purpose (motors and steel disturb it indoors),
  so the heading is still integrated: after a long time and many turns a small error remains.
  The offset re-learning keeps it small. Press `r` to re-zero for long paths.
- **The gyro offset must be right at the start.** A chair that is moving at power-up cannot be
  calibrated. Keep it still, or press `g` as soon as it is.
- **Fast rotations** near the sensor limit (±250 °/s) are not possible for a wheelchair, so this is not
  a practical limit.
- **Flash writes pause the processor briefly.** While a recording is written (about every 4 s), the
  gyro task may be stalled for some tens of milliseconds. The integration uses the real elapsed time,
  so this causes no noticeable error. The encoder interrupts decode quadrature signals in software,
  and missed counts are possible at high pulse rates during such a pause. The ESP32's hardware pulse
  counter (PCNT) avoids this and is the recommended upgrade if you see drift at higher speeds.
- **Replay speed** is limited by the serial link (about 11 KB/s at 115200 baud).
- **Flash space** is limited to about 1.4 MB, roughly 25 minutes of recording at 20 Hz.
  Flash has a finite number of write cycles, so avoid continuous long-term recording.
- The SVG export's scale ratio assumes the viewer's **Scale** field is 1.0 (millimetres).

---

## 13. Quick reference

| I want to …                       | Board                  | Viewer                  | Serial key |
|-----------------------------------|------------------------|-------------------------|------------|
| Start / pause recording           | short press            | –                       | `p`        |
| Replay the next recording         | long press (≥ 1 s)     | **Replay next**         | `d`        |
| Delete all recordings             | –                      | **Delete all files**    | `w`        |
| Reset position and heading        | –                      | **Clear path / center** | `r`        |
| Calibrate the gyro offset         | (power cycle)          | **Calibrate gyro**      | `g`        |
| Switch heading gyro ↔ encoders    | –                      | **Heading source**      | `h`        |
| Save the path as A3 SVG           | –                      | **Export**              | –          |

Data format: `t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg` · 115200 baud · 20 Hz · files `/rec_NNN.csv`

Calibration order: gyro offset (9.1) → gyro scale (9.2) → track width (9.3) → distance and left/right balance (9.4).
