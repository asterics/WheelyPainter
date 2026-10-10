# Wheelchair Odometry – ESP32 firmware + Path Viewer

Odometry for a wheelchair using **two wheel encoders** (one on each drive wheel).
The ESP32 turns the encoder signals into a position and heading, streams them over USB,
and records them to its internal flash. A small Python/Tk program draws the path live,
replays recordings from the board, and exports the path as an SVG for printing on A3.

Until the real encoders are mounted, they are **simulated with two potentiometers**
(centre position = standing still, turn up = wheel forward, turn down = wheel backward).

```
 left  encoder / pot ─┐
                      ├─►  ESP32 (odometry, 20 Hz) ──USB serial──►  path_viewer.py
 right encoder / pot ─┘          │                                     (live path, replay,
                                 ▼                                      delete, SVG export)
          button (GPIO13) ──► recording to flash files /rec_001.csv, /rec_002.csv, …
          LED  (GPIO2)   ◄── blinks while recording
```

## Contents

1. [Repository layout](#1-repository-layout)
2. [Hardware](#2-hardware)
3. [Installation](#3-installation)
4. [Firmware usage](#4-firmware-usage)
5. [Data format](#5-data-format)
6. [Viewer usage](#6-viewer-usage)
7. [Configuration reference](#7-configuration-reference)
8. [Switching from potentiometers to real encoders](#8-switching-from-potentiometers-to-real-encoders)
9. [Calibration](#9-calibration)
10. [Troubleshooting](#10-troubleshooting)
11. [Known limitations](#11-known-limitations)
12. [Quick reference](#12-quick-reference)

---

## 1. Repository layout

```
encoder_odometry/
├── platformio.ini      PlatformIO project (ESP32 DevKit, Arduino framework, LittleFS)
├── src/
│   └── main.cpp        Firmware: odometry, recording, button/LED, serial commands
├── path_viewer.py      PC tool: live path, replay, delete files, SVG export
└── README.md
```

---

## 2. Hardware

### 2.1 Board

[ESP-32 Dev Board 30 PIN](https://www.robotics.org.za/ESP32-DEV) – ESP-WROOM-32, 4 MB flash,
CH340 USB-serial chip, USB-C connector, no PSRAM. Any standard ESP32 DevKit with the same pins
works. The internal LED is assumed to be on GPIO2 (the usual location).

### 2.2 Pin assignment

| Function                            | GPIO   | Notes                                                       |
|-------------------------------------|--------|-------------------------------------------------------------|
| Left potentiometer wiper (sim)      | **35** | ADC1, input only                                            |
| Right potentiometer wiper (sim)     | **34** | ADC1, input only                                            |
| Record / replay button              | **13** | Button between GPIO13 and GND (internal pull-up is enabled) |
| Internal LED                        | **2**  | Blinks while recording                                      |
| Left encoder channel A / B (real)   | 25 / 26| Only used when `SIMULATED_ENCODERS` is 0                    |
| Right encoder channel A / B (real)  | 32 / 33| Only used when `SIMULATED_ENCODERS` is 0                    |

### 2.3 Wiring the simulation

```
 3V3 ──┬── pot L outer ──── wiper ──► GPIO35 (left wheel)
 GND ──┤                  pot L other outer
       │
       └── pot R outer ──── wiper ──► GPIO34 (right wheel)
                          pot R other outer
 GND ── button ── GPIO13
```

Use two 10 kΩ linear potentiometers, outer pins to 3V3 and GND, wiper to the ADC pin.
Mid position means "no movement"; a dead band of about ±80 ADC counts around the centre
keeps the platform still.

### 2.4 Wiring real encoders (later)

Quadrature encoders (A/B channels, 3.3 V logic) are connected to the pins in the table above.
The inputs use the internal pull-ups. The encoders press against the wheelchair tyres, so the
distance per count is set by the diameter of the **measuring roller** on the encoder shaft
(see [section 8](#8-switching-from-potentiometers-to-real-encoders)).

---

## 3. Installation

### 3.1 Firmware (PlatformIO)

**Option A – VS Code**

1. Install [Visual Studio Code](https://code.visualstudio.com/) and its **PlatformIO IDE** extension.
2. `File → Open Folder…` and select the `encoder_odometry` folder.
3. Connect the board with a USB data cable.
4. Click the **Upload** (→) button in the blue PlatformIO status bar.

**Option B – command line**

```bash
pip install platformio
cd encoder_odometry
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
python path_viewer.py
```

`tkinter` is part of the standard Python installer on Windows and macOS. On Linux install it
with e.g. `sudo apt install python3-tk`.

Close any other program that has the COM port open (PlatformIO monitor, Arduino serial
monitor, …). Only one program can use the port at a time.

---

## 4. Firmware usage

### 4.1 What the firmware does

Every 50 ms (20 Hz) it reads the two wheels and computes the platform motion with standard
differential-drive odometry:

```
ds  = (dL + dR) / 2               distance travelled by the platform centre
dθ  = (dL − dR) / track width     left faster than right → turn right (clockwise)
x  += ds · sin(θ + dθ/2)
y  += ds · cos(θ + dθ/2)
θ  += dθ
```

- The start position is (0, 0) and the start heading points along **+Y ("up")**.
- Heading `θ` is measured **clockwise from +Y** and reported in degrees (−180 … +180).
- All distances are in **millimetres**.

Each sample is printed on the serial port and, while recording, appended to a file in flash.

### 4.2 Simulating the wheels with the potentiometers

| Pot position  | Wheel                       |
|---------------|-----------------------------|
| Centre        | stopped                     |
| Turned up     | forward (up to 1000 mm/s)   |
| Turned down   | backward                    |

- Both pots up → platform drives forward. Both down → backward.
- Pots turned by different amounts → the platform drives an arc.
- Left up, right down → the platform turns on the spot to the right.
- If the pots are not exactly at the electrical centre when at rest, press **Center pots** in
  the viewer (or send `c`). The current positions are then taken as "no movement".

### 4.3 Boot behaviour

At power-up the live serial output is **on** and recording is **off**, so the viewer shows
the path immediately. Nothing is written to flash until you start a recording.

### 4.4 Recording

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

### 4.5 Replay and delete

- **Replay** (`d` or long press) sends the next recording over the serial port, one file per
  request. After the last file it wraps around to the first. Progress is printed as
  `# replay 2/3`, followed by the file between `# ---- BEGIN LOG rec_002.csv ----` and
  `# ---- END LOG ----`. The viewer draws it automatically.
- At 115200 baud a 1 MB file takes about **90 seconds** to transfer.
- **Delete** (`w`) removes all recordings.
- Replay and delete are **refused while recording** (the firmware prints a message). Pause the
  recording first.

### 4.6 Serial commands

Open the port at **115200 baud, 8N1** and send single characters (no newline needed).

| Key | Function                                                                          |
|-----|-----------------------------------------------------------------------------------|
| `p` | Start / pause recording and live output (same as short button press)              |
| `d` | Replay next recording (same as long button press)                                 |
| `w` | Delete all recordings                                                             |
| `r` | Reset position and heading to 0 (heading 0 = "up" again)                          |
| `c` | Take the current pot positions as "no movement" (simulation only)                 |
| `s` | Switch steering on/off. Off = the platform only drives straight along ±Y with the average of both wheels, handy for checking signs. On by default. |

### 4.7 Messages from the firmware

Lines starting with `#` are comments. Besides the confirmations of the commands, a status
line is printed twice a second:

```
# L raw=2051 0 mm/s | R raw=3120 410 mm/s | v=205 mm/s hdg=12.4 deg steer=on rec=off
```

It shows the raw ADC value (or encoder count) and speed of each wheel, the platform speed,
heading, steering state and recording state. It is useful to debug the pots. Like the sample
lines it is only sent while the live output is on (not after pausing with `p`).

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
| `n_valid`     | Number of valid wheels (always 2)                                    |
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

Start it with `python path_viewer.py`.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ Port: [COM5 - USB-SERIAL CH340 ▾] [⟳] Baud: [115200] [Start][Stop]           │
│       [Clear path / center] [Center pots] [Export]                           │
│ [ ] Invert X  [ ] Invert Y   Scale: [1.0]   Chair size x: [1.0]              │
│ Recordings on the ESP32: [Replay next] [Delete all files]                    │
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
| **Center pots**         | Sends `c`: current pot positions become "no movement" (simulation)                          |
| **Export**              | Saves the path as SVG (see [6.6](#66-exporting-the-path-as-svg))                           |
| **Invert X / Invert Y** | Mirror the drawing. The heading is mirrored consistently. Both on = rotated by 180°.       |
| **Scale**               | Multiplier applied to incoming distances. 1.0 = millimetres. Applies to **new** samples only; the chair icon and export follow it. |
| **Chair size x**        | Manual size factor for the icon (1.0 = true scale)                                         |
| **Replay next**         | Sends `d`: the board sends the next recording, which is drawn (see below)                  |
| **Delete all files**    | Asks for confirmation, then sends `w`: deletes all recordings in the ESP32                 |

The **status bar** shows the connection state, the periodic status line from the firmware, and
messages such as `recording started: /rec_003.csv` or `3 recording(s) deleted` for a few seconds.

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

### 7.1 Firmware (`src/main.cpp`, top of the file)

| Constant                | Default | Meaning                                                              |
|-------------------------|---------|----------------------------------------------------------------------|
| `SIMULATED_ENCODERS`    | 1       | 1 = potentiometers, 0 = real quadrature encoders                     |
| `SAMPLE_HZ`             | 20      | Odometry and output rate                                             |
| `WHEEL_DIAMETER_MM`     | 50      | Diameter of the **encoder roller** (not the wheelchair wheel)        |
| `TRACK_WIDTH_MM`        | 560     | Distance between the two wheelchair wheel contact points             |
| `POT_L_PIN / POT_R_PIN` | 35 / 34 | ADC pins of the simulation pots                                      |
| `SIM_MAX_SPEED_MM_S`    | 1000    | Wheel speed at full pot deflection                                   |
| `POT_DEADBAND`          | 80      | ADC counts around the centre treated as zero                         |
| `POT_HALF_RANGE`        | 1900    | ADC counts from centre to full speed                                 |
| `ENCODER_PPR`           | 360     | Pulses per revolution and channel (counted 4× internally)            |
| `ENC_L_A/B, ENC_R_A/B`  | 25/26, 32/33 | Encoder pins                                                    |
| `ENC_LEFT_SIGN / ENC_RIGHT_SIGN` | 1 / 1 | Set to −1 if a wheel counts backwards when rolling forward    |
| `BTN_PIN`               | 13      | Button                                                               |
| `LED_PIN`               | 2       | Status LED                                                           |
| `DEBOUNCE_MS`           | 40      | Button debounce time                                                 |
| `LONG_PRESS_MS`         | 1000    | Press duration that counts as "long"                                 |
| `BLINK_MS`              | 250     | LED toggle interval while recording                                  |
| `LOG_MAX_BYTES`         | 1000000 | Maximum size of one recording                                        |
| `MIN_FREE_BYTES`        | 65536   | Minimum free flash required to start a recording                     |

### 7.2 Viewer (`path_viewer.py`, top of the file)

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

## 8. Switching from potentiometers to real encoders

1. Mount the encoders so their rollers press against the tyres without slipping.
2. Wire A/B of the left encoder to GPIO25/26 and the right encoder to GPIO32/33 (3.3 V logic,
   common ground).
3. In `main.cpp` set `#define SIMULATED_ENCODERS 0`.
4. Enter the **diameter of the measuring roller** in `WHEEL_DIAMETER_MM`, the **pulses per
   revolution** in `ENCODER_PPR`, and the **wheel spacing** in `TRACK_WIDTH_MM`
   (and the same spacing in `CHAIR_TRACK_MM` of the viewer).
5. Roll the chair forward: both wheels must count positive. If one counts negative, set its
   `ENC_*_SIGN` to −1 (the right encoder is usually mirrored).
6. Re-flash and calibrate (next section).

Distance per count = π · roller diameter / (4 · `ENCODER_PPR`).

---

## 9. Calibration

Do both tests on the real chair after switching to encoders.

**Distance** – push the chair straight for a known distance (e.g. 5 m) and read `y`
in the viewer. Then:

```
new WHEEL_DIAMETER_MM = old WHEEL_DIAMETER_MM × (true distance / measured distance)
```

**Track width** – rotate the chair on the spot by exactly 360° (right turn) and read the heading.
If the display shows e.g. +8° instead of 0°:

```
new TRACK_WIDTH_MM = old TRACK_WIDTH_MM × (360 + 8) / 360
```

Adjust `CHAIR_TRACK_MM` in the viewer to match.

---

## 10. Troubleshooting

| Problem                                         | What to check                                                                                  |
|-------------------------------------------------|------------------------------------------------------------------------------------------------|
| Upload fails / "Failed to connect"              | Hold **BOOT** while the upload starts; use a data-capable USB cable; install the CH340 driver  |
| Port is not listed in the viewer                | Press ⟳; check the driver and the cable                                                        |
| "Error: could not open port" / access denied    | Another program (serial monitor) is using the port – close it                                  |
| The board resets when a terminal connects       | Some terminals toggle DTR/RTS. The viewer does not. Disable DTR/RTS in the terminal            |
| Viewer connects but nothing moves               | Live output may be paused (`p` again), or the pots are in the dead band. Check the status bar line |
| Path drifts while the pots are at rest          | Press **Center pots** (or `c`) with both pots at rest                                          |
| Backward motion looks wrong / no reverse        | Look at the status line: raw ADC values and mm/s per wheel. Check wiring and pot centring       |
| Heading keeps rotating with both pots "up"      | The pots are never exactly equal. Press `s` to disable steering for straight-line tests        |
| Mirrored left/right or up/down                  | Use **Invert X / Invert Y**; with real encoders check `ENC_*_SIGN`                             |
| "stop the recording first …"                    | Replay and delete are refused while recording. Pause with a short press or `p`                 |
| "flash full – cannot start"                     | Press **Delete all files** (or send `w`)                                                       |
| Replay does not appear                          | Check that a `BEGIN LOG` line arrives (terminal). Check the baud rate. Large files take ~90 s  |
| LED does not blink                              | It blinks only while recording. Some boards have the LED on a different GPIO – change `LED_PIN` |
| `ModuleNotFoundError: serial`                   | `pip install pyserial` (not the package named `serial`)                                        |
| `ModuleNotFoundError: tkinter`                  | Linux: `sudo apt install python3-tk`                                                           |

---

## 11. Known limitations

- **Odometry drifts.** Wheel slip, roller and tyre wear, and calibration errors accumulate;
  heading errors especially. Calibrate and expect to re-zero (`r`) for long paths.
- **Real-encoder mode needs commissioning.** It decodes the quadrature signals in a GPIO
  interrupt. Flash writes during recording can briefly delay interrupts, so counts may be
  missed at high pulse rates. The ESP32's hardware pulse counter (PCNT) avoids this and is the
  recommended upgrade if you see drift at higher speeds.
- **Replay speed** is limited by the serial link (about 11 KB/s at 115200 baud).
- **Flash space** is limited to about 1.4 MB, roughly 25 minutes of recording at 20 Hz.
  Flash has a finite number of write cycles, so avoid continuous long-term recording.
- The SVG export's scale ratio assumes the viewer's **Scale** field is 1.0 (millimetres).

---

## 12. Quick reference

| I want to …                       | Board                  | Viewer                  | Serial key |
|-----------------------------------|------------------------|-------------------------|------------|
| Start / pause recording           | short press            | –                       | `p`        |
| Replay the next recording         | long press (≥ 1 s)     | **Replay next**         | `d`        |
| Delete all recordings             | –                      | **Delete all files**    | `w`        |
| Reset position and heading        | –                      | **Clear path / center** | `r`        |
| Set the pot centre                | –                      | **Center pots**         | `c`        |
| Toggle steering                   | –                      | –                       | `s`        |
| Save the path as A3 SVG           | –                      | **Export**              | –          |

Data format: `t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg` · 115200 baud · 20 Hz · files `/rec_NNN.csv`
