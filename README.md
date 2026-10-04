# Optical Flow Odometry – ESP32-CAM firmware + Path Viewer

Dead-reckoning of a moving platform with a **downward-looking camera** (AZ-Delivery ESP32-CAM, OV2640).
The ESP32 compares successive camera frames of the floor, measures how the floor texture shifts,
converts that to millimetres, streams it over a serial port and logs it to flash. A small Python/Tk
program draws the path live, replays the log from the board, and exports the path as an SVG.

> This is the **optical flow version**. A wheel-encoder version of the firmware (with a record
> button, LED and multiple recording files) lives in a separate branch. Both use the same
> `path_viewer.py`.

```
 floor ──► OV2640 ──► ESP32-CAM ── serial (FTDI) ──► path_viewer.py
           160x120    block matching,                 (live path, replay,
           grayscale  mm conversion,                   delete log, SVG export)
                      log to flash (/flow.csv)
```

## Contents

1. [Repository layout](#1-repository-layout)
2. [Hardware and mounting](#2-hardware-and-mounting)
3. [Installation](#3-installation)
4. [How it works](#4-how-it-works)
5. [Firmware usage](#5-firmware-usage)
6. [Data format](#6-data-format)
7. [Viewer usage with this firmware](#7-viewer-usage-with-this-firmware)
8. [Calibration](#8-calibration)
9. [Configuration reference](#9-configuration-reference)
10. [Troubleshooting](#10-troubleshooting)
11. [Known limitations](#11-known-limitations)
12. [Quick reference](#12-quick-reference)

---

## 1. Repository layout

```
esp32cam_flow/
├── platformio.ini      PlatformIO project (ESP32-CAM, Arduino framework, PSRAM, LittleFS)
├── src/
│   └── main.cpp        Firmware: camera, optical flow, serial output, flash log
├── path_viewer.py      PC tool: live path, replay, delete log, SVG export
└── README.md
```

---

## 2. Hardware and mounting

### 2.1 Board

[AZ-Delivery ESP32-CAM module](https://www.az-delivery.de/en/products/esp32-cam-modul-esp32-wifi-bluetooth-modul-inklusive-kamera)
(AI-Thinker pinout): ESP32, 4 MB flash, PSRAM, OV2640 camera. The
[datasheet](https://cdn.shopify.com/s/files/1/1509/1638/files/ESP32-Cam_datasheet.pdf?v=1689236064)
shows the pinout. The board has **no USB connector**, so you need a USB-to-serial (FTDI) adapter
with **3.3 V logic**.

### 2.2 Wiring the FTDI adapter

| FTDI adapter | ESP32-CAM | Notes                                                        |
|--------------|-----------|--------------------------------------------------------------|
| TX           | U0R (GPIO3) |                                                            |
| RX           | U0T (GPIO1) |                                                            |
| GND          | GND       |                                                              |
| 5 V          | 5V        | Use a supply that can deliver at least 500 mA                |
| –            | GPIO0 → GND | **Only while flashing** (see below)                        |

Poor power is the most common cause of random resets ("brownout"). Use a good USB port or an
external 5 V supply, and short, thick wires.

### 2.3 Camera mounting

- Point the camera **straight down** at the floor, rigidly attached to the platform.
- Mount it so that the **top of the image points forward** and the right of the image points to the
  platform's right. If it is mounted differently, use **Invert X / Invert Y** in the viewer.
- A **fixed, known height** matters: the distance in millimetres per pixel is proportional to
  the height. 200–400 mm is a good start (see [calibration](#8-calibration)).
- Light the floor well and evenly. Short exposure reduces motion blur
  (`MANUAL_EXPOSURE`, see [configuration](#9-configuration-reference)). Avoid shadows that move
  with the platform.
- The floor needs **visible texture** (carpet, wood grain, tiles with grout). Smooth, uniform
  or very shiny floors cannot be tracked.

---

## 3. Installation

### 3.1 Firmware (PlatformIO)

**Option A – VS Code:** install the PlatformIO IDE extension, open the `esp32cam_flow` folder and click
**Upload** (→) in the status bar.

**Option B – command line:**

```bash
pip install platformio
cd esp32cam_flow
pio run -t upload          # build and flash
pio device monitor         # optional: serial terminal at 115200 baud
```

**Flashing procedure (no USB chip on the board):**

1. Connect GPIO0 to GND.
2. Press the board's reset button (or power-cycle) so it enters the bootloader.
3. Start the upload.
4. When it has finished, **remove the GPIO0–GND wire** and press reset to run the firmware.

Notes:

- The first build downloads the ESP32 toolchain and needs internet access.
- The flash file system (LittleFS) is formatted automatically on the first boot.
- A normal firmware upload does not erase the log file.

### 3.2 Viewer (Python)

Requirements: **Python 3.8 or newer**, `pyserial` and `tkinter`.

```bash
pip install pyserial
python path_viewer.py
```

`tkinter` ships with the Windows/macOS installers. On Linux: `sudo apt install python3-tk`.
Close other programs that use the same port (PlatformIO monitor, …); only one program can use
it at a time.

---

## 4. How it works

1. The camera delivers **160×120 8-bit grayscale** frames (QQVGA). The newest frame is copied
   into internal RAM, which is much faster than PSRAM.
2. The previous frame is divided into a **3×3 grid of 24×24 pixel blocks**. Blocks with too
   little texture are skipped.
3. For each block the firmware searches the current frame within **±8 px** for the best match
   (sum of absolute differences, every second pixel), then refines the result to
   **sub-pixel** accuracy with a parabola fit. Ambiguous or bad matches are rejected.
4. The **median** of the valid blocks gives the frame-to-frame shift. With fewer than 3 valid
   blocks the frame reports no movement (`n_valid` < 3).
5. The shift is converted from pixels to **millimetres on the floor** and the sign is inverted,
   because the floor moves opposite to the platform:

```
dx_mm = −(floor shift in x, px) · MM_PER_PX        platform to the right is positive
dy_mm = +(floor shift in y, px) · MM_PER_PX        platform forward ("up") is positive

MM_PER_PX = 2 · CAM_HEIGHT_MM · tan(CAM_HFOV_DEG / 2) / 160
```

(the image y axis points down, so a floor moving down in the image means the platform moves forward).

6. The camera task runs on core 1; a logger task on core 0 writes the samples to flash.

**Maximum speed:** the search range is ±8 px per frame, i.e. about `8 · MM_PER_PX · fps`
(e.g. 8 · 2.4 mm · 25 fps ≈ 480 mm/s at 300 mm height). Faster movement cannot be tracked.

---

## 5. Firmware usage

The firmware starts streaming and logging immediately after boot. There is no button or LED
in this version.

### 5.1 Serial commands

Open the port at **115200 baud, 8N1** and send single characters (no newline needed).

| Key | Function                                                                                |
|-----|-----------------------------------------------------------------------------------------|
| `p` | Toggle the live serial output (the log keeps recording)                                 |
| `d` | Replay: dump the log file over serial                                                   |
| `w` | Delete the log file and start a fresh one (`x` does the same – old name)                |
| `r` | Reset the accumulated position to 0                                                     |
| `c` | Not used in this version (answers with a note; the viewer's *Center pots* button does nothing) |

### 5.2 Messages from the firmware

Lines starting with `#` are comments. Every 2 seconds (while the live output is on):

```
# fps=24.8 flow=3120 us/frame dropped=0
```

- `fps` – frames per second processed.
- `flow` – average processing time of the optical flow per frame in µs.
- `dropped` – samples that could not be queued for the logger (should stay 0).

### 5.3 The log file

- All samples are appended to **`/flow.csv`** in flash, from boot on, **across reboots**.
  Each boot adds a comment line `# session start; …`.
- Data is written in blocks (about 4 KB or every 2 s). After a power loss at most the last 2 s
  are lost.
- Logging stops by itself at about **1 MB** (a message is printed). Send `w` to start over.
- A line is about 40 bytes. At ~25 fps that is about 1 KB/s, so roughly 17 minutes per MB.
- **Replay** (`d`) prints `# replay 1/1`, then the file between
  `# ---- BEGIN LOG flow.csv ----` and `# ---- END LOG ----`. At 115200 baud 1 MB takes about 90 s.
  The live output is paused during the replay and resumed afterwards.

---

## 6. Data format

One line per frame, comma separated, the same on the serial port and in the log:

```
t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid
48210,7.153,0.018,1250.40,18.62,9
```

| Column         | Meaning                                                                    |
|----------------|----------------------------------------------------------------------------|
| `t_ms`         | Time since boot in milliseconds                                            |
| `dx_mm, dy_mm` | Platform motion since the previous frame (+x right, +y forward)            |
| `x_mm, y_mm`   | Accumulated motion since boot or the last `r`                              |
| `n_valid`      | Number of image blocks that produced a valid match (0–9)                   |

There is **no heading column**. The camera only measures translation, so this version does not
know the platform's orientation. See [limitations](#11-known-limitations).

---

## 7. Viewer usage with this firmware

Start `python path_viewer.py`, select the port (⟳ to refresh) and press **Start**. The path is
drawn live and autoscaled.

| Control                 | With this firmware                                                                          |
|-------------------------|---------------------------------------------------------------------------------------------|
| **Start / Stop**        | Open / close the serial port (DTR/RTS stay low, so the board is not reset)                  |
| **Clear path / center** | Clears the drawing and sends `r` (the position restarts at 0). The log file is not touched. |
| **Replay next**         | Sends `d`: the log is replayed and drawn (red label `REPLAY flow.csv (1/1)`)                |
| **Delete all files**    | After a confirmation sends `w`: the log is deleted                                          |
| **Export**              | Saves the path as A3 SVG (see below)                                                        |
| **Invert X / Invert Y** | Mirror the drawing – use this if the camera is mounted rotated or mirrored                  |
| **Scale**               | Leave at **1.0**: the firmware already sends millimetres                                    |
| **Chair size x**        | Size factor of the wheelchair icon                                                          |
| **Center pots**         | Not used (encoder version only)                                                             |

Details:

- **Heading:** with 6-column data the viewer estimates the heading from the **direction of
  travel**. The wheelchair icon therefore shows where the platform is moving, not how it is
  oriented: it points backward when reversing and sideways when moving sideways.
- **Status bar:** shows the `fps` line from the firmware and short messages (for a few seconds).
- **Replay:** the viewer clears the display, draws the file while it arrives and keeps it on screen.
  Live data replaces it again as soon as it arrives. The replay starts at (0,0) and includes
  all recorded sessions one after another.
- **Icon scale:** the icon is drawn to scale for a wheelchair (`CHAIR_*` constants at the top of
  `path_viewer.py`). For a smaller platform, adjust them or use *Chair size x*.
- **Export:** only the path is exported (transparent background), fitted to an **A3** sheet
  (landscape if the path is wider than tall, otherwise portrait) with a 15 mm border,
  1 user unit = 1 mm, 0.5 mm line. The status bar shows the drawing scale
  (e.g. `scale 1:13`) and how many points remain after thinning (points that deviate less than 2 mm
  are dropped). In Inkscape press `N` to edit the nodes, `Ctrl+A` then `Shift+U`/`Shift+S`
  to turn them into smooth curves, or `Ctrl+L` (*Path → Simplify*) to simplify.

---

## 8. Calibration

The conversion from pixels to millimetres depends on the **camera height and the lens' field of
view**. Both are only estimates (`CAM_HEIGHT_MM = 300`, `CAM_HFOV_DEG = 65`), so calibrate:

1. Reset the position (`r` or **Clear path / center**).
2. Push the platform **straight forward** for a known distance, e.g. 2 m.
3. Read `y` in the viewer (top-left text) and correct:

```
new CAM_HEIGHT_MM = old CAM_HEIGHT_MM × (true distance / measured distance)
```

4. Re-flash. Check once more with a sideways movement; if the two axes disagree the camera is
   probably not pointing straight down, or the image aspect differs.

Only the product `height × tan(FOV/2)` matters, so correcting the height is enough.
Calibrate at the height the camera will really have, and keep it constant when driving.

---

## 9. Configuration reference

Constants at the top of `src/main.cpp`:

| Constant                 | Default      | Meaning                                                            |
|--------------------------|--------------|--------------------------------------------------------------------|
| `W`, `H`                 | 160, 120     | Image size (QQVGA). Changing it also needs a new frame size in `initCamera()` |
| `BS`                     | 24           | Block size in pixels                                               |
| `SR`                     | 8            | Search range ± pixels per frame (limits the maximum speed)         |
| `STEP`                   | 2            | Pixel subsampling inside the matching (1 = more precise, slower)   |
| `BX[]`, `BY[]`           | 3×3 grid     | Block origins; keep `SR` pixels of margin to the image border      |
| `TEX_MIN`                | 400          | Minimum texture of a block; lower to track smoother floors         |
| `MAX_SAD`                | 25 per sample| Reject matches that are worse than this                            |
| `MIN_VALID_BLOCKS`       | 3            | Minimum valid blocks for a valid frame                             |
| `CAM_HEIGHT_MM`          | 300          | Camera height above the floor – **calibrate**                      |
| `CAM_HFOV_DEG`           | 65           | Horizontal field of view of the lens                               |
| `MANUAL_EXPOSURE`        | 0            | 0 = auto; otherwise exposure value (0–1200), shorter = less blur   |
| `MANUAL_GAIN`            | 10           | Gain (0–30), only used with manual exposure                        |
| `LOG_FLUSH_MS`           | 2000         | Maximum time between flash writes                                  |
| `LOG_MAX_BYTES`          | 1000000      | Log size limit                                                     |

Camera pins follow the AI-Thinker pinout (`PWDN_GPIO_NUM` … `PCLK_GPIO_NUM`).

---

## 10. Troubleshooting

| Problem                                          | What to check                                                                           |
|--------------------------------------------------|-----------------------------------------------------------------------------------------|
| Upload fails ("Failed to connect")               | GPIO0 must be connected to GND during upload; reset the board first; check TX/RX not swapped |
| Board does not start after flashing              | Remove the GPIO0–GND wire and press reset                                               |
| Random resets / "Brownout detector" message      | Power: use ≥ 500 mA at 5 V and short wires                                              |
| `camera init failed: 0x…`                        | Camera ribbon cable not seated; power problem; wrong board selection                    |
| Viewer shows nothing                             | Wrong port/baud, live output toggled off (send `p` again), another program holds the port |
| No movement although the platform moves          | Floor without texture (`n_valid` low), too dark, camera out of focus; try lowering `TEX_MIN` |
| Movement is jittery or jumps                     | Motion blur (use `MANUAL_EXPOSURE`, more light), repetitive floor pattern, speed above the maximum |
| Path scale is wrong                              | Calibrate `CAM_HEIGHT_MM` ([section 8](#8-calibration))                                 |
| Path is mirrored or rotated                      | Use **Invert X / Invert Y**; check the camera orientation                               |
| Old log shows a huge or tiny path                | A log from an older firmware used pixels. Delete it (**Delete all files** / `w`)        |
| `ModuleNotFoundError: serial`                    | `pip install pyserial` (not the package named `serial`)                                 |
| `ModuleNotFoundError: tkinter`                   | Linux: `sudo apt install python3-tk`                                                    |

---

## 11. Known limitations

- **Translation only.** The camera measures shifts, not rotation. If the platform turns, the
  motion is still integrated in the **camera frame**, so the path is only correct while the
  platform keeps its orientation (e.g. a cart that is pushed sideways and forward) or while turns are
  small. For a vehicle that steers, wheel odometry is the better choice.
- **No true heading.** The viewer's wheelchair icon shows the direction of travel only.
- **Floors matter.** Featureless, very glossy, or strongly repetitive surfaces cause missing or
  false matches; this is the main reason for jitter and dropouts.
- **Speed limit** of about 8 px per frame (see [section 4](#4-how-it-works)).
- **Height changes** (bumps, tilt) change the scale directly.
- **Drift** accumulates, as in any dead-reckoning. Re-zero with `r` for long runs.
- The log keeps recording from boot. Flash has limited write cycles, so don't leave it running
  indefinitely.
- Old logs written by earlier versions of this firmware are in pixels and should be deleted.

---

## 12. Quick reference

| I want to …                      | Viewer                  | Serial key |
|----------------------------------|-------------------------|------------|
| Pause / resume the live output   | –                       | `p`        |
| Replay the log                   | **Replay next**         | `d`        |
| Delete the log                   | **Delete all files**    | `w`        |
| Reset the position               | **Clear path / center** | `r`        |
| Save the path as A3 SVG          | **Export**              | –          |

Data format: `t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid` · 115200 baud · log file `/flow.csv`
