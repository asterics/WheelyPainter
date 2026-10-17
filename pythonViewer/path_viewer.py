#!/usr/bin/env python3
"""
Live path viewer for the ESP32 wheel-odometry firmware (wheelchair, wheel encoders + gyro).

Reads lines of the form
    t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg
from a serial port, integrates dx/dy into a path and draws it autoscaled, with a small
wheelchair icon (two wheels + front triangle) at the current position showing the heading.

The older 6-column format (no heading) is still accepted; the heading is then estimated
from the direction of travel (cannot show turning on the spot or reversing).

Requirements:  pip install pyserial        (tkinter ships with the standard Python installer)
"""

import math
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports

DEFAULT_BAUD = 115200
MARGIN = 60               # canvas margin in screen pixels
MAX_DRAW_POINTS = 4000    # polyline is decimated above this
POLL_MS = 30

# Wheelchair drawn TRUE TO SCALE (dimensions in mm, top view). Keep CHAIR_TRACK_MM equal to
# TRACK_WIDTH_MM in the firmware, otherwise icon and path do not match.
CHAIR_TRACK_MM = 560.0        # distance between the two wheel contact points
CHAIR_WHEEL_W_MM = 50.0       # tyre width
CHAIR_WHEEL_D_MM = 600.0      # wheel diameter (24")
CHAIR_LENGTH_MM = 950.0       # approx. overall length, rear of wheels to tip of front triangle
CHAIR_PAD_MM = 700.0          # free space kept around the path so the chair is never cut off
MIN_VIEW_MM = 2500.0          # the view is never zoomed in further than this width
MIN_ICON_PX_LEN = 22.0        # when zoomed far out the icon is not drawn smaller than this
EXPORT_TOLERANCE_MM = 2.0     # SVG export: thin out points, max. deviation from the path (0 = keep all)
EXPORT_PAGE_MM = (420.0, 297.0)   # A3; orientation (landscape/portrait) is chosen automatically
EXPORT_MARGIN_MM = 15.0       # free border on the sheet
EXPORT_STROKE_MM = 0.5        # line width on the sheet


# --------------------------------------------------------------------------- serial thread
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_q):
        super().__init__(daemon=True)
        self.port, self.baud, self.q = port, baud, out_q
        self.stop_evt = threading.Event()
        self.ser = None

    def send(self, data: bytes):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
        except Exception:        # noqa: BLE001
            pass

    def run(self):
        try:
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baud
            ser.timeout = 0.1
            ser.dtr = False          # avoid resetting the board when the port is opened
            ser.rts = False
            ser.open()
        except Exception as e:       # noqa: BLE001
            self.q.put(("error", str(e)))
            return

        self.ser = ser
        self.q.put(("connected", self.port))
        buf = b""
        in_dump = False              # True while a recording is being replayed ('d' / long press)
        try:
            while not self.stop_evt.is_set():
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("ascii", "ignore").strip()
                    if not line:
                        continue
                    if line.startswith("#"):
                        if "BEGIN LOG" in line:
                            in_dump = True
                            m = re.search(r"BEGIN LOG\s+(\S+)", line)
                            self.q.put(("replay_begin", m.group(1) if m else "recording"))
                        elif "END LOG" in line:
                            in_dump = False
                            self.q.put(("replay_end", None))
                        elif not in_dump:                # skip the file header line inside a replay
                            self.q.put(("info", line.lstrip("# ")))
                        continue
                    parts = line.split(",")
                    if len(parts) not in (6, 7):
                        continue
                    try:
                        vals = [float(p) for p in parts]
                    except ValueError:
                        continue
                    heading = vals[6] if len(vals) == 7 else None
                    kind = "replay_sample" if in_dump else "sample"
                    self.q.put((kind, (vals[0], vals[1], vals[2], int(vals[5]), heading)))
        except Exception as e:       # noqa: BLE001
            self.q.put(("error", str(e)))
        finally:
            try:
                ser.close()
            except Exception:        # noqa: BLE001
                pass
            self.q.put(("closed", None))


# --------------------------------------------------------------------------- helpers
def nice_length(target):
    """Largest 1/2/5 x 10^k value that is <= target."""
    if target <= 0:
        return 1.0
    exp = 10 ** math.floor(math.log10(target))
    best = exp
    for m in (1, 2, 5, 10):
        if m * exp <= target:
            best = m * exp
    return best


def fmt(v):
    return f"{v:.0f}" if abs(v) >= 100 else f"{v:.2f}".rstrip("0").rstrip(".")


# Wheelchair icon in local coordinates [mm]: a = to the right, b = forward (axle at b = 0)
_t, _w, _d = CHAIR_TRACK_MM / 2, CHAIR_WHEEL_W_MM / 2, CHAIR_WHEEL_D_MM / 2
WHEEL_L = [(-_t - _w, -_d), (-_t + _w, -_d), (-_t + _w, _d), (-_t - _w, _d)]
WHEEL_R = [(_t - _w, -_d), (_t + _w, -_d), (_t + _w, _d), (_t - _w, _d)]
BODY = [(-_t + _w + 10, -200), (_t - _w - 10, -200), (_t - _w - 10, 250), (-_t + _w + 10, 250)]
FRONT = [(-200, 250), (200, 250), (0, 650)]   # triangle showing the front
AXLE = [(-_t, 0), (_t, 0)]


# --------------------------------------------------------------------------- GUI
class App:
    def __init__(self, root):
        self.root = root
        root.title("Wheelchair odometry path")
        root.geometry("900x720")

        self.q = queue.Queue()
        self.reader = None
        self.ports = {}
        self.last_info = ""
        self.flash_text = ""
        self.flash_until = 0.0
        self.replay_name = None              # name of the recording currently shown (None = live)
        self.replay_hint = ""                # e.g. "2/3"
        self.replay_active = False           # True while the file is still being received

        self.pts = [(0.0, 0.0)]
        self.x = self.y = 0.0
        self.heading = 0.0                   # degrees, 0 = up, clockwise positive (plot frame)
        self.bounds = [0.0, 0.0, 0.0, 0.0]   # minx, maxx, miny, maxy
        self.n_valid = 0
        self.samples = 0
        self.dirty = True

        self.build_ui()
        self.refresh_ports()
        self.poll()

    # ---- UI
    def build_ui(self):
        bar = ttk.Frame(self.root, padding=6)
        bar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(bar, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=28, state="readonly")
        self.port_box.pack(side=tk.LEFT, padx=(2, 2))
        ttk.Button(bar, text="⟳", width=3, command=self.refresh_ports).pack(side=tk.LEFT)

        ttk.Label(bar, text="Baud:").pack(side=tk.LEFT, padx=(8, 0))
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Entry(bar, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=2)

        self.btn_start = ttk.Button(bar, text="Start", command=self.start)
        self.btn_start.pack(side=tk.LEFT, padx=(12, 2))
        self.btn_stop = ttk.Button(bar, text="Stop", command=self.stop, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text="Clear path / center", command=self.clear).pack(side=tk.LEFT, padx=(12, 2))
        ttk.Button(bar, text="Export", command=self.export_svg).pack(side=tk.LEFT, padx=(12, 2))

        bar2 = ttk.Frame(self.root, padding=(6, 0, 6, 6))
        bar2.pack(side=tk.TOP, fill=tk.X)
        self.inv_x = tk.BooleanVar(value=False)
        self.inv_y = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar2, text="Invert X", variable=self.inv_x).pack(side=tk.LEFT)
        ttk.Checkbutton(bar2, text="Invert Y", variable=self.inv_y).pack(side=tk.LEFT, padx=6)
        ttk.Label(bar2, text="Scale:").pack(side=tk.LEFT, padx=(12, 0))
        self.scale_var = tk.StringVar(value="1.0")
        ttk.Entry(bar2, textvariable=self.scale_var, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Label(bar2, text="Chair size x:").pack(side=tk.LEFT, padx=(12, 0))
        self.icon_var = tk.StringVar(value="1.0")
        ttk.Entry(bar2, textvariable=self.icon_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(bar2, text="(units are mm; chair is drawn to scale)",
                  foreground="#666").pack(side=tk.LEFT, padx=8)

        bar3 = ttk.Frame(self.root, padding=(6, 0, 6, 6))
        bar3.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(bar3, text="Recordings on the ESP32:").pack(side=tk.LEFT)
        ttk.Button(bar3, text="Replay next", command=self.replay_next).pack(side=tk.LEFT, padx=(8, 2))
        ttk.Button(bar3, text="Delete all files", command=self.wipe_files).pack(side=tk.LEFT, padx=2)
        ttk.Label(bar3, text="(start/pause recording: button on the board or 'p'; replay = long press or 'd')",
                  foreground="#666").pack(side=tk.LEFT, padx=8)

        bar4 = ttk.Frame(self.root, padding=(6, 0, 6, 6))
        bar4.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(bar4, text="Gyro:").pack(side=tk.LEFT)
        ttk.Button(bar4, text="Calibrate gyro", command=self.calibrate_gyro).pack(side=tk.LEFT, padx=(8, 2))
        ttk.Button(bar4, text="Heading source", command=self.toggle_heading_source).pack(side=tk.LEFT, padx=2)
        ttk.Label(bar4, text="(calibrate with the chair standing still; heading source: gyro <-> encoders only)",
                  foreground="#666").pack(side=tk.LEFT, padx=8)

        self.canvas = tk.Canvas(self.root, bg="#fafafa", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda e: self.mark_dirty())

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(self.root, textvariable=self.status_var, anchor="w", padding=4,
                  relief=tk.SUNKEN).pack(side=tk.BOTTOM, fill=tk.X)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def refresh_ports(self):
        self.ports = {f"{p.device} - {p.description}": p.device for p in list_ports.comports()}
        names = list(self.ports.keys())
        self.port_box["values"] = names
        if names and self.port_var.get() not in names:
            self.port_var.set(names[0])

    # ---- control
    def start(self):
        if self.reader and self.reader.is_alive():
            return
        dev = self.ports.get(self.port_var.get())
        if not dev:
            self.status_var.set("No port selected")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self.status_var.set("Invalid baud rate")
            return
        self.reader = SerialReader(dev, baud, self.q)
        self.reader.start()
        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        self.status_var.set(f"Connecting to {dev} ...")

    def calibrate_gyro(self):
        """Ask the firmware to measure the gyro offset ('g' command). The chair must stand still."""
        if self.reader and self.reader.ser:
            self.reader.send(b"g")
            self.flash("Calibrating gyro - keep the chair still ...", 4.0)
        else:
            self.status_var.set("Not connected - press Start first")

    def toggle_heading_source(self):
        """Switch the firmware heading between gyro and encoders only ('h' command)."""
        if self.reader and self.reader.ser:
            self.reader.send(b"h")
        else:
            self.status_var.set("Not connected - press Start first")

    def replay_next(self):
        """Ask the firmware to dump the next recording ('d' command); it is drawn as it arrives."""
        if self.reader and self.reader.ser:
            self.reader.send(b"d")
            self.flash("Requesting next recording ...")
        else:
            self.status_var.set("Not connected - press Start first")

    def wipe_files(self):
        """Delete all recordings in the ESP32 flash ('w' command) after a confirmation."""
        if not (self.reader and self.reader.ser):
            self.status_var.set("Not connected - press Start first")
            return
        if messagebox.askyesno("Delete all files",
                               "Delete ALL recordings stored on the ESP32?\nThis cannot be undone."):
            self.reader.send(b"w")
            self.replay_hint = ""

    def flash(self, text, seconds=3.0):
        self.flash_text = text
        self.flash_until = time.time() + seconds
        self.status_var.set(text)

    def stop(self):
        if self.reader:
            self.reader.stop_evt.set()
            self.reader.join(timeout=0.5)
            self.reader = None
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.status_var.set("Stopped")

    def clear(self):
        # also reset position and heading in the firmware so that heading 0 = "up" again
        if self.reader:
            self.reader.send(b"r")
        self.reset_path()

    def reset_path(self):
        self.replay_name = None
        self.replay_active = False
        self.pts = [(0.0, 0.0)]
        self.x = self.y = 0.0
        self.heading = 0.0
        self.bounds = [0.0, 0.0, 0.0, 0.0]
        self.samples = 0
        self.mark_dirty()

    def on_close(self):
        self.stop()
        self.root.destroy()

    # ---- data
    def poll(self):
        try:
            for _ in range(3000):
                kind, val = self.q.get_nowait()
                if kind == "sample":
                    if self.replay_name is not None:     # live data again: leave the replay view
                        self.reset_path()
                    self.add_sample(*val)
                elif kind == "replay_begin":
                    self.reset_path()
                    self.replay_name = val
                    self.replay_active = True
                    self.flash(f"Replaying {val} ...", 5.0)
                elif kind == "replay_sample":
                    self.add_sample(*val)
                elif kind == "replay_end":
                    self.replay_active = False
                    self.flash(f"Replay finished: {self.replay_name}, {self.samples} samples", 6.0)
                    self.dirty = True
                elif kind == "connected":
                    self.status_var.set(f"Connected to {val}")
                elif kind == "info":
                    self.last_info = val
                    if val.startswith("replay "):
                        self.replay_hint = val[len("replay "):]
                    if val.startswith("status "):        # periodic status line
                        if time.time() >= self.flash_until and self.reader and not self.replay_active:
                            self.status_var.set(f"Connected - {val[len('status '):]}")
                    else:                                # messages from the firmware (recording, delete, ...)
                        self.flash(val, 4.0)
                elif kind == "error":
                    self.status_var.set(f"Error: {val}")
                    self.btn_start.config(state=tk.NORMAL)
                    self.btn_stop.config(state=tk.DISABLED)
                    self.reader = None
        except queue.Empty:
            pass

        if self.dirty:
            self.redraw()
        self.root.after(POLL_MS, self.poll)

    def add_sample(self, _t, dx, dy, n, hdg):
        try:
            k = float(self.scale_var.get())
        except ValueError:
            k = 1.0
        sx = -1.0 if self.inv_x.get() else 1.0
        sy = -1.0 if self.inv_y.get() else 1.0
        dx, dy = dx * k * sx, dy * k * sy

        if hdg is not None:
            # mirror the heading together with the axes
            h = hdg
            if self.inv_x.get():
                h = -h
            if self.inv_y.get():
                h = 180.0 - h
            self.heading = (h + 180.0) % 360.0 - 180.0
        elif math.hypot(dx, dy) > 0.2:
            # no heading from the firmware: estimate it from the direction of travel
            self.heading = math.degrees(math.atan2(dx, dy))

        self.x += dx
        self.y += dy
        self.n_valid = n
        self.samples += 1
        self.pts.append((self.x, self.y))
        b = self.bounds
        b[0], b[1] = min(b[0], self.x), max(b[1], self.x)
        b[2], b[3] = min(b[2], self.y), max(b[3], self.y)
        self.dirty = True

    def mark_dirty(self):
        self.dirty = True

    # ---- drawing
    def get_float(self, var, default):
        try:
            v = float(var.get())
            return v if v > 0 else default
        except ValueError:
            return default

    def view_params(self):
        """Padded path bounds shared by the screen drawing and the SVG export."""
        k = self.get_float(self.scale_var, 1.0)        # plot units per mm
        f = self.get_float(self.icon_var, 1.0)         # chair size factor
        pad = CHAIR_PAD_MM * k * f                     # free space so the chair is never cut off
        minx, maxx, miny, maxy = self.bounds
        minx, maxx, miny, maxy = minx - pad, maxx + pad, miny - pad, maxy + pad
        min_span = MIN_VIEW_MM * k                     # never zoom in further than this
        spanx = max(maxx - minx, min_span)
        spany = max(maxy - miny, min_span)
        cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
        return k, f, spanx, spany, cx, cy

    @staticmethod
    def chair_polys(ex, ey, heading_deg, px_per_mm):
        """Screen coordinates of the wheelchair parts; heading 0 = up, clockwise positive."""
        th = math.radians(heading_deg)
        s, c = math.sin(th), math.cos(th)

        def poly(points):
            out = []
            for a, b in points:
                a, b = a * px_per_mm, b * px_per_mm
                px = a * c + b * s           # plot-frame offset (x right)
                py = -a * s + b * c          # plot-frame offset (y up)
                out.extend((ex + px, ey - py))
            return out

        return {"body": poly(BODY), "wheel_l": poly(WHEEL_L), "wheel_r": poly(WHEEL_R),
                "axle": poly(AXLE), "front": poly(FRONT)}

    def draw_chair(self, ex, ey, heading_deg, px_per_mm):
        cv = self.canvas
        p = self.chair_polys(ex, ey, heading_deg, px_per_mm)
        cv.create_polygon(p["body"], fill="#b0bec5", outline="#37474f", width=1)
        cv.create_polygon(p["wheel_l"], fill="#263238", outline="")
        cv.create_polygon(p["wheel_r"], fill="#263238", outline="")
        cv.create_line(*p["axle"], fill="#37474f", width=2)
        cv.create_polygon(p["front"], fill="#c62828", outline="#7f0000", width=1)

    def redraw(self):
        self.dirty = False
        c = self.canvas
        c.delete("all")
        w, h = max(c.winfo_width(), 50), max(c.winfo_height(), 50)

        k, f, spanx, spany, cx, cy = self.view_params()
        scale = min((w - 2 * MARGIN) / spanx, (h - 2 * MARGIN) / spany)   # screen px per plot unit

        def to_screen(px, py):
            return w / 2 + (px - cx) * scale, h / 2 - (py - cy) * scale

        # origin cross
        ox, oy = to_screen(0, 0)
        c.create_line(ox - 10, oy, ox + 10, oy, fill="#bbb")
        c.create_line(ox, oy - 10, ox, oy + 10, fill="#bbb")

        # path (decimated if very long, last point always kept)
        n = len(self.pts)
        step = max(1, n // MAX_DRAW_POINTS)
        coords = []
        for i in range(0, n, step):
            coords.extend(to_screen(*self.pts[i]))
        if (n - 1) % step:
            coords.extend(to_screen(*self.pts[-1]))
        if len(coords) >= 4:
            c.create_line(*coords, fill="#1565c0", width=2, joinstyle=tk.ROUND)

        # start marker and wheelchair at the current position (true scale)
        sx, sy = to_screen(*self.pts[0])
        c.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill="#2e7d32", outline="")
        ex, ey = to_screen(self.x, self.y)
        px_per_mm = max(k * scale * f, MIN_ICON_PX_LEN / CHAIR_LENGTH_MM)
        self.draw_chair(ex, ey, self.heading, px_per_mm)

        # scale bar
        bar_units = nice_length((w * 0.2) / scale)
        bar_px = bar_units * scale
        bx, by = 20, h - 20
        c.create_line(bx, by, bx + bar_px, by, width=3)
        c.create_line(bx, by - 5, bx, by + 5)
        c.create_line(bx + bar_px, by - 5, bx + bar_px, by + 5)
        c.create_text(bx, by - 12, text=fmt(bar_units), anchor="w")

        minx, maxx, miny, maxy = self.bounds
        c.create_text(12, 12, anchor="nw", font=("TkDefaultFont", 10),
                      text=f"x = {self.x:.1f}   y = {self.y:.1f}   heading = {self.heading:.1f}°   "
                           f"samples = {self.samples}\n"
                           f"extent: {fmt(maxx - minx)} x {fmt(maxy - miny)}")
        if self.replay_name is not None:
            label = f"REPLAY {self.replay_name}"
            if self.replay_hint:
                label += f" ({self.replay_hint})"
            label += " - receiving ..." if self.replay_active else " - finished"
            c.create_text(12, 48, anchor="nw", font=("TkDefaultFont", 10, "bold"), fill="#c62828", text=label)

    # ---- SVG export (path only)
    @staticmethod
    def simplify(points, tol):
        """Ramer-Douglas-Peucker: drop points that deviate less than `tol` from the path."""
        n = len(points)
        if tol <= 0 or n < 3:
            return list(points)
        keep = [False] * n
        keep[0] = keep[-1] = True
        stack = [(0, n - 1)]
        while stack:
            a, b = stack.pop()
            ax, ay = points[a]
            bx, by = points[b]
            dx, dy = bx - ax, by - ay
            norm = math.hypot(dx, dy)
            best, idx = -1.0, -1
            for i in range(a + 1, b):
                px, py = points[i]
                if norm == 0:
                    d = math.hypot(px - ax, py - ay)
                else:
                    d = abs(dy * (px - ax) - dx * (py - ay)) / norm
                if d > best:
                    best, idx = d, i
            if best > tol:
                keep[idx] = True
                stack.append((a, idx))
                stack.append((idx, b))
        return [pt for pt, k in zip(points, keep) if k]

    def export_svg(self):
        if len(self.pts) < 2:
            self.status_var.set("Nothing to export yet")
            return
        path = filedialog.asksaveasfilename(
            title="Export path as SVG", defaultextension=".svg",
            filetypes=[("SVG image", "*.svg")], initialfile=time.strftime("path_%Y%m%d_%H%M%S.svg"))
        if not path:
            return

        k = self.get_float(self.scale_var, 1.0)
        pts = self.simplify(self.pts, EXPORT_TOLERANCE_MM * k)

        minx, maxx, miny, maxy = self.bounds
        pw, ph = max(maxx - minx, 1e-6), max(maxy - miny, 1e-6)

        # A3 sheet, landscape if the path is wider than tall, otherwise portrait
        long_side, short_side = EXPORT_PAGE_MM
        page_w, page_h = (long_side, short_side) if pw >= ph else (short_side, long_side)

        # uniform scale: path units -> mm on the sheet, centred inside the margins
        sc = min((page_w - 2 * EXPORT_MARGIN_MM) / pw, (page_h - 2 * EXPORT_MARGIN_MM) / ph)
        off_x = (page_w - pw * sc) / 2
        off_y = (page_h - ph * sc) / 2

        # y axis is flipped: the viewer has y up, SVG has y down
        coords = [f"{(x - minx) * sc + off_x:.3f},{(maxy - y) * sc + off_y:.3f}" for x, y in pts]
        d = "M" + " L".join(coords)

        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_w:.0f}mm" height="{page_h:.0f}mm" '
               f'viewBox="0 0 {page_w:.0f} {page_h:.0f}">\n'
               f'<path id="path" d="{d}" fill="none" stroke="#000000" stroke-width="{EXPORT_STROKE_MM}" '
               f'stroke-linejoin="round" stroke-linecap="round"/>\n</svg>\n')
        ratio = 1.0 / (sc * k)        # real mm per mm on paper (path units = mm * k)
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(svg)
            self.flash(f"Exported A3 {'landscape' if page_w > page_h else 'portrait'}, scale 1:{ratio:.0f}, "
                       f"{len(pts)} of {len(self.pts)} points -> {path}", 6.0)
        except OSError as e:
            self.status_var.set(f"Export failed: {e}")


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
