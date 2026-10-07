"""Tk desktop view for the local V5 program simulator (standard library only)."""
from __future__ import annotations

import copy
import ctypes
import json
import math
import sys
from pathlib import Path
from typing import Any
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


BG = "#101721"
PANEL = "#19232f"
FIELD = "#24313c"
TEXT = "#e6edf3"
MUTED = "#9aafbd"
CYAN = "#54cfdd"
GREEN = "#8ad8a3"
AMBER = "#ffcb70"


def enable_dpi_awareness():
    """Keep Windows desktop coordinates and Tk field pixels on the same scale."""
    if sys.platform == "win32":
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass


def _merge(base, incoming):
    result = copy.deepcopy(base)
    for key, value in incoming.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


class SimulatorApp:
    """Render a SimulatorSession without executing robot code in the UI process."""

    def __init__(self, root, config, program_path):
        self.root = root
        self.config = copy.deepcopy(config)
        self.program_path = Path(program_path)
        self.session = None
        self.paused = False
        self.snapshot = None
        self.world: dict[str, Any] = {"x_mm": -1000.0, "y_mm": -1000.0, "heading_deg": 0.0}
        self.gps = None
        self.target = None
        self.trail = []
        self.dragging = False
        self._closed = False
        self._poll_token = None
        self._log_history = []
        self._brain_text = None
        self._transform = None
        self.start_x = tk.StringVar(value="-1000")
        self.start_y = tk.StringVar(value="-1000")
        self.start_heading = tk.StringVar(value="0")
        self.heading_slider = tk.DoubleVar(value=0)
        self.speed = tk.StringVar(value="1")
        self.gps_mode = tk.StringVar(value="realistic")
        self.status = tk.StringVar(value="Ready — choose a starting pose, then Run")
        self.readouts = {name: tk.StringVar(value="—") for name in
                         ("position", "heading", "gps", "motors", "motion", "clock", "collision")}
        self.root.title("Flex • VEX V5 autonomous simulator")
        self.root.geometry("%dx%d" % (min(1360, self.root.winfo_screenwidth() - 60),
                                      min(900, self.root.winfo_screenheight() - 80)))
        self.root.minsize(1000, 680)
        self.root.configure(bg=BG)
        self._style()
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._update_controls()
        self._poll_token = self.root.after(40, self._poll)

    def _style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", background=PANEL, foreground=TEXT, font=("Segoe UI", 10),
                        bordercolor="#354653", lightcolor=PANEL, darkcolor=PANEL)
        style.configure("TFrame", background=PANEL)
        style.configure("Outer.TFrame", background=BG)
        style.configure("TLabel", background=PANEL, foreground=TEXT)
        style.configure("Muted.TLabel", foreground=MUTED, font=("Segoe UI", 9))
        style.configure("Title.TLabel", background=BG, font=("Segoe UI Semibold", 20))
        style.configure("Subtitle.TLabel", background=BG, foreground=MUTED)
        style.configure("Section.TLabel", foreground=CYAN, font=("Segoe UI Semibold", 9))
        style.configure("TButton", padding=(11, 8))
        style.map("TButton", background=[("active", "#304553"), ("disabled", "#202b35")],
                  foreground=[("disabled", "#657583")])
        style.configure("Run.TButton", background="#126675", foreground="white")
        style.map("Run.TButton", background=[("active", "#197f8f"), ("disabled", "#202b35")])
        style.configure("TEntry", fieldbackground="#0f1822", foreground=TEXT, insertcolor=TEXT,
                        padding=5)
        style.configure("TSpinbox", fieldbackground="#0f1822", foreground=TEXT,
                        arrowcolor=TEXT, insertcolor=TEXT, padding=4)
        style.configure("TCombobox", fieldbackground="#0f1822", foreground=TEXT,
                        arrowcolor=TEXT, padding=4)
        style.map("TCombobox", fieldbackground=[("readonly", "#0f1822")],
                  selectbackground=[("readonly", "#0f1822")],
                  selectforeground=[("readonly", TEXT)])
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=(16, 9), background="#202b36")
        style.map("TNotebook.Tab", background=[("selected", PANEL)],
                  foreground=[("selected", CYAN)])
        style.configure("TScale", background=PANEL, troughcolor="#30414d")
        style.configure("TLabelframe", background=PANEL)
        style.configure("TLabelframe.Label", foreground=CYAN)
        self.root.option_add("*TCombobox*Listbox.background", PANEL)
        self.root.option_add("*TCombobox*Listbox.foreground", TEXT)

    def _build(self):
        outer = ttk.Frame(self.root, style="Outer.TFrame", padding=18)
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer, style="Outer.TFrame")
        header.pack(fill="x", pady=(0, 14))
        title = ttk.Frame(header, style="Outer.TFrame")
        title.pack(side="left")
        ttk.Label(title, text="Flex field simulator", style="Title.TLabel").pack(anchor="w")
        ttk.Label(title, text="Local V5 Python preview  ·  " + self.program_path.name,
                  style="Subtitle.TLabel").pack(anchor="w", pady=(3, 0))
        self.settings_button = ttk.Button(header, text="Model settings…", command=self.open_settings)
        self.settings_button.pack(side="right")
        toolbar = ttk.Frame(outer, padding=9)
        toolbar.pack(fill="x", pady=(0, 12))
        self.run_button = ttk.Button(toolbar, text="▶  Run", style="Run.TButton", command=self.start)
        self.pause_button = ttk.Button(toolbar, text="Pause", command=self.toggle_pause)
        self.step_button = ttk.Button(toolbar, text="Step", command=self.step)
        self.stop_button = ttk.Button(toolbar, text="Stop", command=self.stop)
        self.reset_button = ttk.Button(toolbar, text="Reset", command=self.reset)
        for button in (self.run_button, self.pause_button, self.step_button,
                       self.stop_button, self.reset_button):
            button.pack(side="left", padx=(0, 7))
        ttk.Label(toolbar, textvariable=self.readouts["clock"], style="Muted.TLabel").pack(side="right", padx=10)
        content = ttk.Frame(outer, style="Outer.TFrame")
        content.pack(fill="both", expand=True)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=0)
        content.rowconfigure(0, weight=1)
        field_panel = ttk.Frame(content, padding=5)
        field_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        self.canvas = tk.Canvas(field_panel, bg=BG, highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._drag_start)
        self.canvas.bind("<B1-Motion>", self._drag_move)
        self.canvas.bind("<ButtonRelease-1>", self._drag_end)
        ttk.Label(field_panel, text="Drag robot when stopped  ·  Cyan: physical robot  ·  Green/amber: GPS estimate",
                  style="Muted.TLabel", padding=(10, 8)).pack(anchor="w")
        sidebar = ttk.Notebook(content, width=348)
        sidebar.grid(row=0, column=1, sticky="nsew")
        control_tab = ttk.Frame(sidebar)
        control_canvas = tk.Canvas(control_tab, bg=PANEL, highlightthickness=0, width=330)
        control_scroll = ttk.Scrollbar(control_tab, orient="vertical", command=control_canvas.yview)
        control_scroll.pack(side="right", fill="y")
        control_canvas.pack(side="left", fill="both", expand=True)
        control_canvas.configure(yscrollcommand=control_scroll.set)
        control = ttk.Frame(control_canvas, padding=16)
        control_window = control_canvas.create_window(0, 0, anchor="nw", window=control)
        control.bind("<Configure>", lambda event: control_canvas.configure(scrollregion=control_canvas.bbox("all")))
        control_canvas.bind("<Configure>", lambda event: control_canvas.itemconfigure(control_window, width=event.width))
        def scroll_controls(event):
            if str(event.widget).startswith(str(control_tab)):
                control_canvas.yview_scroll(-int(event.delta / 120), "units")
        self.root.bind("<MouseWheel>", scroll_controls, add="+")
        brain = ttk.Frame(sidebar, padding=14)
        sidebar.add(control_tab, text="Controls")
        sidebar.add(brain, text="Brain & console")
        self._build_controls(control)
        self._build_brain(brain)
        footer = ttk.Frame(outer, style="Outer.TFrame")
        footer.pack(fill="x", pady=(12, 0))
        ttk.Label(footer, textvariable=self.status, style="Subtitle.TLabel",
                  wraplength=1040).pack(anchor="w")

    def _build_controls(self, parent):
        self._section(parent, "STARTING POSITION")
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(6, 10))
        self.pose_entries = []
        for label, variable in (("X (mm)", self.start_x), ("Y (mm)", self.start_y)):
            frame = ttk.Frame(row)
            frame.pack(side="left", fill="x", expand=True, padx=(0, 8))
            ttk.Label(frame, text=label, style="Muted.TLabel").pack(anchor="w")
            entry = ttk.Entry(frame, textvariable=variable, width=10)
            entry.pack(fill="x", pady=(4, 0))
            entry.bind("<Return>", lambda event: self.apply_start())
            entry.bind("<FocusOut>", lambda event: self.apply_start(quiet=True))
            self.pose_entries.append(entry)
        row = ttk.Frame(parent)
        row.pack(fill="x")
        ttk.Label(row, text="Heading (°)", style="Muted.TLabel").pack(side="left")
        self.heading_entry = ttk.Spinbox(row, from_=0, to=359.9, increment=5, width=8,
                                         textvariable=self.start_heading, command=self.apply_start)
        self.heading_entry.pack(side="right")
        self.heading_entry.bind("<Return>", lambda event: self.apply_start())
        self.heading_entry.bind("<FocusOut>", lambda event: self.apply_start(quiet=True))
        self.pose_entries.append(self.heading_entry)
        self.heading_scale = ttk.Scale(parent, from_=0, to=359.9, variable=self.heading_slider,
                                        command=self._heading_changed)
        self.heading_scale.pack(fill="x", pady=8)
        ttk.Label(parent, text="0° = up (+Y)   ·   90° = right (+X)", style="Muted.TLabel").pack(anchor="w")
        self._section(parent, "PLAYBACK", top=22)
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(8, 6))
        ttk.Label(row, text="Simulation speed").pack(side="left")
        speed = ttk.Combobox(row, textvariable=self.speed, values=("0.5", "1", "2", "4"),
                             width=6, state="readonly")
        speed.pack(side="right")
        speed.bind("<<ComboboxSelected>>", self._speed_changed)
        self._section(parent, "GPS SENSOR", top=18)
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(8, 8))
        ttk.Label(row, text="Readings").pack(side="left")
        self.mode_combo = ttk.Combobox(row, textvariable=self.gps_mode,
                                      values=("realistic", "ideal"), width=12, state="readonly")
        self.mode_combo.pack(side="right")
        ttk.Label(parent, text="Realistic adds estimated noise, latency and loss of visual lock during fast turns.",
                  style="Muted.TLabel", wraplength=290).pack(anchor="w")
        self.dropout_button = ttk.Button(parent, text="Inject GPS loss (1 second)", command=self.inject_dropout)
        self.dropout_button.pack(fill="x", pady=(10, 0))
        self._section(parent, "LIVE TELEMETRY", top=20)
        for label, key in (("Position", "position"), ("Heading", "heading"), ("GPS quality", "gps"),
                           ("Motors L / R", "motors"), ("Motion", "motion"), ("Field contact", "collision")):
            line = ttk.Frame(parent)
            line.pack(fill="x", pady=4)
            ttk.Label(line, text=label, style="Muted.TLabel").pack(side="left")
            ttk.Label(line, textvariable=self.readouts[key], font=("Consolas", 9)).pack(side="right")
        ttk.Separator(parent).pack(fill="x", pady=(18, 10))
        ttk.Label(parent, text="Physical dimensions and mass are editable\nestimates. Check Model settings before tuning.",
                  style="Muted.TLabel").pack(anchor="w")

    @staticmethod
    def _section(parent, text, top=0):
        ttk.Label(parent, text=text, style="Section.TLabel").pack(anchor="w", pady=(top, 0))

    def _build_brain(self, parent):
        self._section(parent, "V5 BRAIN SCREEN")
        self.brain_display = tk.Text(parent, height=11, width=33, bg="#090f14", fg=CYAN,
                                     font=("Consolas", 10), relief="flat", padx=10, pady=12,
                                     state="disabled", wrap="none")
        self.brain_display.pack(fill="x", pady=(10, 20))
        self._section(parent, "PROGRAM CONSOLE")
        frame = ttk.Frame(parent)
        frame.pack(fill="both", expand=True, pady=(10, 0))
        self.console = tk.Text(frame, width=33, bg="#0e1720", fg=TEXT, font=("Consolas", 9),
                               relief="flat", padx=8, pady=8, state="disabled", wrap="word")
        self.console.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(frame, command=self.console.yview)
        scrollbar.pack(side="right", fill="y")
        self.console.configure(yscrollcommand=scrollbar.set)

    def _running(self):
        return self.session is not None and self.session.alive

    def _update_controls(self):
        running = self._running()
        for widget in (self.run_button, self.settings_button, *self.pose_entries, self.heading_scale):
            widget.configure(state="disabled" if running else "normal")
        self.mode_combo.configure(state="disabled" if running else "readonly")
        for widget in (self.pause_button, self.stop_button, self.dropout_button):
            widget.configure(state="normal" if running else "disabled")
        self.step_button.configure(state="normal" if running and self.paused else "disabled")
        self.pause_button.configure(text="Resume" if self.paused else "Pause")

    def _starting_pose(self) -> dict[str, float]:
        values = [float(var.get()) for var in (self.start_x, self.start_y, self.start_heading)]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Use finite numbers for X, Y and heading.")
        width = self.config["field"]["width_mm"] / 2
        height = self.config["field"]["height_mm"] / 2
        if abs(values[0]) > width or abs(values[1]) > height:
            raise ValueError("The starting position must be inside the field.")
        return dict(zip(("x_mm", "y_mm", "heading_deg"), (values[0], values[1], values[2] % 360)))

    def apply_start(self, quiet=False):
        if self._running():
            return
        try:
            pose = self._starting_pose()
        except (ValueError, tk.TclError) as error:
            if not quiet:
                self.status.set(str(error))
            return
        self.world = pose
        self.start_heading.set("%.1f" % pose["heading_deg"])
        self.heading_slider.set(pose["heading_deg"])
        self.gps = None
        self.trail = []
        self._telemetry()
        self.redraw()

    def _heading_changed(self, value):
        if not self._running():
            self.start_heading.set("%.1f" % float(value))
            self.apply_start(quiet=True)

    def start(self):
        if self._running():
            return
        try:
            pose = self._starting_pose()
            from .physics import World
            World(self.config, pose)
            if self.session is not None:
                self.session.stop()
            from .runtime import SimulatorSession
            self.session = SimulatorSession(copy.deepcopy(self.config), self.program_path,
                                            start_pose=pose, realtime=True,
                                            speed=float(self.speed.get()), gps_mode=self.gps_mode.get())
            self._clear_outputs()
            self.world = pose
            self.paused = False
            self.session.start()
            self.status.set("Running " + self.program_path.name)
        except Exception as error:
            if self.session is not None:
                self.session.stop()
                self.session = None
            self.status.set("Could not start: " + str(error))
            self._append_console(str(error))
        self._update_controls()

    def toggle_pause(self):
        if self.session is not None and self.session.alive:
            self.paused = not self.paused
            self.session.pause(self.paused)
            self.status.set("Paused — Step advances one simulation tick" if self.paused else "Running")
            self._update_controls()

    def step(self):
        if self.session is not None and self.session.alive and self.paused:
            self.session.step()

    def stop(self):
        if self.session is not None:
            self.session.stop()
            self.session = None
        self.paused = False
        self.status.set("Stopped — drag the robot or edit its starting position")
        self._update_controls()

    def reset(self):
        self.stop()
        self._clear_outputs()
        self.apply_start()
        self.status.set("Reset — ready to run from the starting position")

    def _clear_outputs(self):
        self.snapshot = None
        self.gps = None
        self.target = None
        self.trail = []
        self._log_history = []
        self._brain_text = None
        for widget in (self.brain_display, self.console):
            widget.configure(state="normal")
            widget.delete("1.0", "end")
            widget.configure(state="disabled")
        for variable in self.readouts.values():
            variable.set("—")

    def _speed_changed(self, event=None):
        if self.session is not None and self.session.alive:
            self.session.set_speed(float(self.speed.get()))

    def inject_dropout(self):
        if self.session is not None and self.session.alive:
            self.session.inject_gps_loss(seconds=1.0)
            self._append_console("[Simulator] GPS loss injected for 1 second.")

    def _append_console(self, text):
        self.console.configure(state="normal")
        self.console.insert("end", str(text).rstrip("\n") + "\n")
        # Bound the desktop log without changing the worker's program output.
        if int(self.console.index("end-1c").split(".")[0]) > 1500:
            self.console.delete("1.0", "301.0")
        self.console.see("end")
        self.console.configure(state="disabled")

    def _poll(self):
        if self._closed:
            return
        try:
            if self.session is not None:
                for message in self.session.poll():
                    kind = message.get("type")
                    if kind == "snapshot":
                        self._accept_snapshot(message)
                    elif kind == "exit":
                        self.paused = False
                        error = message.get("error")
                        self.status.set("Program error: " + str(error) if error else
                                        message.get("message", "Program finished"))
                        if error:
                            self._append_console(error)
                self._update_controls()
        except Exception as error:
            self.status.set("Simulator error: " + str(error))
            self._append_console(error)
            if self.session is not None:
                self.session.stop()
                self.session = None
            self._update_controls()
        self._poll_token = self.root.after(40, self._poll)

    def _accept_snapshot(self, snapshot):
        self.snapshot = snapshot
        self.world = snapshot.get("world", self.world)
        self.gps = snapshot.get("gps")
        self.target = snapshot.get("target")
        self.paused = snapshot.get("paused", self.paused)
        position = (self.world.get("x_mm", 0), self.world.get("y_mm", 0))
        if not self.trail or math.hypot(position[0] - self.trail[-1][0], position[1] - self.trail[-1][1]) >= 5:
            self.trail.append(position)
            self.trail = self.trail[-2500:]
        status = snapshot.get("status")
        if snapshot.get("error"):
            self.status.set("Program error: " + str(snapshot["error"]))
        elif status:
            self.status.set(("Paused  ·  " if self.paused else "") + status)
        brain_text = "\n".join(str(line) for line in snapshot.get("brain_lines", []))
        if brain_text != self._brain_text:
            self.brain_display.configure(state="normal")
            self.brain_display.delete("1.0", "end")
            self.brain_display.insert("1.0", brain_text)
            self.brain_display.configure(state="disabled")
            self._brain_text = brain_text
        logs = snapshot.get("logs", [])
        old_count = len(self._log_history) if logs[:len(self._log_history)] == self._log_history else 0
        for line in logs[old_count:]:
            self._append_console(line)
        self._log_history = list(logs)
        self._telemetry()
        self.redraw()

    def _telemetry(self):
        world = self.world
        self.readouts["position"].set("%.0f, %.0f mm" % (world.get("x_mm", 0), world.get("y_mm", 0)))
        self.readouts["heading"].set("%.1f°" % world.get("heading_deg", 0))
        self.readouts["gps"].set("%d%%" % self.gps.get("quality", 0) if self.gps else "—")
        self.readouts["motors"].set("%.0f / %.0f rpm" % (world.get("left_rpm", 0), world.get("right_rpm", 0)))
        self.readouts["motion"].set("%.0f mm/s · %.0f°/s" % (world.get("v_mm_s", 0), world.get("omega_deg_s", 0)))
        self.readouts["collision"].set("CONTACT" if world.get("collision") else "clear")
        if self.snapshot:
            self.readouts["clock"].set("SIM TIME  %.2f s" % self.snapshot.get("time_s", 0))

    def _point(self, x, y):
        assert self._transform is not None
        cx, cy, scale = self._transform
        return cx + x * scale, cy - y * scale

    def _robot_point(self, local_x, local_y):
        angle = math.radians(self.world.get("heading_deg", 0))
        x = self.world.get("x_mm", 0) + local_x * math.cos(angle) + local_y * math.sin(angle)
        y = self.world.get("y_mm", 0) - local_x * math.sin(angle) + local_y * math.cos(angle)
        return self._point(x, y)

    def _polygon(self, points, **kwargs):
        return self.canvas.create_polygon([coordinate for point in points for coordinate in point], **kwargs)

    def _robot_rectangle(self, x0, y0, x1, y1, **kwargs):
        return self._polygon([self._robot_point(x, y) for x, y in
                              ((x0, y0), (x1, y0), (x1, y1), (x0, y1))], **kwargs)

    def redraw(self):
        canvas = self.canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 20 or height < 20:
            return
        field = self.config["field"]
        fw, fh = field["width_mm"], field["height_mm"]
        scale = min((width - 95) / fw, (height - 100) / fh)
        if scale <= 0:
            return
        self._transform = (width / 2 + 12, height / 2 - 5, scale)
        canvas.delete("all")
        left, top = self._point(-fw / 2, fh / 2)
        right, bottom = self._point(fw / 2, -fh / 2)
        canvas.create_rectangle(left, top, right, bottom, fill=FIELD, outline="#667886", width=4)
        tile = field["tile_mm"]
        for index in range(1, math.ceil(fw / tile)):
            x = -fw / 2 + index * tile
            if x < fw / 2:
                px, _ = self._point(x, 0)
                canvas.create_line(px, top, px, bottom, fill="#34434f")
        for index in range(1, math.ceil(fh / tile)):
            y = -fh / 2 + index * tile
            if y < fh / 2:
                _, py = self._point(0, y)
                canvas.create_line(left, py, right, py, fill="#34434f")
        cx, cy = self._point(0, 0)
        canvas.create_line(left, cy, right, cy, fill="#4a626f", dash=(4, 5))
        canvas.create_line(cx, top, cx, bottom, fill="#4a626f", dash=(4, 5))
        for x in (-fw / 2, -fw / 4, 0, fw / 4, fw / 2):
            px, _ = self._point(x, 0)
            canvas.create_text(px, bottom + 17, text="%.0f" % x, fill=MUTED, font=("Consolas", 9))
        for y in (-fh / 2, 0, fh / 2):
            _, py = self._point(0, y)
            canvas.create_text(left - 10, py, text="%.0f" % y, anchor="e", fill=MUTED, font=("Consolas", 9))
        canvas.create_text(cx, bottom + 38, text="X (mm) →", fill=MUTED, font=("Segoe UI", 10))
        canvas.create_text(left, top - 23, text="Y (mm) ↑", anchor="w", fill=MUTED, font=("Segoe UI", 10))
        canvas.create_text(right, top - 23, text="%.2f × %.2f m" % (fw / 1000, fh / 1000),
                           anchor="e", fill=MUTED, font=("Segoe UI", 10))
        if self.target:
            tx, ty = self._point(self.target.get("x_mm", 0), self.target.get("y_mm", 0))
            radius = self.target.get("radius_mm", 100) * scale
            canvas.create_oval(tx - radius, ty - radius, tx + radius, ty + radius,
                               fill="#254c43", outline=GREEN, dash=(4, 3))
            canvas.create_line(tx - 5, ty, tx + 5, ty, fill=GREEN)
            canvas.create_line(tx, ty - 5, tx, ty + 5, fill=GREEN)
            canvas.create_text(tx, ty + radius + 13, text="target", fill=GREEN, font=("Segoe UI", 9))
        if len(self.trail) > 1:
            points = [coordinate for point in self.trail for coordinate in self._point(*point)]
            canvas.create_line(*points, fill="#3b919c", width=2)
        self._draw_robot(scale)
        if self.gps:
            gx, gy = self.gps.get("x_mm"), self.gps.get("y_mm")
            if isinstance(gx, (int, float)) and isinstance(gy, (int, float)) and math.isfinite(gx) and math.isfinite(gy):
                px, py = self._point(gx, gy)
                color = GREEN if self.gps.get("quality", 0) >= 100 else AMBER
                canvas.create_oval(px - 6, py - 6, px + 6, py + 6, outline=color, width=2)
                canvas.create_line(px - 10, py, px + 10, py, fill=color)
                canvas.create_line(px, py - 10, px, py + 10, fill=color)
                canvas.create_text(px + 12, py + 12, text="GPS", anchor="w", fill=color, font=("Segoe UI", 8))

    def _draw_robot(self, scale):
        robot = self.config["robot"]
        length, width = robot["body_length_mm"], robot["body_width_mm"]
        gps_x, gps_y = robot["gps_x_mm"], robot["gps_y_mm"]
        points = [self._robot_point(gps_x, gps_y)]
        for delta in (-35, 35):
            angle = math.radians(robot["gps_heading_deg"] + delta)
            points.append(self._robot_point(gps_x + 480 * math.sin(angle), gps_y + 480 * math.cos(angle)))
        self._polygon(points, fill="#314a58", outline="#517386", stipple="gray25")
        # Dashed outline is the collision footprint, including wheels and claw.
        self._robot_rectangle(-width / 2, -length / 2, width / 2, length / 2,
                              fill="", outline="#527f8c", dash=(3, 3))
        diameter = robot["wheel_diameter_mm"]
        for side in (-1, 1):
            for position in (-0.38, 0, 0.38):
                wx = side * robot["track_width_mm"] / 2
                wy = length * position
                wheel_length = diameter if position == 0 else diameter * 0.7
                self._robot_rectangle(wx - 18, wy - wheel_length / 2, wx + 18, wy + wheel_length / 2,
                                      fill="#0d131b", outline="#83939d", width=1)
        self._robot_rectangle(-width * .34, -length * .48, width * .34, length * .28,
                              fill="#174e5b", outline=CYAN, width=2)
        self._robot_rectangle(-width * .3, -length * .25, width * .3, length * .13,
                              fill="#21333f", outline="#527f8c")
        for side in (-1, 1):
            jaw = [self._robot_point(side * width * .18, length * .28),
                   self._robot_point(side * width * .28, length * .44),
                   self._robot_point(side * width * .08, length * .5)]
            self.canvas.create_line(*[v for point in jaw for v in point], fill="#e57270", width=3)
        start, end = self._robot_point(0, 15), self._robot_point(0, length * .40)
        self.canvas.create_line(*start, *end, fill=CYAN, width=2, arrow="last")
        px, py = self._robot_point(gps_x, gps_y)
        self.canvas.create_rectangle(px - 4, py - 4, px + 4, py + 4, fill="#aac6d6", outline="#e6edf3")
        x, y = self._robot_point(0, 0)
        self.canvas.create_oval(x - 2.5, y - 2.5, x + 2.5, y + 2.5, fill="white", outline="")

    def _drag_start(self, event):
        if self._running() or not self._transform:
            return
        x, y = self._point(self.world.get("x_mm", 0), self.world.get("y_mm", 0))
        radius = max(self.config["robot"]["body_width_mm"], self.config["robot"]["body_length_mm"]) * self._transform[2] / 2 + 15
        if math.hypot(event.x - x, event.y - y) <= radius:
            self.dragging = True
            self._drag_move(event)

    def _drag_move(self, event):
        if not self.dragging or self._running() or not self._transform:
            return
        cx, cy, scale = self._transform
        robot = self.config["robot"]
        margin = math.hypot(robot["body_width_mm"], robot["body_length_mm"]) / 2
        xmax = max(0, self.config["field"]["width_mm"] / 2 - margin)
        ymax = max(0, self.config["field"]["height_mm"] / 2 - margin)
        self.start_x.set("%.0f" % max(-xmax, min(xmax, (event.x - cx) / scale)))
        self.start_y.set("%.0f" % max(-ymax, min(ymax, (cy - event.y) / scale)))
        self.apply_start(quiet=True)

    def _drag_end(self, event):
        self.dragging = False

    def open_settings(self):
        if not self._running():
            ModelSettings(self)

    def close(self):
        self._closed = True
        if self._poll_token is not None:
            self.root.after_cancel(self._poll_token)
        if self.session is not None:
            self.session.stop()
        self.root.destroy()


class ModelSettings:
    """Edit the physical model independently of main.py's device configuration."""

    FIELDS = {
        "Chassis": [
            ("robot", "mass_kg", "Robot mass (kg) — estimated"),
            ("robot", "wheel_diameter_mm", "Drive wheel diameter (mm)"),
            ("robot", "track_width_mm", "Drive wheel centre spacing (mm)"),
            ("robot", "body_length_mm", "Body length (mm)"),
            ("robot", "body_width_mm", "Body width (mm)"),
            ("robot", "external_ratio", "Motor turns / wheel turn"),
            ("robot", "motor_free_rpm", "Motor free speed (RPM)"),
            ("robot", "motor_stall_torque_nm", "Motor stall torque (N·m)"),
        ],
        "GPS": [
            ("robot", "gps_x_mm", "Physical GPS X: right + (mm)"),
            ("robot", "gps_y_mm", "Physical GPS Y: forward + (mm)"),
            ("robot", "gps_heading_deg", "Physical GPS facing: back = 180°"),
            ("gps", "sample_hz", "Position sample rate (Hz)"),
            ("gps", "latency_ms", "Readout latency (ms)"),
            ("gps", "position_noise_mm", "Position noise (mm)"),
            ("gps", "heading_noise_deg", "Heading noise (degrees)"),
            ("gps", "turn_dropout_deg_s", "Visual loss above turn rate (°/s)"),
            ("gps", "wall_min_distance_mm", "Minimum visible wall distance (mm)"),
        ],
        "Field & engine": [
            ("field", "width_mm", "Field width X (mm)"),
            ("field", "height_mm", "Field height Y (mm)"),
            ("field", "tile_mm", "Tile spacing (mm)"),
            ("simulation", "step_ms", "Physics step (ms)"),
            ("simulation", "seed", "Random seed (integer)"),
        ],
        "Physics": [
            ("physics", "traction_mu", "Forward tire friction (estimated)"),
            ("physics", "lateral_mu", "Sideways tire friction (estimated)"),
            ("physics", "motor_response_s", "Motor response time (seconds)"),
            ("physics", "brake_response_s", "Brake response time (seconds)"),
            ("physics", "coast_response_s", "Coast response time (seconds)"),
            ("physics", "inertia_scale", "Yaw inertia multiplier"),
            ("physics", "restitution", "Wall bounce (0 to 1)"),
        ],
    }

    def __init__(self, app):
        self.app = app
        self.values = {}
        self.working = copy.deepcopy(app.config)
        self.window = tk.Toplevel(app.root)
        self.window.title("Physical model settings")
        self.window.configure(bg=PANEL)
        self.window.geometry("740x%d" % min(800, app.root.winfo_screenheight() - 100))
        self.window.resizable(False, False)
        self.window.transient(app.root)
        self.window.grab_set()
        container = ttk.Frame(self.window, padding=18)
        container.pack(fill="both", expand=True)
        ttk.Label(container, text="Physical model", font=("Segoe UI Semibold", 17)).pack(anchor="w")
        ttk.Label(container, text="Estimates affect motion. These settings change the simulated robot;\n"
                  "GPS offsets in main.py still describe what the program believes is mounted.",
                  style="Muted.TLabel").pack(anchor="w", pady=(6, 12))
        notebook = ttk.Notebook(container)
        notebook.pack(fill="both", expand=True)
        for title, fields in self.FIELDS.items():
            tab = ttk.Frame(notebook, padding=14)
            notebook.add(tab, text=title)
            tab.columnconfigure(0, weight=1)
            for row, (section, key, label) in enumerate(fields):
                ttk.Label(tab, text=label).grid(row=row, column=0, sticky="w", pady=6)
                var = tk.StringVar(value=str(self.working[section][key]))
                self.values[(section, key)] = var
                ttk.Entry(tab, textvariable=var, width=13).grid(row=row, column=1, sticky="e", pady=6)
        self.note = tk.StringVar(value="Motor ports 9 / 10 and GPS port 18 follow the project wiring.")
        ttk.Label(container, textvariable=self.note, style="Muted.TLabel", wraplength=610).pack(anchor="w", pady=10)
        actions = ttk.Frame(container)
        actions.pack(fill="x")
        ttk.Button(actions, text="Load JSON…", command=self.load).pack(side="left", padx=(0, 6))
        ttk.Button(actions, text="Save JSON…", command=self.save).pack(side="left")
        ttk.Button(actions, text="Apply", style="Run.TButton", command=self.apply).pack(side="right")
        ttk.Button(actions, text="Cancel", command=self.window.destroy).pack(side="right", padx=6)

    def _read(self):
        result = copy.deepcopy(self.working)
        for (section, key), var in self.values.items():
            try:
                number = float(var.get())
                if not math.isfinite(number):
                    raise ValueError()
                if key in ("step_ms", "seed"):
                    if number != int(number):
                        raise ValueError()
                    number = int(number)
                result[section][key] = number
            except ValueError:
                raise ValueError("Enter a valid number for " + key + ".") from None
        nonnegative = {"latency_ms", "position_noise_mm", "heading_noise_deg", "wall_min_distance_mm",
                       "traction_mu", "lateral_mu", "restitution", "turn_dropout_deg_s"}
        signed = {"gps_x_mm", "gps_y_mm", "gps_heading_deg", "seed"}
        for (section, key) in self.values:
            value = result[section][key]
            if key not in signed and (value < 0 if key in nonnegative else value <= 0):
                raise ValueError(key + (" must be zero or greater." if key in nonnegative else " must be greater than zero."))
        result["robot"]["gps_heading_deg"] %= 360
        if result["field"]["tile_mm"] < 10:
            raise ValueError("Tile spacing must be at least 10 mm.")
        from .config import validate_config
        validate_config(result)
        return result

    def _set_fields(self, config):
        self.working = config
        for (section, key), variable in self.values.items():
            variable.set(str(config[section][key]))

    def load(self):
        filename = filedialog.askopenfilename(parent=self.window, title="Load simulator model",
                                              filetypes=[("JSON model", "*.json")])
        if not filename:
            return
        try:
            with open(filename, encoding="utf-8") as file:
                incoming = json.load(file)
            if not isinstance(incoming, dict):
                raise ValueError("The JSON file must contain a settings object.")
            merged = _merge(self.app.config, incoming)
            for section, key in self.values:
                float(merged[section][key])
            self._set_fields(merged)
            self._read()
            self.note.set("Loaded " + Path(filename).name + ". Apply to use these settings.")
        except (OSError, ValueError, TypeError, KeyError) as error:
            messagebox.showerror("Could not load model", str(error), parent=self.window)

    def save(self):
        try:
            config = self._read()
        except ValueError as error:
            self.note.set(str(error))
            return
        filename = filedialog.asksaveasfilename(parent=self.window, title="Save simulator model",
                                              defaultextension=".json", initialfile="flex-model.json",
                                              filetypes=[("JSON model", "*.json")])
        if filename:
            try:
                with open(filename, "w", encoding="utf-8") as file:
                    json.dump(config, file, indent=2)
                    file.write("\n")
                self.note.set("Saved " + Path(filename).name)
            except OSError as error:
                messagebox.showerror("Could not save model", str(error), parent=self.window)

    def apply(self):
        try:
            config = self._read()
        except ValueError as error:
            self.note.set(str(error))
            return
        self.app.config = config
        self.app.reset()
        self.app.status.set("Model updated — ready to run")
        self.window.destroy()


def run_gui(config, program_path):
    enable_dpi_awareness()
    root = tk.Tk()
    SimulatorApp(root, config, program_path)
    root.mainloop()
