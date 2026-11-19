#!/usr/bin/env python3
"""RFD900x configurator GUI.

Detects USB serial radios, enters SiK AT command mode, and reads/writes every
S-register parameter (local radio via AT..., remote radio via RT...).
"""

import json
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports

DEFAULT_BAUD = 57600

# S1 (SERIAL_SPEED) register value -> real baud rate.
SERIAL_BAUD = {
    1: 1200, 2: 2400, 4: 4800, 9: 9600, 19: 19200, 38: 38400,
    57: 57600, 115: 115200, 230: 230400, 460: 460800, 921: 921600,
}
BAUD_CHOICES = sorted(SERIAL_BAUD.values())
AIR_SPEEDS = [2, 4, 8, 16, 19, 24, 32, 64, 96, 128, 192, 250]

# (number, name, choices, min, max, description, read_only)
PARAMS = [
    (0, "FORMAT", None, 0, 65535, "EEPROM format version (read-only)", True),
    (1, "SERIAL_SPEED", sorted(SERIAL_BAUD), 1, 921,
     "Serial speed: 57=57600, 115=115200, ... (applies after reboot)", False),
    (2, "AIR_SPEED", AIR_SPEEDS, 2, 250, "Air data rate in kbps", False),
    (3, "NETID", list(range(1, 256)), 1, 255, "Network ID 1-255 (must match on both radios)", False),
    (4, "TXPOWER", list(range(0,31)), 0, 30, "Transmit power in dBm", False),
    (5, "ECC", [0, 1], 0, 1, "Error correcting code (0=off, 1=on)", False),
    (6, "MAVLINK", [0, 1, 2], 0, 2, "MAVLink framing (0=off, 1=on, 2=low latency)", False),
    (7, "OP_RESEND", [0, 1], 0, 1, "Opportunistic resend (0=off, 1=on)", False),
    (8, "MIN_FREQ", [f"{k // 1000}e6" for k in range(800000, 1000001, 1000)], 800000, 1000000,
     "Minimum frequency in kHz (shown as MHz, e.g. 915e6)", False),
    (9, "MAX_FREQ", [f"{k // 1000}e6" for k in range(800000, 1000001, 1000)], 800000, 1000000,
     "Maximum frequency in kHz (shown as MHz, e.g. 928e6)", False),
    (10, "NUM_CHANNELS", list(range(1,201)), 1, 200, "Number of hopping channels", False),
    (11, "DUTY_CYCLE", list(range(0, 101)), 0, 100, "Transmit duty cycle in percent", False),
    (12, "LBT_RSSI", list(range(0, 221)), 0, 220, "Listen-before-talk RSSI threshold (0=off, else 25-220)", False),
    (13, "MANCHESTER", [0, 1], 0, 1, "Manchester encoding (0=off, 1=on)", False),
    (14, "RTSCTS", [0, 1], 0, 1, "Hardware flow control (0=off, 1=on)", False),
    (15, "MAX_WINDOW", list(range(0, 401)), 0, 400, "Max transmit window in ms (max 400)", False),
]

PARAM_RE = re.compile(r"S(\d+):\s*([A-Za-z0-9_]+)\s*=\s*(-?\d+)")


def read_idle(ser, total, idle):
    """Read until `idle` seconds pass with no data (after some data), or `total` elapses."""
    end = time.monotonic() + total
    last = None
    buf = bytearray()
    while time.monotonic() < end:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            last = time.monotonic()
        elif last is not None and time.monotonic() - last >= idle:
            break
    return bytes(buf)


def scan_ports(show_all=False):
    """Return a list of (label, device, likely_rfd)."""
    found = []
    for p in list_ports.comports():
        if p.vid is None and not show_all:
            continue
        text = " ".join(x or "" for x in (p.description, p.manufacturer, p.product)).lower()
        likely = p.vid == 0x0403 or "rfd" in text
        ident = f"{p.vid:04x}:{p.pid:04x}" if p.vid is not None else "no-usb-id"
        label = f"{p.device} - {p.description} [{ident}]"
        found.append((label, p.device, likely))
    return found


FREQ_PARAMS = (8, 9)  # radio stores these in kHz; GUI shows MHz as e.g. 915e6


def fmt_freq(khz):
    """915000 (kHz) -> '915e6'."""
    return f"{int(khz) // 1000}e6"


def parse_freq(text):
    """'915e6' -> 915000 (kHz)."""
    return round(float(text.strip()) / 1000)


class Row:
    def __init__(self, num, name, var, lo, hi, read_only):
        self.num, self.name, self.var = num, name, var
        self.lo, self.hi, self.read_only = lo, hi, read_only
        self.is_freq = num in FREQ_PARAMS

    def to_display(self, val):
        return fmt_freq(val) if self.is_freq else str(val)

    def parse(self, text):
        return parse_freq(text) if self.is_freq else int(text)


class App:
    def __init__(self, root):
        self.root = root
        root.title("RFD900x Configurator")
        root.geometry("1000x860")

        self.jobs = queue.Queue()
        self.ui_q = queue.Queue()
        self.ser = None          # worker-thread only
        self.in_cmd = False      # worker-thread only
        self.connected = False   # UI mirror
        self.cmd_mode = False    # UI mirror
        self.rows = {}
        self.orig = {}
        self.port_map = {}
        self.port_labels = ()

        self._build_ui()
        threading.Thread(target=self._worker, daemon=True).start()
        self._poll_ui()
        self.refresh_ports()
        self._auto_refresh()
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------ UI
    def _theme(self, dark):
        return {
            "background": "#111827" if dark else "#f3f5f9",
            "surface": "#1f2937" if dark else "#ffffff",
            "field": "#0f172a" if dark else "#ffffff",
            "foreground": "#e5e7eb" if dark else "#1e293b",
            "muted": "#9ca3af" if dark else "#64748b",
            "border": "#4b5563" if dark else "#d5dce7",
            "title": "#bfdbfe" if dark else "#233c65",
            "log": "#0b1220" if dark else "#eaf0f7",
            "log_foreground": "#dbeafe" if dark else "#334155",
        }

    def _build_ui(self):
        pad = {"padx": 6, "pady": 3}

        self.style = ttk.Style()
        self.style.theme_use("clam")
        self.dark_var = tk.BooleanVar(value=False)

        conn = ttk.LabelFrame(self.root, text="Connection")
        conn.pack(fill="x", padx=8, pady=4)
        ttk.Label(conn, text="Port:").grid(row=0, column=0, **pad)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, width=55)
        self.port_box.grid(row=0, column=1, **pad)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, **pad)
        self.show_all_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(conn, text="Show all ports", variable=self.show_all_var,
                        command=self.refresh_ports).grid(row=0, column=3, **pad)
        ttk.Checkbutton(conn, text="Dark mode", variable=self.dark_var,
                        command=self.apply_theme).grid(row=0, column=5, **pad)

        ttk.Label(conn, text="Baud:").grid(row=1, column=0, **pad)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Combobox(conn, textvariable=self.baud_var, values=BAUD_CHOICES,
                     width=10).grid(row=1, column=1, sticky="w", **pad)
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.on_connect_toggle)
        self.connect_btn.grid(row=0, column=4, rowspan=2, **pad)

        self.detect_var = tk.StringVar()
        ttk.Label(conn, textvariable=self.detect_var).grid(row=2, column=0, columnspan=5, sticky="w", **pad)

        radio = ttk.LabelFrame(self.root, text="Radio")
        radio.pack(fill="x", padx=8, pady=4)
        self.enter_btn = ttk.Button(radio, text="Enter Command Mode", command=self.on_enter)
        self.exit_btn = ttk.Button(radio, text="Exit Command Mode (ATO)", command=self.on_exit)
        self.read_btn = ttk.Button(radio, text="Read All", command=self.on_read)
        self.write_btn = ttk.Button(radio, text="Write Changes & Save", command=self.on_write)
        self.reboot_btn = ttk.Button(radio, text="Reboot", command=self.on_reboot)
        self.factory_btn = ttk.Button(radio, text="Factory Reset", command=self.on_factory)
        for i, b in enumerate((self.enter_btn, self.exit_btn, self.read_btn,
                               self.write_btn, self.reboot_btn, self.factory_btn)):
            b.grid(row=0, column=i, **pad)

        self.target_var = tk.StringVar(value="AT")
        ttk.Label(radio, text="Target:").grid(row=1, column=0, sticky="e", **pad)
        tf = ttk.Frame(radio)
        tf.grid(row=1, column=1, columnspan=2, sticky="w")
        ttk.Radiobutton(tf, text="Local (AT)", variable=self.target_var, value="AT").pack(side="left", padx=4)
        ttk.Radiobutton(tf, text="Remote (RT)", variable=self.target_var, value="RT").pack(side="left", padx=4)
        self.reboot_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(radio, text="Reboot after saving", variable=self.reboot_var
                        ).grid(row=1, column=3, columnspan=2, sticky="w", **pad)

        self.state_var = tk.StringVar()
        self.version_var = tk.StringVar()
        ttk.Label(radio, textvariable=self.state_var).grid(row=2, column=0, columnspan=3, sticky="w", **pad)
        ttk.Label(radio, textvariable=self.version_var).grid(row=2, column=3, columnspan=3, sticky="w", **pad)

        self.bottom_pane = tk.PanedWindow(self.root, orient="vertical", sashrelief="raised", sashwidth=6)
        self.bottom_pane.pack(fill="both", expand=True, padx=8, pady=4)
        bottom = self.bottom_pane

        top_pane = ttk.Frame(bottom)
        params_outer = ttk.LabelFrame(top_pane, text="Parameters")
        params_outer.pack(fill="both", expand=True)
        self.params_canvas = tk.Canvas(params_outer, highlightthickness=0)
        params_sb = ttk.Scrollbar(params_outer, orient="vertical", command=self.params_canvas.yview)
        self.params_canvas.configure(yscrollcommand=params_sb.set)
        params_sb.pack(side="right", fill="y")
        self.params_canvas.pack(side="left", fill="both", expand=True)
        self.params_frame = ttk.Frame(self.params_canvas)
        self.params_canvas.create_window((0, 0), window=self.params_frame, anchor="nw")
        self.params_frame.bind(
            "<Configure>",
            lambda e: self.params_canvas.configure(scrollregion=self.params_canvas.bbox("all")),
        )
        for widget in (self.params_canvas, self.params_frame):
            widget.bind("<MouseWheel>", self._on_params_mousewheel)
            widget.bind("<Button-4>", self._on_params_mousewheel)
            widget.bind("<Button-5>", self._on_params_mousewheel)
        for num, name, choices, lo, hi, desc, ro in PARAMS:
            self._add_row(num, name, choices, lo, hi, desc, ro)

        params_btns = ttk.Frame(top_pane)
        params_btns.pack(fill="x", pady=(4, 0))
        ttk.Button(params_btns, text="Copy Parameters", command=self.on_copy_params).pack(side="left", padx=(0, 6))
        ttk.Button(params_btns, text="Paste Parameters", command=self.on_paste_params).pack(side="left")
        bottom.add(top_pane, stretch="always")

        term = ttk.LabelFrame(bottom, text="Log / Terminal")
        self.log_text = tk.Text(term, height=10, state="disabled", wrap="word")
        sb = ttk.Scrollbar(term, command=self.log_text.yview)
        self.log_text.config(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.bind("<MouseWheel>", self._on_log_mousewheel)
        self.log_text.bind("<Button-4>", self._on_log_mousewheel)
        self.log_text.bind("<Button-5>", self._on_log_mousewheel)
        bottom.add(term, stretch="always")

        send = ttk.Frame(self.root)
        send.pack(fill="x", padx=8, pady=(0, 8))
        self.raw_var = tk.StringVar()
        entry = ttk.Entry(send, textvariable=self.raw_var)
        entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
        entry.bind("<Return>", lambda e: self.on_raw())
        self.raw_btn = ttk.Button(send, text="Send Raw", command=self.on_raw)
        self.raw_btn.pack(side="left")

        self.update_buttons()
        self.apply_theme()

    def _on_log_mousewheel(self, event):
        if event.num == 4:
            self.log_text.yview_scroll(-1, "units")
        elif event.num == 5:
            self.log_text.yview_scroll(1, "units")
        else:
            self.log_text.yview_scroll(-1 if event.delta > 0 else 1, "units")
        return "break"

    def _on_params_mousewheel(self, event):
        if event.num == 4:
            self.params_canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self.params_canvas.yview_scroll(1, "units")
        else:
            self.params_canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")
        return "break"

    def apply_theme(self):
        theme = self._theme(self.dark_var.get())
        bg, fg = theme["background"], theme["foreground"]
        field, border = theme["field"], theme["border"]
        surface, title = theme["surface"], theme["title"]

        self.root.configure(bg=bg)
        style = self.style
        for cls in ("TFrame", "TLabelframe", "TLabel", "TCheckbutton", "TRadiobutton"):
            style.configure(cls, background=surface, foreground=fg)
        style.map("TCheckbutton", background=[("active", surface)], foreground=[("active", fg)])
        style.map("TRadiobutton", background=[("active", surface)], foreground=[("active", fg)])
        style.configure("TLabelframe", background=surface, bordercolor=border,
                        lightcolor=surface, darkcolor=surface)
        style.configure("TLabelframe.Label", background=surface, foreground=title,
                        font=("TkDefaultFont", 10, "bold"))
        style.configure("TButton", background=field, foreground=fg,
                        bordercolor=border, lightcolor=field, darkcolor=field)
        style.map("TButton", background=[("active", field)])
        style.configure("TEntry", fieldbackground=field, foreground=fg,
                        insertcolor=fg, bordercolor=border,
                        lightcolor=field, darkcolor=field)
        style.configure("TCombobox", fieldbackground=field, foreground=fg,
                        background=field, insertcolor=fg, bordercolor=border,
                        lightcolor=field, darkcolor=field, arrowcolor=fg)
        style.map("TCombobox", fieldbackground=[("readonly", field)],
                  foreground=[("readonly", fg)])

        if hasattr(self, "log_text"):
            self.log_text.config(bg=theme["log"], fg=theme["log_foreground"],
                                 insertbackground=theme["log_foreground"],
                                 highlightbackground=border, highlightcolor=border)
        if hasattr(self, "bottom_pane"):
            self.bottom_pane.config(bg=surface, sashpad=0, bd=0)
        if hasattr(self, "params_canvas"):
            self.params_canvas.config(bg=surface, highlightbackground=border)

    def _add_row(self, num, name, choices, lo, hi, desc, read_only):
        r = len(self.rows)
        var = tk.StringVar()
        ttk.Label(self.params_frame, text=f"S{num}  {name}", width=22).grid(row=r, column=0, sticky="w", padx=6, pady=1)
        if num in FREQ_PARAMS:
            w = ttk.Combobox(self.params_frame, textvariable=var, values=choices, width=10)
        elif choices is not None:
            w = ttk.Combobox(self.params_frame, textvariable=var, values=choices, width=10)
        else:
            w = ttk.Entry(self.params_frame, textvariable=var, width=12)
        if read_only:
            w.state(["disabled"])
        w.grid(row=r, column=1, padx=6, pady=1)
        ttk.Label(self.params_frame, text=desc).grid(row=r, column=2, sticky="w", padx=6, pady=1)
        self.rows[num] = Row(num, name, var, lo, hi, read_only)

    def update_buttons(self):
        def st(on):
            return "normal" if on else "disabled"
        self.connect_btn.config(text="Disconnect" if self.connected else "Connect")
        self.enter_btn.config(state=st(not self.cmd_mode))
        for b in (self.exit_btn, self.read_btn, self.write_btn, self.reboot_btn, self.factory_btn):
            b.config(state=st(self.cmd_mode))
        self.raw_btn.config(state=st(self.connected))
        if not self.connected:
            self.state_var.set("Not connected")
        elif self.cmd_mode:
            self.state_var.set("Connected - COMMAND MODE")
        else:
            self.state_var.set("Connected - data mode (use Enter Command Mode)")

    def set_state(self, connected, cmd_mode):
        self.connected, self.cmd_mode = connected, cmd_mode
        if not connected:
            self.version_var.set("")
        self.update_buttons()

    # ------------------------------------------------------- thread plumbing
    def ui(self, fn):
        self.ui_q.put(fn)

    def log(self, text):
        def append():
            self.log_text.config(state="normal")
            self.log_text.insert("end", text + "\n")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
        self.ui(append)

    def _poll_ui(self):
        try:
            while True:
                self.ui_q.get_nowait()()
        except queue.Empty:
            pass
        self.root.after(50, self._poll_ui)

    def submit(self, fn):
        self.jobs.put(fn)

    def _worker(self):
        while True:
            fn = self.jobs.get()
            try:
                fn()
            except serial.SerialException as exc:
                self.log(f"SERIAL ERROR: {exc}")
                if "ermission" in str(exc):
                    self.log("Hint: add your user to the 'dialout' group and log in again.")
                self._close()
            except Exception as exc:  # noqa: BLE001
                self.log(f"ERROR: {exc}")

    # ----------------------------------------------------------- port scan
    def refresh_ports(self):
        found = scan_ports(self.show_all_var.get())
        labels = tuple(l for l, _, _ in found)
        self.port_map = {l: d for l, d, _ in found}
        if labels != self.port_labels:
            self.port_labels = labels
            self.port_box.config(values=labels)
            cur = self.port_var.get()
            if cur not in self.port_map and cur not in self.port_map.values():
                likely = [l for l, _, lk in found if lk]
                pick = likely[0] if likely else (labels[0] if labels else "")
                if not self.connected:
                    self.port_var.set(pick)
        likely = [d for _, d, lk in found if lk]
        if likely:
            self.detect_var.set(f"RFD900 detected: {', '.join(likely)}")
        elif found:
            self.detect_var.set("USB serial device(s) found, but none identified as an RFD900")
        else:
            self.detect_var.set("No USB serial device found - plug in the RFD900")

    def _auto_refresh(self):
        if not self.connected:
            self.refresh_ports()
        self.root.after(2000, self._auto_refresh)

    def selected_port(self):
        text = self.port_var.get().strip()
        return self.port_map.get(text, text)

    def selected_baud(self):
        try:
            return int(self.baud_var.get())
        except ValueError:
            raise ValueError("Baud must be an integer")

    # --------------------------------------------------------- UI handlers
    def on_connect_toggle(self):
        if self.connected:
            self.submit(self._job_disconnect)
            return
        try:
            port, baud = self.selected_port(), self.selected_baud()
        except ValueError as exc:
            messagebox.showerror("Error", str(exc))
            return
        if not port:
            messagebox.showerror("Error", "No port selected")
            return
        self.submit(lambda: self._open(port, baud))

    def on_enter(self):
        try:
            port, baud = self.selected_port(), self.selected_baud()
        except ValueError as exc:
            messagebox.showerror("Error", str(exc))
            return
        if not port:
            messagebox.showerror("Error", "No port selected")
            return
        self.submit(lambda: self._job_enter(port, baud))

    def on_exit(self):
        self.submit(self._job_exit)

    def on_read(self):
        prefix = self.target_var.get()
        self.submit(lambda: self._read_all(prefix))

    def on_write(self):
        try:
            changes = self._gather_changes()
        except ValueError as exc:
            messagebox.showerror("Invalid value", str(exc))
            return
        if not changes:
            messagebox.showinfo("Nothing to write", "No parameters have changed since the last read.")
            return
        prefix = self.target_var.get()
        reboot = self.reboot_var.get()
        self.submit(lambda: self._job_write(prefix, changes, reboot))

    def on_reboot(self):
        prefix = self.target_var.get()
        self.submit(lambda: self._reboot(prefix, []))

    def on_factory(self):
        if not messagebox.askyesno("Factory reset",
                                   "Reset all parameters to factory defaults and save?"):
            return
        prefix = self.target_var.get()
        self.submit(lambda: self._job_factory(prefix))

    def on_raw(self):
        cmd = self.raw_var.get().strip()
        if not cmd:
            return
        self.raw_var.set("")
        self.submit(lambda: self._job_raw(cmd))

    def on_copy_params(self):
        data = {}
        for num, row in self.rows.items():
            raw = row.var.get().strip()
            if raw == "":
                continue
            try:
                data[str(num)] = row.parse(raw)
            except ValueError:
                continue
        if not data:
            messagebox.showinfo("Copy Parameters", "No parameters to copy. Read from a radio first.")
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(json.dumps(data))
        self.log(f"Copied {len(data)} parameters to clipboard")

    def on_paste_params(self):
        try:
            text = self.root.clipboard_get()
        except tk.TclError:
            messagebox.showerror("Paste Parameters", "Clipboard is empty")
            return
        try:
            data = json.loads(text)
        except ValueError:
            messagebox.showerror("Paste Parameters", "Clipboard does not contain valid parameter data")
            return
        applied = 0
        for key, val in data.items():
            try:
                num = int(key)
            except ValueError:
                continue
            row = self.rows.get(num)
            if row is None or row.read_only:
                continue
            row.var.set(row.to_display(val))
            applied += 1
        self.log(f"Pasted {applied} parameters (review values, then Write Changes & Save)")

    def on_close(self):
        self._close()
        self.root.destroy()

    def _gather_changes(self):
        changes = []
        for num in sorted(self.rows):
            row = self.rows[num]
            if row.read_only:
                continue
            raw = row.var.get().strip()
            if raw == "":
                continue
            try:
                val = row.parse(raw)
            except ValueError:
                raise ValueError(f"S{num} {row.name}: '{raw}' is not a valid number")
            if str(val) == str(self.orig.get(num)):
                continue
            if not row.lo <= val <= row.hi:
                raise ValueError(f"S{num} {row.name}: {raw} is outside the allowed range")
            changes.append((num, val))
        return changes

    def _apply_params(self, params):
        for num, name, val in params:
            if num not in self.rows:
                self._add_row(num, name, None, 0, 65535, "(reported by radio)", False)
            self.rows[num].var.set(self.rows[num].to_display(val))
            self.orig[num] = val

    # ------------------------------------------------------- worker: serial
    def _open(self, port, baud):
        self._close()
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=2)
        self.in_cmd = False
        self.log(f"Opened {port} at {baud} baud")
        self.ui(lambda: self.set_state(True, False))

    def _close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
        self.ser = None
        self.in_cmd = False
        self.ui(lambda: self.set_state(False, False))

    def _job_disconnect(self):
        if self.ser is not None and self.in_cmd:
            try:
                self.ser.write(b"ATO\r\n")
                self.ser.flush()
            except serial.SerialException:
                pass
        self._close()
        self.log("Disconnected")

    def _cmd(self, command, total=1.5, idle=0.3):
        self.ser.reset_input_buffer()
        self.ser.write(command.encode("ascii") + b"\r\n")
        self.ser.flush()
        self.log(f">> {command}")
        text = read_idle(self.ser, total, idle).decode("ascii", "replace").strip()
        if text:
            self.log(text)
        return text

    def _try_enter(self):
        """Send +++ with SiK guard times; fall back to probing in case we are already in command mode."""
        ser = self.ser
        ser.reset_input_buffer()
        time.sleep(1.2)
        ser.write(b"+++")
        ser.flush()
        time.sleep(1.2)
        if b"OK" in read_idle(ser, 1.0, 0.3):
            return True
        resp = self._cmd("ATI", total=1.0)
        return "SiK" in resp or "RFD" in resp

    def _job_enter(self, port, baud):
        if self.ser is None:
            self._open(port, baud)
        self.ser.baudrate = baud
        self.log(f"Trying command mode at {baud} baud...")
        if self._try_enter():
            self.in_cmd = True
            self.log(f"Command mode active at {baud} baud")
            self.ui(lambda: self.set_state(True, True))
            ver = self._cmd("ATI")
            self.ui(lambda: self.version_var.set(ver.splitlines()[-1] if ver else ""))
            self._read_all("AT")
            return
        self.log("Could not enter command mode. Check port, baud, and that the radio is powered.")

    def _job_exit(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"ATO\r\n")
        self.ser.flush()
        self.log(">> ATO")
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.log("Left command mode (data mode)")
        self.in_cmd = False
        self.ui(lambda: self.set_state(True, False))

    def _read_all(self, prefix):
        text = self._cmd(f"{prefix}I5", total=4.0, idle=0.6)
        params = [(int(n), name, val) for n, name, val in PARAM_RE.findall(text)]
        if not params:
            self.log("No parameters returned.")
            return
        self.ui(lambda: self._apply_params(params))

    def _job_write(self, prefix, changes, reboot):
        failed = []
        for num, val in changes:
            if "OK" not in self._cmd(f"{prefix}S{num}={val}"):
                failed.append(num)
        if failed:
            self.log("ERROR: radio rejected " + ", ".join(f"S{n}" for n in failed) + "; nothing saved.")
            return
        if "OK" not in self._cmd(f"{prefix}&W", total=2.5):
            self.log("WARNING: no OK received for save (&W)")
        if reboot:
            self._reboot(prefix, changes)
        else:
            self._read_all(prefix)

    def _reboot(self, prefix, changes):
        self._cmd(f"{prefix}Z", total=1.0)
        if prefix != "AT":
            self.log("Remote radio rebooting.")
            return
        self.in_cmd = False
        self.ui(lambda: self.set_state(True, False))
        new = dict(changes).get(1)
        if new in SERIAL_BAUD:
            self.ser.baudrate = SERIAL_BAUD[new]
            self.ui(lambda: self.baud_var.set(str(SERIAL_BAUD[new])))
            self.log(f"Serial port switched to {SERIAL_BAUD[new]} baud")
        self.log("Local radio rebooting. Click 'Enter Command Mode' to continue.")
        time.sleep(3.0)

    def _job_factory(self, prefix):
        if "OK" not in self._cmd(f"{prefix}&F"):
            self.log("ERROR: factory reset failed")
            return
        self._cmd(f"{prefix}&W", total=2.5)
        self._read_all(prefix)

    def _job_raw(self, cmd):
        if self.ser is None:
            self.log("Not connected")
            return
        if cmd == "+++":
            ok = self._try_enter()
            self.in_cmd = ok
            self.ui(lambda: self.set_state(True, ok))
            return
        self._cmd(cmd, total=3.0)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
