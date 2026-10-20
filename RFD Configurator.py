#!/usr/bin/env python3
"""RFD900x configurator GUI.

Detects USB serial radios, enters SiK AT command mode, and reads/writes every
S-register parameter (local radio via AT..., remote radio via RT...).
"""

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
    (3, "NETID", None, 0, 499, "Network ID (must match on both radios)", False),
    (4, "TXPOWER", None, 0, 30, "Transmit power in dBm", False),
    (5, "ECC", [0, 1], 0, 1, "Error correcting code (0=off, 1=on)", False),
    (6, "MAVLINK", [0, 1, 2], 0, 2, "MAVLink framing (0=off, 1=on, 2=low latency)", False),
    (7, "OP_RESEND", [0, 1], 0, 1, "Opportunistic resend (0=off, 1=on)", False),
    (8, "MIN_FREQ", None, 0, 1000000, "Minimum frequency in kHz", False),
    (9, "MAX_FREQ", None, 0, 1000000, "Maximum frequency in kHz", False),
    (10, "NUM_CHANNELS", None, 1, 200, "Number of hopping channels", False),
    (11, "DUTY_CYCLE", None, 0, 100, "Transmit duty cycle in percent", False),
    (12, "LBT_RSSI", None, 0, 220, "Listen-before-talk RSSI threshold (0=off, else 25-220)", False),
    (13, "MANCHESTER", [0, 1], 0, 1, "Manchester encoding (0=off, 1=on)", False),
    (14, "RTSCTS", [0, 1], 0, 1, "Hardware flow control (0=off, 1=on)", False),
    (15, "MAX_WINDOW", None, 0, 400, "Max transmit window in ms (max 400)", False),
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


class Row:
    def __init__(self, num, name, var, lo, hi, read_only):
        self.num, self.name, self.var = num, name, var
        self.lo, self.hi, self.read_only = lo, hi, read_only


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
    def _build_ui(self):
        pad = {"padx": 6, "pady": 3}

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

        ttk.Label(conn, text="Baud:").grid(row=1, column=0, **pad)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Combobox(conn, textvariable=self.baud_var, values=BAUD_CHOICES,
                     width=10).grid(row=1, column=1, sticky="w", **pad)
        self.auto_baud_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(conn, text="Auto-detect baud when entering command mode",
                        variable=self.auto_baud_var).grid(row=1, column=2, columnspan=2, sticky="w", **pad)
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.on_connect_toggle)
        self.connect_btn.grid(row=0, column=4, rowspan=2, **pad)

        self.detect_var = tk.StringVar()
        ttk.Label(conn, textvariable=self.detect_var).grid(row=2, column=0, columnspan=5, sticky="w", **pad)

        radio = ttk.LabelFrame(self.root, text="Radio")
        radio.pack(fill="x", padx=8, pady=4)
        self.enter_btn = ttk.Button(radio, text="Enter Command Mode (+++)", command=self.on_enter)
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

        self.params_frame = ttk.LabelFrame(self.root, text="Parameters")
        self.params_frame.pack(fill="x", padx=8, pady=4)
        for num, name, choices, lo, hi, desc, ro in PARAMS:
            self._add_row(num, name, choices, lo, hi, desc, ro)

        term = ttk.LabelFrame(self.root, text="Log / Terminal")
        term.pack(fill="both", expand=True, padx=8, pady=4)
        self.log_text = tk.Text(term, height=10, state="disabled", wrap="word")
        sb = ttk.Scrollbar(term, command=self.log_text.yview)
        self.log_text.config(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)

        send = ttk.Frame(self.root)
        send.pack(fill="x", padx=8, pady=(0, 8))
        self.raw_var = tk.StringVar()
        entry = ttk.Entry(send, textvariable=self.raw_var)
        entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
        entry.bind("<Return>", lambda e: self.on_raw())
        self.raw_btn = ttk.Button(send, text="Send Raw", command=self.on_raw)
        self.raw_btn.pack(side="left")

        self.update_buttons()

    def _add_row(self, num, name, choices, lo, hi, desc, read_only):
        r = len(self.rows)
        var = tk.StringVar()
        ttk.Label(self.params_frame, text=f"S{num}  {name}", width=22).grid(row=r, column=0, sticky="w", padx=6, pady=1)
        if choices is not None:
            w = ttk.Combobox(self.params_frame, textvariable=var, values=choices, width=10)
        else:
            w = ttk.Spinbox(self.params_frame, textvariable=var, from_=lo, to=hi, width=10)
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
        auto = self.auto_baud_var.get()
        self.submit(lambda: self._job_enter(port, baud, auto))

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
            if raw == "" or raw == self.orig.get(num):
                continue
            try:
                val = int(raw)
            except ValueError:
                raise ValueError(f"S{num} {row.name}: '{raw}' is not an integer")
            if not row.lo <= val <= row.hi:
                raise ValueError(f"S{num} {row.name}: {val} is outside {row.lo}..{row.hi}")
            changes.append((num, val))
        return changes

    def _apply_params(self, params):
        for num, name, val in params:
            if num not in self.rows:
                self._add_row(num, name, None, 0, 65535, "(reported by radio)", False)
            self.rows[num].var.set(val)
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
                self._cmd("ATO")
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

    def _job_enter(self, port, baud, auto):
        if self.ser is None:
            self._open(port, baud)
        bauds = [baud] + ([b for b in BAUD_CHOICES if b != baud] if auto else [])
        for b in bauds:
            self.ser.baudrate = b
            self.log(f"Trying command mode at {b} baud...")
            if self._try_enter():
                self.in_cmd = True
                self.log(f"Command mode active at {b} baud")
                self.ui(lambda b=b: (self.baud_var.set(str(b)), self.set_state(True, True)))
                ver = self._cmd("ATI")
                self.ui(lambda: self.version_var.set(ver.splitlines()[-1] if ver else ""))
                self._read_all("AT")
                return
        self.ser.baudrate = baud
        self.log("Could not enter command mode. Check port, baud, and that the radio is powered.")

    def _job_exit(self):
        self._cmd("ATO")
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
