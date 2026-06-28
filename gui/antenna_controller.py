#!/usr/bin/env python3
"""
Antenna Rotator Controller — Raspberry Pi Desktop GUI
======================================================
Controls an AZ/EL antenna rotator via an Arduino Nano over a serial link.
Displays live IMU-derived position data and logs readings to CSV.

Dependencies:  tkinter (stdlib), pyserial
  pip install pyserial

Serial protocol (115200 baud, ASCII, newline-terminated):
  Commands → Arduino:
    STATUS                   request current position / IMU data
    AZ <degrees>             move to azimuth target (0–360)
    EL <degrees>             move to elevation target (−90–90)
    MOVE AZ <f> EL <f>       move to both targets simultaneously
    STOP                     stop all motors immediately
    DECL <degrees>           update magnetic declination on the Arduino

  Responses ← Arduino:
    POS AZ=<f> EL=<f> HDG=<f> PITCH=<f> ROLL=<f> MOVING=<0|1>
    OK <echo>
    ERR <message>
    INIT MPU6050=<OK|FAIL> QMC5883L=<OK|FAIL>
"""

from __future__ import annotations

import csv
import os
import queue
import re
import sys
import threading
from datetime import datetime
from typing import Optional

import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit(
        "pyserial is not installed.  Run:  pip install pyserial\n"
        "Then restart this program."
    )

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
VERSION = "1.0.0"
DEFAULT_BAUD = 115200
BAUD_CHOICES = ["9600", "19200", "38400", "57600", "115200"]
DEFAULT_POLL_MS = 1000
MAX_CONSOLE_LINES = 500

# Matches: POS AZ=45.3 EL=15.2 HDG=44.8 PITCH=15.2 ROLL=2.1 MOVING=0
_STATUS_RE = re.compile(
    r"POS\s+"
    r"AZ=([\-\d.]+)\s+"
    r"EL=([\-\d.]+)\s+"
    r"HDG=([\-\d.]+)\s+"
    r"PITCH=([\-\d.]+)\s+"
    r"ROLL=([\-\d.]+)\s+"
    r"MOVING=([01])"
)

PROTOCOL_HELP = """\
Serial Protocol Reference (115200 baud, ASCII, newline-terminated)
==================================================================

Commands (Pi → Arduino):
  STATUS               Request current status / IMU data
  AZ <degrees>         Move to AZ target  (0 – 360)
  EL <degrees>         Move to EL target  (−90 – 90)
  MOVE AZ <f> EL <f>   Move to both targets simultaneously
  STOP                 Stop all motors immediately
  DECL <degrees>       Set magnetic declination (degrees, + east)

Responses (Arduino → Pi):
  POS AZ=<f> EL=<f> HDG=<f> PITCH=<f> ROLL=<f> MOVING=<0|1>
      Broadcast every ~500 ms and also sent on STATUS.
  OK <echo of command>
  ERR <message>
  INIT MPU6050=<OK|FAIL> QMC5883L=<OK|FAIL|DISABLED>  (at startup)

Field descriptions:
  AZ      Current azimuth from compass or dead-reckoning (0–360 °)
  EL      Current elevation from IMU pitch (−90–90 °)
  HDG     Tilt-compensated compass heading
  PITCH   IMU pitch angle
  ROLL    IMU roll angle
  MOVING  1 = motors running, 0 = idle

CSV log columns:
  timestamp, az_deg, el_deg, heading_deg, pitch_deg, roll_deg
"""


# ──────────────────────────────────────────────────────────────────────────────
# Background serial reader thread
# ──────────────────────────────────────────────────────────────────────────────
class SerialReaderThread(threading.Thread):
    """Daemon thread: reads lines from the serial port and puts them on a queue."""

    def __init__(self, port: serial.Serial, rx_queue: queue.Queue) -> None:
        super().__init__(daemon=True, name="SerialReader")
        self._port = port
        self._queue = rx_queue
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                if self._port and self._port.is_open:
                    raw = self._port.readline()
                    if raw:
                        text = raw.decode("utf-8", errors="replace").strip()
                        if text:
                            self._queue.put(("data", text))
            except serial.SerialException as exc:
                self._queue.put(("error", str(exc)))
                break
            except Exception:
                pass


# ──────────────────────────────────────────────────────────────────────────────
# Main application
# ──────────────────────────────────────────────────────────────────────────────
class AntennaControllerApp:
    """AZ/EL antenna rotator controller GUI."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(f"Antenna Rotator Controller  v{VERSION}")
        self.root.geometry("900x680")
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Serial state
        self._ser: Optional[serial.Serial] = None
        self._reader: Optional[SerialReaderThread] = None
        self._rx_queue: queue.Queue = queue.Queue()
        self._tx_lock = threading.Lock()

        # Live data
        self._az = 0.0
        self._el = 0.0
        self._hdg = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._moving = False

        # CSV logging
        self._log_handle: Optional[object] = None
        self._csv_writer: Optional[csv.writer] = None
        self._row_count = 0

        # Polling job handle
        self._poll_job: Optional[str] = None

        self._build_ui()
        self._refresh_ports()
        # Start queue drainer
        self._drain_queue()

    # ──────────────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self._build_menu()
        self._build_connection_bar()
        self._build_main_panel()
        self._build_logging_bar()
        self._build_console_panel()
        self._build_status_bar()

    def _build_menu(self) -> None:
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        file_menu = tk.Menu(menubar, tearoff=False)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Choose Log File…", command=self._browse_log)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self._on_close)

        view_menu = tk.Menu(menubar, tearoff=False)
        menubar.add_cascade(label="View", menu=view_menu)
        view_menu.add_command(label="Clear Console", command=self._clear_console)

        help_menu = tk.Menu(menubar, tearoff=False)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="Protocol Reference", command=self._show_protocol)
        help_menu.add_command(label="About", command=self._show_about)

    def _build_connection_bar(self) -> None:
        frm = ttk.LabelFrame(self.root, text="Connection", padding=6)
        frm.pack(fill=tk.X, padx=8, pady=(8, 2))

        ttk.Label(frm, text="Port:").pack(side=tk.LEFT)
        self._port_var = tk.StringVar()
        self._port_cb = ttk.Combobox(
            frm, textvariable=self._port_var, width=18, state="readonly"
        )
        self._port_cb.pack(side=tk.LEFT, padx=(2, 8))

        ttk.Label(frm, text="Baud:").pack(side=tk.LEFT)
        self._baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Combobox(
            frm, textvariable=self._baud_var, values=BAUD_CHOICES, width=8, state="readonly"
        ).pack(side=tk.LEFT, padx=(2, 12))

        self._connect_btn = ttk.Button(
            frm, text="Connect", command=self._toggle_connect, width=12
        )
        self._connect_btn.pack(side=tk.LEFT, padx=4)

        ttk.Button(frm, text="⟳ Ports", command=self._refresh_ports).pack(
            side=tk.LEFT, padx=4
        )

        # Indicator lamp
        self._indicator = tk.Label(
            frm, text="●", font=("TkDefaultFont", 16), fg="red",
            bg=frm.cget("background")
        )
        self._indicator.pack(side=tk.LEFT, padx=(12, 4))
        self._conn_label_var = tk.StringVar(value="Disconnected")
        ttk.Label(frm, textvariable=self._conn_label_var).pack(side=tk.LEFT)

    def _build_main_panel(self) -> None:
        outer = ttk.Frame(self.root)
        outer.pack(fill=tk.BOTH, expand=False, padx=8, pady=4)

        # ── Left: live position display ──────────────────────────────────────
        pos_frm = ttk.LabelFrame(outer, text="Current Position / IMU", padding=8)
        pos_frm.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        self._var_az    = self._make_readout(pos_frm, "Azimuth:",   "---°", 0)
        self._var_el    = self._make_readout(pos_frm, "Elevation:", "---°", 1)
        self._var_hdg   = self._make_readout(pos_frm, "Heading:",   "---°", 2)
        self._var_pitch = self._make_readout(pos_frm, "Pitch:",     "---°", 3)
        self._var_roll  = self._make_readout(pos_frm, "Roll:",      "---°", 4)

        self._moving_lbl = tk.Label(
            pos_frm, text="● Idle",
            font=("TkDefaultFont", 11, "bold"), fg="green"
        )
        self._moving_lbl.grid(row=5, column=0, columnspan=2, pady=(10, 0))

        # ── Right: motor controls ────────────────────────────────────────────
        ctrl_frm = ttk.LabelFrame(outer, text="Motor Control", padding=8)
        ctrl_frm.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # AZ row
        ttk.Label(ctrl_frm, text="AZ Target (°):").grid(
            row=0, column=0, sticky=tk.W, pady=3
        )
        self._az_target = tk.StringVar(value="0.0")
        ttk.Entry(ctrl_frm, textvariable=self._az_target, width=9).grid(
            row=0, column=1, padx=4
        )
        ttk.Button(ctrl_frm, text="Move AZ", command=self._cmd_move_az, width=10).grid(
            row=0, column=2, padx=4
        )

        # EL row
        ttk.Label(ctrl_frm, text="EL Target (°):").grid(
            row=1, column=0, sticky=tk.W, pady=3
        )
        self._el_target = tk.StringVar(value="0.0")
        ttk.Entry(ctrl_frm, textvariable=self._el_target, width=9).grid(
            row=1, column=1, padx=4
        )
        ttk.Button(ctrl_frm, text="Move EL", command=self._cmd_move_el, width=10).grid(
            row=1, column=2, padx=4
        )

        # Move Both
        ttk.Button(
            ctrl_frm, text="Move Both (AZ + EL)", command=self._cmd_move_both, width=24
        ).grid(row=2, column=0, columnspan=3, pady=6)

        # STOP (big, red)
        tk.Button(
            ctrl_frm, text="⏹  STOP",
            command=self._cmd_stop,
            bg="#cc2200", fg="white",
            font=("TkDefaultFont", 13, "bold"),
            width=18, height=2,
            activebackground="#ff4422", activeforeground="white",
            relief=tk.RAISED,
        ).grid(row=3, column=0, columnspan=3, pady=6)

        ttk.Button(
            ctrl_frm, text="⟳  Refresh Status", command=self._cmd_status, width=22
        ).grid(row=4, column=0, columnspan=3, pady=2)

        ttk.Separator(ctrl_frm, orient=tk.HORIZONTAL).grid(
            row=5, column=0, columnspan=3, sticky=tk.EW, pady=8
        )

        # Poll settings
        ttk.Label(ctrl_frm, text="Poll interval (ms):").grid(
            row=6, column=0, sticky=tk.W, pady=2
        )
        self._poll_ms = tk.IntVar(value=DEFAULT_POLL_MS)
        ttk.Spinbox(
            ctrl_frm, from_=200, to=10000, increment=100,
            textvariable=self._poll_ms, width=7
        ).grid(row=6, column=1, columnspan=2, sticky=tk.W)

        self._auto_poll = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            ctrl_frm, text="Auto-poll", variable=self._auto_poll,
            command=self._schedule_poll
        ).grid(row=7, column=0, columnspan=3, sticky=tk.W)

        # Declination
        ttk.Separator(ctrl_frm, orient=tk.HORIZONTAL).grid(
            row=8, column=0, columnspan=3, sticky=tk.EW, pady=8
        )
        ttk.Label(ctrl_frm, text="Mag Declination (°):").grid(
            row=9, column=0, sticky=tk.W, pady=2
        )
        self._decl_var = tk.StringVar(value="0.0")
        ttk.Entry(ctrl_frm, textvariable=self._decl_var, width=9).grid(
            row=9, column=1, padx=4
        )
        ttk.Button(ctrl_frm, text="Set", command=self._cmd_set_decl, width=6).grid(
            row=9, column=2
        )

    @staticmethod
    def _make_readout(parent: tk.Widget, label: str, initial: str, row: int) -> tk.StringVar:
        ttk.Label(parent, text=label, width=12, anchor=tk.E).grid(
            row=row, column=0, sticky=tk.E, pady=2
        )
        var = tk.StringVar(value=initial)
        ttk.Label(
            parent, textvariable=var, width=10, anchor=tk.W,
            font=("Courier", 13, "bold")
        ).grid(row=row, column=1, sticky=tk.W, padx=6)
        return var

    def _build_logging_bar(self) -> None:
        frm = ttk.LabelFrame(self.root, text="CSV Logging", padding=6)
        frm.pack(fill=tk.X, padx=8, pady=2)

        ttk.Label(frm, text="File:").pack(side=tk.LEFT)
        default_log = os.path.join(
            os.path.expanduser("~"), "antenna_log.csv"
        )
        self._log_path = tk.StringVar(value=default_log)
        ttk.Entry(frm, textvariable=self._log_path, width=42).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(frm, text="Browse…", command=self._browse_log).pack(
            side=tk.LEFT, padx=2
        )
        self._log_btn = ttk.Button(
            frm, text="Start Logging", command=self._toggle_logging, width=14
        )
        self._log_btn.pack(side=tk.LEFT, padx=8)

        self._log_count_var = tk.StringVar(value="0 rows logged")
        ttk.Label(frm, textvariable=self._log_count_var).pack(side=tk.LEFT)

    def _build_console_panel(self) -> None:
        frm = ttk.LabelFrame(self.root, text="Serial Console", padding=6)
        frm.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 6))

        toolbar = ttk.Frame(frm)
        toolbar.pack(fill=tk.X, pady=(0, 2))
        ttk.Button(toolbar, text="Clear", command=self._clear_console).pack(
            side=tk.RIGHT
        )

        self._console = scrolledtext.ScrolledText(
            frm, height=8, font=("Courier", 9),
            state="disabled", wrap=tk.NONE
        )
        self._console.pack(fill=tk.BOTH, expand=True)

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self.root, relief=tk.SUNKEN)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        self._status_var = tk.StringVar(value="Ready")
        ttk.Label(bar, textvariable=self._status_var, anchor=tk.W).pack(
            side=tk.LEFT, padx=4
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Port management
    # ──────────────────────────────────────────────────────────────────────────

    def _refresh_ports(self) -> None:
        ports = sorted(p.device for p in serial.tools.list_ports.comports())
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connect(self) -> None:
        if self._ser and self._ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self._port_var.get()
        if not port:
            messagebox.showerror("No Port Selected", "Please choose a serial port.")
            return
        baud = int(self._baud_var.get())
        try:
            self._ser = serial.Serial(port, baud, timeout=1)
            self._reader = SerialReaderThread(self._ser, self._rx_queue)
            self._reader.start()
            self._connect_btn.config(text="Disconnect")
            self._indicator.config(fg="green")
            self._conn_label_var.set(f"Connected — {port} @ {baud}")
            self._status_var.set(f"Connected: {port}")
            self._log_console(f"[Connected to {port} at {baud} baud]")
            self._schedule_poll()
        except serial.SerialException as exc:
            messagebox.showerror("Connection Error", str(exc))

    def _disconnect(self) -> None:
        if self._poll_job:
            self.root.after_cancel(self._poll_job)
            self._poll_job = None
        if self._reader:
            self._reader.stop()
            self._reader = None
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._connect_btn.config(text="Connect")
        self._indicator.config(fg="red")
        self._conn_label_var.set("Disconnected")
        self._status_var.set("Disconnected")
        self._log_console("[Disconnected]")

    # ──────────────────────────────────────────────────────────────────────────
    # Serial I/O
    # ──────────────────────────────────────────────────────────────────────────

    def _send(self, cmd: str) -> None:
        if not self._ser or not self._ser.is_open:
            self._log_console("[Not connected — command not sent]")
            return
        with self._tx_lock:
            try:
                self._ser.write((cmd.strip() + "\n").encode("utf-8"))
                self._log_console(f">> {cmd.strip()}")
            except serial.SerialException as exc:
                self._log_console(f"[TX Error: {exc}]")

    def _drain_queue(self) -> None:
        """Process up to all pending RX items; reschedule every 50 ms."""
        for _ in range(50):   # bound per cycle to keep UI responsive
            try:
                kind, text = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "data":
                self._handle_line(text)
            elif kind == "error":
                self._log_console(f"[Serial error: {text}]")
                self._disconnect()
        self.root.after(50, self._drain_queue)

    def _handle_line(self, line: str) -> None:
        self._log_console(f"<< {line}")
        m = _STATUS_RE.match(line)
        if m:
            az, el, hdg, pitch, roll, moving = m.groups()
            self._az      = float(az)
            self._el      = float(el)
            self._hdg     = float(hdg)
            self._pitch   = float(pitch)
            self._roll    = float(roll)
            self._moving  = moving == "1"
            self._refresh_display()
            if self._log_handle:
                self._write_csv_row()

    def _refresh_display(self) -> None:
        self._var_az.set(f"{self._az:.1f}°")
        self._var_el.set(f"{self._el:.1f}°")
        self._var_hdg.set(f"{self._hdg:.1f}°")
        self._var_pitch.set(f"{self._pitch:.1f}°")
        self._var_roll.set(f"{self._roll:.1f}°")
        if self._moving:
            self._moving_lbl.config(text="● Moving…", fg="orange")
        else:
            self._moving_lbl.config(text="● Idle", fg="green")

    # ──────────────────────────────────────────────────────────────────────────
    # Command helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _cmd_move_az(self) -> None:
        try:
            az = float(self._az_target.get())
            if not 0.0 <= az <= 360.0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid Input", "AZ must be 0 – 360 degrees.")
            return
        self._send(f"AZ {az:.1f}")

    def _cmd_move_el(self) -> None:
        try:
            el = float(self._el_target.get())
            if not -90.0 <= el <= 90.0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid Input", "EL must be −90 – 90 degrees.")
            return
        self._send(f"EL {el:.1f}")

    def _cmd_move_both(self) -> None:
        try:
            az = float(self._az_target.get())
            el = float(self._el_target.get())
            if not (0.0 <= az <= 360.0 and -90.0 <= el <= 90.0):
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid Input", "AZ: 0–360 °   EL: −90–90 °")
            return
        self._send(f"MOVE AZ {az:.1f} EL {el:.1f}")

    def _cmd_stop(self) -> None:
        self._send("STOP")

    def _cmd_status(self) -> None:
        self._send("STATUS")

    def _cmd_set_decl(self) -> None:
        try:
            decl = float(self._decl_var.get())
        except ValueError:
            messagebox.showerror("Invalid Input", "Declination must be a number.")
            return
        self._send(f"DECL {decl:.2f}")

    # ──────────────────────────────────────────────────────────────────────────
    # Polling
    # ──────────────────────────────────────────────────────────────────────────

    def _schedule_poll(self) -> None:
        if self._poll_job:
            self.root.after_cancel(self._poll_job)
            self._poll_job = None
        if self._auto_poll.get():
            interval = max(200, self._poll_ms.get())
            self._poll_job = self.root.after(interval, self._do_poll)

    def _do_poll(self) -> None:
        if self._ser and self._ser.is_open:
            self._cmd_status()
        self._schedule_poll()

    # ──────────────────────────────────────────────────────────────────────────
    # CSV Logging
    # ──────────────────────────────────────────────────────────────────────────

    def _browse_log(self) -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile="antenna_log.csv",
            title="Choose log file",
        )
        if path:
            self._log_path.set(path)

    def _toggle_logging(self) -> None:
        if self._log_handle:
            self._stop_logging()
        else:
            self._start_logging()

    def _start_logging(self) -> None:
        path = self._log_path.get().strip()
        if not path:
            messagebox.showerror("No File", "Please choose a log file path first.")
            return
        try:
            is_new = not os.path.exists(path)
            self._log_handle = open(path, "a", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._log_handle)
            if is_new:
                self._csv_writer.writerow(
                    ["timestamp", "az_deg", "el_deg", "heading_deg", "pitch_deg", "roll_deg"]
                )
                self._log_handle.flush()
            self._row_count = 0
            self._log_btn.config(text="Stop Logging")
            self._log_count_var.set(f"{self._row_count} rows logged")
            self._log_console(f"[Logging to: {path}]")
        except OSError as exc:
            messagebox.showerror("File Error", str(exc))
            self._log_handle = None
            self._csv_writer = None

    def _stop_logging(self) -> None:
        if self._log_handle:
            try:
                self._log_handle.close()
            except Exception:
                pass
            self._log_handle = None
            self._csv_writer = None
        self._log_btn.config(text="Start Logging")
        self._log_console("[Logging stopped]")

    def _write_csv_row(self) -> None:
        if not self._csv_writer:
            return
        ts = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        try:
            self._csv_writer.writerow([
                ts,
                f"{self._az:.2f}",
                f"{self._el:.2f}",
                f"{self._hdg:.2f}",
                f"{self._pitch:.2f}",
                f"{self._roll:.2f}",
            ])
            self._log_handle.flush()
            self._row_count += 1
            self._log_count_var.set(f"{self._row_count} rows logged")
        except OSError as exc:
            self._log_console(f"[Log write error: {exc}]")
            self._stop_logging()

    # ──────────────────────────────────────────────────────────────────────────
    # Console helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _log_console(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}]  {text}\n"
        self._console.config(state="normal")
        self._console.insert(tk.END, line)
        # Trim oldest lines to keep memory usage bounded
        total = int(self._console.index(tk.END).split(".")[0])
        if total > MAX_CONSOLE_LINES + 100:
            self._console.delete("1.0", f"{total - MAX_CONSOLE_LINES}.0")
        self._console.see(tk.END)
        self._console.config(state="disabled")

    def _clear_console(self) -> None:
        self._console.config(state="normal")
        self._console.delete("1.0", tk.END)
        self._console.config(state="disabled")

    # ──────────────────────────────────────────────────────────────────────────
    # Dialogs
    # ──────────────────────────────────────────────────────────────────────────

    def _show_protocol(self) -> None:
        win = tk.Toplevel(self.root)
        win.title("Protocol Reference")
        win.geometry("540x360")
        txt = scrolledtext.ScrolledText(win, font=("Courier", 9), wrap=tk.NONE)
        txt.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        txt.insert(tk.END, PROTOCOL_HELP)
        txt.config(state="disabled")

    def _show_about(self) -> None:
        messagebox.showinfo(
            "About",
            f"Antenna Rotator Controller  v{VERSION}\n\n"
            "Raspberry Pi desktop GUI for AZ/EL antenna control\n"
            "via Arduino Nano + MPU6050 + QMC5883L + L298N.\n\n"
            "Serial: 115200 baud, ASCII protocol.",
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Clean shutdown
    # ──────────────────────────────────────────────────────────────────────────

    def _on_close(self) -> None:
        self._disconnect()
        self._stop_logging()
        self.root.destroy()


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main() -> None:
    root = tk.Tk()
    AntennaControllerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
