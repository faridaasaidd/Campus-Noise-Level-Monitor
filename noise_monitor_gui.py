"""
Campus Noise Monitor — PC GUI
Requires: pip install pyserial matplotlib

Usage: python noise_monitor_gui.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import queue
import time
import re

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# ── Colour palette ────────────────────────────────────────────────────────────
BG        = "#1e1e2e"
PANEL     = "#2a2a3e"
ACCENT    = "#7c6af7"
GREEN     = "#50fa7b"
YELLOW    = "#f1fa8c"
RED       = "#ff5555"
TEXT      = "#cdd6f4"
SUBTEXT   = "#6c7086"
BORDER    = "#45475a"

MAX_POINTS = 120   # ~12 seconds of history at 10 Hz

# ── Regex to parse log lines ──────────────────────────────────────────────────
LOG_RE = re.compile(
    r"\[(QUIET|WARN\s*|ALARM)\]\s+pp=\s*(\d+)\s+avg=\s*(\d+)\s+"
    r"WARN=(\d+)\s+ALARM=(\d+)\s+preset=(.+)"
)

class NoiseMonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Campus Noise Monitor")
        self.root.configure(bg=BG)
        self.root.minsize(900, 620)

        self.serial_port  = None
        self.rx_queue     = queue.Queue()
        self.running      = False
        self.read_thread  = None

        # Data series
        self.avg_history  = []
        self.pp_history   = []
        self.warn_line    = 100
        self.alarm_line   = 200
        self.state        = "QUIET"
        self._sliders_synced = False

        self._build_ui()
        self._refresh_ports()
        self.root.after(50, self._poll_queue)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Top bar ──
        top = tk.Frame(self.root, bg=BG, pady=8)
        top.pack(fill="x", padx=12)

        tk.Label(top, text="🔊 Campus Noise Monitor",
                 bg=BG, fg=TEXT, font=("Segoe UI", 14, "bold")).pack(side="left")

        # State badge
        self.state_var = tk.StringVar(value="QUIET")
        self.state_lbl = tk.Label(top, textvariable=self.state_var,
                                  bg=GREEN, fg=BG,
                                  font=("Segoe UI", 10, "bold"),
                                  padx=10, pady=3, relief="flat")
        self.state_lbl.pack(side="left", padx=14)

        # ── Main area: left panel + right chart ──
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=12, pady=(0, 8))

        left  = tk.Frame(main, bg=BG, width=280)
        left.pack(side="left", fill="y", padx=(0, 8))
        left.pack_propagate(False)

        right = tk.Frame(main, bg=BG)
        right.pack(side="left", fill="both", expand=True)

        self._build_connection_panel(left)
        self._build_threshold_panel(left)
        self._build_chart(right)
        self._build_log(right)

    def _panel(self, parent, title):
        frame = tk.LabelFrame(parent, text=f"  {title}  ",
                              bg=PANEL, fg=ACCENT,
                              font=("Segoe UI", 9, "bold"),
                              bd=1, relief="solid",
                              labelanchor="nw")
        frame.pack(fill="x", pady=(0, 8))
        return frame

    def _build_connection_panel(self, parent):
        p = self._panel(parent, "Connection")

        row = tk.Frame(p, bg=PANEL)
        row.pack(fill="x", padx=8, pady=6)

        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(row, textvariable=self.port_var, width=12,
                                     state="readonly")
        self.port_cb.pack(side="left")

        tk.Button(row, text="⟳", bg=PANEL, fg=TEXT, bd=0,
                  font=("Segoe UI", 11),
                  command=self._refresh_ports).pack(side="left", padx=4)

        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(row, textvariable=self.baud_var, width=8,
                     values=["9600","19200","38400","57600","115200"],
                     state="readonly").pack(side="left", padx=4)

        self.conn_btn = tk.Button(p, text="Connect",
                                  bg=ACCENT, fg="white",
                                  font=("Segoe UI", 9, "bold"),
                                  bd=0, padx=12, pady=4,
                                  command=self._toggle_connection)
        self.conn_btn.pack(padx=8, pady=(0, 8))

        self.conn_status = tk.Label(p, text="Disconnected",
                                    bg=PANEL, fg=SUBTEXT,
                                    font=("Segoe UI", 8))
        self.conn_status.pack(pady=(0, 6))

    def _build_threshold_panel(self, parent):
        p = self._panel(parent, "Thresholds")

        for label, attr, default, color in [
            ("WARN",  "warn",  100, YELLOW),
            ("ALARM", "alarm", 200, RED),
        ]:
            row = tk.Frame(p, bg=PANEL)
            row.pack(fill="x", padx=8, pady=4)

            tk.Label(row, text=label, bg=PANEL, fg=color,
                     font=("Segoe UI", 9, "bold"), width=6,
                     anchor="w").pack(side="left")

            var = tk.IntVar(value=default)
            setattr(self, f"{attr}_var", var)

            scale = tk.Scale(row, from_=100, to=4000,
                             orient="horizontal", variable=var,
                             bg=PANEL, fg=TEXT, highlightthickness=0,
                             troughcolor=BORDER, activebackground=color,
                             length=120, showvalue=False,
                             command=lambda v, a=attr: self._on_slider(a))
            scale.pack(side="left", padx=6)

            lbl = tk.Label(row, textvariable=var, bg=PANEL, fg=color,
                           font=("Segoe UI Mono", 9), width=5)
            lbl.pack(side="left")

            btn = tk.Button(row, text="Set",
                            bg=color, fg=BG,
                            font=("Segoe UI", 8, "bold"),
                            bd=0, padx=6,
                            command=lambda a=attr: self._send_threshold(a))
            btn.pack(side="left", padx=4)

    def _build_chart(self, parent):
        if not MATPLOTLIB_AVAILABLE:
            tk.Label(parent, text="matplotlib not installed\npip install matplotlib",
                     bg=BG, fg=SUBTEXT).pack(expand=True)
            self.canvas = None
            return

        fig = Figure(figsize=(5, 2.4), dpi=96, facecolor=BG)
        self.ax = fig.add_subplot(111)
        self.ax.set_facecolor(PANEL)
        self.ax.tick_params(colors=SUBTEXT, labelsize=7)
        for spine in self.ax.spines.values():
            spine.set_edgecolor(BORDER)
        self.ax.set_ylabel("ADC units", color=SUBTEXT, fontsize=7)
        self.ax.set_ylim(0, 500)

        self.line_avg,  = self.ax.plot([], [], color=ACCENT, lw=1.5, label="avg")
        self.line_pp,   = self.ax.plot([], [], color=SUBTEXT, lw=0.8,
                                       alpha=0.5, label="peak-peak")
        self.h_warn     = self.ax.axhline(self.warn_line,  color=YELLOW,
                                          lw=1, ls="--", label="WARN")
        self.h_alarm    = self.ax.axhline(self.alarm_line, color=RED,
                                          lw=1, ls="--", label="ALARM")
        self.ax.legend(loc="upper right", fontsize=7,
                       facecolor=PANEL, edgecolor=BORDER, labelcolor=TEXT)
        fig.tight_layout(pad=0.6)

        self.canvas = FigureCanvasTkAgg(fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="x", padx=0, pady=(0, 4))

    def _build_log(self, parent):
        lf = tk.LabelFrame(parent, text="  Log  ",
                           bg=PANEL, fg=ACCENT,
                           font=("Segoe UI", 9, "bold"),
                           bd=1, relief="solid", labelanchor="nw")
        lf.pack(fill="both", expand=True)

        self.log_box = scrolledtext.ScrolledText(
            lf, bg="#11111b", fg=TEXT,
            font=("Consolas", 8), bd=0,
            state="disabled", wrap="none",
            height=6)
        self.log_box.pack(fill="both", expand=True, padx=4, pady=4)

        # Tag colours
        self.log_box.tag_config("quiet", foreground=GREEN)
        self.log_box.tag_config("warn",  foreground=YELLOW)
        self.log_box.tag_config("alarm", foreground=RED)
        self.log_box.tag_config("ok",    foreground=GREEN)
        self.log_box.tag_config("err",   foreground=RED)
        self.log_box.tag_config("info",  foreground=ACCENT)

        btns = tk.Frame(lf, bg=PANEL)
        btns.pack(fill="x", padx=4, pady=(0, 4))
        tk.Button(btns, text="Clear log", bg=BORDER, fg=TEXT,
                  font=("Segoe UI", 8), bd=0, padx=8,
                  command=self._clear_log).pack(side="right")

    # ── Serial connection ─────────────────────────────────────────────────────

    def _refresh_ports(self):
        if not SERIAL_AVAILABLE:
            self.port_cb["values"] = ["pyserial not installed"]
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if not SERIAL_AVAILABLE:
            self._log("pyserial not installed — pip install pyserial", "err")
            return
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            self.conn_btn.config(text="Disconnect", bg=RED)
            self.conn_status.config(text=f"Connected — {port} @ {baud}", fg=GREEN)
            self._log(f"Connected to {port} @ {baud}", "info")
        except Exception as e:
            self._log(f"Connect failed: {e}", "err")

    def _disconnect(self):
        self.running = False
        self._sliders_synced = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.conn_btn.config(text="Connect", bg=ACCENT)
        self.conn_status.config(text="Disconnected", fg=SUBTEXT)
        self._log("Disconnected", "info")

    def _read_loop(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode("ascii", errors="replace").strip()
                    if line:
                        self.rx_queue.put(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.rx_queue.put(f"[READ ERROR] {e}")
                break

    # ── Queue polling (runs on main thread) ───────────────────────────────────

    def _poll_queue(self):
        updated = False
        try:
            for _ in range(20):          # drain up to 20 lines per cycle
                line = self.rx_queue.get_nowait()
                self._handle_line(line)
                updated = True
        except queue.Empty:
            pass
        if updated and self.canvas:
            self._redraw_chart()
        self.root.after(50, self._poll_queue)

    def _handle_line(self, line):
        m = LOG_RE.match(line)
        if m:
            state, pp, avg, warn, alarm, preset = m.groups()
            state = state.strip()
            self.avg_history.append(int(avg))
            self.pp_history.append(int(pp))
            if len(self.avg_history) > MAX_POINTS:
                self.avg_history.pop(0)
                self.pp_history.pop(0)
            self.warn_line  = int(warn)
            self.alarm_line = int(alarm)
            # Sync sliders once on first line so they start at firmware values
            if not self._sliders_synced:
                self.warn_var.set(int(warn))
                self.alarm_var.set(int(alarm))
                self._sliders_synced = True
            # State badge
            self.state_var.set(state)
            colors = {"QUIET": GREEN, "WARN": YELLOW, "ALARM": RED}
            self.state_lbl.config(bg=colors.get(state, GREEN))
            tag = state.lower().rstrip()
            self._log(line, tag)
        elif line.strip() == "OK":
            self._log(line, "ok")
        elif line.strip() == "ERR":
            self._log(line, "err")
        else:
            self._log(line, "info")

    # ── Chart ─────────────────────────────────────────────────────────────────

    def _redraw_chart(self):
        if not self.canvas:
            return
        xs = list(range(len(self.avg_history)))
        self.line_avg.set_data(xs, self.avg_history)
        self.line_pp.set_data(xs, self.pp_history)
        self.h_warn.set_ydata([self.warn_line,  self.warn_line])
        self.h_alarm.set_ydata([self.alarm_line, self.alarm_line])
        self.ax.set_xlim(0, max(MAX_POINTS, len(xs)))
        top = max(max(self.avg_history or [0]),
                  max(self.pp_history  or [0]),
                  self.alarm_line) * 1.15
        self.ax.set_ylim(0, max(top, 200))
        self.canvas.draw_idle()

    # ── Controls ──────────────────────────────────────────────────────────────

    def _on_slider(self, attr):
        pass   # live preview; send on button press

    def _send_threshold(self, attr):
        val = self.warn_var.get() if attr == "warn" else self.alarm_var.get()
        # Use live firmware values for validation
        current_warn  = self.warn_line
        current_alarm = self.alarm_line
        if attr == "warn" and val >= current_alarm:
            self._log(f"WARN ({val}) must be less than current ALARM ({current_alarm})", "err")
            return
        if attr == "alarm" and val <= current_warn:
            self._log(f"ALARM ({val}) must be greater than current WARN ({current_warn})", "err")
            return
        cmd = f"SET THR {'WARN' if attr == 'warn' else 'ALARM'} {val}\r\n"
        self._log(f"Sending: {cmd.strip()}", "info")
        self._send(cmd)

    def _send(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd.encode("ascii"))
            except Exception as e:
                self._log(f"Send error: {e}", "err")
        else:
            self._log("Not connected", "err")

    # ── Log ───────────────────────────────────────────────────────────────────

    def _log(self, text, tag="info"):
        self.log_box.config(state="normal")
        self.log_box.insert("end", text + "\n", tag)
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def _clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    app  = NoiseMonitorApp(root)

    # Style ttk widgets
    style = ttk.Style()
    style.theme_use("clam")
    style.configure("TCombobox",
                    fieldbackground=PANEL, background=PANEL,
                    foreground=TEXT, selectbackground=ACCENT)

    root.mainloop()

if __name__ == "__main__":
    main()