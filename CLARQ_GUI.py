# drone_gui.py
# Runs on Windows alongside Mission Planner
# Sends MAVLink commands directly to Pixhawk via UDP from Mission Planner
# pip install pymavlink

import tkinter as tk
from tkinter import scrolledtext
from pymavlink import mavutil
import threading
import time

# ── Config ────────────────────────────────────────────────────
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

# ── Colors ────────────────────────────────────────────────────
BG     = "#0f0f16"
PANEL  = "#191926"
ACCENT = "#1ec882"
WARN   = "#dc6438"
TEXT   = "#d2d2dc"
DIM    = "#646478"
GREEN  = "#1a6b42"
RED    = "#6b1a1a"
BLUE   = "#1a3d6b"
PURPLE = "#4a1a6b"

class DroneGUI:
    def __init__(self, root):
        self.root      = root
        self.root.title("Drone MAVLink Control")
        self.root.geometry("1000x700")
        self.root.configure(bg=BG)

        self.mav       = None
        self.connected = False
        self.telem_on  = False

        self.build_ui()

    # ── UI ────────────────────────────────────────────────────
    def build_ui(self):

        # Title bar
        title = tk.Frame(self.root, bg=PANEL, pady=12)
        title.pack(fill=tk.X)

        tk.Label(
            title,
            text="DRONE MAVLINK CONTROL",
            font=("Consolas", 16, "bold"),
            bg=PANEL, fg=ACCENT
        ).pack(side=tk.LEFT, padx=20)

        self.conn_label = tk.Label(
            title,
            text="● DISCONNECTED",
            font=("Consolas", 12),
            bg=PANEL, fg=WARN
        )
        self.conn_label.pack(side=tk.RIGHT, padx=20)

        # Main layout
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # ── Left scrollable panel ─────────────────────────────
        left_outer = tk.Frame(main, bg=PANEL, width=280)
        left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_outer.pack_propagate(False)

        left_canvas = tk.Canvas(
            left_outer, bg=PANEL, width=260,
            highlightthickness=0)
        left_scrollbar = tk.Scrollbar(
            left_outer,
            orient=tk.VERTICAL,
            command=left_canvas.yview)
        left_canvas.configure(
            yscrollcommand=left_scrollbar.set)

        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        left = tk.Frame(left_canvas, bg=PANEL)
        left_canvas.create_window(
            (0, 0), window=left, anchor=tk.NW)

        def on_frame_configure(event):
            left_canvas.configure(
                scrollregion=left_canvas.bbox("all"))

        def on_mousewheel(event):
            left_canvas.yview_scroll(
                int(-1 * (event.delta / 120)), "units")

        left.bind("<Configure>", on_frame_configure)
        left_canvas.bind("<MouseWheel>", on_mousewheel)
        left.bind("<MouseWheel>", on_mousewheel)

        # ── Right panel ───────────────────────────────────────
        right = tk.Frame(main, bg=PANEL)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.build_buttons(left)
        self.build_output(right)

    def build_buttons(self, parent):

        def section(text):
            tk.Frame(parent, bg=DIM, height=1).pack(
                fill=tk.X, padx=15, pady=(15, 0))
            tk.Label(
                parent,
                text=text,
                font=("Consolas", 9),
                bg=PANEL, fg=DIM
            ).pack(anchor=tk.W, padx=15, pady=(4, 6))

        def btn(text, color, cmd, state=tk.NORMAL):
            b = tk.Button(
                parent,
                text=text,
                font=("Consolas", 12, "bold"),
                bg=color, fg=TEXT,
                activebackground=color,
                relief=tk.FLAT,
                cursor="hand2",
                pady=10,
                state=state,
                command=cmd
            )
            b.pack(fill=tk.X, padx=15, pady=3)
            return b

        # ── Connection ────────────────────────────────────────
        section("CONNECTION")

        conn_frame = tk.Frame(parent, bg=PANEL)
        conn_frame.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(
            conn_frame,
            text="Port:",
            font=("Consolas", 11),
            bg=PANEL, fg=TEXT
        ).pack(side=tk.LEFT)
        self.conn_entry = tk.Entry(
            conn_frame,
            font=("Consolas", 11),
            bg="#2a2a3a", fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT,
            width=18
        )
        self.conn_entry.insert(0, MAVLINK_CONNECTION)
        self.conn_entry.pack(side=tk.LEFT, padx=5)

        btn("CONNECT",    BLUE, self.connect)
        btn("DISCONNECT", RED,  self.disconnect)

        # ── Telemetry ─────────────────────────────────────────
        section("TELEMETRY")

        telem = tk.Frame(parent, bg="#12121e")
        telem.pack(fill=tk.X, padx=15, pady=5)

        self.telem = {}
        fields = [
            ("Mode",    "UNKNOWN"),
            ("Armed",   "NO"),
            ("Battery", "--V"),
            ("Alt",     "--m"),
            ("Lat",     "--"),
            ("Lon",     "--"),
            ("Heading", "--°"),
            ("Speed",   "--m/s"),
        ]

        for label, default in fields:
            row = tk.Frame(telem, bg="#12121e")
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(
                row,
                text=f"{label}:",
                font=("Consolas", 11),
                bg="#12121e", fg=DIM,
                width=9, anchor=tk.W
            ).pack(side=tk.LEFT)
            lbl = tk.Label(
                row,
                text=default,
                font=("Consolas", 11, "bold"),
                bg="#12121e", fg=ACCENT
            )
            lbl.pack(side=tk.LEFT)
            self.telem[label] = lbl

        # ── Flight modes ──────────────────────────────────────
        section("FLIGHT MODES")

        self.flight_btns = []
        for mode, color in [
            ("STABILIZE", BLUE),
            ("GUIDED",    BLUE),
            ("LOITER",    BLUE),
            ("RTL",       WARN),
            ("LAND",      WARN),
            ("AUTO",      BLUE),
            ("ALTHOLD",   BLUE),
            ("POSHOLD",   BLUE),
        ]:
            b = btn(
                mode, color,
                lambda m=mode: self.set_mode(m),
                tk.DISABLED)
            self.flight_btns.append(b)

        # ── Arm / Disarm ──────────────────────────────────────
        section("ARM / DISARM")

        self.arm_btn = btn(
            "ARM",    GREEN, self.arm,    tk.DISABLED)
        self.disarm_btn = btn(
            "DISARM", RED,   self.disarm, tk.DISABLED)

        # ── Commands ──────────────────────────────────────────
        section("COMMANDS")

        self.cmd_btns = []
        commands = [
            ("TAKEOFF 2m",        BLUE,   self.takeoff),
            ("TAKEOFF 5m",        BLUE,   self.takeoff_5),
            ("TAKEOFF 10m",       BLUE,   self.takeoff_10),
            ("RETURN TO LAUNCH",  WARN,   self.rtl),
            ("ACTIVATE LANDING",  PURPLE, self.activate_landing),
        ]
        for text, color, cmd in commands:
            b = btn(text, color, cmd, tk.DISABLED)
            self.cmd_btns.append(b)

        # ── Precision landing ─────────────────────────────────
        section("PRECISION LANDING")

        self.prec_btns = []
        prec_commands = [
            ("ENABLE PREC LAND",  GREEN, self.enable_prec_land),
            ("DISABLE PREC LAND", RED,   self.disable_prec_land),
        ]
        for text, color, cmd in prec_commands:
            b = btn(text, color, cmd, tk.DISABLED)
            self.prec_btns.append(b)

        # Bottom padding
        tk.Frame(parent, bg=PANEL, height=20).pack()

    def build_output(self, parent):

        header = tk.Frame(parent, bg=PANEL)
        header.pack(fill=tk.X, padx=10, pady=(10, 5))

        tk.Label(
            header,
            text="MAVLINK OUTPUT",
            font=("Consolas", 12),
            bg=PANEL, fg=DIM
        ).pack(side=tk.LEFT)

        tk.Button(
            header,
            text="CLEAR",
            font=("Consolas", 10),
            bg="#2a2a3a", fg=TEXT,
            relief=tk.FLAT,
            cursor="hand2",
            command=self.clear
        ).pack(side=tk.RIGHT, padx=5)

        tk.Button(
            header,
            text="SAVE LOG",
            font=("Consolas", 10),
            bg="#2a2a3a", fg=TEXT,
            relief=tk.FLAT,
            cursor="hand2",
            command=self.save_log
        ).pack(side=tk.RIGHT, padx=5)

        self.output = scrolledtext.ScrolledText(
            parent,
            font=("Consolas", 11),
            bg="#0a0a14",
            fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.output.pack(
            fill=tk.BOTH, expand=True, padx=10, pady=(0, 5))

        self.output.tag_config("info",   foreground=ACCENT)
        self.output.tag_config("warn",   foreground=WARN)
        self.output.tag_config("error",  foreground="#dc3838")
        self.output.tag_config("cmd",    foreground="#6496c8")
        self.output.tag_config("normal", foreground=TEXT)
        self.output.tag_config("dim",    foreground=DIM)

    # ── Logging ───────────────────────────────────────────────
    def log(self, message, tag="normal"):
        def _log():
            self.output.config(state=tk.NORMAL)
            ts = time.strftime("%H:%M:%S")
            self.output.insert(
                tk.END, f"[{ts}] {message}\n", tag)
            self.output.see(tk.END)
            self.output.config(state=tk.DISABLED)
        self.root.after(0, _log)

    def clear(self):
        self.output.config(state=tk.NORMAL)
        self.output.delete(1.0, tk.END)
        self.output.config(state=tk.DISABLED)

    def save_log(self):
        content = self.output.get(1.0, tk.END)
        fname   = f"mavlink_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        with open(fname, 'w') as f:
            f.write(content)
        self.log(f"Log saved to {fname}", "info")

    def update_telem(self, key, value):
        def _u():
            if key in self.telem:
                self.telem[key].config(text=value)
        self.root.after(0, _u)

    def enable_buttons(self):
        for b in (self.flight_btns +
                  self.cmd_btns +
                  self.prec_btns):
            b.config(state=tk.NORMAL)
        self.arm_btn.config(state=tk.NORMAL)
        self.disarm_btn.config(state=tk.NORMAL)

    # ── Connection ────────────────────────────────────────────
    def connect(self):
        def _connect():
            try:
                conn = self.conn_entry.get()
                self.log(f"Connecting to {conn}...", "info")
                self.mav = mavutil.mavlink_connection(conn)
                self.log("Waiting for heartbeat...", "dim")
                self.mav.wait_heartbeat(timeout=10)
                self.connected = True
                self.log(
                    f"Connected! "
                    f"System {self.mav.target_system} "
                    f"Component {self.mav.target_component}",
                    "info")
                self.root.after(0, lambda: self.conn_label.config(
                    text="● CONNECTED", fg=ACCENT))
                self.root.after(0, self.enable_buttons)
                self.start_telemetry()
            except Exception as e:
                self.log(f"Connection failed: {e}", "error")

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect(self):
        self.connected = False
        self.telem_on  = False
        self.log("Disconnected", "warn")
        self.root.after(0, lambda: self.conn_label.config(
            text="● DISCONNECTED", fg=WARN))

    # ── Telemetry loop ────────────────────────────────────────
    def start_telemetry(self):
        self.telem_on = True
        threading.Thread(
            target=self.telem_loop, daemon=True).start()

    def telem_loop(self):
        while self.telem_on and self.connected:
            try:
                msg = self.mav.recv_match(
                    type=[
                        'HEARTBEAT',
                        'SYS_STATUS',
                        'GLOBAL_POSITION_INT',
                        'VFR_HUD',
                        'STATUSTEXT'
                    ],
                    blocking=True,
                    timeout=1
                )
                if msg is None:
                    continue

                t = msg.get_type()

                if t == 'HEARTBEAT':
                    mode  = mavutil.mode_string_v10(msg)
                    armed = bool(
                        msg.base_mode &
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.update_telem('Mode',  mode)
                    self.update_telem(
                        'Armed', 'YES' if armed else 'NO')

                elif t == 'SYS_STATUS':
                    v = msg.voltage_battery / 1000.0
                    self.update_telem('Battery', f'{v:.1f}V')

                elif t == 'GLOBAL_POSITION_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000.0
                    hdg = msg.hdg / 100.0
                    self.update_telem('Lat',     f'{lat:.6f}')
                    self.update_telem('Lon',     f'{lon:.6f}')
                    self.update_telem('Alt',     f'{alt:.1f}m')
                    self.update_telem('Heading', f'{hdg:.0f}°')

                elif t == 'VFR_HUD':
                    self.update_telem(
                        'Speed', f'{msg.groundspeed:.1f}m/s')

                elif t == 'STATUSTEXT':
                    self.log(
                        f"FC: {msg.text.strip()}", "dim")

            except Exception:
                break

    # ── MAVLink commands ──────────────────────────────────────
    def set_mode(self, mode):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log(f"Setting mode: {mode}", "cmd")
        self.mav.set_mode(mode)
        self.log(f"Mode {mode} sent", "info")

    def arm(self):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log("Arming...", "warn")
        self.mav.arducopter_arm()
        self.log("Arm command sent", "info")

    def disarm(self):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log("Disarming...", "warn")
        self.mav.arducopter_disarm()
        self.log("Disarm command sent", "info")

    def takeoff(self):
        self._takeoff(2.0)

    def takeoff_5(self):
        self._takeoff(5.0)

    def takeoff_10(self):
        self._takeoff(10.0)

    def _takeoff(self, alt):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log(f"Sending takeoff to {alt}m...", "cmd")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )
        self.log(f"Takeoff {alt}m command sent", "info")

    def rtl(self):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log("Return to launch...", "warn")
        self.mav.set_mode("RTL")
        self.log("RTL command sent", "info")

    def activate_landing(self):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log("Activating precision landing...", "warn")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.log("Landing command sent!", "info")

    def enable_prec_land(self):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log("Enabling precision landing...", "cmd")
        self.mav.mav.param_set_send(
            self.mav.target_system,
            self.mav.target_component,
            b'PLND_ENABLED',
            1,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8
        )
        self.log("PLND_ENABLED = 1 sent", "info")

    def disable_prec_land(self):
        if not self.connected:
            self.log("Not connected!", "error")
            return
        self.log("Disabling precision landing...", "cmd")
        self.mav.mav.param_set_send(
            self.mav.target_system,
            self.mav.target_component,
            b'PLND_ENABLED',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8
        )
        self.log("PLND_ENABLED = 0 sent", "info")


# ── Run ───────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = DroneGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()