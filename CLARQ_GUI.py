# CLARQ_GUI.py
# Runs on Windows alongside Mission Planner
# Single UDP port 14550 for both send and receive.
# Commands:   GUI → NRF24L01 via Arduino (primary, direct to Jetson)
#             GUI → UDP 14550 → Mission Planner → Pixhawk (MAVLink fallback)
#
# Mission Planner setup:
#   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
#   (only ONE entry needed — no inbound port required)
#
# pip install pymavlink pyserial
#
# Hardware: Arduino Nano + NRF24L01 plugged into Windows USB
# Flash Arduino with clarq_rf_arduino.ino first

import tkinter as tk
from tkinter import scrolledtext
from pymavlink import mavutil
import threading
import time
import serial
import serial.tools.list_ports

# ── Config ────────────────────────────────────────────────────
# Single UDP port for both telemetry receive and command send
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"
ARDUINO_BAUD       = 115200

# ── Command IDs — must match clarq_rf_listener.py ─────────────
CMD_ID_PING         = 1
CMD_ID_LAND         = 2
CMD_ID_START_TAG    = 3
CMD_ID_STOP_TAG     = 4
CMD_ID_STOP_ALL     = 5
CMD_ID_LAUNCH       = 6
CMD_ID_LAUNCH_SIM   = 7
# ── New navigation commands ───────────────────────────────────
CMD_ID_SET_HOME     = 8   # run set_origin.py  — save home point
CMD_ID_START_SCAN   = 9   # run save_position.py — start mission scan
CMD_ID_SAVE_END     = 10  # save current pos as mission end point
CMD_ID_GO_HOME      = 11  # run return_land.py + activate AprilTag

CMD_START_LANDING   = "CLARQ_LAND"
CMD_START_APRILTAG  = "CLARQ_START_TAG"
CMD_STOP_APRILTAG   = "CLARQ_STOP_TAG"
CMD_PING            = "CLARQ_PING"
CMD_LAUNCH          = "CLARQ_LAUNCH"
CMD_LAUNCH_SIM      = "CLARQ_LAUNCH_SIM"
CMD_SET_HOME        = "CLARQ_SET_HOME"
CMD_START_SCAN      = "CLARQ_START_SCAN"
CMD_SAVE_END        = "CLARQ_SAVE_END"
CMD_GO_HOME         = "CLARQ_GO_HOME"

# ── Response IDs from Jetson via RF ───────────────────────────
RESP_PONG           = 1
RESP_LAUNCHED       = 6
RESP_LAUNCHED_SIM   = 7
RESP_STOPPED        = 5
RESP_HOME_SET       = 8
RESP_SCAN_STARTED   = 9
RESP_END_SAVED      = 10
RESP_GO_HOME        = 11

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
        self.root         = root
        self.root.title("CLARQ Drone Control")
        self.root.geometry("1000x800")
        self.root.configure(bg=BG)

        self.mav          = None
        self.mav_send     = None
        self.connected    = False
        self.telem_on     = False

        self.arduino      = None
        self.rf_connected = False
        self.rf_read_on   = False

        self.build_ui()

    def build_ui(self):
        title = tk.Frame(self.root, bg=PANEL, pady=12)
        title.pack(fill=tk.X)

        tk.Label(
            title,
            text="CLARQ DRONE NAVIGATION CONTROL",
            font=("Consolas", 16, "bold"),
            bg=PANEL, fg=ACCENT
        ).pack(side=tk.LEFT, padx=20)

        self.rf_label = tk.Label(
            title, text="RF ●",
            font=("Consolas", 11),
            bg=PANEL, fg=DIM)
        self.rf_label.pack(side=tk.RIGHT, padx=10)

        self.conn_label = tk.Label(
            title, text="● DISCONNECTED",
            font=("Consolas", 12),
            bg=PANEL, fg=WARN)
        self.conn_label.pack(side=tk.RIGHT, padx=20)

        main = tk.Frame(self.root, bg=BG)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_outer = tk.Frame(main, bg=PANEL, width=280)
        left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_outer.pack_propagate(False)

        left_canvas = tk.Canvas(
            left_outer, bg=PANEL, width=260,
            highlightthickness=0)
        left_scrollbar = tk.Scrollbar(
            left_outer, orient=tk.VERTICAL,
            command=left_canvas.yview)
        left_canvas.configure(
            yscrollcommand=left_scrollbar.set)
        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        left_canvas.pack(
            side=tk.LEFT, fill=tk.BOTH, expand=True)

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

        right = tk.Frame(main, bg=PANEL)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.build_buttons(left)
        self.build_output(right)

    def build_buttons(self, parent):

        def section(text):
            tk.Frame(parent, bg=DIM, height=1).pack(
                fill=tk.X, padx=15, pady=(15, 0))
            tk.Label(
                parent, text=text,
                font=("Consolas", 9),
                bg=PANEL, fg=DIM
            ).pack(anchor=tk.W, padx=15, pady=(4, 6))

        def btn(text, color, cmd, state=tk.NORMAL):
            b = tk.Button(
                parent, text=text,
                font=("Consolas", 12, "bold"),
                bg=color, fg=TEXT,
                activebackground=color,
                relief=tk.FLAT, cursor="hand2",
                pady=10, state=state, command=cmd)
            b.pack(fill=tk.X, padx=15, pady=3)
            return b

        # ── RF / Arduino ──────────────────────────────────────
        section("RF RADIO (NRF24L01)")

        port_frame = tk.Frame(parent, bg=PANEL)
        port_frame.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(
            port_frame, text="Port:",
            font=("Consolas", 11),
            bg=PANEL, fg=TEXT
        ).pack(side=tk.LEFT)
        self.rf_port_entry = tk.Entry(
            port_frame, font=("Consolas", 11),
            bg="#2a2a3a", fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT, width=10)
        self.rf_port_entry.insert(0, "COM3")
        self.rf_port_entry.pack(side=tk.LEFT, padx=5)

        btn("SCAN PORTS",    BLUE,  self.scan_ports)
        btn("CONNECT RF",    GREEN, self.connect_rf)
        btn("DISCONNECT RF", RED,   self.disconnect_rf)

        # ── Camera navigation mission ─────────────────────────
        section("CAMERA NAVIGATION")

        # Status labels showing current nav state
        self.nav_status_lbl = tk.Label(
            parent, text="Nav: IDLE",
            font=("Consolas", 11, "bold"),
            bg=PANEL, fg=DIM)
        self.nav_status_lbl.pack(
            anchor=tk.W, padx=15, pady=(2, 6))

        self.nav_btns = []

        b = btn(
            "1. SET HOME POINT", GREEN,
            self.set_home_point)
        self.nav_btns.append(b)

        b = btn(
            "2. START SCAN MISSION", BLUE,
            self.start_scan_mission)
        self.nav_btns.append(b)

        b = btn(
            "3. SAVE END POINT", PURPLE,
            self.save_end_point)
        self.nav_btns.append(b)

        b = btn(
            "4. GO HOME + LAND", WARN,
            self.go_home_and_land)
        self.nav_btns.append(b)

        # ── Jetson system ─────────────────────────────────────
        section("JETSON")
        btn("LAUNCH JETSON",     GREEN, self.launch_jetson)
        btn("LAUNCH JETSON SIM", BLUE,  self.launch_jetson_sim)
        btn("KILL JETSON",       RED,   self.kill_jetson)

        # ── MAVLink ───────────────────────────────────────────
        section("MAVLINK (TELEMETRY)")

        conn_frame = tk.Frame(parent, bg=PANEL)
        conn_frame.pack(fill=tk.X, padx=15, pady=4)
        tk.Label(
            conn_frame, text="Port:",
            font=("Consolas", 11),
            bg=PANEL, fg=TEXT
        ).pack(side=tk.LEFT)
        self.conn_entry = tk.Entry(
            conn_frame, font=("Consolas", 11),
            bg="#2a2a3a", fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT, width=18)
        self.conn_entry.insert(0, MAVLINK_CONNECTION)
        self.conn_entry.pack(side=tk.LEFT, padx=5)

        btn("CONNECT MAV",    BLUE, self.connect)
        btn("DISCONNECT MAV", RED,  self.disconnect)

        # ── Telemetry ─────────────────────────────────────────
        section("TELEMETRY")

        telem = tk.Frame(parent, bg="#12121e")
        telem.pack(fill=tk.X, padx=15, pady=5)

        self.telem = {}
        for label, default in [
            ("Mode",    "UNKNOWN"),
            ("Armed",   "NO"),
            ("Battery", "--V"),
            ("Alt",     "--m"),
            ("Lat",     "--"),
            ("Lon",     "--"),
            ("Heading", "--°"),
            ("Speed",   "--m/s"),
        ]:
            row = tk.Frame(telem, bg="#12121e")
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(
                row, text=f"{label}:",
                font=("Consolas", 11),
                bg="#12121e", fg=DIM,
                width=9, anchor=tk.W
            ).pack(side=tk.LEFT)
            lbl = tk.Label(
                row, text=default,
                font=("Consolas", 11, "bold"),
                bg="#12121e", fg=ACCENT)
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
            b = btn(mode, color,
                    lambda m=mode: self.set_mode(m),
                    tk.DISABLED)
            self.flight_btns.append(b)

        # ── Arm / Disarm ──────────────────────────────────────
        section("ARM / DISARM")

        self.arm_btn = btn(
            "ARM",       GREEN, self.arm,       tk.DISABLED)
        self.disarm_btn = btn(
            "DISARM",    RED,   self.disarm,    tk.DISABLED)
        self.force_arm_btn = btn(
            "FORCE ARM", WARN,  self.force_arm, tk.DISABLED)

        # ── Commands ──────────────────────────────────────────
        section("COMMANDS")

        self.cmd_btns = []
        for text, color, cmd in [
            ("TAKEOFF 2m",       BLUE, self.takeoff),
            ("TAKEOFF 5m",       BLUE, self.takeoff_5),
            ("TAKEOFF 10m",      BLUE, self.takeoff_10),
            ("RETURN TO LAUNCH", WARN, self.rtl),
        ]:
            b = btn(text, color, cmd, tk.DISABLED)
            self.cmd_btns.append(b)

        # ── Precision landing ─────────────────────────────────
        section("PRECISION LANDING")

        self.jetson_status_lbl = tk.Label(
            parent, text="Jetson: IDLE",
            font=("Consolas", 11, "bold"),
            bg=PANEL, fg=DIM)
        self.jetson_status_lbl.pack(
            anchor=tk.W, padx=15, pady=(4, 2))

        self.jetson_phase_lbl = tk.Label(
            parent, text="Phase: --",
            font=("Consolas", 10),
            bg=PANEL, fg=DIM)
        self.jetson_phase_lbl.pack(
            anchor=tk.W, padx=15, pady=(0, 6))

        self.prec_btns = []
        for text, color, cmd in [
            ("START PREC LAND", GREEN, self.start_precision_landing),
            ("STOP PREC LAND",  RED,   self.stop_precision_landing),
            ("PING JETSON",     BLUE,  self.ping_jetson),
        ]:
            b = btn(text, color, cmd, tk.DISABLED)
            self.prec_btns.append(b)

        tk.Frame(parent, bg=PANEL, height=20).pack()

    def build_output(self, parent):
        header = tk.Frame(parent, bg=PANEL)
        header.pack(fill=tk.X, padx=10, pady=(10, 5))

        tk.Label(
            header, text="OUTPUT LOG",
            font=("Consolas", 12),
            bg=PANEL, fg=DIM
        ).pack(side=tk.LEFT)

        tk.Button(
            header, text="CLEAR",
            font=("Consolas", 10),
            bg="#2a2a3a", fg=TEXT,
            relief=tk.FLAT, cursor="hand2",
            command=self.clear
        ).pack(side=tk.RIGHT, padx=5)

        tk.Button(
            header, text="SAVE LOG",
            font=("Consolas", 10),
            bg="#2a2a3a", fg=TEXT,
            relief=tk.FLAT, cursor="hand2",
            command=self.save_log
        ).pack(side=tk.RIGHT, padx=5)

        self.output = scrolledtext.ScrolledText(
            parent, font=("Consolas", 11),
            bg="#0a0a14", fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT, wrap=tk.WORD,
            state=tk.DISABLED)
        self.output.pack(
            fill=tk.BOTH, expand=True,
            padx=10, pady=(0, 5))

        self.output.tag_config("info",   foreground=ACCENT)
        self.output.tag_config("warn",   foreground=WARN)
        self.output.tag_config("error",  foreground="#dc3838")
        self.output.tag_config("cmd",    foreground="#6496c8")
        self.output.tag_config("rf",     foreground="#c850c8")
        self.output.tag_config("nav",    foreground="#50c8c8")
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
        fname = f"clarq_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        with open(fname, 'w') as f:
            f.write(content)
        self.log(f"Log saved to {fname}", "info")

    def update_telem(self, key, value):
        def _u():
            if key in self.telem:
                self.telem[key].config(text=value)
        self.root.after(0, _u)

    def update_nav_status(self, text, color=None):
        c = color or ACCENT
        def _u():
            self.nav_status_lbl.config(
                text=f"Nav: {text}", fg=c)
        self.root.after(0, _u)

    def update_jetson_status(self, resp_id):
        status_map = {
            RESP_PONG:         ("ONLINE ✓",        ACCENT,  "--"),
            RESP_LAUNCHED:     ("LAUNCHED ✓",       ACCENT,  "All systems starting"),
            RESP_LAUNCHED_SIM: ("LAUNCHED SIM ✓",   ACCENT,  "Sim stack starting"),
            RESP_STOPPED:      ("STOPPED",           DIM,     "All processes stopped"),
            RESP_HOME_SET:     ("HOME SET ✓",        ACCENT,  "origin_T0.json saved"),
            RESP_SCAN_STARTED: ("SCANNING ✓",        ACCENT,  "Mission scan running"),
            RESP_END_SAVED:    ("END SAVED ✓",       ACCENT,  "scan_end_pos.json saved"),
            RESP_GO_HOME:      ("RETURNING HOME...", WARN,    "Phase 1 — navigating"),
        }
        if resp_id in status_map:
            status, color, phase = status_map[resp_id]
            # Update nav status for navigation responses
            if resp_id in (RESP_HOME_SET, RESP_SCAN_STARTED,
                           RESP_END_SAVED, RESP_GO_HOME):
                self.update_nav_status(status, color)
            def _update():
                self.jetson_status_lbl.config(
                    text=f"Jetson: {status}", fg=color)
                self.jetson_phase_lbl.config(
                    text=f"Phase: {phase}", fg=color)
            self.root.after(0, _update)

    def enable_buttons(self):
        for b in (self.flight_btns +
                  self.cmd_btns +
                  self.prec_btns):
            b.config(state=tk.NORMAL)
        self.arm_btn.config(state=tk.NORMAL)
        self.disarm_btn.config(state=tk.NORMAL)
        self.force_arm_btn.config(state=tk.NORMAL)

    # ── RF / Arduino ──────────────────────────────────────────
    def scan_ports(self):
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.log("No serial ports found", "warn")
            return
        self.log("Available ports:", "info")
        for p in ports:
            self.log(f"  {p.device} — {p.description}", "dim")

    def connect_rf(self):
        def _connect():
            try:
                port = self.rf_port_entry.get()
                self.log(f"Connecting RF on {port}...", "rf")
                self.arduino = serial.Serial(
                    port, ARDUINO_BAUD, timeout=2)
                time.sleep(2)

                deadline = time.time() + 5
                while time.time() < deadline:
                    if self.arduino.in_waiting:
                        line = (self.arduino.readline()
                                .decode('utf-8', errors='ignore')
                                .strip())
                        if line == "CLARQ_RF_READY":
                            break
                        self.log(f"Arduino: {line}", "dim")

                self.rf_connected = True
                self.log("RF connected! Arduino ready.", "rf")
                self.root.after(
                    0, lambda: self.rf_label.config(
                        text="RF ●", fg=ACCENT))

                self.rf_read_on = True
                threading.Thread(
                    target=self._rf_read_loop,
                    daemon=True).start()

            except Exception as e:
                self.log(f"RF connect failed: {e}", "error")

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect_rf(self):
        self.rf_connected = False
        self.rf_read_on   = False
        if self.arduino:
            try:
                self.arduino.close()
            except Exception:
                pass
            self.arduino = None
        self.log("RF disconnected", "warn")
        self.root.after(
            0, lambda: self.rf_label.config(
                text="RF ●", fg=DIM))

    def _rf_read_loop(self):
        while self.rf_read_on and self.rf_connected:
            try:
                if self.arduino and self.arduino.in_waiting:
                    line = (self.arduino.readline()
                            .decode('utf-8', errors='ignore')
                            .strip())
                    if not line:
                        continue

                    if line.startswith("TX_OK:"):
                        cmd_id = int(line.split(":")[1])
                        self.log(
                            f"RF TX OK — cmd {cmd_id} sent",
                            "rf")
                    elif line.startswith("TX_FAIL:"):
                        cmd_id = int(line.split(":")[1])
                        self.log(
                            f"RF TX FAILED — cmd {cmd_id}",
                            "error")
                    elif line.startswith("RX:"):
                        resp_id = int(line.split(":")[1])
                        resp_names = {
                            1:  "CLARQ_PONG",
                            5:  "CLARQ_ALL_STOPPED",
                            6:  "CLARQ_LAUNCHED",
                            7:  "CLARQ_LAUNCHED_SIM",
                            8:  "CLARQ_HOME_SET",
                            9:  "CLARQ_SCAN_STARTED",
                            10: "CLARQ_END_SAVED",
                            11: "CLARQ_GOING_HOME",
                        }
                        name = resp_names.get(
                            resp_id, f"RESP_{resp_id}")
                        self.log(
                            f"JETSON RF: {name}", "info")
                        self.update_jetson_status(resp_id)
                    else:
                        self.log(f"Arduino: {line}", "dim")
                else:
                    time.sleep(0.02)
            except Exception as e:
                if self.rf_read_on:
                    self.log(f"RF read error: {e}", "error")
                break

    def send_rf_cmd(self, cmd_id):
        if not self.rf_connected or not self.arduino:
            return False
        try:
            self.arduino.write(f"CMD:{cmd_id}\n".encode())
            self.log(f"RF → Arduino CMD:{cmd_id}", "rf")
            return True
        except Exception as e:
            self.log(f"RF send error: {e}", "error")
            return False

    def send_jetson_cmd(self, cmd_id, text):
        """RF primary, MAVLink fallback."""
        self.log(
            f"Sending to Jetson: {text} (id={cmd_id})", "cmd")

        rf_sent = False
        if self.rf_connected:
            rf_sent = self.send_rf_cmd(cmd_id)

        if not rf_sent:
            self.log(
                "RF not available — trying MAVLink fallback",
                "warn")

        if self.connected and self.mav_send:
            try:
                self.mav_send.mav.command_long_send(
                    self.mav_send.target_system,
                    self.mav_send.target_component,
                    mavutil.mavlink.MAV_CMD_USER_1,
                    0, float(cmd_id),
                    0, 0, 0, 0, 0, 0)
                self.log(
                    f"MAVLink fallback cmd={cmd_id}", "dim")
            except Exception as e:
                self.log(
                    f"MAVLink fallback failed: {e}", "warn")

    # ── Camera navigation mission buttons ─────────────────────
    def set_home_point(self):
        """
        Step 1 — Tell Jetson to run set_origin.py.
        Saves current ZED pose as origin_T0.json.
        Gimbal auto-tilts to 30° after saving.
        """
        self.log("─" * 44, "nav")
        self.log("  STEP 1 — SET HOME POINT", "nav")
        self.log("  Jetson will run set_origin.py", "nav")
        self.log("  Gimbal tilts to 30° after save", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_SET_HOME, CMD_SET_HOME)
        self.update_nav_status("Setting home...", WARN)

    def start_scan_mission(self):
        """
        Step 2 — Tell Jetson to run save_position.py.
        Starts recording position during the mission scan.
        """
        self.log("─" * 44, "nav")
        self.log("  STEP 2 — START SCAN MISSION", "nav")
        self.log("  Jetson will run save_position.py", "nav")
        self.log("  Fly the mission area now", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_START_SCAN, CMD_START_SCAN)
        self.update_nav_status("Scanning...", ACCENT)

    def save_end_point(self):
        """
        Step 3 — Tell Jetson to save current position as
        mission end point (scan_end_pos.json).
        """
        self.log("─" * 44, "nav")
        self.log("  STEP 3 — SAVE END POINT", "nav")
        self.log("  Saves scan_end_pos.json on Jetson", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_SAVE_END, CMD_SAVE_END)
        self.update_nav_status("End point saved", ACCENT)

    def go_home_and_land(self):
        """
        Step 4 — Tell Jetson to run return_land.py and
        activate AprilTag precision landing.
        Drone navigates back to origin_T0 then lands on rover.
        """
        self.log("─" * 44, "nav")
        self.log("  STEP 4 — GO HOME + LAND", "nav")
        self.log("  Jetson runs return_land.py", "nav")
        self.log("  AprilTag activated automatically", "nav")
        self.log("  Phase 1 → ZED navigate to T0", "nav")
        self.log("  Phase 2 → AprilTag align at 90°", "nav")
        self.log("  Phase 3 → Land on rover", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_GO_HOME, CMD_GO_HOME)
        self.update_nav_status("Returning home...", WARN)
        # Also set GUIDED mode via MAVLink if connected
        if self.connected:
            self.set_mode("GUIDED")

    # ── MAVLink connection ────────────────────────────────────
    def connect(self):
        def _connect():
            try:
                conn = self.conn_entry.get()
                self.log(
                    f"Connecting MAVLink to {conn}...", "info")

                # Single connection handles both send and receive
                # pymavlink UDP is bidirectional — same port works
                # for telemetry IN and commands OUT
                self.mav = mavutil.mavlink_connection(conn)
                self.log("Waiting for heartbeat...", "dim")
                self.mav.wait_heartbeat(timeout=10)
                self.mav.target_system    = 1
                self.mav.target_component = 1

                # mav_send points to same connection
                self.mav_send = self.mav

                self.connected = True
                self.log(
                    f"Connected on {conn} "
                    f"(send + recv)", "info")

                self.root.after(
                    0, lambda: self.conn_label.config(
                        text="● CONNECTED", fg=ACCENT))
                self.root.after(0, self.enable_buttons)
                self.start_telemetry()

            except Exception as e:
                self.log(
                    f"MAVLink connection failed: {e}", "error")

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect(self):
        self.connected = False
        self.telem_on  = False
        self.log("MAVLink disconnected", "warn")
        self.root.after(
            0, lambda: self.conn_label.config(
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
                        'HEARTBEAT', 'SYS_STATUS',
                        'GLOBAL_POSITION_INT',
                        'VFR_HUD', 'STATUSTEXT'
                    ],
                    blocking=True, timeout=1)
                if msg is None:
                    continue

                t = msg.get_type()

                if t == 'STATUSTEXT':
                    text = msg.text.strip()
                    if text.startswith('CLARQ'):
                        self.log(
                            f"JETSON MAV: {text}", "info")
                    else:
                        self.log(f"FC: {text}", "dim")
                    continue

                if t == 'HEARTBEAT':
                    src = (msg.get_srcSystem()
                           if hasattr(msg, 'get_srcSystem')
                           else 1)
                    if src != 1:
                        continue
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

            except Exception:
                break

    # ── MAVLink flight commands ───────────────────────────────
    def set_mode(self, mode):
        if not self.connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log(f"Setting mode: {mode}", "cmd")
        mode_map = {
            "STABILIZE": 0, "ACRO": 1,  "ALTHOLD": 2,
            "AUTO":      3, "GUIDED": 4, "LOITER":  5,
            "RTL":       6, "CIRCLE": 7, "LAND":    9,
            "POSHOLD":  16,
        }
        if mode not in mode_map:
            self.log(f"Unknown mode: {mode}", "error")
            return

        mode_num = mode_map[mode]

        # Send via two methods for reliability
        # Method 1 — SET_MODE message
        self.mav_send.mav.set_mode_send(
            self.mav_send.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num)

        # Method 2 — COMMAND_LONG MAV_CMD_DO_SET_MODE
        # More reliable through Mission Planner forwarding
        self.mav_send.mav.command_long_send(
            self.mav_send.target_system,
            self.mav_send.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num,
            0, 0, 0, 0, 0)

        self.log(f"Mode {mode} ({mode_num}) sent", "info")

        # Watch for ACK in background to confirm
        def _watch_ack():
            try:
                ack = self.mav.recv_match(
                    type='COMMAND_ACK',
                    blocking=True,
                    timeout=3)
                if ack and ack.result == 0:
                    self.log(
                        f"Mode {mode} confirmed ✓", "info")
                elif ack:
                    self.log(
                        f"Mode {mode} rejected "
                        f"(result={ack.result})", "warn")
                else:
                    self.log(
                        f"Mode {mode} — no ACK received",
                        "dim")
            except Exception:
                pass
        threading.Thread(
            target=_watch_ack, daemon=True).start()

    def arm(self):
        if not self.connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log("Arming...", "warn")
        self.mav_send.arducopter_arm()
        self.log("Arm command sent", "info")

    def disarm(self):
        if not self.connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log("Disarming...", "warn")
        self.mav_send.arducopter_disarm()
        self.log("Disarm command sent", "info")

    def force_arm(self):
        if not self.connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log("Force arming...", "warn")
        self.mav_send.mav.command_long_send(
            self.mav_send.target_system,
            self.mav_send.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0)
        self.log("Force arm sent!", "info")

    def takeoff(self):    self._takeoff(2.0)
    def takeoff_5(self):  self._takeoff(5.0)
    def takeoff_10(self): self._takeoff(10.0)

    def _takeoff(self, alt):
        if not self.connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log(f"Sending takeoff to {alt}m...", "cmd")
        self.mav_send.mav.command_long_send(
            self.mav_send.target_system,
            self.mav_send.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt)
        self.log(f"Takeoff {alt}m sent", "info")

    def rtl(self):
        if not self.connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log("Return to launch...", "warn")
        self.set_mode("RTL")

    # ── Jetson commands ───────────────────────────────────────
    def ping_jetson(self):
        self.send_jetson_cmd(CMD_ID_PING, CMD_PING)

    def launch_jetson(self):
        self.send_jetson_cmd(CMD_ID_LAUNCH, CMD_LAUNCH)

    def launch_jetson_sim(self):
        self.send_jetson_cmd(CMD_ID_LAUNCH_SIM, CMD_LAUNCH_SIM)

    def kill_jetson(self):
        self.send_jetson_cmd(CMD_ID_STOP_ALL, "CLARQ_STOP_ALL")

    def start_precision_landing(self):
        self.log("=" * 44, "info")
        self.log("  PRECISION LANDING STARTED", "info")
        self.log("=" * 44, "info")
        self.send_jetson_cmd(CMD_ID_LAND, CMD_START_LANDING)
        if self.connected:
            self.set_mode("GUIDED")
        self.log("Jetson now handles phases 1-3", "dim")

    def stop_precision_landing(self):
        self.log("Stopping precision landing!", "warn")
        self.send_jetson_cmd(CMD_ID_STOP_TAG, CMD_STOP_APRILTAG)
        if self.connected:
            self.set_mode("LOITER")


# ── Run ───────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = DroneGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
