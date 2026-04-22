# # CLARQ_GUI.py
# # Runs on Windows alongside Mission Planner
# #
# # Command routing (nRF path):
# #   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
# #
# # Telemetry routing (MAVLink receive only):
# #   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
# #
# # Mission Planner setup:
# #   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
# #
# # pip install pymavlink pyserial

# import tkinter as tk
# from tkinter import scrolledtext, ttk
# from pymavlink import mavutil
# import threading
# import time
# import serial
# import serial.tools.list_ports

# # ── Config ────────────────────────────────────────────────────
# MAVLINK_CONNECTION = "udp:127.0.0.1:14550"   # receive telemetry from MP
# SERIAL_BAUD        = 115200                   # must match Arduino sketch

# # ── Command IDs — must match clarq_rf_listener.py ─────────────
# CMD_ID_PING       = 1
# CMD_ID_LAND       = 2
# CMD_ID_START_TAG  = 3
# CMD_ID_STOP_TAG   = 4
# CMD_ID_STOP_ALL   = 5
# CMD_ID_LAUNCH     = 6
# CMD_ID_LAUNCH_SIM = 7
# CMD_ID_SET_HOME   = 8
# CMD_ID_START_SCAN = 9
# CMD_ID_SAVE_END   = 10
# CMD_ID_GO_HOME    = 11

# CMD_START_LANDING  = "CLARQ_LAND"
# CMD_START_APRILTAG = "CLARQ_START_TAG"
# CMD_STOP_APRILTAG  = "CLARQ_STOP_TAG"
# CMD_PING           = "CLARQ_PING"
# CMD_LAUNCH         = "CLARQ_LAUNCH"
# CMD_LAUNCH_SIM     = "CLARQ_LAUNCH_SIM"
# CMD_SET_HOME       = "CLARQ_SET_HOME"
# CMD_START_SCAN     = "CLARQ_START_SCAN"
# CMD_SAVE_END       = "CLARQ_SAVE_END"
# CMD_GO_HOME        = "CLARQ_GO_HOME"

# # ── Jetson response map (STATUSTEXT → status label) ───────────
# JETSON_STATUS_MAP = {
#     "CLARQ_PONG":         ("ONLINE ✓",          "#1ec882", "--"),
#     "CLARQ_HOME_SET":     ("HOME SET ✓",         "#1ec882", "origin_T0.json saved"),
#     "CLARQ_SCAN_STARTED": ("SCANNING ✓",         "#1ec882", "Mission scan running"),
#     "CLARQ_END_SAVED":    ("END SAVED ✓",        "#1ec882", "scan_end_pos.json saved"),
#     "CLARQ_GOING_HOME":   ("RETURNING HOME...",  "#dc6438", "Phase 1 — navigating"),
#     "CLARQ_LAUNCHED":     ("LAUNCHED ✓",         "#1ec882", "All systems starting"),
#     "CLARQ_LAUNCHED_SIM": ("LAUNCHED SIM ✓",     "#1ec882", "Sim stack starting"),
#     "CLARQ_ALL_STOPPED":  ("STOPPED",            "#646478", "All processes stopped"),
#     "CLARQ_TAG_RUNNING":  ("TAG RUNNING ✓",      "#1ec882", "AprilTag active"),
#     "CLARQ_TAG_STOPPED":  ("TAG STOPPED",        "#646478", "AprilTag stopped"),
# }

# # ── RX response codes from Arduino ────────────────────────────
# # Arduino sends back: TX_OK:<id>, TX_FAIL:<id>, RX:<id>
# ARDUINO_RX_MAP = {
#     CMD_ID_PING:       "CLARQ_PONG",
#     CMD_ID_LAND:       "CLARQ_LAND_ACK",
#     CMD_ID_START_TAG:  "CLARQ_TAG_RUNNING",
#     CMD_ID_STOP_TAG:   "CLARQ_TAG_STOPPED",
#     CMD_ID_STOP_ALL:   "CLARQ_ALL_STOPPED",
#     CMD_ID_LAUNCH:     "CLARQ_LAUNCHED",
#     CMD_ID_LAUNCH_SIM: "CLARQ_LAUNCHED_SIM",
#     CMD_ID_SET_HOME:   "CLARQ_HOME_SET",
#     CMD_ID_START_SCAN: "CLARQ_SCAN_STARTED",
#     CMD_ID_SAVE_END:   "CLARQ_END_SAVED",
#     CMD_ID_GO_HOME:    "CLARQ_GOING_HOME",
# }

# # ── Colors ────────────────────────────────────────────────────
# BG     = "#0f0f16"
# PANEL  = "#191926"
# ACCENT = "#1ec882"
# WARN   = "#dc6438"
# TEXT   = "#d2d2dc"
# DIM    = "#646478"
# GREEN  = "#1a6b42"
# RED    = "#6b1a1a"
# BLUE   = "#1a3d6b"
# PURPLE = "#4a1a6b"


# class DroneGUI:
#     def __init__(self, root):
#         self.root      = root
#         self.root.title("CLARQ Drone Control")
#         self.root.geometry("1000x800")
#         self.root.configure(bg=BG)

#         # MAVLink (telemetry receive only)
#         self.mav       = None
#         self.mav_connected = False
#         self.telem_on  = False

#         # nRF Arduino serial bridge
#         self.ser       = None
#         self.rf_connected = False

#         self.build_ui()

#     # ═══════════════════════════════════════════════════════════
#     # UI BUILD
#     # ═══════════════════════════════════════════════════════════
#     def build_ui(self):
#         title = tk.Frame(self.root, bg=PANEL, pady=12)
#         title.pack(fill=tk.X)

#         tk.Label(
#             title,
#             text="CLARQ DRONE NAVIGATION CONTROL",
#             font=("Consolas", 16, "bold"),
#             bg=PANEL, fg=ACCENT
#         ).pack(side=tk.LEFT, padx=20)

#         # Status indicators (right side of title bar)
#         status_frame = tk.Frame(title, bg=PANEL)
#         status_frame.pack(side=tk.RIGHT, padx=20)

#         self.rf_label = tk.Label(
#             status_frame, text="● RF DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.rf_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.conn_label = tk.Label(
#             status_frame, text="● MAV DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.conn_label.pack(side=tk.RIGHT)

#         main = tk.Frame(self.root, bg=BG)
#         main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

#         left_outer = tk.Frame(main, bg=PANEL, width=280)
#         left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
#         left_outer.pack_propagate(False)

#         left_canvas = tk.Canvas(
#             left_outer, bg=PANEL, width=260,
#             highlightthickness=0)
#         left_scrollbar = tk.Scrollbar(
#             left_outer, orient=tk.VERTICAL,
#             command=left_canvas.yview)
#         left_canvas.configure(
#             yscrollcommand=left_scrollbar.set)
#         left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
#         left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         left = tk.Frame(left_canvas, bg=PANEL)
#         left_canvas.create_window(
#             (0, 0), window=left, anchor=tk.NW)

#         def on_frame_configure(event):
#             left_canvas.configure(
#                 scrollregion=left_canvas.bbox("all"))

#         def on_mousewheel(event):
#             left_canvas.yview_scroll(
#                 int(-1 * (event.delta / 120)), "units")

#         left.bind("<Configure>", on_frame_configure)
#         left_canvas.bind("<MouseWheel>", on_mousewheel)
#         left.bind("<MouseWheel>", on_mousewheel)

#         right = tk.Frame(main, bg=PANEL)
#         right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         self.build_buttons(left)
#         self.build_output(right)

#     def build_buttons(self, parent):

#         def section(text):
#             tk.Frame(parent, bg=DIM, height=1).pack(
#                 fill=tk.X, padx=15, pady=(15, 0))
#             tk.Label(
#                 parent, text=text,
#                 font=("Consolas", 9),
#                 bg=PANEL, fg=DIM
#             ).pack(anchor=tk.W, padx=15, pady=(4, 6))

#         def btn(text, color, cmd, state=tk.NORMAL):
#             b = tk.Button(
#                 parent, text=text,
#                 font=("Consolas", 12, "bold"),
#                 bg=color, fg=TEXT,
#                 activebackground=color,
#                 relief=tk.FLAT, cursor="hand2",
#                 pady=10, state=state, command=cmd)
#             b.pack(fill=tk.X, padx=15, pady=3)
#             return b

#         # ── nRF / Arduino serial connection ───────────────────
#         section("NRF RADIO (COMMAND LINK)")

#         port_frame = tk.Frame(parent, bg=PANEL)
#         port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.port_var = tk.StringVar()
#         self.port_combo = ttk.Combobox(
#             port_frame,
#             textvariable=self.port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         self.refresh_ports()

#         btn("CONNECT RF",    BLUE, self.connect_rf)
#         btn("DISCONNECT RF", RED,  self.disconnect_rf)

#         # ── MAVLink telemetry connection ───────────────────────
#         section("MAVLINK (TELEMETRY ONLY)")

#         conn_frame = tk.Frame(parent, bg=PANEL)
#         conn_frame.pack(fill=tk.X, padx=15, pady=4)
#         tk.Label(
#             conn_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)
#         self.conn_entry = tk.Entry(
#             conn_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=22)
#         self.conn_entry.insert(0, MAVLINK_CONNECTION)
#         self.conn_entry.pack(side=tk.LEFT, padx=5)

#         btn("CONNECT MAV",    BLUE, self.connect_mav)
#         btn("DISCONNECT MAV", RED,  self.disconnect_mav)

#         # ── Camera navigation ─────────────────────────────────
#         section("CAMERA NAVIGATION")

#         self.jetson_status_lbl = tk.Label(
#             parent, text="Jetson: IDLE",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.jetson_status_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 0))

#         self.jetson_phase_lbl = tk.Label(
#             parent, text="Phase: --",
#             font=("Consolas", 10),
#             bg=PANEL, fg=DIM)
#         self.jetson_phase_lbl.pack(
#             anchor=tk.W, padx=15, pady=(0, 6))

#         self.nav_btns = []
#         b = btn("1. SET HOME POINT",     GREEN,  self.set_home_point)
#         self.nav_btns.append(b)
#         b = btn("2. START SCAN MISSION", BLUE,   self.start_scan_mission)
#         self.nav_btns.append(b)
#         b = btn("3. SAVE END POINT",     PURPLE, self.save_end_point)
#         self.nav_btns.append(b)
#         b = btn("4. GO HOME + LAND",     WARN,   self.go_home_and_land)
#         self.nav_btns.append(b)

#         # ── Jetson ────────────────────────────────────────────
#         section("JETSON")
#         btn("LAUNCH JETSON",     GREEN, self.launch_jetson)
#         btn("LAUNCH JETSON SIM", BLUE,  self.launch_jetson_sim)
#         btn("KILL JETSON",       RED,   self.kill_jetson)
#         btn("PING JETSON",       BLUE,  self.ping_jetson)

#         # ── Telemetry ─────────────────────────────────────────
#         section("TELEMETRY")

#         telem = tk.Frame(parent, bg="#12121e")
#         telem.pack(fill=tk.X, padx=15, pady=5)

#         self.telem = {}
#         for label, default in [
#             ("Mode",    "UNKNOWN"),
#             ("Armed",   "NO"),
#             ("Battery", "--V"),
#             ("Alt",     "--m"),
#             ("Lat",     "--"),
#             ("Lon",     "--"),
#             ("Heading", "--°"),
#             ("Speed",   "--m/s"),
#         ]:
#             row = tk.Frame(telem, bg="#12121e")
#             row.pack(fill=tk.X, padx=10, pady=2)
#             tk.Label(
#                 row, text=f"{label}:",
#                 font=("Consolas", 11),
#                 bg="#12121e", fg=DIM,
#                 width=9, anchor=tk.W
#             ).pack(side=tk.LEFT)
#             lbl = tk.Label(
#                 row, text=default,
#                 font=("Consolas", 11, "bold"),
#                 bg="#12121e", fg=ACCENT)
#             lbl.pack(side=tk.LEFT)
#             self.telem[label] = lbl

#         # ── Flight modes ──────────────────────────────────────
#         section("FLIGHT MODES")

#         self.flight_btns = []
#         for mode, color in [
#             ("STABILIZE", BLUE),
#             ("GUIDED",    BLUE),
#             ("LOITER",    BLUE),
#             ("RTL",       WARN),
#             ("LAND",      WARN),
#             ("AUTO",      BLUE),
#             ("ALTHOLD",   BLUE),
#             ("POSHOLD",   BLUE),
#         ]:
#             b = btn(mode, color,
#                     lambda m=mode: self.set_mode(m),
#                     tk.DISABLED)
#             self.flight_btns.append(b)

#         # ── Arm / Disarm ──────────────────────────────────────
#         section("ARM / DISARM")

#         self.arm_btn       = btn("ARM",       GREEN, self.arm,       tk.DISABLED)
#         self.disarm_btn    = btn("DISARM",    RED,   self.disarm,    tk.DISABLED)
#         self.force_arm_btn = btn("FORCE ARM", WARN,  self.force_arm, tk.DISABLED)

#         # ── Commands ──────────────────────────────────────────
#         section("COMMANDS")

#         self.cmd_btns = []
#         for text, color, cmd in [
#             ("TAKEOFF 2m",       BLUE, self.takeoff),
#             ("TAKEOFF 5m",       BLUE, self.takeoff_5),
#             ("TAKEOFF 10m",      BLUE, self.takeoff_10),
#             ("RETURN TO LAUNCH", WARN, self.rtl),
#         ]:
#             b = btn(text, color, cmd, tk.DISABLED)
#             self.cmd_btns.append(b)

#         tk.Frame(parent, bg=PANEL, height=20).pack()

#     def build_output(self, parent):
#         header = tk.Frame(parent, bg=PANEL)
#         header.pack(fill=tk.X, padx=10, pady=(10, 5))

#         tk.Label(
#             header, text="OUTPUT LOG",
#             font=("Consolas", 12),
#             bg=PANEL, fg=DIM
#         ).pack(side=tk.LEFT)

#         tk.Button(
#             header, text="CLEAR",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.clear
#         ).pack(side=tk.RIGHT, padx=5)

#         tk.Button(
#             header, text="SAVE LOG",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.save_log
#         ).pack(side=tk.RIGHT, padx=5)

#         self.output = scrolledtext.ScrolledText(
#             parent, font=("Consolas", 11),
#             bg="#0a0a14", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, wrap=tk.WORD,
#             state=tk.DISABLED)
#         self.output.pack(
#             fill=tk.BOTH, expand=True,
#             padx=10, pady=(0, 5))

#         self.output.tag_config("info",   foreground=ACCENT)
#         self.output.tag_config("warn",   foreground=WARN)
#         self.output.tag_config("error",  foreground="#dc3838")
#         self.output.tag_config("cmd",    foreground="#6496c8")
#         self.output.tag_config("nav",    foreground="#50c8c8")
#         self.output.tag_config("normal", foreground=TEXT)
#         self.output.tag_config("dim",    foreground=DIM)

#     # ═══════════════════════════════════════════════════════════
#     # LOGGING
#     # ═══════════════════════════════════════════════════════════
#     def log(self, message, tag="normal"):
#         def _log():
#             self.output.config(state=tk.NORMAL)
#             ts = time.strftime("%H:%M:%S")
#             self.output.insert(
#                 tk.END, f"[{ts}] {message}\n", tag)
#             self.output.see(tk.END)
#             self.output.config(state=tk.DISABLED)
#         self.root.after(0, _log)

#     def clear(self):
#         self.output.config(state=tk.NORMAL)
#         self.output.delete(1.0, tk.END)
#         self.output.config(state=tk.DISABLED)

#     def save_log(self):
#         content = self.output.get(1.0, tk.END)
#         fname = f"clarq_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
#         with open(fname, 'w') as f:
#             f.write(content)
#         self.log(f"Log saved to {fname}", "info")

#     def update_telem(self, key, value):
#         def _u():
#             if key in self.telem:
#                 self.telem[key].config(text=value)
#         self.root.after(0, _u)

#     def update_jetson_status(self, text):
#         if text in JETSON_STATUS_MAP:
#             status, color, phase = JETSON_STATUS_MAP[text]
#             def _u():
#                 self.jetson_status_lbl.config(
#                     text=f"Jetson: {status}", fg=color)
#                 self.jetson_phase_lbl.config(
#                     text=f"Phase: {phase}", fg=color)
#             self.root.after(0, _u)

#     def enable_mav_buttons(self):
#         for b in (self.flight_btns + self.cmd_btns):
#             b.config(state=tk.NORMAL)
#         self.arm_btn.config(state=tk.NORMAL)
#         self.disarm_btn.config(state=tk.NORMAL)
#         self.force_arm_btn.config(state=tk.NORMAL)

#     # ═══════════════════════════════════════════════════════════
#     # NRF / ARDUINO SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def refresh_ports(self):
#         """Scan for available COM ports and populate dropdown."""
#         ports = [p.device for p in serial.tools.list_ports.comports()]
#         self.port_combo['values'] = ports
#         if ports:
#             self.port_combo.set(ports[0])
#         else:
#             self.port_combo.set("")
#         self.log(f"Found ports: {ports if ports else 'none'}", "dim")

#     def connect_rf(self):
#         def _connect():
#             port = self.port_var.get()
#             if not port:
#                 self.log("No COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to Arduino on {port}...", "info")
#                 self.ser = serial.Serial(port, SERIAL_BAUD, timeout=2)
#                 time.sleep(2)  # wait for Arduino reset after serial open

#                 # Wait for CLARQ_RF_READY handshake from Arduino
#                 self.log("Waiting for Arduino ready signal...", "dim")
#                 deadline = time.time() + 5
#                 ready = False
#                 while time.time() < deadline:
#                     if self.ser.in_waiting:
#                         line = self.ser.readline().decode(
#                             'utf-8', errors='ignore').strip()
#                         if line == "CLARQ_RF_READY":
#                             ready = True
#                             break
#                         self.log(f"Arduino: {line}", "dim")

#                 if ready:
#                     self.rf_connected = True
#                     self.log(
#                         f"RF bridge connected on {port} ✓", "info")
#                     self.root.after(
#                         0, lambda: self.rf_label.config(
#                             text="● RF CONNECTED", fg=ACCENT))
#                     # Start background reader for Arduino responses
#                     threading.Thread(
#                         target=self.rf_read_loop,
#                         daemon=True).start()
#                 else:
#                     self.log(
#                         "Arduino did not respond — check port/baud",
#                         "error")
#                     self.ser.close()
#                     self.ser = None

#             except Exception as e:
#                 self.log(f"RF connect failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_rf(self):
#         self.rf_connected = False
#         if self.ser:
#             try:
#                 self.ser.close()
#             except Exception:
#                 pass
#             self.ser = None
#         self.log("RF bridge disconnected", "warn")
#         self.root.after(
#             0, lambda: self.rf_label.config(
#                 text="● RF DISCONNECTED", fg=WARN))

#     def rf_read_loop(self):
#         """Background thread — reads responses from Arduino."""
#         while self.rf_connected and self.ser:
#             try:
#                 if self.ser.in_waiting:
#                     line = self.ser.readline().decode(
#                         'utf-8', errors='ignore').strip()
#                     if not line:
#                         continue

#                     if line.startswith("TX_OK:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX OK — cmd {cmd_id} delivered ✓",
#                             "info")

#                     elif line.startswith("TX_FAIL:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX FAIL — cmd {cmd_id} not delivered",
#                             "error")

#                     elif line.startswith("RX:"):
#                         # Jetson sent an ACK back over nRF
#                         cmd_id = int(line.split(":")[1])
#                         label = ARDUINO_RX_MAP.get(cmd_id, f"id={cmd_id}")
#                         self.log(
#                             f"JETSON ACK: {label}", "info")
#                         self.update_jetson_status(label)

#                     elif line.startswith("ERR:"):
#                         self.log(f"Arduino error: {line}", "warn")

#                     else:
#                         self.log(f"Arduino: {line}", "dim")

#                 else:
#                     time.sleep(0.05)

#             except Exception as e:
#                 if self.rf_connected:
#                     self.log(f"RF read error: {e}", "error")
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # SEND COMMAND TO JETSON VIA NRF ARDUINO BRIDGE
#     # ═══════════════════════════════════════════════════════════
#     def send_jetson_cmd(self, cmd_id, text):
#         """
#         Send command to Jetson via Arduino nRF bridge.
#         Writes 'CMD:<id>\n' over USB serial to Arduino.
#         Arduino transmits over NRF24L01 to Jetson.
#         """
#         self.log(f"Sending to Jetson: {text} (id={cmd_id})", "cmd")

#         if not self.rf_connected or not self.ser:
#             self.log(
#                 "RF bridge not connected — cannot send command",
#                 "error")
#             return

#         try:
#             packet = f"CMD:{cmd_id}\n"
#             self.ser.write(packet.encode('utf-8'))
#             self.log(
#                 f"→ Arduino: {packet.strip()}", "dim")
#         except Exception as e:
#             self.log(f"Serial write failed: {e}", "error")

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK TELEMETRY (RECEIVE ONLY — no send)
#     # ═══════════════════════════════════════════════════════════
#     def connect_mav(self):
#         def _connect():
#             try:
#                 conn = self.conn_entry.get()
#                 self.log(f"Connecting MAVLink to {conn}...", "info")
#                 self.mav = mavutil.mavlink_connection(conn)
#                 self.log("Waiting for heartbeat...", "dim")
#                 self.mav.wait_heartbeat(timeout=10)
#                 self.mav_connected = True
#                 self.log(
#                     f"MAVLink connected — telemetry active ✓", "info")
#                 self.root.after(
#                     0, lambda: self.conn_label.config(
#                         text="● MAV CONNECTED", fg=ACCENT))
#                 self.root.after(0, self.enable_mav_buttons)
#                 self.start_telemetry()
#             except Exception as e:
#                 self.log(
#                     f"MAVLink connection failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_mav(self):
#         self.mav_connected = False
#         self.telem_on = False
#         self.log("MAVLink disconnected", "warn")
#         self.root.after(
#             0, lambda: self.conn_label.config(
#                 text="● MAV DISCONNECTED", fg=WARN))

#     def start_telemetry(self):
#         self.telem_on = True
#         threading.Thread(
#             target=self.telem_loop, daemon=True).start()

#     def telem_loop(self):
#         while self.telem_on and self.mav_connected:
#             try:
#                 msg = self.mav.recv_match(
#                     type=[
#                         'HEARTBEAT', 'SYS_STATUS',
#                         'GLOBAL_POSITION_INT',
#                         'VFR_HUD', 'STATUSTEXT'
#                     ],
#                     blocking=True, timeout=1)
#                 if msg is None:
#                     continue

#                 t = msg.get_type()

#                 if t == 'STATUSTEXT':
#                     text = msg.text.strip()
#                     if text.startswith('CLARQ'):
#                         self.log(f"JETSON: {text}", "info")
#                         self.update_jetson_status(text)
#                     else:
#                         self.log(f"FC: {text}", "dim")
#                     continue

#                 if t == 'HEARTBEAT':
#                     src = (msg.get_srcSystem()
#                            if hasattr(msg, 'get_srcSystem')
#                            else 1)
#                     if src != 1:
#                         continue
#                     mode  = mavutil.mode_string_v10(msg)
#                     armed = bool(
#                         msg.base_mode &
#                         mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
#                     self.update_telem('Mode',  mode)
#                     self.update_telem(
#                         'Armed', 'YES' if armed else 'NO')

#                 elif t == 'SYS_STATUS':
#                     v = msg.voltage_battery / 1000.0
#                     self.update_telem('Battery', f'{v:.1f}V')

#                 elif t == 'GLOBAL_POSITION_INT':
#                     lat = msg.lat / 1e7
#                     lon = msg.lon / 1e7
#                     alt = msg.relative_alt / 1000.0
#                     hdg = msg.hdg / 100.0
#                     self.update_telem('Lat',     f'{lat:.6f}')
#                     self.update_telem('Lon',     f'{lon:.6f}')
#                     self.update_telem('Alt',     f'{alt:.1f}m')
#                     self.update_telem('Heading', f'{hdg:.0f}°')

#                 elif t == 'VFR_HUD':
#                     self.update_telem(
#                         'Speed', f'{msg.groundspeed:.1f}m/s')

#             except Exception:
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK FLIGHT COMMANDS (still sent via MAVLink)
#     # ═══════════════════════════════════════════════════════════
#     def set_mode(self, mode):
#         if not self.mav_connected or not self.mav:
#             self.log("MAVLink not connected!", "error")
#             return
#         mode_map = {
#             "STABILIZE": 0, "ACRO": 1,  "ALTHOLD": 2,
#             "AUTO":      3, "GUIDED": 4, "LOITER":  5,
#             "RTL":       6, "CIRCLE": 7, "LAND":    9,
#             "POSHOLD":  16,
#         }
#         if mode not in mode_map:
#             self.log(f"Unknown mode: {mode}", "error")
#             return
#         mode_num = mode_map[mode]
#         self.mav.mav.set_mode_send(
#             self.mav.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num)
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#             0,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num, 0, 0, 0, 0, 0)
#         self.log(f"Mode {mode} ({mode_num}) sent", "cmd")

#         def _watch_ack():
#             try:
#                 ack = self.mav.recv_match(
#                     type='COMMAND_ACK',
#                     blocking=True, timeout=3)
#                 if ack and ack.result == 0:
#                     self.log(f"Mode {mode} confirmed ✓", "info")
#                 elif ack:
#                     self.log(
#                         f"Mode {mode} rejected (result={ack.result})",
#                         "warn")
#                 else:
#                     self.log(f"Mode {mode} — no ACK", "dim")
#             except Exception:
#                 pass
#         threading.Thread(
#             target=_watch_ack, daemon=True).start()

#     def arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Arming...", "warn")
#         self.mav.arducopter_arm()
#         self.log("Arm command sent", "info")

#     def disarm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Disarming...", "warn")
#         self.mav.arducopter_disarm()
#         self.log("Disarm command sent", "info")

#     def force_arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Force arming...", "warn")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0, 1, 21196, 0, 0, 0, 0, 0)
#         self.log("Force arm sent!", "info")

#     def takeoff(self):    self._takeoff(2.0)
#     def takeoff_5(self):  self._takeoff(5.0)
#     def takeoff_10(self): self._takeoff(10.0)

#     def _takeoff(self, alt):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log(f"Sending takeoff to {alt}m...", "cmd")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#             0, 0, 0, 0, 0, 0, 0, alt)
#         self.log(f"Takeoff {alt}m sent", "info")

#     def rtl(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.set_mode("RTL")

#     # ═══════════════════════════════════════════════════════════
#     # CAMERA NAVIGATION
#     # ═══════════════════════════════════════════════════════════
#     def set_home_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 1 — SET HOME POINT", "nav")
#         self.log("  Saves ZED pose as origin_T0.json", "nav")
#         self.log("  Gimbal tilts to 30° after save", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SET_HOME, CMD_SET_HOME)

#     def start_scan_mission(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 2 — START SCAN MISSION", "nav")
#         self.log("  Jetson runs save_position.py", "nav")
#         self.log("  Fly the mission area now", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_START_SCAN, CMD_START_SCAN)

#     def save_end_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 3 — SAVE END POINT", "nav")
#         self.log("  Saves scan_end_pos.json on Jetson", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SAVE_END, CMD_SAVE_END)

#     def go_home_and_land(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 4 — GO HOME + LAND", "nav")
#         self.log("  Jetson runs return_land.py", "nav")
#         self.log("  AprilTag activated automatically", "nav")
#         self.log("  Phase 1 → ZED navigate to T0", "nav")
#         self.log("  Phase 2 → AprilTag align at 90°", "nav")
#         self.log("  Phase 3 → Land on rover", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_GO_HOME, CMD_GO_HOME)
#         if self.mav_connected:
#             self.set_mode("GUIDED")

#     # ── Jetson system commands ────────────────────────────────
#     def ping_jetson(self):
#         self.send_jetson_cmd(CMD_ID_PING, CMD_PING)

#     def launch_jetson(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH, CMD_LAUNCH)

#     def launch_jetson_sim(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH_SIM, CMD_LAUNCH_SIM)

#     def kill_jetson(self):
#         self.send_jetson_cmd(CMD_ID_STOP_ALL, "CLARQ_STOP_ALL")


# # ── Run ───────────────────────────────────────────────────────
# def main():
#     root = tk.Tk()
#     app  = DroneGUI(root)
#     root.mainloop()

# if __name__ == '__main__':
#     main()




#WITH GIMBAL CONTROLS
# CLARQ_GUI.py
# Runs on Windows alongside Mission Planner
#
# Command routing (nRF path):
#   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
#
# Telemetry routing (MAVLink receive only):
#   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
#
# Mission Planner setup:
#   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
#
# Gimbal control:
#   GUI → USB Serial (COM port) → SimpleBGC Controller → Gimbal Motor
#
# pip install pymavlink pyserial










# import tkinter as tk
# from tkinter import scrolledtext, ttk
# from pymavlink import mavutil
# import threading
# import time
# import serial
# import serial.tools.list_ports
# import struct

# # ── Config ────────────────────────────────────────────────────
# MAVLINK_CONNECTION = "udp:127.0.0.1:14550"   # receive telemetry from MP
# SERIAL_BAUD        = 115200                   # must match Arduino sketch
# GIMBAL_BAUD        = 115200                   # gimbal serial baud rate

# # Valid gimbal angles (from your script)
# VALID_GIMBAL_ANGLES = [
#     0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30, 35, -35, 
#     40, -40, 45, -45, 50, -50, 55, -55, 60, -60, 65, -65, 70, -70, 
#     75, -75, 80, -80, 85, -85, 90, -90, 95, -95, 100, -100, 105, -105, 
#     110, -110, 115, -115, 120, -120, 125, -125, 130, -130, 135, -135, 
#     140, -140, 145, -145, 150, -150, 155, -155, 160, -160, 165, -165, 
#     170, -170, 175, -175, 180, -180, 360, -360
# ]

# # ── Command IDs — must match clarq_rf_listener.py ─────────────
# CMD_ID_PING       = 1
# CMD_ID_LAND       = 2
# CMD_ID_START_TAG  = 3
# CMD_ID_STOP_TAG   = 4
# CMD_ID_STOP_ALL   = 5
# CMD_ID_LAUNCH     = 6
# CMD_ID_LAUNCH_SIM = 7
# CMD_ID_SET_HOME   = 8
# CMD_ID_START_SCAN = 9
# CMD_ID_SAVE_END   = 10
# CMD_ID_GO_HOME    = 11

# CMD_START_LANDING  = "CLARQ_LAND"
# CMD_START_APRILTAG = "CLARQ_START_TAG"
# CMD_STOP_APRILTAG  = "CLARQ_STOP_TAG"
# CMD_PING           = "CLARQ_PING"
# CMD_LAUNCH         = "CLARQ_LAUNCH"
# CMD_LAUNCH_SIM     = "CLARQ_LAUNCH_SIM"
# CMD_SET_HOME       = "CLARQ_SET_HOME"
# CMD_START_SCAN     = "CLARQ_START_SCAN"
# CMD_SAVE_END       = "CLARQ_SAVE_END"
# CMD_GO_HOME        = "CLARQ_GO_HOME"

# # ── Jetson response map (STATUSTEXT → status label) ───────────
# JETSON_STATUS_MAP = {
#     "CLARQ_PONG":         ("ONLINE ✓",          "#1ec882", "--"),
#     "CLARQ_HOME_SET":     ("HOME SET ✓",         "#1ec882", "origin_T0.json saved"),
#     "CLARQ_SCAN_STARTED": ("SCANNING ✓",         "#1ec882", "Mission scan running"),
#     "CLARQ_END_SAVED":    ("END SAVED ✓",        "#1ec882", "scan_end_pos.json saved"),
#     "CLARQ_GOING_HOME":   ("RETURNING HOME...",  "#dc6438", "Phase 1 — navigating"),
#     "CLARQ_LAUNCHED":     ("LAUNCHED ✓",         "#1ec882", "All systems starting"),
#     "CLARQ_LAUNCHED_SIM": ("LAUNCHED SIM ✓",     "#1ec882", "Sim stack starting"),
#     "CLARQ_ALL_STOPPED":  ("STOPPED",            "#646478", "All processes stopped"),
#     "CLARQ_TAG_RUNNING":  ("TAG RUNNING ✓",      "#1ec882", "AprilTag active"),
#     "CLARQ_TAG_STOPPED":  ("TAG STOPPED",        "#646478", "AprilTag stopped"),
# }

# # ── RX response codes from Arduino ────────────────────────────
# # Arduino sends back: TX_OK:<id>, TX_FAIL:<id>, RX:<id>
# ARDUINO_RX_MAP = {
#     CMD_ID_PING:       "CLARQ_PONG",
#     CMD_ID_LAND:       "CLARQ_LAND_ACK",
#     CMD_ID_START_TAG:  "CLARQ_TAG_RUNNING",
#     CMD_ID_STOP_TAG:   "CLARQ_TAG_STOPPED",
#     CMD_ID_STOP_ALL:   "CLARQ_ALL_STOPPED",
#     CMD_ID_LAUNCH:     "CLARQ_LAUNCHED",
#     CMD_ID_LAUNCH_SIM: "CLARQ_LAUNCHED_SIM",
#     CMD_ID_SET_HOME:   "CLARQ_HOME_SET",
#     CMD_ID_START_SCAN: "CLARQ_SCAN_STARTED",
#     CMD_ID_SAVE_END:   "CLARQ_END_SAVED",
#     CMD_ID_GO_HOME:    "CLARQ_GOING_HOME",
# }

# # ── Colors ────────────────────────────────────────────────────
# BG     = "#0f0f16"
# PANEL  = "#191926"
# ACCENT = "#1ec882"
# WARN   = "#dc6438"
# TEXT   = "#d2d2dc"
# DIM    = "#646478"
# GREEN  = "#1a6b42"
# RED    = "#6b1a1a"
# BLUE   = "#1a3d6b"
# PURPLE = "#4a1a6b"


# class DroneGUI:
#     def __init__(self, root):
#         self.root      = root
#         self.root.title("CLARQ Drone Control")
#         self.root.geometry("1000x800")
#         self.root.configure(bg=BG)

#         # MAVLink (telemetry receive only)
#         self.mav       = None
#         self.mav_connected = False
#         self.telem_on  = False

#         # nRF Arduino serial bridge
#         self.ser       = None
#         self.rf_connected = False

#         # Gimbal serial connection
#         self.gimbal_ser = None
#         self.gimbal_connected = False
#         self.current_gimbal_angle = 0

#         self.build_ui()

#     # ═══════════════════════════════════════════════════════════
#     # GIMBAL CONTROL FUNCTIONS
#     # ═══════════════════════════════════════════════════════════
#     def gimbal_send_cmd(self, cmd, data=bytes([])):
#         """Send command to SimpleBGC gimbal controller."""
#         size = len(data)
#         header_checksum = (cmd + size) & 0xFF
#         body_checksum = 0
#         for b in data:
#             body_checksum = (body_checksum + b) & 0xFF
#         packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
#         self.gimbal_ser.write(packet)
#         time.sleep(0.1)
#         response = self.gimbal_ser.read(100)
#         return response

#     def gimbal_set_pitch(self, angle):
#         """Set gimbal pitch to absolute angle with drift correction."""
#         if not self.gimbal_connected or not self.gimbal_ser:
#             self.log("Gimbal not connected!", "error")
#             return

#         # Mode 2: absolute angle control
#         mode = 2
#         pitch = int(angle / 0.02197265625)
#         data = struct.pack('<Bhhhhhh', mode, 0, 0, 300, pitch, 0, 0)
#         self.gimbal_send_cmd(67, data)
#         time.sleep(0.5)
        
#         # Mode 0: release control (drift correction)
#         mode = 0
#         data = struct.pack('<Bhhhhhh', mode, 0, 0, 0, 0, 0, 0)
#         self.gimbal_send_cmd(67, data)
        
#         self.current_gimbal_angle = angle
#         self.log(f"Gimbal pitch set to {angle}°", "info")
        
#         # Update current angle display
#         self.root.after(0, lambda: self.gimbal_current_lbl.config(
#             text=f"Current: {angle}°", fg=ACCENT))

#     # ═══════════════════════════════════════════════════════════
#     # UI BUILD
#     # ═══════════════════════════════════════════════════════════
#     def build_ui(self):
#         title = tk.Frame(self.root, bg=PANEL, pady=12)
#         title.pack(fill=tk.X)

#         tk.Label(
#             title,
#             text="CLARQ DRONE NAVIGATION CONTROL",
#             font=("Consolas", 16, "bold"),
#             bg=PANEL, fg=ACCENT
#         ).pack(side=tk.LEFT, padx=20)

#         # Status indicators (right side of title bar)
#         status_frame = tk.Frame(title, bg=PANEL)
#         status_frame.pack(side=tk.RIGHT, padx=20)

#         self.gimbal_label = tk.Label(
#             status_frame, text="● GIMBAL DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.gimbal_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.rf_label = tk.Label(
#             status_frame, text="● RF DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.rf_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.conn_label = tk.Label(
#             status_frame, text="● MAV DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.conn_label.pack(side=tk.RIGHT)

#         main = tk.Frame(self.root, bg=BG)
#         main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

#         left_outer = tk.Frame(main, bg=PANEL, width=280)
#         left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
#         left_outer.pack_propagate(False)

#         left_canvas = tk.Canvas(
#             left_outer, bg=PANEL, width=260,
#             highlightthickness=0)
#         left_scrollbar = tk.Scrollbar(
#             left_outer, orient=tk.VERTICAL,
#             command=left_canvas.yview)
#         left_canvas.configure(
#             yscrollcommand=left_scrollbar.set)
#         left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
#         left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         left = tk.Frame(left_canvas, bg=PANEL)
#         left_canvas.create_window(
#             (0, 0), window=left, anchor=tk.NW)

#         def on_frame_configure(event):
#             left_canvas.configure(
#                 scrollregion=left_canvas.bbox("all"))

#         def on_mousewheel(event):
#             left_canvas.yview_scroll(
#                 int(-1 * (event.delta / 120)), "units")

#         left.bind("<Configure>", on_frame_configure)
#         left_canvas.bind("<MouseWheel>", on_mousewheel)
#         left.bind("<MouseWheel>", on_mousewheel)

#         right = tk.Frame(main, bg=PANEL)
#         right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         self.build_buttons(left)
#         self.build_output(right)

#     def build_buttons(self, parent):

#         def section(text):
#             tk.Frame(parent, bg=DIM, height=1).pack(
#                 fill=tk.X, padx=15, pady=(15, 0))
#             tk.Label(
#                 parent, text=text,
#                 font=("Consolas", 9),
#                 bg=PANEL, fg=DIM
#             ).pack(anchor=tk.W, padx=15, pady=(4, 6))

#         def btn(text, color, cmd, state=tk.NORMAL):
#             b = tk.Button(
#                 parent, text=text,
#                 font=("Consolas", 12, "bold"),
#                 bg=color, fg=TEXT,
#                 activebackground=color,
#                 relief=tk.FLAT, cursor="hand2",
#                 pady=10, state=state, command=cmd)
#             b.pack(fill=tk.X, padx=15, pady=3)
#             return b

#         # ── nRF / Arduino serial connection ───────────────────
#         section("NRF RADIO (COMMAND LINK)")

#         port_frame = tk.Frame(parent, bg=PANEL)
#         port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.port_var = tk.StringVar()
#         self.port_combo = ttk.Combobox(
#             port_frame,
#             textvariable=self.port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         self.refresh_ports()

#         btn("CONNECT RF",    BLUE, self.connect_rf)
#         btn("DISCONNECT RF", RED,  self.disconnect_rf)

#         # ── Gimbal control ────────────────────────────────────
#         section("GIMBAL CONTROL")

#         gimbal_port_frame = tk.Frame(parent, bg=PANEL)
#         gimbal_port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             gimbal_port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.gimbal_port_var = tk.StringVar()
#         self.gimbal_port_combo = ttk.Combobox(
#             gimbal_port_frame,
#             textvariable=self.gimbal_port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.gimbal_port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             gimbal_port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         btn("CONNECT GIMBAL",    GREEN, self.connect_gimbal)
#         btn("DISCONNECT GIMBAL", RED,   self.disconnect_gimbal)

#         # Gimbal angle input
#         angle_frame = tk.Frame(parent, bg=PANEL)
#         angle_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             angle_frame, text="Angle:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.gimbal_angle_entry = tk.Entry(
#             angle_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=10)
#         self.gimbal_angle_entry.insert(0, "0")
#         self.gimbal_angle_entry.pack(side=tk.LEFT, padx=5)
        
#         # Bind Enter key to set angle
#         self.gimbal_angle_entry.bind('<Return>', lambda e: self.set_gimbal_angle())

#         tk.Button(
#             angle_frame, text="SET",
#             font=("Consolas", 11, "bold"),
#             bg=BLUE, fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.set_gimbal_angle
#         ).pack(side=tk.LEFT)

#         # Current angle display
#         self.gimbal_current_lbl = tk.Label(
#             parent, text="Current: 0°",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.gimbal_current_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 6))

#         # Quick angle buttons
#         quick_angles_frame = tk.Frame(parent, bg=PANEL)
#         quick_angles_frame.pack(fill=tk.X, padx=15, pady=4)

#         for angle in [0, 30, 45, 60, 90]:
#             tk.Button(
#                 quick_angles_frame, text=f"{angle}°",
#                 font=("Consolas", 10),
#                 bg="#2a2a3a", fg=TEXT,
#                 relief=tk.FLAT, cursor="hand2",
#                 command=lambda a=angle: self.quick_set_angle(a)
#             ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

#         quick_angles_frame2 = tk.Frame(parent, bg=PANEL)
#         quick_angles_frame2.pack(fill=tk.X, padx=15, pady=(0, 4))

#         for angle in [-30, -45, -60, -90]:
#             tk.Button(
#                 quick_angles_frame2, text=f"{angle}°",
#                 font=("Consolas", 10),
#                 bg="#2a2a3a", fg=TEXT,
#                 relief=tk.FLAT, cursor="hand2",
#                 command=lambda a=angle: self.quick_set_angle(a)
#             ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

#         # ── MAVLink telemetry connection ───────────────────────
#         section("MAVLINK (TELEMETRY ONLY)")

#         conn_frame = tk.Frame(parent, bg=PANEL)
#         conn_frame.pack(fill=tk.X, padx=15, pady=4)
#         tk.Label(
#             conn_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)
#         self.conn_entry = tk.Entry(
#             conn_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=22)
#         self.conn_entry.insert(0, MAVLINK_CONNECTION)
#         self.conn_entry.pack(side=tk.LEFT, padx=5)

#         btn("CONNECT MAV",    BLUE, self.connect_mav)
#         btn("DISCONNECT MAV", RED,  self.disconnect_mav)

#         # ── Camera navigation ─────────────────────────────────
#         section("CAMERA NAVIGATION")

#         self.jetson_status_lbl = tk.Label(
#             parent, text="Jetson: IDLE",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.jetson_status_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 0))

#         self.jetson_phase_lbl = tk.Label(
#             parent, text="Phase: --",
#             font=("Consolas", 10),
#             bg=PANEL, fg=DIM)
#         self.jetson_phase_lbl.pack(
#             anchor=tk.W, padx=15, pady=(0, 6))

#         self.nav_btns = []
#         b = btn("1. SET HOME POINT",     GREEN,  self.set_home_point)
#         self.nav_btns.append(b)
#         b = btn("2. START SCAN MISSION", BLUE,   self.start_scan_mission)
#         self.nav_btns.append(b)
#         b = btn("3. SAVE END POINT",     PURPLE, self.save_end_point)
#         self.nav_btns.append(b)
#         b = btn("4. GO HOME + LAND",     WARN,   self.go_home_and_land)
#         self.nav_btns.append(b)

#         # ── Jetson ────────────────────────────────────────────
#         section("JETSON")
#         btn("LAUNCH JETSON",     GREEN, self.launch_jetson)
#         btn("LAUNCH JETSON SIM", BLUE,  self.launch_jetson_sim)
#         btn("KILL JETSON",       RED,   self.kill_jetson)
#         btn("PING JETSON",       BLUE,  self.ping_jetson)

#         # ── Telemetry ─────────────────────────────────────────
#         section("TELEMETRY")

#         telem = tk.Frame(parent, bg="#12121e")
#         telem.pack(fill=tk.X, padx=15, pady=5)

#         self.telem = {}
#         for label, default in [
#             ("Mode",    "UNKNOWN"),
#             ("Armed",   "NO"),
#             ("Battery", "--V"),
#             ("Alt",     "--m"),
#             ("Lat",     "--"),
#             ("Lon",     "--"),
#             ("Heading", "--°"),
#             ("Speed",   "--m/s"),
#         ]:
#             row = tk.Frame(telem, bg="#12121e")
#             row.pack(fill=tk.X, padx=10, pady=2)
#             tk.Label(
#                 row, text=f"{label}:",
#                 font=("Consolas", 11),
#                 bg="#12121e", fg=DIM,
#                 width=9, anchor=tk.W
#             ).pack(side=tk.LEFT)
#             lbl = tk.Label(
#                 row, text=default,
#                 font=("Consolas", 11, "bold"),
#                 bg="#12121e", fg=ACCENT)
#             lbl.pack(side=tk.LEFT)
#             self.telem[label] = lbl

#         # ── Flight modes ──────────────────────────────────────
#         section("FLIGHT MODES")

#         self.flight_btns = []
#         for mode, color in [
#             ("STABILIZE", BLUE),
#             ("GUIDED",    BLUE),
#             ("LOITER",    BLUE),
#             ("RTL",       WARN),
#             ("LAND",      WARN),
#             ("AUTO",      BLUE),
#             ("ALTHOLD",   BLUE),
#             ("POSHOLD",   BLUE),
#         ]:
#             b = btn(mode, color,
#                     lambda m=mode: self.set_mode(m),
#                     tk.DISABLED)
#             self.flight_btns.append(b)

#         # ── Arm / Disarm ──────────────────────────────────────
#         section("ARM / DISARM")

#         self.arm_btn       = btn("ARM",       GREEN, self.arm,       tk.DISABLED)
#         self.disarm_btn    = btn("DISARM",    RED,   self.disarm,    tk.DISABLED)
#         self.force_arm_btn = btn("FORCE ARM", WARN,  self.force_arm, tk.DISABLED)

#         # ── Commands ──────────────────────────────────────────
#         section("COMMANDS")

#         self.cmd_btns = []
#         for text, color, cmd in [
#             ("TAKEOFF 2m",       BLUE, self.takeoff),
#             ("TAKEOFF 5m",       BLUE, self.takeoff_5),
#             ("TAKEOFF 10m",      BLUE, self.takeoff_10),
#             ("RETURN TO LAUNCH", WARN, self.rtl),
#         ]:
#             b = btn(text, color, cmd, tk.DISABLED)
#             self.cmd_btns.append(b)

#         tk.Frame(parent, bg=PANEL, height=20).pack()

#     def build_output(self, parent):
#         header = tk.Frame(parent, bg=PANEL)
#         header.pack(fill=tk.X, padx=10, pady=(10, 5))

#         tk.Label(
#             header, text="OUTPUT LOG",
#             font=("Consolas", 12),
#             bg=PANEL, fg=DIM
#         ).pack(side=tk.LEFT)

#         tk.Button(
#             header, text="CLEAR",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.clear
#         ).pack(side=tk.RIGHT, padx=5)

#         tk.Button(
#             header, text="SAVE LOG",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.save_log
#         ).pack(side=tk.RIGHT, padx=5)

#         self.output = scrolledtext.ScrolledText(
#             parent, font=("Consolas", 11),
#             bg="#0a0a14", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, wrap=tk.WORD,
#             state=tk.DISABLED)
#         self.output.pack(
#             fill=tk.BOTH, expand=True,
#             padx=10, pady=(0, 5))

#         self.output.tag_config("info",   foreground=ACCENT)
#         self.output.tag_config("warn",   foreground=WARN)
#         self.output.tag_config("error",  foreground="#dc3838")
#         self.output.tag_config("cmd",    foreground="#6496c8")
#         self.output.tag_config("nav",    foreground="#50c8c8")
#         self.output.tag_config("normal", foreground=TEXT)
#         self.output.tag_config("dim",    foreground=DIM)

#     # ═══════════════════════════════════════════════════════════
#     # LOGGING
#     # ═══════════════════════════════════════════════════════════
#     def log(self, message, tag="normal"):
#         def _log():
#             self.output.config(state=tk.NORMAL)
#             ts = time.strftime("%H:%M:%S")
#             self.output.insert(
#                 tk.END, f"[{ts}] {message}\n", tag)
#             self.output.see(tk.END)
#             self.output.config(state=tk.DISABLED)
#         self.root.after(0, _log)

#     def clear(self):
#         self.output.config(state=tk.NORMAL)
#         self.output.delete(1.0, tk.END)
#         self.output.config(state=tk.DISABLED)

#     def save_log(self):
#         content = self.output.get(1.0, tk.END)
#         fname = f"clarq_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
#         with open(fname, 'w') as f:
#             f.write(content)
#         self.log(f"Log saved to {fname}", "info")

#     def update_telem(self, key, value):
#         def _u():
#             if key in self.telem:
#                 self.telem[key].config(text=value)
#         self.root.after(0, _u)

#     def update_jetson_status(self, text):
#         if text in JETSON_STATUS_MAP:
#             status, color, phase = JETSON_STATUS_MAP[text]
#             def _u():
#                 self.jetson_status_lbl.config(
#                     text=f"Jetson: {status}", fg=color)
#                 self.jetson_phase_lbl.config(
#                     text=f"Phase: {phase}", fg=color)
#             self.root.after(0, _u)

#     def enable_mav_buttons(self):
#         for b in (self.flight_btns + self.cmd_btns):
#             b.config(state=tk.NORMAL)
#         self.arm_btn.config(state=tk.NORMAL)
#         self.disarm_btn.config(state=tk.NORMAL)
#         self.force_arm_btn.config(state=tk.NORMAL)

#     # ═══════════════════════════════════════════════════════════
#     # GIMBAL SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def connect_gimbal(self):
#         def _connect():
#             port = self.gimbal_port_var.get()
#             if not port:
#                 self.log("No gimbal COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to gimbal on {port}...", "info")
#                 self.gimbal_ser = serial.Serial(port, GIMBAL_BAUD, timeout=1)
#                 time.sleep(2)  # wait for controller to be ready

#                 # Turn motors on (CMD 77)
#                 self.log("Turning gimbal motors on...", "dim")
#                 self.gimbal_send_cmd(77)
#                 time.sleep(1)

#                 # Set to 0 degrees initially
#                 self.gimbal_set_pitch(0)

#                 self.gimbal_connected = True
#                 self.log(f"Gimbal connected on {port} ✓", "info")
#                 self.root.after(
#                     0, lambda: self.gimbal_label.config(
#                         text="● GIMBAL CONNECTED", fg=ACCENT))

#             except Exception as e:
#                 self.log(f"Gimbal connect failed: {e}", "error")
#                 if self.gimbal_ser:
#                     try:
#                         self.gimbal_ser.close()
#                     except:
#                         pass
#                     self.gimbal_ser = None

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_gimbal(self):
#         self.gimbal_connected = False
#         if self.gimbal_ser:
#             try:
#                 self.gimbal_ser.close()
#             except Exception:
#                 pass
#             self.gimbal_ser = None
#         self.log("Gimbal disconnected", "warn")
#         self.root.after(
#             0, lambda: self.gimbal_label.config(
#                 text="● GIMBAL DISCONNECTED", fg=WARN))

#     def set_gimbal_angle(self):
#         """Set gimbal angle from entry field."""
#         try:
#             angle = float(self.gimbal_angle_entry.get())
            
#             if angle not in VALID_GIMBAL_ANGLES:
#                 self.log(f"Invalid angle: {angle}° (not in valid range)", "error")
#                 return
            
#             self.gimbal_set_pitch(angle)
            
#         except ValueError:
#             self.log("Invalid angle input — enter a number", "error")

#     def quick_set_angle(self, angle):
#         """Quick set button handler."""
#         self.gimbal_angle_entry.delete(0, tk.END)
#         self.gimbal_angle_entry.insert(0, str(angle))
#         self.gimbal_set_pitch(angle)

#     # ═══════════════════════════════════════════════════════════
#     # NRF / ARDUINO SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def refresh_ports(self):
#         """Scan for available COM ports and populate dropdown."""
#         ports = [p.device for p in serial.tools.list_ports.comports()]
#         self.port_combo['values'] = ports
        
#         # Only update gimbal combo if it exists (it's created after RF section)
#         if hasattr(self, 'gimbal_port_combo'):
#             self.gimbal_port_combo['values'] = ports
        
#         if ports:
#             if not self.port_var.get():
#                 self.port_combo.set(ports[0])
#             if hasattr(self, 'gimbal_port_combo') and not self.gimbal_port_var.get():
#                 # Try to set a different port for gimbal if available
#                 self.gimbal_port_combo.set(ports[1] if len(ports) > 1 else ports[0])
#         else:
#             self.port_combo.set("")
#             if hasattr(self, 'gimbal_port_combo'):
#                 self.gimbal_port_combo.set("")
        
#         self.log(f"Found ports: {ports if ports else 'none'}", "dim")

#     def connect_rf(self):
#         def _connect():
#             port = self.port_var.get()
#             if not port:
#                 self.log("No COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to Arduino on {port}...", "info")
#                 self.ser = serial.Serial(port, SERIAL_BAUD, timeout=2)
#                 time.sleep(2)  # wait for Arduino reset after serial open

#                 # Wait for CLARQ_RF_READY handshake from Arduino
#                 self.log("Waiting for Arduino ready signal...", "dim")
#                 deadline = time.time() + 5
#                 ready = False
#                 while time.time() < deadline:
#                     if self.ser.in_waiting:
#                         line = self.ser.readline().decode(
#                             'utf-8', errors='ignore').strip()
#                         if line == "CLARQ_RF_READY":
#                             ready = True
#                             break
#                         self.log(f"Arduino: {line}", "dim")

#                 if ready:
#                     self.rf_connected = True
#                     self.log(
#                         f"RF bridge connected on {port} ✓", "info")
#                     self.root.after(
#                         0, lambda: self.rf_label.config(
#                             text="● RF CONNECTED", fg=ACCENT))
#                     # Start background reader for Arduino responses
#                     threading.Thread(
#                         target=self.rf_read_loop,
#                         daemon=True).start()
#                 else:
#                     self.log(
#                         "Arduino did not respond — check port/baud",
#                         "error")
#                     self.ser.close()
#                     self.ser = None

#             except Exception as e:
#                 self.log(f"RF connect failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_rf(self):
#         self.rf_connected = False
#         if self.ser:
#             try:
#                 self.ser.close()
#             except Exception:
#                 pass
#             self.ser = None
#         self.log("RF bridge disconnected", "warn")
#         self.root.after(
#             0, lambda: self.rf_label.config(
#                 text="● RF DISCONNECTED", fg=WARN))

#     def rf_read_loop(self):
#         """Background thread — reads responses from Arduino."""
#         while self.rf_connected and self.ser:
#             try:
#                 if self.ser.in_waiting:
#                     line = self.ser.readline().decode(
#                         'utf-8', errors='ignore').strip()
#                     if not line:
#                         continue

#                     if line.startswith("TX_OK:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX OK — cmd {cmd_id} delivered ✓",
#                             "info")

#                     elif line.startswith("TX_FAIL:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX FAIL — cmd {cmd_id} not delivered",
#                             "error")

#                     elif line.startswith("RX:"):
#                         # Jetson sent an ACK back over nRF
#                         cmd_id = int(line.split(":")[1])
#                         label = ARDUINO_RX_MAP.get(cmd_id, f"id={cmd_id}")
#                         self.log(
#                             f"JETSON ACK: {label}", "info")
#                         self.update_jetson_status(label)

#                     elif line.startswith("ERR:"):
#                         self.log(f"Arduino error: {line}", "warn")

#                     else:
#                         self.log(f"Arduino: {line}", "dim")

#                 else:
#                     time.sleep(0.05)

#             except Exception as e:
#                 if self.rf_connected:
#                     self.log(f"RF read error: {e}", "error")
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # SEND COMMAND TO JETSON VIA NRF ARDUINO BRIDGE
#     # ═══════════════════════════════════════════════════════════
#     def send_jetson_cmd(self, cmd_id, text):
#         """
#         Send command to Jetson via Arduino nRF bridge.
#         Writes 'CMD:<id>\n' over USB serial to Arduino.
#         Arduino transmits over NRF24L01 to Jetson.
#         """
#         self.log(f"Sending to Jetson: {text} (id={cmd_id})", "cmd")

#         if not self.rf_connected or not self.ser:
#             self.log(
#                 "RF bridge not connected — cannot send command",
#                 "error")
#             return

#         try:
#             packet = f"CMD:{cmd_id}\n"
#             self.ser.write(packet.encode('utf-8'))
#             self.log(
#                 f"→ Arduino: {packet.strip()}", "dim")
#         except Exception as e:
#             self.log(f"Serial write failed: {e}", "error")

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK TELEMETRY (RECEIVE ONLY — no send)
#     # ═══════════════════════════════════════════════════════════
#     def connect_mav(self):
#         def _connect():
#             try:
#                 conn = self.conn_entry.get()
#                 self.log(f"Connecting MAVLink to {conn}...", "info")
#                 self.mav = mavutil.mavlink_connection(conn)
#                 self.log("Waiting for heartbeat...", "dim")
#                 self.mav.wait_heartbeat(timeout=10)
#                 self.mav_connected = True
#                 self.log(
#                     f"MAVLink connected — telemetry active ✓", "info")
#                 self.root.after(
#                     0, lambda: self.conn_label.config(
#                         text="● MAV CONNECTED", fg=ACCENT))
#                 self.root.after(0, self.enable_mav_buttons)
#                 self.start_telemetry()
#             except Exception as e:
#                 self.log(
#                     f"MAVLink connection failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_mav(self):
#         self.mav_connected = False
#         self.telem_on = False
#         self.log("MAVLink disconnected", "warn")
#         self.root.after(
#             0, lambda: self.conn_label.config(
#                 text="● MAV DISCONNECTED", fg=WARN))

#     def start_telemetry(self):
#         self.telem_on = True
#         threading.Thread(
#             target=self.telem_loop, daemon=True).start()

#     def telem_loop(self):
#         while self.telem_on and self.mav_connected:
#             try:
#                 msg = self.mav.recv_match(
#                     type=[
#                         'HEARTBEAT', 'SYS_STATUS',
#                         'GLOBAL_POSITION_INT',
#                         'VFR_HUD', 'STATUSTEXT'
#                     ],
#                     blocking=True, timeout=1)
#                 if msg is None:
#                     continue

#                 t = msg.get_type()

#                 if t == 'STATUSTEXT':
#                     text = msg.text.strip()
#                     if text.startswith('CLARQ'):
#                         self.log(f"JETSON: {text}", "info")
#                         self.update_jetson_status(text)
#                     else:
#                         self.log(f"FC: {text}", "dim")
#                     continue

#                 if t == 'HEARTBEAT':
#                     src = (msg.get_srcSystem()
#                            if hasattr(msg, 'get_srcSystem')
#                            else 1)
#                     if src != 1:
#                         continue
#                     mode  = mavutil.mode_string_v10(msg)
#                     armed = bool(
#                         msg.base_mode &
#                         mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
#                     self.update_telem('Mode',  mode)
#                     self.update_telem(
#                         'Armed', 'YES' if armed else 'NO')

#                 elif t == 'SYS_STATUS':
#                     v = msg.voltage_battery / 1000.0
#                     self.update_telem('Battery', f'{v:.1f}V')

#                 elif t == 'GLOBAL_POSITION_INT':
#                     lat = msg.lat / 1e7
#                     lon = msg.lon / 1e7
#                     alt = msg.relative_alt / 1000.0
#                     hdg = msg.hdg / 100.0
#                     self.update_telem('Lat',     f'{lat:.6f}')
#                     self.update_telem('Lon',     f'{lon:.6f}')
#                     self.update_telem('Alt',     f'{alt:.1f}m')
#                     self.update_telem('Heading', f'{hdg:.0f}°')

#                 elif t == 'VFR_HUD':
#                     self.update_telem(
#                         'Speed', f'{msg.groundspeed:.1f}m/s')

#             except Exception:
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK FLIGHT COMMANDS (still sent via MAVLink)
#     # ═══════════════════════════════════════════════════════════
#     def set_mode(self, mode):
#         if not self.mav_connected or not self.mav:
#             self.log("MAVLink not connected!", "error")
#             return
#         mode_map = {
#             "STABILIZE": 0, "ACRO": 1,  "ALTHOLD": 2,
#             "AUTO":      3, "GUIDED": 4, "LOITER":  5,
#             "RTL":       6, "CIRCLE": 7, "LAND":    9,
#             "POSHOLD":  16,
#         }
#         if mode not in mode_map:
#             self.log(f"Unknown mode: {mode}", "error")
#             return
#         mode_num = mode_map[mode]
#         self.mav.mav.set_mode_send(
#             self.mav.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num)
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#             0,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num, 0, 0, 0, 0, 0)
#         self.log(f"Mode {mode} ({mode_num}) sent", "cmd")

#         def _watch_ack():
#             try:
#                 ack = self.mav.recv_match(
#                     type='COMMAND_ACK',
#                     blocking=True, timeout=3)
#                 if ack and ack.result == 0:
#                     self.log(f"Mode {mode} confirmed ✓", "info")
#                 elif ack:
#                     self.log(
#                         f"Mode {mode} rejected (result={ack.result})",
#                         "warn")
#                 else:
#                     self.log(f"Mode {mode} — no ACK", "dim")
#             except Exception:
#                 pass
#         threading.Thread(
#             target=_watch_ack, daemon=True).start()

#     def arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Arming...", "warn")
#         self.mav.arducopter_arm()
#         self.log("Arm command sent", "info")

#     def disarm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Disarming...", "warn")
#         self.mav.arducopter_disarm()
#         self.log("Disarm command sent", "info")

#     def force_arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Force arming...", "warn")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0, 1, 21196, 0, 0, 0, 0, 0)
#         self.log("Force arm sent!", "info")

#     def takeoff(self):    self._takeoff(2.0)
#     def takeoff_5(self):  self._takeoff(5.0)
#     def takeoff_10(self): self._takeoff(10.0)

#     def _takeoff(self, alt):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log(f"Sending takeoff to {alt}m...", "cmd")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#             0, 0, 0, 0, 0, 0, 0, alt)
#         self.log(f"Takeoff {alt}m sent", "info")

#     def rtl(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.set_mode("RTL")

#     # ═══════════════════════════════════════════════════════════
#     # CAMERA NAVIGATION
#     # ═══════════════════════════════════════════════════════════
#     def set_home_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 1 — SET HOME POINT", "nav")
#         self.log("  Saves ZED pose as origin_T0.json", "nav")
#         self.log("  Gimbal tilts to 30° after save", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SET_HOME, CMD_SET_HOME)

#     def start_scan_mission(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 2 — START SCAN MISSION", "nav")
#         self.log("  Jetson runs save_position.py", "nav")
#         self.log("  Fly the mission area now", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_START_SCAN, CMD_START_SCAN)

#     def save_end_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 3 — SAVE END POINT", "nav")
#         self.log("  Saves scan_end_pos.json on Jetson", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SAVE_END, CMD_SAVE_END)

#     def go_home_and_land(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 4 — GO HOME + LAND", "nav")
#         self.log("  Jetson runs return_land.py", "nav")
#         self.log("  AprilTag activated automatically", "nav")
#         self.log("  Phase 1 → ZED navigate to T0", "nav")
#         self.log("  Phase 2 → AprilTag align at 90°", "nav")
#         self.log("  Phase 3 → Land on rover", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_GO_HOME, CMD_GO_HOME)
#         if self.mav_connected:
#             self.set_mode("GUIDED")

#     # ── Jetson system commands ────────────────────────────────
#     def ping_jetson(self):
#         self.send_jetson_cmd(CMD_ID_PING, CMD_PING)

#     def launch_jetson(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH, CMD_LAUNCH)

#     def launch_jetson_sim(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH_SIM, CMD_LAUNCH_SIM)

#     def kill_jetson(self):
#         self.send_jetson_cmd(CMD_ID_STOP_ALL, "CLARQ_STOP_ALL")


# # ── Run ───────────────────────────────────────────────────────
# def main():
#     root = tk.Tk()
#     app  = DroneGUI(root)
#     root.mainloop()

# if __name__ == '__main__':
#     main()

# CLARQ_GUI.py
# Runs on Windows alongside Mission Planner
#
# Command routing (nRF path):
#   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
#
# Telemetry routing (MAVLink receive only):
#   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
#
# Mission Planner setup:
#   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
#
# Gimbal control:
#   GUI → USB Serial (COM port) → SimpleBGC Controller → Gimbal Motor
#
# pip install pymavlink pyserial
# CLARQ_GUI.py
# Runs on Windows alongside Mission Planner
#
# Command routing (nRF path):
#   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
#
# Telemetry routing (MAVLink receive only):
#   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
#
# Mission Planner setup:
#   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
#
# Gimbal control:
#   GUI → USB Serial (COM port) → SimpleBGC Controller → Gimbal Motor
#
# pip install pymavlink pyserial





# #SECOND SCRIPT
# import tkinter as tk
# from tkinter import scrolledtext, ttk
# from pymavlink import mavutil
# import threading
# import time
# import serial
# import serial.tools.list_ports
# import struct

# # ── Config ────────────────────────────────────────────────────
# MAVLINK_CONNECTION = "udp:127.0.0.1:14550"   # receive telemetry from MP
# SERIAL_BAUD        = 115200                   # must match Arduino sketch
# GIMBAL_BAUD        = 115200                   # gimbal serial baud rate

# # Valid gimbal angles (from your script)
# VALID_GIMBAL_ANGLES = [
#     0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30, 35, -35, 
#     40, -40, 45, -45, 50, -50, 55, -55, 60, -60, 65, -65, 70, -70, 
#     75, -75, 80, -80, 85, -85, 90, -90, 95, -95, 100, -100, 105, -105, 
#     110, -110, 115, -115, 120, -120, 125, -125, 130, -130, 135, -135, 
#     140, -140, 145, -145, 150, -150, 155, -155, 160, -160, 165, -165, 
#     170, -170, 175, -175, 180, -180, 360, -360
# ]

# # ── Command IDs — must match clarq_rf_listener.py ─────────────
# CMD_ID_PING       = 1
# CMD_ID_LAND       = 2
# CMD_ID_START_TAG  = 3
# CMD_ID_STOP_TAG       = 4
# CMD_ID_STOP_ALL       = 5
# CMD_ID_LAUNCH         = 6
# CMD_ID_LAUNCH_SIM     = 7
# CMD_ID_SET_HOME       = 8
# CMD_ID_START_SCAN     = 9   # Position tracking only (legacy)
# CMD_ID_SAVE_END       = 10
# CMD_ID_GO_HOME        = 11
# CMD_ID_START_3D_FUSION = 12  # Start ZED Fusion + position tracking
# CMD_ID_STOP_3D_FUSION  = 13  # Stop ZED Fusion recording
# CMD_ID_START_PHOTO     = 14  # Start photo capture mode
# CMD_ID_START_VIDEO     = 15  # Start video recording mode
# CMD_ID_STOP_CAPTURE    = 16  # Stop photo/video capture

# CMD_START_LANDING  = "CLARQ_LAND"
# CMD_START_APRILTAG = "CLARQ_START_TAG"
# CMD_STOP_APRILTAG  = "CLARQ_STOP_TAG"
# CMD_PING           = "CLARQ_PING"
# CMD_LAUNCH         = "CLARQ_LAUNCH"
# CMD_LAUNCH_SIM     = "CLARQ_LAUNCH_SIM"
# CMD_SET_HOME         = "CLARQ_SET_HOME"
# CMD_START_SCAN       = "CLARQ_START_SCAN"
# CMD_START_3D_FUSION  = "CLARQ_START_3D_FUSION"
# CMD_STOP_3D_FUSION   = "CLARQ_STOP_3D_FUSION"
# CMD_START_PHOTO      = "CLARQ_START_PHOTO"
# CMD_START_VIDEO      = "CLARQ_START_VIDEO"
# CMD_STOP_CAPTURE     = "CLARQ_STOP_CAPTURE"
# CMD_SAVE_END         = "CLARQ_SAVE_END"
# CMD_GO_HOME          = "CLARQ_GO_HOME"

# # ── Jetson response map (STATUSTEXT → status label) ───────────
# JETSON_STATUS_MAP = {
#     "CLARQ_PONG":              ("ONLINE ✓",          "#1ec882", "--"),
#     "CLARQ_HOME_SET":          ("HOME SET ✓",        "#1ec882", "origin_T0.json saved"),
#     "CLARQ_SCAN_STARTED":      ("SCANNING ✓",        "#1ec882", "Position tracking only"),
#     "CLARQ_3D_FUSION_STARTED": ("3D FUSION ✓",       "#1ec882", "ZED Fusion + tracking"),
#     "CLARQ_3D_FUSION_STOPPED": ("FUSION STOPPED",    "#646478", "Recording saved"),
#     "CLARQ_PHOTO_STARTED":     ("PHOTO MODE ✓",      "#1ec882", "Capturing images"),
#     "CLARQ_VIDEO_STARTED":     ("VIDEO MODE ✓",      "#1ec882", "Recording video"),
#     "CLARQ_CAPTURE_STOPPED":   ("CAPTURE STOPPED",   "#646478", "Files saved"),
#     "CLARQ_END_SAVED":         ("END SAVED ✓",       "#1ec882", "scan_end_pos.json saved"),
#     "CLARQ_GOING_HOME":        ("RETURNING HOME...", "#dc6438", "Phase 1 — navigating"),
#     "CLARQ_LAUNCHED":          ("LAUNCHED ✓",        "#1ec882", "All systems starting"),
#     "CLARQ_LAUNCHED_SIM":      ("LAUNCHED SIM ✓",    "#1ec882", "Sim stack starting"),
#     "CLARQ_ALL_STOPPED":       ("STOPPED",           "#646478", "All processes stopped"),
#     "CLARQ_TAG_RUNNING":       ("TAG RUNNING ✓",     "#1ec882", "AprilTag active"),
#     "CLARQ_TAG_STOPPED":       ("TAG STOPPED",       "#646478", "AprilTag stopped"),
# }

# # ── RX response codes from Arduino ────────────────────────────
# # Arduino sends back: TX_OK:<id>, TX_FAIL:<id>, RX:<id>
# ARDUINO_RX_MAP = {
#     CMD_ID_PING:            "CLARQ_PONG",
#     CMD_ID_LAND:            "CLARQ_LAND_ACK",
#     CMD_ID_START_TAG:       "CLARQ_TAG_RUNNING",
#     CMD_ID_STOP_TAG:        "CLARQ_TAG_STOPPED",
#     CMD_ID_STOP_ALL:        "CLARQ_ALL_STOPPED",
#     CMD_ID_LAUNCH:          "CLARQ_LAUNCHED",
#     CMD_ID_LAUNCH_SIM:      "CLARQ_LAUNCHED_SIM",
#     CMD_ID_SET_HOME:        "CLARQ_HOME_SET",
#     CMD_ID_START_SCAN:      "CLARQ_SCAN_STARTED",
#     CMD_ID_START_3D_FUSION: "CLARQ_3D_FUSION_STARTED",
#     CMD_ID_STOP_3D_FUSION:  "CLARQ_3D_FUSION_STOPPED",
#     CMD_ID_START_PHOTO:     "CLARQ_PHOTO_STARTED",
#     CMD_ID_START_VIDEO:     "CLARQ_VIDEO_STARTED",
#     CMD_ID_STOP_CAPTURE:    "CLARQ_CAPTURE_STOPPED",
#     CMD_ID_SAVE_END:        "CLARQ_END_SAVED",
#     CMD_ID_GO_HOME:         "CLARQ_GOING_HOME",
# }

# # ── Colors ────────────────────────────────────────────────────
# BG     = "#0f0f16"
# PANEL  = "#191926"
# ACCENT = "#1ec882"
# WARN   = "#dc6438"
# TEXT   = "#d2d2dc"
# DIM    = "#646478"
# GREEN  = "#1a6b42"
# RED    = "#6b1a1a"
# BLUE   = "#1a3d6b"
# PURPLE = "#4a1a6b"


# class DroneGUI:
#     def __init__(self, root):
#         self.root      = root
#         self.root.title("CLARQ Drone Control")
#         self.root.geometry("1000x800")
#         self.root.configure(bg=BG)

#         # MAVLink (telemetry receive only)
#         self.mav       = None
#         self.mav_connected = False
#         self.telem_on  = False

#         # nRF Arduino serial bridge
#         self.ser       = None
#         self.rf_connected = False

#         # Gimbal serial connection
#         self.gimbal_ser = None
#         self.gimbal_connected = False
#         self.current_gimbal_angle = 0

#         # Capture quality settings
#         self.capture_quality = "HIGH"  # LOW, MEDIUM, HIGH, ULTRA

#         self.build_ui()

#     # ═══════════════════════════════════════════════════════════
#     # GIMBAL CONTROL FUNCTIONS
#     # ═══════════════════════════════════════════════════════════
#     def gimbal_send_cmd(self, cmd, data=bytes([])):
#         """Send command to SimpleBGC gimbal controller."""
#         size = len(data)
#         header_checksum = (cmd + size) & 0xFF
#         body_checksum = 0
#         for b in data:
#             body_checksum = (body_checksum + b) & 0xFF
#         packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
#         self.gimbal_ser.write(packet)
#         time.sleep(0.1)
#         response = self.gimbal_ser.read(100)
#         return response

#     def gimbal_set_pitch(self, angle):
#         """Set gimbal pitch to absolute angle with drift correction."""
#         if not self.gimbal_connected or not self.gimbal_ser:
#             self.log("Gimbal not connected!", "error")
#             return

#         # Mode 2: absolute angle control
#         mode = 2
#         pitch = int(angle / 0.02197265625)
#         data = struct.pack('<Bhhhhhh', mode, 0, 0, 300, pitch, 0, 0)
#         self.gimbal_send_cmd(67, data)
#         time.sleep(0.5)
        
#         # Mode 0: release control (drift correction)
#         mode = 0
#         data = struct.pack('<Bhhhhhh', mode, 0, 0, 0, 0, 0, 0)
#         self.gimbal_send_cmd(67, data)
        
#         self.current_gimbal_angle = angle
#         self.log(f"Gimbal pitch set to {angle}°", "info")
        
#         # Update current angle display
#         self.root.after(0, lambda: self.gimbal_current_lbl.config(
#             text=f"Current: {angle}°", fg=ACCENT))

#     # ═══════════════════════════════════════════════════════════
#     # UI BUILD
#     # ═══════════════════════════════════════════════════════════
#     def build_ui(self):
#         title = tk.Frame(self.root, bg=PANEL, pady=12)
#         title.pack(fill=tk.X)

#         tk.Label(
#             title,
#             text="CLARQ DRONE NAVIGATION CONTROL",
#             font=("Consolas", 16, "bold"),
#             bg=PANEL, fg=ACCENT
#         ).pack(side=tk.LEFT, padx=20)

#         # Status indicators (right side of title bar)
#         status_frame = tk.Frame(title, bg=PANEL)
#         status_frame.pack(side=tk.RIGHT, padx=20)

#         self.gimbal_label = tk.Label(
#             status_frame, text="● GIMBAL DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.gimbal_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.rf_label = tk.Label(
#             status_frame, text="● RF DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.rf_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.conn_label = tk.Label(
#             status_frame, text="● MAV DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.conn_label.pack(side=tk.RIGHT)

#         main = tk.Frame(self.root, bg=BG)
#         main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

#         left_outer = tk.Frame(main, bg=PANEL, width=280)
#         left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
#         left_outer.pack_propagate(False)

#         left_canvas = tk.Canvas(
#             left_outer, bg=PANEL, width=260,
#             highlightthickness=0)
#         left_scrollbar = tk.Scrollbar(
#             left_outer, orient=tk.VERTICAL,
#             command=left_canvas.yview)
#         left_canvas.configure(
#             yscrollcommand=left_scrollbar.set)
#         left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
#         left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         left = tk.Frame(left_canvas, bg=PANEL)
#         left_canvas.create_window(
#             (0, 0), window=left, anchor=tk.NW)

#         def on_frame_configure(event):
#             left_canvas.configure(
#                 scrollregion=left_canvas.bbox("all"))

#         def on_mousewheel(event):
#             left_canvas.yview_scroll(
#                 int(-1 * (event.delta / 120)), "units")

#         left.bind("<Configure>", on_frame_configure)
#         left_canvas.bind("<MouseWheel>", on_mousewheel)
#         left.bind("<MouseWheel>", on_mousewheel)

#         right = tk.Frame(main, bg=PANEL)
#         right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         self.build_buttons(left)
#         self.build_output(right)

#     def build_buttons(self, parent):

#         def section(text):
#             tk.Frame(parent, bg=DIM, height=1).pack(
#                 fill=tk.X, padx=15, pady=(15, 0))
#             tk.Label(
#                 parent, text=text,
#                 font=("Consolas", 9),
#                 bg=PANEL, fg=DIM
#             ).pack(anchor=tk.W, padx=15, pady=(4, 6))

#         def btn(text, color, cmd, state=tk.NORMAL):
#             b = tk.Button(
#                 parent, text=text,
#                 font=("Consolas", 12, "bold"),
#                 bg=color, fg=TEXT,
#                 activebackground=color,
#                 relief=tk.FLAT, cursor="hand2",
#                 pady=10, state=state, command=cmd)
#             b.pack(fill=tk.X, padx=15, pady=3)
#             return b

#         # ── nRF / Arduino serial connection ───────────────────
#         section("NRF RADIO (COMMAND LINK)")

#         port_frame = tk.Frame(parent, bg=PANEL)
#         port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.port_var = tk.StringVar()
#         self.port_combo = ttk.Combobox(
#             port_frame,
#             textvariable=self.port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         self.refresh_ports()

#         btn("CONNECT RF",    BLUE, self.connect_rf)
#         btn("DISCONNECT RF", RED,  self.disconnect_rf)

#         # ── Gimbal control ────────────────────────────────────
#         section("GIMBAL CONTROL")

#         gimbal_port_frame = tk.Frame(parent, bg=PANEL)
#         gimbal_port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             gimbal_port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.gimbal_port_var = tk.StringVar()
#         self.gimbal_port_combo = ttk.Combobox(
#             gimbal_port_frame,
#             textvariable=self.gimbal_port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.gimbal_port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             gimbal_port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         btn("CONNECT GIMBAL",    GREEN, self.connect_gimbal)
#         btn("DISCONNECT GIMBAL", RED,   self.disconnect_gimbal)

#         # Gimbal angle input
#         angle_frame = tk.Frame(parent, bg=PANEL)
#         angle_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             angle_frame, text="Angle:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.gimbal_angle_entry = tk.Entry(
#             angle_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=10)
#         self.gimbal_angle_entry.insert(0, "0")
#         self.gimbal_angle_entry.pack(side=tk.LEFT, padx=5)
        
#         # Bind Enter key to set angle
#         self.gimbal_angle_entry.bind('<Return>', lambda e: self.set_gimbal_angle())

#         tk.Button(
#             angle_frame, text="SET",
#             font=("Consolas", 11, "bold"),
#             bg=BLUE, fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.set_gimbal_angle
#         ).pack(side=tk.LEFT)

#         # Current angle display
#         self.gimbal_current_lbl = tk.Label(
#             parent, text="Current: 0°",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.gimbal_current_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 6))

#         # Quick angle buttons
#         quick_angles_frame = tk.Frame(parent, bg=PANEL)
#         quick_angles_frame.pack(fill=tk.X, padx=15, pady=4)

#         for angle in [0, 30, 45, 60, 90]:
#             tk.Button(
#                 quick_angles_frame, text=f"{angle}°",
#                 font=("Consolas", 10),
#                 bg="#2a2a3a", fg=TEXT,
#                 relief=tk.FLAT, cursor="hand2",
#                 command=lambda a=angle: self.quick_set_angle(a)
#             ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

#         quick_angles_frame2 = tk.Frame(parent, bg=PANEL)
#         quick_angles_frame2.pack(fill=tk.X, padx=15, pady=(0, 4))

#         for angle in [-30, -45, -60, -90]:
#             tk.Button(
#                 quick_angles_frame2, text=f"{angle}°",
#                 font=("Consolas", 10),
#                 bg="#2a2a3a", fg=TEXT,
#                 relief=tk.FLAT, cursor="hand2",
#                 command=lambda a=angle: self.quick_set_angle(a)
#             ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

#         # ── MAVLink telemetry connection ───────────────────────
#         section("MAVLINK (TELEMETRY ONLY)")

#         conn_frame = tk.Frame(parent, bg=PANEL)
#         conn_frame.pack(fill=tk.X, padx=15, pady=4)
#         tk.Label(
#             conn_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)
#         self.conn_entry = tk.Entry(
#             conn_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=22)
#         self.conn_entry.insert(0, MAVLINK_CONNECTION)
#         self.conn_entry.pack(side=tk.LEFT, padx=5)

#         btn("CONNECT MAV",    BLUE, self.connect_mav)
#         btn("DISCONNECT MAV", RED,  self.disconnect_mav)

#         # ── Camera navigation ─────────────────────────────────
#         section("CAMERA NAVIGATION")

#         self.jetson_status_lbl = tk.Label(
#             parent, text="Jetson: IDLE",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.jetson_status_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 0))

#         self.jetson_phase_lbl = tk.Label(
#             parent, text="Phase: --",
#             font=("Consolas", 10),
#             bg=PANEL, fg=DIM)
#         self.jetson_phase_lbl.pack(
#             anchor=tk.W, padx=15, pady=(0, 6))

#         self.nav_btns = []
#         b = btn("1. SET HOME POINT",     GREEN,  self.set_home_point)
#         self.nav_btns.append(b)
        
#         # Step 2: 3D Fusion Scan with START/STOP control
#         tk.Label(
#             parent, text="2. 3D FUSION SCAN",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=TEXT
#         ).pack(anchor=tk.W, padx=15, pady=(8, 4))
        
#         tk.Label(
#             parent, text="ZED Fusion + Position Tracking",
#             font=("Consolas", 9),
#             bg=PANEL, fg=DIM
#         ).pack(anchor=tk.W, padx=15, pady=(0, 4))
        
#         # 3D Fusion control buttons
#         fusion_frame = tk.Frame(parent, bg=PANEL)
#         fusion_frame.pack(fill=tk.X, padx=15, pady=(0, 4))
        
#         tk.Button(
#             fusion_frame, text="START 3D FUSION",
#             font=("Consolas", 11, "bold"),
#             bg="#1a6b5a", fg=TEXT,
#             activebackground="#1a6b5a",
#             relief=tk.FLAT, cursor="hand2",
#             pady=10, command=self.start_3d_fusion
#         ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        
#         tk.Button(
#             fusion_frame, text="STOP FUSION",
#             font=("Consolas", 11, "bold"),
#             bg="#6b4a1a", fg=TEXT,
#             activebackground="#6b4a1a",
#             relief=tk.FLAT, cursor="hand2",
#             pady=10, command=self.stop_3d_fusion
#         ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))
        
#         # Photo/Video Capture Section
#         tk.Label(
#             parent, text="PHOTO / VIDEO CAPTURE",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=TEXT
#         ).pack(anchor=tk.W, padx=15, pady=(12, 4))
        
#         # Quality selector
#         quality_frame = tk.Frame(parent, bg=PANEL)
#         quality_frame.pack(fill=tk.X, padx=15, pady=(0, 4))
        
#         tk.Label(
#             quality_frame, text="Quality:",
#             font=("Consolas", 10),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)
        
#         self.quality_var = tk.StringVar(value="HIGH")
#         quality_dropdown = ttk.Combobox(
#             quality_frame,
#             textvariable=self.quality_var,
#             font=("Consolas", 10),
#             width=10, state="readonly",
#             values=["LOW", "MEDIUM", "HIGH", "ULTRA"])
#         quality_dropdown.pack(side=tk.LEFT, padx=5)
        
#         # Photo/Video control buttons
#         capture_frame = tk.Frame(parent, bg=PANEL)
#         capture_frame.pack(fill=tk.X, padx=15, pady=(0, 4))
        
#         tk.Button(
#             capture_frame, text="PHOTO MODE",
#             font=("Consolas", 10, "bold"),
#             bg="#4a1a6b", fg=TEXT,
#             activebackground="#4a1a6b",
#             relief=tk.FLAT, cursor="hand2",
#             pady=8, command=self.start_photo_capture
#         ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        
#         tk.Button(
#             capture_frame, text="VIDEO MODE",
#             font=("Consolas", 10, "bold"),
#             bg="#1a4a6b", fg=TEXT,
#             activebackground="#1a4a6b",
#             relief=tk.FLAT, cursor="hand2",
#             pady=8, command=self.start_video_capture
#         ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))
        
#         tk.Button(
#             parent, text="STOP CAPTURE",
#             font=("Consolas", 10, "bold"),
#             bg="#6b3a1a", fg=TEXT,
#             activebackground="#6b3a1a",
#             relief=tk.FLAT, cursor="hand2",
#             pady=8, command=self.stop_capture
#         ).pack(fill=tk.X, padx=15, pady=(0, 4))
        
#         b = btn("3. SAVE END POINT",     PURPLE, self.save_end_point)
#         self.nav_btns.append(b)
#         b = btn("4. GO HOME + LAND",     WARN,   self.go_home_and_land)
#         self.nav_btns.append(b)

#         # ── Jetson ────────────────────────────────────────────
#         section("JETSON")
#         btn("LAUNCH JETSON",     GREEN, self.launch_jetson)
#         btn("LAUNCH JETSON SIM", BLUE,  self.launch_jetson_sim)
#         btn("KILL JETSON",       RED,   self.kill_jetson)
#         btn("PING JETSON",       BLUE,  self.ping_jetson)

#         # ── Telemetry ─────────────────────────────────────────
#         section("TELEMETRY")

#         telem = tk.Frame(parent, bg="#12121e")
#         telem.pack(fill=tk.X, padx=15, pady=5)

#         self.telem = {}
#         for label, default in [
#             ("Mode",    "UNKNOWN"),
#             ("Armed",   "NO"),
#             ("Battery", "--V"),
#             ("Alt",     "--m"),
#             ("Lat",     "--"),
#             ("Lon",     "--"),
#             ("Heading", "--°"),
#             ("Speed",   "--m/s"),
#         ]:
#             row = tk.Frame(telem, bg="#12121e")
#             row.pack(fill=tk.X, padx=10, pady=2)
#             tk.Label(
#                 row, text=f"{label}:",
#                 font=("Consolas", 11),
#                 bg="#12121e", fg=DIM,
#                 width=9, anchor=tk.W
#             ).pack(side=tk.LEFT)
#             lbl = tk.Label(
#                 row, text=default,
#                 font=("Consolas", 11, "bold"),
#                 bg="#12121e", fg=ACCENT)
#             lbl.pack(side=tk.LEFT)
#             self.telem[label] = lbl

#         # ── Flight modes ──────────────────────────────────────
#         section("FLIGHT MODES")

#         self.flight_btns = []
#         for mode, color in [
#             ("STABILIZE", BLUE),
#             ("GUIDED",    BLUE),
#             ("LOITER",    BLUE),
#             ("RTL",       WARN),
#             ("LAND",      WARN),
#             ("AUTO",      BLUE),
#             ("ALTHOLD",   BLUE),
#             ("POSHOLD",   BLUE),
#         ]:
#             b = btn(mode, color,
#                     lambda m=mode: self.set_mode(m),
#                     tk.DISABLED)
#             self.flight_btns.append(b)

#         # ── Arm / Disarm ──────────────────────────────────────
#         section("ARM / DISARM")

#         self.arm_btn       = btn("ARM",       GREEN, self.arm,       tk.DISABLED)
#         self.disarm_btn    = btn("DISARM",    RED,   self.disarm,    tk.DISABLED)
#         self.force_arm_btn = btn("FORCE ARM", WARN,  self.force_arm, tk.DISABLED)

#         # ── Commands ──────────────────────────────────────────
#         section("COMMANDS")

#         self.cmd_btns = []
#         for text, color, cmd in [
#             ("TAKEOFF 2m",       BLUE, self.takeoff),
#             ("TAKEOFF 5m",       BLUE, self.takeoff_5),
#             ("TAKEOFF 10m",      BLUE, self.takeoff_10),
#             ("RETURN TO LAUNCH", WARN, self.rtl),
#         ]:
#             b = btn(text, color, cmd, tk.DISABLED)
#             self.cmd_btns.append(b)

#         tk.Frame(parent, bg=PANEL, height=20).pack()

#     def build_output(self, parent):
#         header = tk.Frame(parent, bg=PANEL)
#         header.pack(fill=tk.X, padx=10, pady=(10, 5))

#         tk.Label(
#             header, text="OUTPUT LOG",
#             font=("Consolas", 12),
#             bg=PANEL, fg=DIM
#         ).pack(side=tk.LEFT)

#         tk.Button(
#             header, text="CLEAR",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.clear
#         ).pack(side=tk.RIGHT, padx=5)

#         tk.Button(
#             header, text="SAVE LOG",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.save_log
#         ).pack(side=tk.RIGHT, padx=5)

#         self.output = scrolledtext.ScrolledText(
#             parent, font=("Consolas", 11),
#             bg="#0a0a14", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, wrap=tk.WORD,
#             state=tk.DISABLED)
#         self.output.pack(
#             fill=tk.BOTH, expand=True,
#             padx=10, pady=(0, 5))

#         self.output.tag_config("info",   foreground=ACCENT)
#         self.output.tag_config("warn",   foreground=WARN)
#         self.output.tag_config("error",  foreground="#dc3838")
#         self.output.tag_config("cmd",    foreground="#6496c8")
#         self.output.tag_config("nav",    foreground="#50c8c8")
#         self.output.tag_config("normal", foreground=TEXT)
#         self.output.tag_config("dim",    foreground=DIM)

#     # ═══════════════════════════════════════════════════════════
#     # LOGGING
#     # ═══════════════════════════════════════════════════════════
#     def log(self, message, tag="normal"):
#         def _log():
#             self.output.config(state=tk.NORMAL)
#             ts = time.strftime("%H:%M:%S")
#             self.output.insert(
#                 tk.END, f"[{ts}] {message}\n", tag)
#             self.output.see(tk.END)
#             self.output.config(state=tk.DISABLED)
#         self.root.after(0, _log)

#     def clear(self):
#         self.output.config(state=tk.NORMAL)
#         self.output.delete(1.0, tk.END)
#         self.output.config(state=tk.DISABLED)

#     def save_log(self):
#         content = self.output.get(1.0, tk.END)
#         fname = f"clarq_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
#         with open(fname, 'w') as f:
#             f.write(content)
#         self.log(f"Log saved to {fname}", "info")

#     def update_telem(self, key, value):
#         def _u():
#             if key in self.telem:
#                 self.telem[key].config(text=value)
#         self.root.after(0, _u)

#     def update_jetson_status(self, text):
#         if text in JETSON_STATUS_MAP:
#             status, color, phase = JETSON_STATUS_MAP[text]
#             def _u():
#                 self.jetson_status_lbl.config(
#                     text=f"Jetson: {status}", fg=color)
#                 self.jetson_phase_lbl.config(
#                     text=f"Phase: {phase}", fg=color)
#             self.root.after(0, _u)

#     def enable_mav_buttons(self):
#         for b in (self.flight_btns + self.cmd_btns):
#             b.config(state=tk.NORMAL)
#         self.arm_btn.config(state=tk.NORMAL)
#         self.disarm_btn.config(state=tk.NORMAL)
#         self.force_arm_btn.config(state=tk.NORMAL)

#     # ═══════════════════════════════════════════════════════════
#     # GIMBAL SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def connect_gimbal(self):
#         def _connect():
#             port = self.gimbal_port_var.get()
#             if not port:
#                 self.log("No gimbal COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to gimbal on {port}...", "info")
#                 self.gimbal_ser = serial.Serial(port, GIMBAL_BAUD, timeout=1)
#                 time.sleep(2)  # wait for controller to be ready

#                 # Turn motors on (CMD 77)
#                 self.log("Turning gimbal motors on...", "dim")
#                 self.gimbal_send_cmd(77)
#                 time.sleep(1)

#                 # Set to 0 degrees initially
#                 self.gimbal_set_pitch(0)

#                 self.gimbal_connected = True
#                 self.log(f"Gimbal connected on {port} ✓", "info")
#                 self.root.after(
#                     0, lambda: self.gimbal_label.config(
#                         text="● GIMBAL CONNECTED", fg=ACCENT))

#             except Exception as e:
#                 self.log(f"Gimbal connect failed: {e}", "error")
#                 if self.gimbal_ser:
#                     try:
#                         self.gimbal_ser.close()
#                     except:
#                         pass
#                     self.gimbal_ser = None

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_gimbal(self):
#         self.gimbal_connected = False
#         if self.gimbal_ser:
#             try:
#                 self.gimbal_ser.close()
#             except Exception:
#                 pass
#             self.gimbal_ser = None
#         self.log("Gimbal disconnected", "warn")
#         self.root.after(
#             0, lambda: self.gimbal_label.config(
#                 text="● GIMBAL DISCONNECTED", fg=WARN))

#     def set_gimbal_angle(self):
#         """Set gimbal angle from entry field."""
#         try:
#             angle = float(self.gimbal_angle_entry.get())
            
#             if angle not in VALID_GIMBAL_ANGLES:
#                 self.log(f"Invalid angle: {angle}° (not in valid range)", "error")
#                 return
            
#             self.gimbal_set_pitch(angle)
            
#         except ValueError:
#             self.log("Invalid angle input — enter a number", "error")

#     def quick_set_angle(self, angle):
#         """Quick set button handler."""
#         self.gimbal_angle_entry.delete(0, tk.END)
#         self.gimbal_angle_entry.insert(0, str(angle))
#         self.gimbal_set_pitch(angle)

#     # ═══════════════════════════════════════════════════════════
#     # NRF / ARDUINO SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def refresh_ports(self):
#         """Scan for available COM ports and populate dropdown."""
#         ports = [p.device for p in serial.tools.list_ports.comports()]
#         self.port_combo['values'] = ports
        
#         # Only update gimbal combo if it exists (it's created after RF section)
#         if hasattr(self, 'gimbal_port_combo'):
#             self.gimbal_port_combo['values'] = ports
        
#         if ports:
#             if not self.port_var.get():
#                 self.port_combo.set(ports[0])
#             if hasattr(self, 'gimbal_port_combo') and not self.gimbal_port_var.get():
#                 # Try to set a different port for gimbal if available
#                 self.gimbal_port_combo.set(ports[1] if len(ports) > 1 else ports[0])
#         else:
#             self.port_combo.set("")
#             if hasattr(self, 'gimbal_port_combo'):
#                 self.gimbal_port_combo.set("")
        
#         self.log(f"Found ports: {ports if ports else 'none'}", "dim")

#     def connect_rf(self):
#         def _connect():
#             port = self.port_var.get()
#             if not port:
#                 self.log("No COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to Arduino on {port}...", "info")
#                 self.ser = serial.Serial(port, SERIAL_BAUD, timeout=2)
#                 time.sleep(2)  # wait for Arduino reset after serial open

#                 # Wait for CLARQ_RF_READY handshake from Arduino
#                 self.log("Waiting for Arduino ready signal...", "dim")
#                 deadline = time.time() + 5
#                 ready = False
#                 while time.time() < deadline:
#                     if self.ser.in_waiting:
#                         line = self.ser.readline().decode(
#                             'utf-8', errors='ignore').strip()
#                         if line == "CLARQ_RF_READY":
#                             ready = True
#                             break
#                         self.log(f"Arduino: {line}", "dim")

#                 if ready:
#                     self.rf_connected = True
#                     self.log(
#                         f"RF bridge connected on {port} ✓", "info")
#                     self.root.after(
#                         0, lambda: self.rf_label.config(
#                             text="● RF CONNECTED", fg=ACCENT))
#                     # Start background reader for Arduino responses
#                     threading.Thread(
#                         target=self.rf_read_loop,
#                         daemon=True).start()
#                 else:
#                     self.log(
#                         "Arduino did not respond — check port/baud",
#                         "error")
#                     self.ser.close()
#                     self.ser = None

#             except Exception as e:
#                 self.log(f"RF connect failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_rf(self):
#         self.rf_connected = False
#         if self.ser:
#             try:
#                 self.ser.close()
#             except Exception:
#                 pass
#             self.ser = None
#         self.log("RF bridge disconnected", "warn")
#         self.root.after(
#             0, lambda: self.rf_label.config(
#                 text="● RF DISCONNECTED", fg=WARN))

#     def rf_read_loop(self):
#         """Background thread — reads responses from Arduino."""
#         while self.rf_connected and self.ser:
#             try:
#                 if self.ser.in_waiting:
#                     line = self.ser.readline().decode(
#                         'utf-8', errors='ignore').strip()
#                     if not line:
#                         continue

#                     if line.startswith("TX_OK:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX OK — cmd {cmd_id} delivered ✓",
#                             "info")

#                     elif line.startswith("TX_FAIL:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX FAIL — cmd {cmd_id} not delivered",
#                             "error")

#                     elif line.startswith("RX:"):
#                         # Jetson sent an ACK back over nRF
#                         cmd_id = int(line.split(":")[1])
#                         label = ARDUINO_RX_MAP.get(cmd_id, f"id={cmd_id}")
#                         self.log(
#                             f"JETSON ACK: {label}", "info")
#                         self.update_jetson_status(label)

#                     elif line.startswith("ERR:"):
#                         self.log(f"Arduino error: {line}", "warn")

#                     else:
#                         self.log(f"Arduino: {line}", "dim")

#                 else:
#                     time.sleep(0.05)

#             except Exception as e:
#                 if self.rf_connected:
#                     self.log(f"RF read error: {e}", "error")
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # SEND COMMAND TO JETSON VIA NRF ARDUINO BRIDGE
#     # ═══════════════════════════════════════════════════════════
#     def send_jetson_cmd(self, cmd_id, text):
#         """
#         Send command to Jetson via Arduino nRF bridge.
#         Writes 'CMD:<id>\n' over USB serial to Arduino.
#         Arduino transmits over NRF24L01 to Jetson.
#         """
#         self.log(f"Sending to Jetson: {text} (id={cmd_id})", "cmd")

#         if not self.rf_connected or not self.ser:
#             self.log(
#                 "RF bridge not connected — cannot send command",
#                 "error")
#             return

#         try:
#             packet = f"CMD:{cmd_id}\n"
#             self.ser.write(packet.encode('utf-8'))
#             self.log(
#                 f"→ Arduino: {packet.strip()}", "dim")
#         except Exception as e:
#             self.log(f"Serial write failed: {e}", "error")

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK TELEMETRY (RECEIVE ONLY — no send)
#     # ═══════════════════════════════════════════════════════════
#     def connect_mav(self):
#         def _connect():
#             try:
#                 conn = self.conn_entry.get()
#                 self.log(f"Connecting MAVLink to {conn}...", "info")
#                 self.mav = mavutil.mavlink_connection(conn)
#                 self.log("Waiting for heartbeat...", "dim")
#                 self.mav.wait_heartbeat(timeout=10)
#                 self.mav_connected = True
#                 self.log(
#                     f"MAVLink connected — telemetry active ✓", "info")
#                 self.root.after(
#                     0, lambda: self.conn_label.config(
#                         text="● MAV CONNECTED", fg=ACCENT))
#                 self.root.after(0, self.enable_mav_buttons)
#                 self.start_telemetry()
#             except Exception as e:
#                 self.log(
#                     f"MAVLink connection failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_mav(self):
#         self.mav_connected = False
#         self.telem_on = False
#         self.log("MAVLink disconnected", "warn")
#         self.root.after(
#             0, lambda: self.conn_label.config(
#                 text="● MAV DISCONNECTED", fg=WARN))

#     def start_telemetry(self):
#         self.telem_on = True
#         threading.Thread(
#             target=self.telem_loop, daemon=True).start()

#     def telem_loop(self):
#         while self.telem_on and self.mav_connected:
#             try:
#                 msg = self.mav.recv_match(
#                     type=[
#                         'HEARTBEAT', 'SYS_STATUS',
#                         'GLOBAL_POSITION_INT',
#                         'VFR_HUD', 'STATUSTEXT'
#                     ],
#                     blocking=True, timeout=1)
#                 if msg is None:
#                     continue

#                 t = msg.get_type()

#                 if t == 'STATUSTEXT':
#                     text = msg.text.strip()
#                     if text.startswith('CLARQ'):
#                         self.log(f"JETSON: {text}", "info")
#                         self.update_jetson_status(text)
#                     else:
#                         self.log(f"FC: {text}", "dim")
#                     continue

#                 if t == 'HEARTBEAT':
#                     src = (msg.get_srcSystem()
#                            if hasattr(msg, 'get_srcSystem')
#                            else 1)
#                     if src != 1:
#                         continue
#                     mode  = mavutil.mode_string_v10(msg)
#                     armed = bool(
#                         msg.base_mode &
#                         mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
#                     self.update_telem('Mode',  mode)
#                     self.update_telem(
#                         'Armed', 'YES' if armed else 'NO')

#                 elif t == 'SYS_STATUS':
#                     v = msg.voltage_battery / 1000.0
#                     self.update_telem('Battery', f'{v:.1f}V')

#                 elif t == 'GLOBAL_POSITION_INT':
#                     lat = msg.lat / 1e7
#                     lon = msg.lon / 1e7
#                     alt = msg.relative_alt / 1000.0
#                     hdg = msg.hdg / 100.0
#                     self.update_telem('Lat',     f'{lat:.6f}')
#                     self.update_telem('Lon',     f'{lon:.6f}')
#                     self.update_telem('Alt',     f'{alt:.1f}m')
#                     self.update_telem('Heading', f'{hdg:.0f}°')

#                 elif t == 'VFR_HUD':
#                     self.update_telem(
#                         'Speed', f'{msg.groundspeed:.1f}m/s')

#             except Exception:
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK FLIGHT COMMANDS (still sent via MAVLink)
#     # ═══════════════════════════════════════════════════════════
#     def set_mode(self, mode):
#         if not self.mav_connected or not self.mav:
#             self.log("MAVLink not connected!", "error")
#             return
#         mode_map = {
#             "STABILIZE": 0, "ACRO": 1,  "ALTHOLD": 2,
#             "AUTO":      3, "GUIDED": 4, "LOITER":  5,
#             "RTL":       6, "CIRCLE": 7, "LAND":    9,
#             "POSHOLD":  16,
#         }
#         if mode not in mode_map:
#             self.log(f"Unknown mode: {mode}", "error")
#             return
#         mode_num = mode_map[mode]
#         self.mav.mav.set_mode_send(
#             self.mav.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num)
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#             0,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num, 0, 0, 0, 0, 0)
#         self.log(f"Mode {mode} ({mode_num}) sent", "cmd")

#         def _watch_ack():
#             try:
#                 ack = self.mav.recv_match(
#                     type='COMMAND_ACK',
#                     blocking=True, timeout=3)
#                 if ack and ack.result == 0:
#                     self.log(f"Mode {mode} confirmed ✓", "info")
#                 elif ack:
#                     self.log(
#                         f"Mode {mode} rejected (result={ack.result})",
#                         "warn")
#                 else:
#                     self.log(f"Mode {mode} — no ACK", "dim")
#             except Exception:
#                 pass
#         threading.Thread(
#             target=_watch_ack, daemon=True).start()

#     def arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Arming...", "warn")
#         self.mav.arducopter_arm()
#         self.log("Arm command sent", "info")

#     def disarm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Disarming...", "warn")
#         self.mav.arducopter_disarm()
#         self.log("Disarm command sent", "info")

#     def force_arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Force arming...", "warn")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0, 1, 21196, 0, 0, 0, 0, 0)
#         self.log("Force arm sent!", "info")

#     def takeoff(self):    self._takeoff(2.0)
#     def takeoff_5(self):  self._takeoff(5.0)
#     def takeoff_10(self): self._takeoff(10.0)

#     def _takeoff(self, alt):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log(f"Sending takeoff to {alt}m...", "cmd")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#             0, 0, 0, 0, 0, 0, 0, alt)
#         self.log(f"Takeoff {alt}m sent", "info")

#     def rtl(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.set_mode("RTL")

#     # ═══════════════════════════════════════════════════════════
#     # CAMERA NAVIGATION
#     # ═══════════════════════════════════════════════════════════
#     def set_home_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 1 — SET HOME POINT", "nav")
#         self.log("  Saves ZED pose as origin_T0.json", "nav")
#         self.log("  Gimbal tilts to 30° after save", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SET_HOME, CMD_SET_HOME)

#     def start_3d_fusion(self):
#         """Start ZED Fusion 3D reconstruction + position tracking."""
#         self.log("─" * 44, "nav")
#         self.log("  STEP 2 — START 3D FUSION SCAN", "nav")
#         self.log("  Jetson starting:", "nav")
#         self.log("    • ZED Fusion (3D reconstruction)", "nav")
#         self.log("    • Position tracking (save_position.py)", "nav")
#         self.log("  Recording to .SVO file", "nav")
#         self.log("  Fly mission area with Mission Planner", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_START_3D_FUSION, CMD_START_3D_FUSION)

#     def stop_3d_fusion(self):
#         """Stop ZED Fusion recording and save the 3D scan."""
#         self.log("─" * 44, "nav")
#         self.log("  STOPPING 3D FUSION", "nav")
#         self.log("  Saving 3D reconstruction file", "nav")
#         self.log("  Position data saved", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_STOP_3D_FUSION, CMD_STOP_3D_FUSION)

#     def start_photo_capture(self):
#         """Start photo capture mode with selected quality."""
#         quality = self.quality_var.get()
#         self.log("─" * 44, "nav")
#         self.log("  PHOTO CAPTURE MODE", "nav")
#         self.log(f"  Quality: {quality}", "nav")
#         self.log("  Capturing images during flight", "nav")
#         self.log("  Fly mission area with Mission Planner", "nav")
#         self.log("─" * 44, "nav")
        
#         # Encode quality in the command (will be parsed by Jetson)
#         self.send_jetson_cmd(CMD_ID_START_PHOTO, f"{CMD_START_PHOTO}:{quality}")

#     def start_video_capture(self):
#         """Start video recording mode with selected quality."""
#         quality = self.quality_var.get()
#         self.log("─" * 44, "nav")
#         self.log("  VIDEO RECORDING MODE", "nav")
#         self.log(f"  Quality: {quality}", "nav")
#         self.log("  Recording video during flight", "nav")
#         self.log("  Fly mission area with Mission Planner", "nav")
#         self.log("─" * 44, "nav")
        
#         # Encode quality in the command (will be parsed by Jetson)
#         self.send_jetson_cmd(CMD_ID_START_VIDEO, f"{CMD_START_VIDEO}:{quality}")

#     def stop_capture(self):
#         """Stop photo/video capture."""
#         self.log("─" * 44, "nav")
#         self.log("  STOPPING CAPTURE", "nav")
#         self.log("  Saving files...", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_STOP_CAPTURE, CMD_STOP_CAPTURE)

#     def save_end_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 3 — SAVE END POINT", "nav")
#         self.log("  Saves scan_end_pos.json on Jetson", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SAVE_END, CMD_SAVE_END)

#     def go_home_and_land(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 4 — GO HOME + LAND", "nav")
#         self.log("  Jetson runs return_land.py", "nav")
#         self.log("  AprilTag activated automatically", "nav")
#         self.log("  Phase 1 → ZED navigate to T0", "nav")
#         self.log("  Phase 2 → AprilTag align at 90°", "nav")
#         self.log("  Phase 3 → Land on rover", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_GO_HOME, CMD_GO_HOME)
#         if self.mav_connected:
#             self.set_mode("GUIDED")

#     # ── Jetson system commands ────────────────────────────────
#     def ping_jetson(self):
#         self.send_jetson_cmd(CMD_ID_PING, CMD_PING)

#     def launch_jetson(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH, CMD_LAUNCH)

#     def launch_jetson_sim(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH_SIM, CMD_LAUNCH_SIM)

#     def kill_jetson(self):
#         self.send_jetson_cmd(CMD_ID_STOP_ALL, "CLARQ_STOP_ALL")


# # ── Run ───────────────────────────────────────────────────────
# def main():
#     root = tk.Tk()
#     app  = DroneGUI(root)
#     root.mainloop()

# if __name__ == '__main__':
#     main()






#REMOTE
# # CLARQ_GUI.py
# # Runs on Windows alongside Mission Planner
# #
# # Command routing (nRF path):
# #   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
# #
# # Telemetry routing (MAVLink receive only):
# #   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
# #
# # Mission Planner setup:
# #   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
# #
# # pip install pymavlink pyserial

# import tkinter as tk
# from tkinter import scrolledtext, ttk
# from pymavlink import mavutil
# import threading
# import time
# import serial
# import serial.tools.list_ports

# # ── Config ────────────────────────────────────────────────────
# MAVLINK_CONNECTION = "udp:127.0.0.1:14550"   # receive telemetry from MP
# SERIAL_BAUD        = 115200                   # must match Arduino sketch

# # ── Command IDs — must match clarq_rf_listener.py ─────────────
# CMD_ID_PING       = 1
# CMD_ID_LAND       = 2
# CMD_ID_START_TAG  = 3
# CMD_ID_STOP_TAG   = 4
# CMD_ID_STOP_ALL   = 5
# CMD_ID_LAUNCH     = 6
# CMD_ID_LAUNCH_SIM = 7
# CMD_ID_SET_HOME   = 8
# CMD_ID_START_SCAN = 9
# CMD_ID_SAVE_END   = 10
# CMD_ID_GO_HOME    = 11

# CMD_START_LANDING  = "CLARQ_LAND"
# CMD_START_APRILTAG = "CLARQ_START_TAG"
# CMD_STOP_APRILTAG  = "CLARQ_STOP_TAG"
# CMD_PING           = "CLARQ_PING"
# CMD_LAUNCH         = "CLARQ_LAUNCH"
# CMD_LAUNCH_SIM     = "CLARQ_LAUNCH_SIM"
# CMD_SET_HOME       = "CLARQ_SET_HOME"
# CMD_START_SCAN     = "CLARQ_START_SCAN"
# CMD_SAVE_END       = "CLARQ_SAVE_END"
# CMD_GO_HOME        = "CLARQ_GO_HOME"

# # ── Jetson response map (STATUSTEXT → status label) ───────────
# JETSON_STATUS_MAP = {
#     "CLARQ_PONG":         ("ONLINE ✓",          "#1ec882", "--"),
#     "CLARQ_HOME_SET":     ("HOME SET ✓",         "#1ec882", "origin_T0.json saved"),
#     "CLARQ_SCAN_STARTED": ("SCANNING ✓",         "#1ec882", "Mission scan running"),
#     "CLARQ_END_SAVED":    ("END SAVED ✓",        "#1ec882", "scan_end_pos.json saved"),
#     "CLARQ_GOING_HOME":   ("RETURNING HOME...",  "#dc6438", "Phase 1 — navigating"),
#     "CLARQ_LAUNCHED":     ("LAUNCHED ✓",         "#1ec882", "All systems starting"),
#     "CLARQ_LAUNCHED_SIM": ("LAUNCHED SIM ✓",     "#1ec882", "Sim stack starting"),
#     "CLARQ_ALL_STOPPED":  ("STOPPED",            "#646478", "All processes stopped"),
#     "CLARQ_TAG_RUNNING":  ("TAG RUNNING ✓",      "#1ec882", "AprilTag active"),
#     "CLARQ_TAG_STOPPED":  ("TAG STOPPED",        "#646478", "AprilTag stopped"),
# }

# # ── RX response codes from Arduino ────────────────────────────
# # Arduino sends back: TX_OK:<id>, TX_FAIL:<id>, RX:<id>
# ARDUINO_RX_MAP = {
#     CMD_ID_PING:       "CLARQ_PONG",
#     CMD_ID_LAND:       "CLARQ_LAND_ACK",
#     CMD_ID_START_TAG:  "CLARQ_TAG_RUNNING",
#     CMD_ID_STOP_TAG:   "CLARQ_TAG_STOPPED",
#     CMD_ID_STOP_ALL:   "CLARQ_ALL_STOPPED",
#     CMD_ID_LAUNCH:     "CLARQ_LAUNCHED",
#     CMD_ID_LAUNCH_SIM: "CLARQ_LAUNCHED_SIM",
#     CMD_ID_SET_HOME:   "CLARQ_HOME_SET",
#     CMD_ID_START_SCAN: "CLARQ_SCAN_STARTED",
#     CMD_ID_SAVE_END:   "CLARQ_END_SAVED",
#     CMD_ID_GO_HOME:    "CLARQ_GOING_HOME",
# }

# # ── Colors ────────────────────────────────────────────────────
# BG     = "#0f0f16"
# PANEL  = "#191926"
# ACCENT = "#1ec882"
# WARN   = "#dc6438"
# TEXT   = "#d2d2dc"
# DIM    = "#646478"
# GREEN  = "#1a6b42"
# RED    = "#6b1a1a"
# BLUE   = "#1a3d6b"
# PURPLE = "#4a1a6b"


# class DroneGUI:
#     def __init__(self, root):
#         self.root      = root
#         self.root.title("CLARQ Drone Control")
#         self.root.geometry("1000x800")
#         self.root.configure(bg=BG)

#         # MAVLink (telemetry receive only)
#         self.mav       = None
#         self.mav_connected = False
#         self.telem_on  = False

#         # nRF Arduino serial bridge
#         self.ser       = None
#         self.rf_connected = False

#         self.build_ui()

#     # ═══════════════════════════════════════════════════════════
#     # UI BUILD
#     # ═══════════════════════════════════════════════════════════
#     def build_ui(self):
#         title = tk.Frame(self.root, bg=PANEL, pady=12)
#         title.pack(fill=tk.X)

#         tk.Label(
#             title,
#             text="CLARQ DRONE NAVIGATION CONTROL",
#             font=("Consolas", 16, "bold"),
#             bg=PANEL, fg=ACCENT
#         ).pack(side=tk.LEFT, padx=20)

#         # Status indicators (right side of title bar)
#         status_frame = tk.Frame(title, bg=PANEL)
#         status_frame.pack(side=tk.RIGHT, padx=20)

#         self.rf_label = tk.Label(
#             status_frame, text="● RF DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.rf_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.conn_label = tk.Label(
#             status_frame, text="● MAV DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.conn_label.pack(side=tk.RIGHT)

#         main = tk.Frame(self.root, bg=BG)
#         main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

#         left_outer = tk.Frame(main, bg=PANEL, width=280)
#         left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
#         left_outer.pack_propagate(False)

#         left_canvas = tk.Canvas(
#             left_outer, bg=PANEL, width=260,
#             highlightthickness=0)
#         left_scrollbar = tk.Scrollbar(
#             left_outer, orient=tk.VERTICAL,
#             command=left_canvas.yview)
#         left_canvas.configure(
#             yscrollcommand=left_scrollbar.set)
#         left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
#         left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         left = tk.Frame(left_canvas, bg=PANEL)
#         left_canvas.create_window(
#             (0, 0), window=left, anchor=tk.NW)

#         def on_frame_configure(event):
#             left_canvas.configure(
#                 scrollregion=left_canvas.bbox("all"))

#         def on_mousewheel(event):
#             left_canvas.yview_scroll(
#                 int(-1 * (event.delta / 120)), "units")

#         left.bind("<Configure>", on_frame_configure)
#         left_canvas.bind("<MouseWheel>", on_mousewheel)
#         left.bind("<MouseWheel>", on_mousewheel)

#         right = tk.Frame(main, bg=PANEL)
#         right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         self.build_buttons(left)
#         self.build_output(right)

#     def build_buttons(self, parent):

#         def section(text):
#             tk.Frame(parent, bg=DIM, height=1).pack(
#                 fill=tk.X, padx=15, pady=(15, 0))
#             tk.Label(
#                 parent, text=text,
#                 font=("Consolas", 9),
#                 bg=PANEL, fg=DIM
#             ).pack(anchor=tk.W, padx=15, pady=(4, 6))

#         def btn(text, color, cmd, state=tk.NORMAL):
#             b = tk.Button(
#                 parent, text=text,
#                 font=("Consolas", 12, "bold"),
#                 bg=color, fg=TEXT,
#                 activebackground=color,
#                 relief=tk.FLAT, cursor="hand2",
#                 pady=10, state=state, command=cmd)
#             b.pack(fill=tk.X, padx=15, pady=3)
#             return b

#         # ── nRF / Arduino serial connection ───────────────────
#         section("NRF RADIO (COMMAND LINK)")

#         port_frame = tk.Frame(parent, bg=PANEL)
#         port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.port_var = tk.StringVar()
#         self.port_combo = ttk.Combobox(
#             port_frame,
#             textvariable=self.port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         self.refresh_ports()

#         btn("CONNECT RF",    BLUE, self.connect_rf)
#         btn("DISCONNECT RF", RED,  self.disconnect_rf)

#         # ── MAVLink telemetry connection ───────────────────────
#         section("MAVLINK (TELEMETRY ONLY)")

#         conn_frame = tk.Frame(parent, bg=PANEL)
#         conn_frame.pack(fill=tk.X, padx=15, pady=4)
#         tk.Label(
#             conn_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)
#         self.conn_entry = tk.Entry(
#             conn_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=22)
#         self.conn_entry.insert(0, MAVLINK_CONNECTION)
#         self.conn_entry.pack(side=tk.LEFT, padx=5)

#         btn("CONNECT MAV",    BLUE, self.connect_mav)
#         btn("DISCONNECT MAV", RED,  self.disconnect_mav)

#         # ── Camera navigation ─────────────────────────────────
#         section("CAMERA NAVIGATION")

#         self.jetson_status_lbl = tk.Label(
#             parent, text="Jetson: IDLE",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.jetson_status_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 0))

#         self.jetson_phase_lbl = tk.Label(
#             parent, text="Phase: --",
#             font=("Consolas", 10),
#             bg=PANEL, fg=DIM)
#         self.jetson_phase_lbl.pack(
#             anchor=tk.W, padx=15, pady=(0, 6))

#         self.nav_btns = []
#         b = btn("1. SET HOME POINT",     GREEN,  self.set_home_point)
#         self.nav_btns.append(b)
#         b = btn("2. START SCAN MISSION", BLUE,   self.start_scan_mission)
#         self.nav_btns.append(b)
#         b = btn("3. SAVE END POINT",     PURPLE, self.save_end_point)
#         self.nav_btns.append(b)
#         b = btn("4. GO HOME + LAND",     WARN,   self.go_home_and_land)
#         self.nav_btns.append(b)

#         # ── Jetson ────────────────────────────────────────────
#         section("JETSON")
#         btn("LAUNCH JETSON",     GREEN, self.launch_jetson)
#         btn("LAUNCH JETSON SIM", BLUE,  self.launch_jetson_sim)
#         btn("KILL JETSON",       RED,   self.kill_jetson)
#         btn("PING JETSON",       BLUE,  self.ping_jetson)

#         # ── Telemetry ─────────────────────────────────────────
#         section("TELEMETRY")

#         telem = tk.Frame(parent, bg="#12121e")
#         telem.pack(fill=tk.X, padx=15, pady=5)

#         self.telem = {}
#         for label, default in [
#             ("Mode",    "UNKNOWN"),
#             ("Armed",   "NO"),
#             ("Battery", "--V"),
#             ("Alt",     "--m"),
#             ("Lat",     "--"),
#             ("Lon",     "--"),
#             ("Heading", "--°"),
#             ("Speed",   "--m/s"),
#         ]:
#             row = tk.Frame(telem, bg="#12121e")
#             row.pack(fill=tk.X, padx=10, pady=2)
#             tk.Label(
#                 row, text=f"{label}:",
#                 font=("Consolas", 11),
#                 bg="#12121e", fg=DIM,
#                 width=9, anchor=tk.W
#             ).pack(side=tk.LEFT)
#             lbl = tk.Label(
#                 row, text=default,
#                 font=("Consolas", 11, "bold"),
#                 bg="#12121e", fg=ACCENT)
#             lbl.pack(side=tk.LEFT)
#             self.telem[label] = lbl

#         # ── Flight modes ──────────────────────────────────────
#         section("FLIGHT MODES")

#         self.flight_btns = []
#         for mode, color in [
#             ("STABILIZE", BLUE),
#             ("GUIDED",    BLUE),
#             ("LOITER",    BLUE),
#             ("RTL",       WARN),
#             ("LAND",      WARN),
#             ("AUTO",      BLUE),
#             ("ALTHOLD",   BLUE),
#             ("POSHOLD",   BLUE),
#         ]:
#             b = btn(mode, color,
#                     lambda m=mode: self.set_mode(m),
#                     tk.DISABLED)
#             self.flight_btns.append(b)

#         # ── Arm / Disarm ──────────────────────────────────────
#         section("ARM / DISARM")

#         self.arm_btn       = btn("ARM",       GREEN, self.arm,       tk.DISABLED)
#         self.disarm_btn    = btn("DISARM",    RED,   self.disarm,    tk.DISABLED)
#         self.force_arm_btn = btn("FORCE ARM", WARN,  self.force_arm, tk.DISABLED)

#         # ── Commands ──────────────────────────────────────────
#         section("COMMANDS")

#         self.cmd_btns = []
#         for text, color, cmd in [
#             ("TAKEOFF 2m",       BLUE, self.takeoff),
#             ("TAKEOFF 5m",       BLUE, self.takeoff_5),
#             ("TAKEOFF 10m",      BLUE, self.takeoff_10),
#             ("RETURN TO LAUNCH", WARN, self.rtl),
#         ]:
#             b = btn(text, color, cmd, tk.DISABLED)
#             self.cmd_btns.append(b)

#         tk.Frame(parent, bg=PANEL, height=20).pack()

#     def build_output(self, parent):
#         header = tk.Frame(parent, bg=PANEL)
#         header.pack(fill=tk.X, padx=10, pady=(10, 5))

#         tk.Label(
#             header, text="OUTPUT LOG",
#             font=("Consolas", 12),
#             bg=PANEL, fg=DIM
#         ).pack(side=tk.LEFT)

#         tk.Button(
#             header, text="CLEAR",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.clear
#         ).pack(side=tk.RIGHT, padx=5)

#         tk.Button(
#             header, text="SAVE LOG",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.save_log
#         ).pack(side=tk.RIGHT, padx=5)

#         self.output = scrolledtext.ScrolledText(
#             parent, font=("Consolas", 11),
#             bg="#0a0a14", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, wrap=tk.WORD,
#             state=tk.DISABLED)
#         self.output.pack(
#             fill=tk.BOTH, expand=True,
#             padx=10, pady=(0, 5))

#         self.output.tag_config("info",   foreground=ACCENT)
#         self.output.tag_config("warn",   foreground=WARN)
#         self.output.tag_config("error",  foreground="#dc3838")
#         self.output.tag_config("cmd",    foreground="#6496c8")
#         self.output.tag_config("nav",    foreground="#50c8c8")
#         self.output.tag_config("normal", foreground=TEXT)
#         self.output.tag_config("dim",    foreground=DIM)

#     # ═══════════════════════════════════════════════════════════
#     # LOGGING
#     # ═══════════════════════════════════════════════════════════
#     def log(self, message, tag="normal"):
#         def _log():
#             self.output.config(state=tk.NORMAL)
#             ts = time.strftime("%H:%M:%S")
#             self.output.insert(
#                 tk.END, f"[{ts}] {message}\n", tag)
#             self.output.see(tk.END)
#             self.output.config(state=tk.DISABLED)
#         self.root.after(0, _log)

#     def clear(self):
#         self.output.config(state=tk.NORMAL)
#         self.output.delete(1.0, tk.END)
#         self.output.config(state=tk.DISABLED)

#     def save_log(self):
#         content = self.output.get(1.0, tk.END)
#         fname = f"clarq_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
#         with open(fname, 'w') as f:
#             f.write(content)
#         self.log(f"Log saved to {fname}", "info")

#     def update_telem(self, key, value):
#         def _u():
#             if key in self.telem:
#                 self.telem[key].config(text=value)
#         self.root.after(0, _u)

#     def update_jetson_status(self, text):
#         if text in JETSON_STATUS_MAP:
#             status, color, phase = JETSON_STATUS_MAP[text]
#             def _u():
#                 self.jetson_status_lbl.config(
#                     text=f"Jetson: {status}", fg=color)
#                 self.jetson_phase_lbl.config(
#                     text=f"Phase: {phase}", fg=color)
#             self.root.after(0, _u)

#     def enable_mav_buttons(self):
#         for b in (self.flight_btns + self.cmd_btns):
#             b.config(state=tk.NORMAL)
#         self.arm_btn.config(state=tk.NORMAL)
#         self.disarm_btn.config(state=tk.NORMAL)
#         self.force_arm_btn.config(state=tk.NORMAL)

#     # ═══════════════════════════════════════════════════════════
#     # NRF / ARDUINO SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def refresh_ports(self):
#         """Scan for available COM ports and populate dropdown."""
#         ports = [p.device for p in serial.tools.list_ports.comports()]
#         self.port_combo['values'] = ports
#         if ports:
#             self.port_combo.set(ports[0])
#         else:
#             self.port_combo.set("")
#         self.log(f"Found ports: {ports if ports else 'none'}", "dim")

#     def connect_rf(self):
#         def _connect():
#             port = self.port_var.get()
#             if not port:
#                 self.log("No COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to Arduino on {port}...", "info")
#                 self.ser = serial.Serial(port, SERIAL_BAUD, timeout=2)
#                 time.sleep(2)  # wait for Arduino reset after serial open

#                 # Wait for CLARQ_RF_READY handshake from Arduino
#                 self.log("Waiting for Arduino ready signal...", "dim")
#                 deadline = time.time() + 5
#                 ready = False
#                 while time.time() < deadline:
#                     if self.ser.in_waiting:
#                         line = self.ser.readline().decode(
#                             'utf-8', errors='ignore').strip()
#                         if line == "CLARQ_RF_READY":
#                             ready = True
#                             break
#                         self.log(f"Arduino: {line}", "dim")

#                 if ready:
#                     self.rf_connected = True
#                     self.log(
#                         f"RF bridge connected on {port} ✓", "info")
#                     self.root.after(
#                         0, lambda: self.rf_label.config(
#                             text="● RF CONNECTED", fg=ACCENT))
#                     # Start background reader for Arduino responses
#                     threading.Thread(
#                         target=self.rf_read_loop,
#                         daemon=True).start()
#                 else:
#                     self.log(
#                         "Arduino did not respond — check port/baud",
#                         "error")
#                     self.ser.close()
#                     self.ser = None

#             except Exception as e:
#                 self.log(f"RF connect failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_rf(self):
#         self.rf_connected = False
#         if self.ser:
#             try:
#                 self.ser.close()
#             except Exception:
#                 pass
#             self.ser = None
#         self.log("RF bridge disconnected", "warn")
#         self.root.after(
#             0, lambda: self.rf_label.config(
#                 text="● RF DISCONNECTED", fg=WARN))

#     def rf_read_loop(self):
#         """Background thread — reads responses from Arduino."""
#         while self.rf_connected and self.ser:
#             try:
#                 if self.ser.in_waiting:
#                     line = self.ser.readline().decode(
#                         'utf-8', errors='ignore').strip()
#                     if not line:
#                         continue

#                     if line.startswith("TX_OK:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX OK — cmd {cmd_id} delivered ✓",
#                             "info")

#                     elif line.startswith("TX_FAIL:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX FAIL — cmd {cmd_id} not delivered",
#                             "error")

#                     elif line.startswith("RX:"):
#                         # Jetson sent an ACK back over nRF
#                         cmd_id = int(line.split(":")[1])
#                         label = ARDUINO_RX_MAP.get(cmd_id, f"id={cmd_id}")
#                         self.log(
#                             f"JETSON ACK: {label}", "info")
#                         self.update_jetson_status(label)

#                     elif line.startswith("ERR:"):
#                         self.log(f"Arduino error: {line}", "warn")

#                     else:
#                         self.log(f"Arduino: {line}", "dim")

#                 else:
#                     time.sleep(0.05)

#             except Exception as e:
#                 if self.rf_connected:
#                     self.log(f"RF read error: {e}", "error")
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # SEND COMMAND TO JETSON VIA NRF ARDUINO BRIDGE
#     # ═══════════════════════════════════════════════════════════
#     def send_jetson_cmd(self, cmd_id, text):
#         """
#         Send command to Jetson via Arduino nRF bridge.
#         Writes 'CMD:<id>\n' over USB serial to Arduino.
#         Arduino transmits over NRF24L01 to Jetson.
#         """
#         self.log(f"Sending to Jetson: {text} (id={cmd_id})", "cmd")

#         if not self.rf_connected or not self.ser:
#             self.log(
#                 "RF bridge not connected — cannot send command",
#                 "error")
#             return

#         try:
#             packet = f"CMD:{cmd_id}\n"
#             self.ser.write(packet.encode('utf-8'))
#             self.log(
#                 f"→ Arduino: {packet.strip()}", "dim")
#         except Exception as e:
#             self.log(f"Serial write failed: {e}", "error")

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK TELEMETRY (RECEIVE ONLY — no send)
#     # ═══════════════════════════════════════════════════════════
#     def connect_mav(self):
#         def _connect():
#             try:
#                 conn = self.conn_entry.get()
#                 self.log(f"Connecting MAVLink to {conn}...", "info")
#                 self.mav = mavutil.mavlink_connection(conn)
#                 self.log("Waiting for heartbeat...", "dim")
#                 self.mav.wait_heartbeat(timeout=10)
#                 self.mav_connected = True
#                 self.log(
#                     f"MAVLink connected — telemetry active ✓", "info")
#                 self.root.after(
#                     0, lambda: self.conn_label.config(
#                         text="● MAV CONNECTED", fg=ACCENT))
#                 self.root.after(0, self.enable_mav_buttons)
#                 self.start_telemetry()
#             except Exception as e:
#                 self.log(
#                     f"MAVLink connection failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_mav(self):
#         self.mav_connected = False
#         self.telem_on = False
#         self.log("MAVLink disconnected", "warn")
#         self.root.after(
#             0, lambda: self.conn_label.config(
#                 text="● MAV DISCONNECTED", fg=WARN))

#     def start_telemetry(self):
#         self.telem_on = True
#         threading.Thread(
#             target=self.telem_loop, daemon=True).start()

#     def telem_loop(self):
#         while self.telem_on and self.mav_connected:
#             try:
#                 msg = self.mav.recv_match(
#                     type=[
#                         'HEARTBEAT', 'SYS_STATUS',
#                         'GLOBAL_POSITION_INT',
#                         'VFR_HUD', 'STATUSTEXT'
#                     ],
#                     blocking=True, timeout=1)
#                 if msg is None:
#                     continue

#                 t = msg.get_type()

#                 if t == 'STATUSTEXT':
#                     text = msg.text.strip()
#                     if text.startswith('CLARQ'):
#                         self.log(f"JETSON: {text}", "info")
#                         self.update_jetson_status(text)
#                     else:
#                         self.log(f"FC: {text}", "dim")
#                     continue

#                 if t == 'HEARTBEAT':
#                     src = (msg.get_srcSystem()
#                            if hasattr(msg, 'get_srcSystem')
#                            else 1)
#                     if src != 1:
#                         continue
#                     mode  = mavutil.mode_string_v10(msg)
#                     armed = bool(
#                         msg.base_mode &
#                         mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
#                     self.update_telem('Mode',  mode)
#                     self.update_telem(
#                         'Armed', 'YES' if armed else 'NO')

#                 elif t == 'SYS_STATUS':
#                     v = msg.voltage_battery / 1000.0
#                     self.update_telem('Battery', f'{v:.1f}V')

#                 elif t == 'GLOBAL_POSITION_INT':
#                     lat = msg.lat / 1e7
#                     lon = msg.lon / 1e7
#                     alt = msg.relative_alt / 1000.0
#                     hdg = msg.hdg / 100.0
#                     self.update_telem('Lat',     f'{lat:.6f}')
#                     self.update_telem('Lon',     f'{lon:.6f}')
#                     self.update_telem('Alt',     f'{alt:.1f}m')
#                     self.update_telem('Heading', f'{hdg:.0f}°')

#                 elif t == 'VFR_HUD':
#                     self.update_telem(
#                         'Speed', f'{msg.groundspeed:.1f}m/s')

#             except Exception:
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK FLIGHT COMMANDS (still sent via MAVLink)
#     # ═══════════════════════════════════════════════════════════
#     def set_mode(self, mode):
#         if not self.mav_connected or not self.mav:
#             self.log("MAVLink not connected!", "error")
#             return
#         mode_map = {
#             "STABILIZE": 0, "ACRO": 1,  "ALTHOLD": 2,
#             "AUTO":      3, "GUIDED": 4, "LOITER":  5,
#             "RTL":       6, "CIRCLE": 7, "LAND":    9,
#             "POSHOLD":  16,
#         }
#         if mode not in mode_map:
#             self.log(f"Unknown mode: {mode}", "error")
#             return
#         mode_num = mode_map[mode]
#         self.mav.mav.set_mode_send(
#             self.mav.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num)
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#             0,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num, 0, 0, 0, 0, 0)
#         self.log(f"Mode {mode} ({mode_num}) sent", "cmd")

#         def _watch_ack():
#             try:
#                 ack = self.mav.recv_match(
#                     type='COMMAND_ACK',
#                     blocking=True, timeout=3)
#                 if ack and ack.result == 0:
#                     self.log(f"Mode {mode} confirmed ✓", "info")
#                 elif ack:
#                     self.log(
#                         f"Mode {mode} rejected (result={ack.result})",
#                         "warn")
#                 else:
#                     self.log(f"Mode {mode} — no ACK", "dim")
#             except Exception:
#                 pass
#         threading.Thread(
#             target=_watch_ack, daemon=True).start()

#     def arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Arming...", "warn")
#         self.mav.arducopter_arm()
#         self.log("Arm command sent", "info")

#     def disarm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Disarming...", "warn")
#         self.mav.arducopter_disarm()
#         self.log("Disarm command sent", "info")

#     def force_arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Force arming...", "warn")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0, 1, 21196, 0, 0, 0, 0, 0)
#         self.log("Force arm sent!", "info")

#     def takeoff(self):    self._takeoff(2.0)
#     def takeoff_5(self):  self._takeoff(5.0)
#     def takeoff_10(self): self._takeoff(10.0)

#     def _takeoff(self, alt):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log(f"Sending takeoff to {alt}m...", "cmd")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#             0, 0, 0, 0, 0, 0, 0, alt)
#         self.log(f"Takeoff {alt}m sent", "info")

#     def rtl(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.set_mode("RTL")

#     # ═══════════════════════════════════════════════════════════
#     # CAMERA NAVIGATION
#     # ═══════════════════════════════════════════════════════════
#     def set_home_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 1 — SET HOME POINT", "nav")
#         self.log("  Saves ZED pose as origin_T0.json", "nav")
#         self.log("  Gimbal tilts to 30° after save", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SET_HOME, CMD_SET_HOME)

#     def start_scan_mission(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 2 — START SCAN MISSION", "nav")
#         self.log("  Jetson runs save_position.py", "nav")
#         self.log("  Fly the mission area now", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_START_SCAN, CMD_START_SCAN)

#     def save_end_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 3 — SAVE END POINT", "nav")
#         self.log("  Saves scan_end_pos.json on Jetson", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SAVE_END, CMD_SAVE_END)

#     def go_home_and_land(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 4 — GO HOME + LAND", "nav")
#         self.log("  Jetson runs return_land.py", "nav")
#         self.log("  AprilTag activated automatically", "nav")
#         self.log("  Phase 1 → ZED navigate to T0", "nav")
#         self.log("  Phase 2 → AprilTag align at 90°", "nav")
#         self.log("  Phase 3 → Land on rover", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_GO_HOME, CMD_GO_HOME)
#         if self.mav_connected:
#             self.set_mode("GUIDED")

#     # ── Jetson system commands ────────────────────────────────
#     def ping_jetson(self):
#         self.send_jetson_cmd(CMD_ID_PING, CMD_PING)

#     def launch_jetson(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH, CMD_LAUNCH)

#     def launch_jetson_sim(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH_SIM, CMD_LAUNCH_SIM)

#     def kill_jetson(self):
#         self.send_jetson_cmd(CMD_ID_STOP_ALL, "CLARQ_STOP_ALL")


# # ── Run ───────────────────────────────────────────────────────
# def main():
#     root = tk.Tk()
#     app  = DroneGUI(root)
#     root.mainloop()

# if __name__ == '__main__':
#     main()




#WITH GIMBAL CONTROLS
# CLARQ_GUI.py
# Runs on Windows alongside Mission Planner
#
# Command routing (nRF path):
#   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
#
# Telemetry routing (MAVLink receive only):
#   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
#
# Mission Planner setup:
#   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
#
# Gimbal control:
#   GUI → USB Serial (COM port) → SimpleBGC Controller → Gimbal Motor
#
# pip install pymavlink pyserial










# import tkinter as tk
# from tkinter import scrolledtext, ttk
# from pymavlink import mavutil
# import threading
# import time
# import serial
# import serial.tools.list_ports
# import struct

# # ── Config ────────────────────────────────────────────────────
# MAVLINK_CONNECTION = "udp:127.0.0.1:14550"   # receive telemetry from MP
# SERIAL_BAUD        = 115200                   # must match Arduino sketch
# GIMBAL_BAUD        = 115200                   # gimbal serial baud rate

# # Valid gimbal angles (from your script)
# VALID_GIMBAL_ANGLES = [
#     0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30, 35, -35, 
#     40, -40, 45, -45, 50, -50, 55, -55, 60, -60, 65, -65, 70, -70, 
#     75, -75, 80, -80, 85, -85, 90, -90, 95, -95, 100, -100, 105, -105, 
#     110, -110, 115, -115, 120, -120, 125, -125, 130, -130, 135, -135, 
#     140, -140, 145, -145, 150, -150, 155, -155, 160, -160, 165, -165, 
#     170, -170, 175, -175, 180, -180, 360, -360
# ]

# # ── Command IDs — must match clarq_rf_listener.py ─────────────
# CMD_ID_PING       = 1
# CMD_ID_LAND       = 2
# CMD_ID_START_TAG  = 3
# CMD_ID_STOP_TAG   = 4
# CMD_ID_STOP_ALL   = 5
# CMD_ID_LAUNCH     = 6
# CMD_ID_LAUNCH_SIM = 7
# CMD_ID_SET_HOME   = 8
# CMD_ID_START_SCAN = 9
# CMD_ID_SAVE_END   = 10
# CMD_ID_GO_HOME    = 11

# CMD_START_LANDING  = "CLARQ_LAND"
# CMD_START_APRILTAG = "CLARQ_START_TAG"
# CMD_STOP_APRILTAG  = "CLARQ_STOP_TAG"
# CMD_PING           = "CLARQ_PING"
# CMD_LAUNCH         = "CLARQ_LAUNCH"
# CMD_LAUNCH_SIM     = "CLARQ_LAUNCH_SIM"
# CMD_SET_HOME       = "CLARQ_SET_HOME"
# CMD_START_SCAN     = "CLARQ_START_SCAN"
# CMD_SAVE_END       = "CLARQ_SAVE_END"
# CMD_GO_HOME        = "CLARQ_GO_HOME"

# # ── Jetson response map (STATUSTEXT → status label) ───────────
# JETSON_STATUS_MAP = {
#     "CLARQ_PONG":         ("ONLINE ✓",          "#1ec882", "--"),
#     "CLARQ_HOME_SET":     ("HOME SET ✓",         "#1ec882", "origin_T0.json saved"),
#     "CLARQ_SCAN_STARTED": ("SCANNING ✓",         "#1ec882", "Mission scan running"),
#     "CLARQ_END_SAVED":    ("END SAVED ✓",        "#1ec882", "scan_end_pos.json saved"),
#     "CLARQ_GOING_HOME":   ("RETURNING HOME...",  "#dc6438", "Phase 1 — navigating"),
#     "CLARQ_LAUNCHED":     ("LAUNCHED ✓",         "#1ec882", "All systems starting"),
#     "CLARQ_LAUNCHED_SIM": ("LAUNCHED SIM ✓",     "#1ec882", "Sim stack starting"),
#     "CLARQ_ALL_STOPPED":  ("STOPPED",            "#646478", "All processes stopped"),
#     "CLARQ_TAG_RUNNING":  ("TAG RUNNING ✓",      "#1ec882", "AprilTag active"),
#     "CLARQ_TAG_STOPPED":  ("TAG STOPPED",        "#646478", "AprilTag stopped"),
# }

# # ── RX response codes from Arduino ────────────────────────────
# # Arduino sends back: TX_OK:<id>, TX_FAIL:<id>, RX:<id>
# ARDUINO_RX_MAP = {
#     CMD_ID_PING:       "CLARQ_PONG",
#     CMD_ID_LAND:       "CLARQ_LAND_ACK",
#     CMD_ID_START_TAG:  "CLARQ_TAG_RUNNING",
#     CMD_ID_STOP_TAG:   "CLARQ_TAG_STOPPED",
#     CMD_ID_STOP_ALL:   "CLARQ_ALL_STOPPED",
#     CMD_ID_LAUNCH:     "CLARQ_LAUNCHED",
#     CMD_ID_LAUNCH_SIM: "CLARQ_LAUNCHED_SIM",
#     CMD_ID_SET_HOME:   "CLARQ_HOME_SET",
#     CMD_ID_START_SCAN: "CLARQ_SCAN_STARTED",
#     CMD_ID_SAVE_END:   "CLARQ_END_SAVED",
#     CMD_ID_GO_HOME:    "CLARQ_GOING_HOME",
# }

# # ── Colors ────────────────────────────────────────────────────
# BG     = "#0f0f16"
# PANEL  = "#191926"
# ACCENT = "#1ec882"
# WARN   = "#dc6438"
# TEXT   = "#d2d2dc"
# DIM    = "#646478"
# GREEN  = "#1a6b42"
# RED    = "#6b1a1a"
# BLUE   = "#1a3d6b"
# PURPLE = "#4a1a6b"


# class DroneGUI:
#     def __init__(self, root):
#         self.root      = root
#         self.root.title("CLARQ Drone Control")
#         self.root.geometry("1000x800")
#         self.root.configure(bg=BG)

#         # MAVLink (telemetry receive only)
#         self.mav       = None
#         self.mav_connected = False
#         self.telem_on  = False

#         # nRF Arduino serial bridge
#         self.ser       = None
#         self.rf_connected = False

#         # Gimbal serial connection
#         self.gimbal_ser = None
#         self.gimbal_connected = False
#         self.current_gimbal_angle = 0

#         self.build_ui()

#     # ═══════════════════════════════════════════════════════════
#     # GIMBAL CONTROL FUNCTIONS
#     # ═══════════════════════════════════════════════════════════
#     def gimbal_send_cmd(self, cmd, data=bytes([])):
#         """Send command to SimpleBGC gimbal controller."""
#         size = len(data)
#         header_checksum = (cmd + size) & 0xFF
#         body_checksum = 0
#         for b in data:
#             body_checksum = (body_checksum + b) & 0xFF
#         packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
#         self.gimbal_ser.write(packet)
#         time.sleep(0.1)
#         response = self.gimbal_ser.read(100)
#         return response

#     def gimbal_set_pitch(self, angle):
#         """Set gimbal pitch to absolute angle with drift correction."""
#         if not self.gimbal_connected or not self.gimbal_ser:
#             self.log("Gimbal not connected!", "error")
#             return

#         # Mode 2: absolute angle control
#         mode = 2
#         pitch = int(angle / 0.02197265625)
#         data = struct.pack('<Bhhhhhh', mode, 0, 0, 300, pitch, 0, 0)
#         self.gimbal_send_cmd(67, data)
#         time.sleep(0.5)
        
#         # Mode 0: release control (drift correction)
#         mode = 0
#         data = struct.pack('<Bhhhhhh', mode, 0, 0, 0, 0, 0, 0)
#         self.gimbal_send_cmd(67, data)
        
#         self.current_gimbal_angle = angle
#         self.log(f"Gimbal pitch set to {angle}°", "info")
        
#         # Update current angle display
#         self.root.after(0, lambda: self.gimbal_current_lbl.config(
#             text=f"Current: {angle}°", fg=ACCENT))

#     # ═══════════════════════════════════════════════════════════
#     # UI BUILD
#     # ═══════════════════════════════════════════════════════════
#     def build_ui(self):
#         title = tk.Frame(self.root, bg=PANEL, pady=12)
#         title.pack(fill=tk.X)

#         tk.Label(
#             title,
#             text="CLARQ DRONE NAVIGATION CONTROL",
#             font=("Consolas", 16, "bold"),
#             bg=PANEL, fg=ACCENT
#         ).pack(side=tk.LEFT, padx=20)

#         # Status indicators (right side of title bar)
#         status_frame = tk.Frame(title, bg=PANEL)
#         status_frame.pack(side=tk.RIGHT, padx=20)

#         self.gimbal_label = tk.Label(
#             status_frame, text="● GIMBAL DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.gimbal_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.rf_label = tk.Label(
#             status_frame, text="● RF DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.rf_label.pack(side=tk.RIGHT, padx=(10, 0))

#         self.conn_label = tk.Label(
#             status_frame, text="● MAV DISCONNECTED",
#             font=("Consolas", 11),
#             bg=PANEL, fg=WARN)
#         self.conn_label.pack(side=tk.RIGHT)

#         main = tk.Frame(self.root, bg=BG)
#         main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

#         left_outer = tk.Frame(main, bg=PANEL, width=280)
#         left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
#         left_outer.pack_propagate(False)

#         left_canvas = tk.Canvas(
#             left_outer, bg=PANEL, width=260,
#             highlightthickness=0)
#         left_scrollbar = tk.Scrollbar(
#             left_outer, orient=tk.VERTICAL,
#             command=left_canvas.yview)
#         left_canvas.configure(
#             yscrollcommand=left_scrollbar.set)
#         left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
#         left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         left = tk.Frame(left_canvas, bg=PANEL)
#         left_canvas.create_window(
#             (0, 0), window=left, anchor=tk.NW)

#         def on_frame_configure(event):
#             left_canvas.configure(
#                 scrollregion=left_canvas.bbox("all"))

#         def on_mousewheel(event):
#             left_canvas.yview_scroll(
#                 int(-1 * (event.delta / 120)), "units")

#         left.bind("<Configure>", on_frame_configure)
#         left_canvas.bind("<MouseWheel>", on_mousewheel)
#         left.bind("<MouseWheel>", on_mousewheel)

#         right = tk.Frame(main, bg=PANEL)
#         right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

#         self.build_buttons(left)
#         self.build_output(right)

#     def build_buttons(self, parent):

#         def section(text):
#             tk.Frame(parent, bg=DIM, height=1).pack(
#                 fill=tk.X, padx=15, pady=(15, 0))
#             tk.Label(
#                 parent, text=text,
#                 font=("Consolas", 9),
#                 bg=PANEL, fg=DIM
#             ).pack(anchor=tk.W, padx=15, pady=(4, 6))

#         def btn(text, color, cmd, state=tk.NORMAL):
#             b = tk.Button(
#                 parent, text=text,
#                 font=("Consolas", 12, "bold"),
#                 bg=color, fg=TEXT,
#                 activebackground=color,
#                 relief=tk.FLAT, cursor="hand2",
#                 pady=10, state=state, command=cmd)
#             b.pack(fill=tk.X, padx=15, pady=3)
#             return b

#         # ── nRF / Arduino serial connection ───────────────────
#         section("NRF RADIO (COMMAND LINK)")

#         port_frame = tk.Frame(parent, bg=PANEL)
#         port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.port_var = tk.StringVar()
#         self.port_combo = ttk.Combobox(
#             port_frame,
#             textvariable=self.port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         self.refresh_ports()

#         btn("CONNECT RF",    BLUE, self.connect_rf)
#         btn("DISCONNECT RF", RED,  self.disconnect_rf)

#         # ── Gimbal control ────────────────────────────────────
#         section("GIMBAL CONTROL")

#         gimbal_port_frame = tk.Frame(parent, bg=PANEL)
#         gimbal_port_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             gimbal_port_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.gimbal_port_var = tk.StringVar()
#         self.gimbal_port_combo = ttk.Combobox(
#             gimbal_port_frame,
#             textvariable=self.gimbal_port_var,
#             font=("Consolas", 11),
#             width=12, state="readonly")
#         self.gimbal_port_combo.pack(side=tk.LEFT, padx=5)

#         tk.Button(
#             gimbal_port_frame, text="↻",
#             font=("Consolas", 12),
#             bg="#2a2a3a", fg=ACCENT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.refresh_ports
#         ).pack(side=tk.LEFT)

#         btn("CONNECT GIMBAL",    GREEN, self.connect_gimbal)
#         btn("DISCONNECT GIMBAL", RED,   self.disconnect_gimbal)

#         # Gimbal angle input
#         angle_frame = tk.Frame(parent, bg=PANEL)
#         angle_frame.pack(fill=tk.X, padx=15, pady=4)

#         tk.Label(
#             angle_frame, text="Angle:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)

#         self.gimbal_angle_entry = tk.Entry(
#             angle_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=10)
#         self.gimbal_angle_entry.insert(0, "0")
#         self.gimbal_angle_entry.pack(side=tk.LEFT, padx=5)
        
#         # Bind Enter key to set angle
#         self.gimbal_angle_entry.bind('<Return>', lambda e: self.set_gimbal_angle())

#         tk.Button(
#             angle_frame, text="SET",
#             font=("Consolas", 11, "bold"),
#             bg=BLUE, fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.set_gimbal_angle
#         ).pack(side=tk.LEFT)

#         # Current angle display
#         self.gimbal_current_lbl = tk.Label(
#             parent, text="Current: 0°",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.gimbal_current_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 6))

#         # Quick angle buttons
#         quick_angles_frame = tk.Frame(parent, bg=PANEL)
#         quick_angles_frame.pack(fill=tk.X, padx=15, pady=4)

#         for angle in [0, 30, 45, 60, 90]:
#             tk.Button(
#                 quick_angles_frame, text=f"{angle}°",
#                 font=("Consolas", 10),
#                 bg="#2a2a3a", fg=TEXT,
#                 relief=tk.FLAT, cursor="hand2",
#                 command=lambda a=angle: self.quick_set_angle(a)
#             ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

#         quick_angles_frame2 = tk.Frame(parent, bg=PANEL)
#         quick_angles_frame2.pack(fill=tk.X, padx=15, pady=(0, 4))

#         for angle in [-30, -45, -60, -90]:
#             tk.Button(
#                 quick_angles_frame2, text=f"{angle}°",
#                 font=("Consolas", 10),
#                 bg="#2a2a3a", fg=TEXT,
#                 relief=tk.FLAT, cursor="hand2",
#                 command=lambda a=angle: self.quick_set_angle(a)
#             ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

#         # ── MAVLink telemetry connection ───────────────────────
#         section("MAVLINK (TELEMETRY ONLY)")

#         conn_frame = tk.Frame(parent, bg=PANEL)
#         conn_frame.pack(fill=tk.X, padx=15, pady=4)
#         tk.Label(
#             conn_frame, text="Port:",
#             font=("Consolas", 11),
#             bg=PANEL, fg=TEXT
#         ).pack(side=tk.LEFT)
#         self.conn_entry = tk.Entry(
#             conn_frame, font=("Consolas", 11),
#             bg="#2a2a3a", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, width=22)
#         self.conn_entry.insert(0, MAVLINK_CONNECTION)
#         self.conn_entry.pack(side=tk.LEFT, padx=5)

#         btn("CONNECT MAV",    BLUE, self.connect_mav)
#         btn("DISCONNECT MAV", RED,  self.disconnect_mav)

#         # ── Camera navigation ─────────────────────────────────
#         section("CAMERA NAVIGATION")

#         self.jetson_status_lbl = tk.Label(
#             parent, text="Jetson: IDLE",
#             font=("Consolas", 11, "bold"),
#             bg=PANEL, fg=DIM)
#         self.jetson_status_lbl.pack(
#             anchor=tk.W, padx=15, pady=(2, 0))

#         self.jetson_phase_lbl = tk.Label(
#             parent, text="Phase: --",
#             font=("Consolas", 10),
#             bg=PANEL, fg=DIM)
#         self.jetson_phase_lbl.pack(
#             anchor=tk.W, padx=15, pady=(0, 6))

#         self.nav_btns = []
#         b = btn("1. SET HOME POINT",     GREEN,  self.set_home_point)
#         self.nav_btns.append(b)
#         b = btn("2. START SCAN MISSION", BLUE,   self.start_scan_mission)
#         self.nav_btns.append(b)
#         b = btn("3. SAVE END POINT",     PURPLE, self.save_end_point)
#         self.nav_btns.append(b)
#         b = btn("4. GO HOME + LAND",     WARN,   self.go_home_and_land)
#         self.nav_btns.append(b)

#         # ── Jetson ────────────────────────────────────────────
#         section("JETSON")
#         btn("LAUNCH JETSON",     GREEN, self.launch_jetson)
#         btn("LAUNCH JETSON SIM", BLUE,  self.launch_jetson_sim)
#         btn("KILL JETSON",       RED,   self.kill_jetson)
#         btn("PING JETSON",       BLUE,  self.ping_jetson)

#         # ── Telemetry ─────────────────────────────────────────
#         section("TELEMETRY")

#         telem = tk.Frame(parent, bg="#12121e")
#         telem.pack(fill=tk.X, padx=15, pady=5)

#         self.telem = {}
#         for label, default in [
#             ("Mode",    "UNKNOWN"),
#             ("Armed",   "NO"),
#             ("Battery", "--V"),
#             ("Alt",     "--m"),
#             ("Lat",     "--"),
#             ("Lon",     "--"),
#             ("Heading", "--°"),
#             ("Speed",   "--m/s"),
#         ]:
#             row = tk.Frame(telem, bg="#12121e")
#             row.pack(fill=tk.X, padx=10, pady=2)
#             tk.Label(
#                 row, text=f"{label}:",
#                 font=("Consolas", 11),
#                 bg="#12121e", fg=DIM,
#                 width=9, anchor=tk.W
#             ).pack(side=tk.LEFT)
#             lbl = tk.Label(
#                 row, text=default,
#                 font=("Consolas", 11, "bold"),
#                 bg="#12121e", fg=ACCENT)
#             lbl.pack(side=tk.LEFT)
#             self.telem[label] = lbl

#         # ── Flight modes ──────────────────────────────────────
#         section("FLIGHT MODES")

#         self.flight_btns = []
#         for mode, color in [
#             ("STABILIZE", BLUE),
#             ("GUIDED",    BLUE),
#             ("LOITER",    BLUE),
#             ("RTL",       WARN),
#             ("LAND",      WARN),
#             ("AUTO",      BLUE),
#             ("ALTHOLD",   BLUE),
#             ("POSHOLD",   BLUE),
#         ]:
#             b = btn(mode, color,
#                     lambda m=mode: self.set_mode(m),
#                     tk.DISABLED)
#             self.flight_btns.append(b)

#         # ── Arm / Disarm ──────────────────────────────────────
#         section("ARM / DISARM")

#         self.arm_btn       = btn("ARM",       GREEN, self.arm,       tk.DISABLED)
#         self.disarm_btn    = btn("DISARM",    RED,   self.disarm,    tk.DISABLED)
#         self.force_arm_btn = btn("FORCE ARM", WARN,  self.force_arm, tk.DISABLED)

#         # ── Commands ──────────────────────────────────────────
#         section("COMMANDS")

#         self.cmd_btns = []
#         for text, color, cmd in [
#             ("TAKEOFF 2m",       BLUE, self.takeoff),
#             ("TAKEOFF 5m",       BLUE, self.takeoff_5),
#             ("TAKEOFF 10m",      BLUE, self.takeoff_10),
#             ("RETURN TO LAUNCH", WARN, self.rtl),
#         ]:
#             b = btn(text, color, cmd, tk.DISABLED)
#             self.cmd_btns.append(b)

#         tk.Frame(parent, bg=PANEL, height=20).pack()

#     def build_output(self, parent):
#         header = tk.Frame(parent, bg=PANEL)
#         header.pack(fill=tk.X, padx=10, pady=(10, 5))

#         tk.Label(
#             header, text="OUTPUT LOG",
#             font=("Consolas", 12),
#             bg=PANEL, fg=DIM
#         ).pack(side=tk.LEFT)

#         tk.Button(
#             header, text="CLEAR",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.clear
#         ).pack(side=tk.RIGHT, padx=5)

#         tk.Button(
#             header, text="SAVE LOG",
#             font=("Consolas", 10),
#             bg="#2a2a3a", fg=TEXT,
#             relief=tk.FLAT, cursor="hand2",
#             command=self.save_log
#         ).pack(side=tk.RIGHT, padx=5)

#         self.output = scrolledtext.ScrolledText(
#             parent, font=("Consolas", 11),
#             bg="#0a0a14", fg=TEXT,
#             insertbackground=TEXT,
#             relief=tk.FLAT, wrap=tk.WORD,
#             state=tk.DISABLED)
#         self.output.pack(
#             fill=tk.BOTH, expand=True,
#             padx=10, pady=(0, 5))

#         self.output.tag_config("info",   foreground=ACCENT)
#         self.output.tag_config("warn",   foreground=WARN)
#         self.output.tag_config("error",  foreground="#dc3838")
#         self.output.tag_config("cmd",    foreground="#6496c8")
#         self.output.tag_config("nav",    foreground="#50c8c8")
#         self.output.tag_config("normal", foreground=TEXT)
#         self.output.tag_config("dim",    foreground=DIM)

#     # ═══════════════════════════════════════════════════════════
#     # LOGGING
#     # ═══════════════════════════════════════════════════════════
#     def log(self, message, tag="normal"):
#         def _log():
#             self.output.config(state=tk.NORMAL)
#             ts = time.strftime("%H:%M:%S")
#             self.output.insert(
#                 tk.END, f"[{ts}] {message}\n", tag)
#             self.output.see(tk.END)
#             self.output.config(state=tk.DISABLED)
#         self.root.after(0, _log)

#     def clear(self):
#         self.output.config(state=tk.NORMAL)
#         self.output.delete(1.0, tk.END)
#         self.output.config(state=tk.DISABLED)

#     def save_log(self):
#         content = self.output.get(1.0, tk.END)
#         fname = f"clarq_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
#         with open(fname, 'w') as f:
#             f.write(content)
#         self.log(f"Log saved to {fname}", "info")

#     def update_telem(self, key, value):
#         def _u():
#             if key in self.telem:
#                 self.telem[key].config(text=value)
#         self.root.after(0, _u)

#     def update_jetson_status(self, text):
#         if text in JETSON_STATUS_MAP:
#             status, color, phase = JETSON_STATUS_MAP[text]
#             def _u():
#                 self.jetson_status_lbl.config(
#                     text=f"Jetson: {status}", fg=color)
#                 self.jetson_phase_lbl.config(
#                     text=f"Phase: {phase}", fg=color)
#             self.root.after(0, _u)

#     def enable_mav_buttons(self):
#         for b in (self.flight_btns + self.cmd_btns):
#             b.config(state=tk.NORMAL)
#         self.arm_btn.config(state=tk.NORMAL)
#         self.disarm_btn.config(state=tk.NORMAL)
#         self.force_arm_btn.config(state=tk.NORMAL)

#     # ═══════════════════════════════════════════════════════════
#     # GIMBAL SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def connect_gimbal(self):
#         def _connect():
#             port = self.gimbal_port_var.get()
#             if not port:
#                 self.log("No gimbal COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to gimbal on {port}...", "info")
#                 self.gimbal_ser = serial.Serial(port, GIMBAL_BAUD, timeout=1)
#                 time.sleep(2)  # wait for controller to be ready

#                 # Turn motors on (CMD 77)
#                 self.log("Turning gimbal motors on...", "dim")
#                 self.gimbal_send_cmd(77)
#                 time.sleep(1)

#                 # Set to 0 degrees initially
#                 self.gimbal_set_pitch(0)

#                 self.gimbal_connected = True
#                 self.log(f"Gimbal connected on {port} ✓", "info")
#                 self.root.after(
#                     0, lambda: self.gimbal_label.config(
#                         text="● GIMBAL CONNECTED", fg=ACCENT))

#             except Exception as e:
#                 self.log(f"Gimbal connect failed: {e}", "error")
#                 if self.gimbal_ser:
#                     try:
#                         self.gimbal_ser.close()
#                     except:
#                         pass
#                     self.gimbal_ser = None

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_gimbal(self):
#         self.gimbal_connected = False
#         if self.gimbal_ser:
#             try:
#                 self.gimbal_ser.close()
#             except Exception:
#                 pass
#             self.gimbal_ser = None
#         self.log("Gimbal disconnected", "warn")
#         self.root.after(
#             0, lambda: self.gimbal_label.config(
#                 text="● GIMBAL DISCONNECTED", fg=WARN))

#     def set_gimbal_angle(self):
#         """Set gimbal angle from entry field."""
#         try:
#             angle = float(self.gimbal_angle_entry.get())
            
#             if angle not in VALID_GIMBAL_ANGLES:
#                 self.log(f"Invalid angle: {angle}° (not in valid range)", "error")
#                 return
            
#             self.gimbal_set_pitch(angle)
            
#         except ValueError:
#             self.log("Invalid angle input — enter a number", "error")

#     def quick_set_angle(self, angle):
#         """Quick set button handler."""
#         self.gimbal_angle_entry.delete(0, tk.END)
#         self.gimbal_angle_entry.insert(0, str(angle))
#         self.gimbal_set_pitch(angle)

#     # ═══════════════════════════════════════════════════════════
#     # NRF / ARDUINO SERIAL CONNECTION
#     # ═══════════════════════════════════════════════════════════
#     def refresh_ports(self):
#         """Scan for available COM ports and populate dropdown."""
#         ports = [p.device for p in serial.tools.list_ports.comports()]
#         self.port_combo['values'] = ports
        
#         # Only update gimbal combo if it exists (it's created after RF section)
#         if hasattr(self, 'gimbal_port_combo'):
#             self.gimbal_port_combo['values'] = ports
        
#         if ports:
#             if not self.port_var.get():
#                 self.port_combo.set(ports[0])
#             if hasattr(self, 'gimbal_port_combo') and not self.gimbal_port_var.get():
#                 # Try to set a different port for gimbal if available
#                 self.gimbal_port_combo.set(ports[1] if len(ports) > 1 else ports[0])
#         else:
#             self.port_combo.set("")
#             if hasattr(self, 'gimbal_port_combo'):
#                 self.gimbal_port_combo.set("")
        
#         self.log(f"Found ports: {ports if ports else 'none'}", "dim")

#     def connect_rf(self):
#         def _connect():
#             port = self.port_var.get()
#             if not port:
#                 self.log("No COM port selected", "error")
#                 return
#             try:
#                 self.log(f"Connecting to Arduino on {port}...", "info")
#                 self.ser = serial.Serial(port, SERIAL_BAUD, timeout=2)
#                 time.sleep(2)  # wait for Arduino reset after serial open

#                 # Wait for CLARQ_RF_READY handshake from Arduino
#                 self.log("Waiting for Arduino ready signal...", "dim")
#                 deadline = time.time() + 5
#                 ready = False
#                 while time.time() < deadline:
#                     if self.ser.in_waiting:
#                         line = self.ser.readline().decode(
#                             'utf-8', errors='ignore').strip()
#                         if line == "CLARQ_RF_READY":
#                             ready = True
#                             break
#                         self.log(f"Arduino: {line}", "dim")

#                 if ready:
#                     self.rf_connected = True
#                     self.log(
#                         f"RF bridge connected on {port} ✓", "info")
#                     self.root.after(
#                         0, lambda: self.rf_label.config(
#                             text="● RF CONNECTED", fg=ACCENT))
#                     # Start background reader for Arduino responses
#                     threading.Thread(
#                         target=self.rf_read_loop,
#                         daemon=True).start()
#                 else:
#                     self.log(
#                         "Arduino did not respond — check port/baud",
#                         "error")
#                     self.ser.close()
#                     self.ser = None

#             except Exception as e:
#                 self.log(f"RF connect failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_rf(self):
#         self.rf_connected = False
#         if self.ser:
#             try:
#                 self.ser.close()
#             except Exception:
#                 pass
#             self.ser = None
#         self.log("RF bridge disconnected", "warn")
#         self.root.after(
#             0, lambda: self.rf_label.config(
#                 text="● RF DISCONNECTED", fg=WARN))

#     def rf_read_loop(self):
#         """Background thread — reads responses from Arduino."""
#         while self.rf_connected and self.ser:
#             try:
#                 if self.ser.in_waiting:
#                     line = self.ser.readline().decode(
#                         'utf-8', errors='ignore').strip()
#                     if not line:
#                         continue

#                     if line.startswith("TX_OK:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX OK — cmd {cmd_id} delivered ✓",
#                             "info")

#                     elif line.startswith("TX_FAIL:"):
#                         cmd_id = int(line.split(":")[1])
#                         self.log(
#                             f"RF TX FAIL — cmd {cmd_id} not delivered",
#                             "error")

#                     elif line.startswith("RX:"):
#                         # Jetson sent an ACK back over nRF
#                         cmd_id = int(line.split(":")[1])
#                         label = ARDUINO_RX_MAP.get(cmd_id, f"id={cmd_id}")
#                         self.log(
#                             f"JETSON ACK: {label}", "info")
#                         self.update_jetson_status(label)

#                     elif line.startswith("ERR:"):
#                         self.log(f"Arduino error: {line}", "warn")

#                     else:
#                         self.log(f"Arduino: {line}", "dim")

#                 else:
#                     time.sleep(0.05)

#             except Exception as e:
#                 if self.rf_connected:
#                     self.log(f"RF read error: {e}", "error")
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # SEND COMMAND TO JETSON VIA NRF ARDUINO BRIDGE
#     # ═══════════════════════════════════════════════════════════
#     def send_jetson_cmd(self, cmd_id, text):
#         """
#         Send command to Jetson via Arduino nRF bridge.
#         Writes 'CMD:<id>\n' over USB serial to Arduino.
#         Arduino transmits over NRF24L01 to Jetson.
#         """
#         self.log(f"Sending to Jetson: {text} (id={cmd_id})", "cmd")

#         if not self.rf_connected or not self.ser:
#             self.log(
#                 "RF bridge not connected — cannot send command",
#                 "error")
#             return

#         try:
#             packet = f"CMD:{cmd_id}\n"
#             self.ser.write(packet.encode('utf-8'))
#             self.log(
#                 f"→ Arduino: {packet.strip()}", "dim")
#         except Exception as e:
#             self.log(f"Serial write failed: {e}", "error")

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK TELEMETRY (RECEIVE ONLY — no send)
#     # ═══════════════════════════════════════════════════════════
#     def connect_mav(self):
#         def _connect():
#             try:
#                 conn = self.conn_entry.get()
#                 self.log(f"Connecting MAVLink to {conn}...", "info")
#                 self.mav = mavutil.mavlink_connection(conn)
#                 self.log("Waiting for heartbeat...", "dim")
#                 self.mav.wait_heartbeat(timeout=10)
#                 self.mav_connected = True
#                 self.log(
#                     f"MAVLink connected — telemetry active ✓", "info")
#                 self.root.after(
#                     0, lambda: self.conn_label.config(
#                         text="● MAV CONNECTED", fg=ACCENT))
#                 self.root.after(0, self.enable_mav_buttons)
#                 self.start_telemetry()
#             except Exception as e:
#                 self.log(
#                     f"MAVLink connection failed: {e}", "error")

#         threading.Thread(target=_connect, daemon=True).start()

#     def disconnect_mav(self):
#         self.mav_connected = False
#         self.telem_on = False
#         self.log("MAVLink disconnected", "warn")
#         self.root.after(
#             0, lambda: self.conn_label.config(
#                 text="● MAV DISCONNECTED", fg=WARN))

#     def start_telemetry(self):
#         self.telem_on = True
#         threading.Thread(
#             target=self.telem_loop, daemon=True).start()

#     def telem_loop(self):
#         while self.telem_on and self.mav_connected:
#             try:
#                 msg = self.mav.recv_match(
#                     type=[
#                         'HEARTBEAT', 'SYS_STATUS',
#                         'GLOBAL_POSITION_INT',
#                         'VFR_HUD', 'STATUSTEXT'
#                     ],
#                     blocking=True, timeout=1)
#                 if msg is None:
#                     continue

#                 t = msg.get_type()

#                 if t == 'STATUSTEXT':
#                     text = msg.text.strip()
#                     if text.startswith('CLARQ'):
#                         self.log(f"JETSON: {text}", "info")
#                         self.update_jetson_status(text)
#                     else:
#                         self.log(f"FC: {text}", "dim")
#                     continue

#                 if t == 'HEARTBEAT':
#                     src = (msg.get_srcSystem()
#                            if hasattr(msg, 'get_srcSystem')
#                            else 1)
#                     if src != 1:
#                         continue
#                     mode  = mavutil.mode_string_v10(msg)
#                     armed = bool(
#                         msg.base_mode &
#                         mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
#                     self.update_telem('Mode',  mode)
#                     self.update_telem(
#                         'Armed', 'YES' if armed else 'NO')

#                 elif t == 'SYS_STATUS':
#                     v = msg.voltage_battery / 1000.0
#                     self.update_telem('Battery', f'{v:.1f}V')

#                 elif t == 'GLOBAL_POSITION_INT':
#                     lat = msg.lat / 1e7
#                     lon = msg.lon / 1e7
#                     alt = msg.relative_alt / 1000.0
#                     hdg = msg.hdg / 100.0
#                     self.update_telem('Lat',     f'{lat:.6f}')
#                     self.update_telem('Lon',     f'{lon:.6f}')
#                     self.update_telem('Alt',     f'{alt:.1f}m')
#                     self.update_telem('Heading', f'{hdg:.0f}°')

#                 elif t == 'VFR_HUD':
#                     self.update_telem(
#                         'Speed', f'{msg.groundspeed:.1f}m/s')

#             except Exception:
#                 break

#     # ═══════════════════════════════════════════════════════════
#     # MAVLINK FLIGHT COMMANDS (still sent via MAVLink)
#     # ═══════════════════════════════════════════════════════════
#     def set_mode(self, mode):
#         if not self.mav_connected or not self.mav:
#             self.log("MAVLink not connected!", "error")
#             return
#         mode_map = {
#             "STABILIZE": 0, "ACRO": 1,  "ALTHOLD": 2,
#             "AUTO":      3, "GUIDED": 4, "LOITER":  5,
#             "RTL":       6, "CIRCLE": 7, "LAND":    9,
#             "POSHOLD":  16,
#         }
#         if mode not in mode_map:
#             self.log(f"Unknown mode: {mode}", "error")
#             return
#         mode_num = mode_map[mode]
#         self.mav.mav.set_mode_send(
#             self.mav.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num)
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#             0,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_num, 0, 0, 0, 0, 0)
#         self.log(f"Mode {mode} ({mode_num}) sent", "cmd")

#         def _watch_ack():
#             try:
#                 ack = self.mav.recv_match(
#                     type='COMMAND_ACK',
#                     blocking=True, timeout=3)
#                 if ack and ack.result == 0:
#                     self.log(f"Mode {mode} confirmed ✓", "info")
#                 elif ack:
#                     self.log(
#                         f"Mode {mode} rejected (result={ack.result})",
#                         "warn")
#                 else:
#                     self.log(f"Mode {mode} — no ACK", "dim")
#             except Exception:
#                 pass
#         threading.Thread(
#             target=_watch_ack, daemon=True).start()

#     def arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Arming...", "warn")
#         self.mav.arducopter_arm()
#         self.log("Arm command sent", "info")

#     def disarm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Disarming...", "warn")
#         self.mav.arducopter_disarm()
#         self.log("Disarm command sent", "info")

#     def force_arm(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log("Force arming...", "warn")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0, 1, 21196, 0, 0, 0, 0, 0)
#         self.log("Force arm sent!", "info")

#     def takeoff(self):    self._takeoff(2.0)
#     def takeoff_5(self):  self._takeoff(5.0)
#     def takeoff_10(self): self._takeoff(10.0)

#     def _takeoff(self, alt):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.log(f"Sending takeoff to {alt}m...", "cmd")
#         self.mav.mav.command_long_send(
#             self.mav.target_system,
#             self.mav.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#             0, 0, 0, 0, 0, 0, 0, alt)
#         self.log(f"Takeoff {alt}m sent", "info")

#     def rtl(self):
#         if not self.mav_connected:
#             self.log("MAVLink not connected!", "error")
#             return
#         self.set_mode("RTL")

#     # ═══════════════════════════════════════════════════════════
#     # CAMERA NAVIGATION
#     # ═══════════════════════════════════════════════════════════
#     def set_home_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 1 — SET HOME POINT", "nav")
#         self.log("  Saves ZED pose as origin_T0.json", "nav")
#         self.log("  Gimbal tilts to 30° after save", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SET_HOME, CMD_SET_HOME)

#     def start_scan_mission(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 2 — START SCAN MISSION", "nav")
#         self.log("  Jetson runs save_position.py", "nav")
#         self.log("  Fly the mission area now", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_START_SCAN, CMD_START_SCAN)

#     def save_end_point(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 3 — SAVE END POINT", "nav")
#         self.log("  Saves scan_end_pos.json on Jetson", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_SAVE_END, CMD_SAVE_END)

#     def go_home_and_land(self):
#         self.log("─" * 44, "nav")
#         self.log("  STEP 4 — GO HOME + LAND", "nav")
#         self.log("  Jetson runs return_land.py", "nav")
#         self.log("  AprilTag activated automatically", "nav")
#         self.log("  Phase 1 → ZED navigate to T0", "nav")
#         self.log("  Phase 2 → AprilTag align at 90°", "nav")
#         self.log("  Phase 3 → Land on rover", "nav")
#         self.log("─" * 44, "nav")
#         self.send_jetson_cmd(CMD_ID_GO_HOME, CMD_GO_HOME)
#         if self.mav_connected:
#             self.set_mode("GUIDED")

#     # ── Jetson system commands ────────────────────────────────
#     def ping_jetson(self):
#         self.send_jetson_cmd(CMD_ID_PING, CMD_PING)

#     def launch_jetson(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH, CMD_LAUNCH)

#     def launch_jetson_sim(self):
#         self.send_jetson_cmd(CMD_ID_LAUNCH_SIM, CMD_LAUNCH_SIM)

#     def kill_jetson(self):
#         self.send_jetson_cmd(CMD_ID_STOP_ALL, "CLARQ_STOP_ALL")


# # ── Run ───────────────────────────────────────────────────────
# def main():
#     root = tk.Tk()
#     app  = DroneGUI(root)
#     root.mainloop()

# if __name__ == '__main__':
#     main()

# CLARQ_GUI.py
# Runs on Windows alongside Mission Planner
#
# Command routing (nRF path):
#   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
#
# Telemetry routing (MAVLink receive only):
#   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
#
# Mission Planner setup:
#   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
#
# Gimbal control:
#   GUI → USB Serial (COM port) → SimpleBGC Controller → Gimbal Motor
#
# pip install pymavlink pyserial
# CLARQ_GUI.py
# Runs on Windows alongside Mission Planner
#
# Command routing (nRF path):
#   GUI → USB Serial (COM port) → Arduino Nano → NRF24L01 → Jetson
#
# Telemetry routing (MAVLink receive only):
#   Mission Planner → UDP 14550 → GUI (read telemetry + STATUSTEXT)
#
# Mission Planner setup:
#   MAVLink Mirror → Outbound → 127.0.0.1:14550 → Write=checked
#
# Gimbal control:
#   GUI → USB Serial (COM port) → SimpleBGC Controller → Gimbal Motor
#
# pip install pymavlink pyserial

import tkinter as tk
from tkinter import scrolledtext, ttk
from pymavlink import mavutil
import threading
import time
import serial
import serial.tools.list_ports
import struct

# ── Config ────────────────────────────────────────────────────
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"   # receive telemetry from MP
SERIAL_BAUD        = 115200                   # must match Arduino sketch
GIMBAL_BAUD        = 115200                   # gimbal serial baud rate

# Valid gimbal angles (from your script)
VALID_GIMBAL_ANGLES = [
    0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30, 35, -35, 
    40, -40, 45, -45, 50, -50, 55, -55, 60, -60, 65, -65, 70, -70, 
    75, -75, 80, -80, 85, -85, 90, -90, 95, -95, 100, -100, 105, -105, 
    110, -110, 115, -115, 120, -120, 125, -125, 130, -130, 135, -135, 
    140, -140, 145, -145, 150, -150, 155, -155, 160, -160, 165, -165, 
    170, -170, 175, -175, 180, -180, 360, -360
]

# ── Command IDs — must match clarq_rf_listener.py ─────────────
CMD_ID_PING       = 1
CMD_ID_LAND       = 2
CMD_ID_START_TAG  = 3
CMD_ID_STOP_TAG       = 4
CMD_ID_STOP_ALL       = 5
CMD_ID_LAUNCH         = 6
CMD_ID_LAUNCH_SIM     = 7
CMD_ID_SET_HOME       = 8
CMD_ID_START_SCAN     = 9   # Position tracking only (legacy)
CMD_ID_SAVE_END       = 10
CMD_ID_GO_HOME        = 11
CMD_ID_START_3D_FUSION = 12  # Start ZED Fusion + position tracking
CMD_ID_STOP_3D_FUSION  = 13  # Stop ZED Fusion recording
CMD_ID_START_PHOTO     = 14  # Start photo capture mode
CMD_ID_START_VIDEO     = 15  # Start video recording mode
CMD_ID_STOP_CAPTURE    = 16  # Stop photo/video capture
CMD_ID_SET_GIMBAL      = 17  # Set gimbal pitch angle

CMD_START_LANDING  = "CLARQ_LAND"
CMD_START_APRILTAG = "CLARQ_START_TAG"
CMD_STOP_APRILTAG  = "CLARQ_STOP_TAG"
CMD_PING           = "CLARQ_PING"
CMD_LAUNCH         = "CLARQ_LAUNCH"
CMD_LAUNCH_SIM     = "CLARQ_LAUNCH_SIM"
CMD_SET_HOME         = "CLARQ_SET_HOME"
CMD_START_SCAN       = "CLARQ_START_SCAN"
CMD_START_3D_FUSION  = "CLARQ_START_3D_FUSION"
CMD_STOP_3D_FUSION   = "CLARQ_STOP_3D_FUSION"
CMD_START_PHOTO      = "CLARQ_START_PHOTO"
CMD_START_VIDEO      = "CLARQ_START_VIDEO"
CMD_STOP_CAPTURE     = "CLARQ_STOP_CAPTURE"
CMD_SET_GIMBAL       = "CLARQ_SET_GIMBAL"
CMD_SAVE_END         = "CLARQ_SAVE_END"
CMD_GO_HOME          = "CLARQ_GO_HOME"

# ── Jetson response map (STATUSTEXT → status label) ───────────
JETSON_STATUS_MAP = {
    "CLARQ_PONG":              ("ONLINE ✓",          "#1ec882", "--"),
    "CLARQ_HOME_SET":          ("HOME SET ✓",        "#1ec882", "origin_T0.json saved"),
    "CLARQ_SCAN_STARTED":      ("SCANNING ✓",        "#1ec882", "Position tracking only"),
    "CLARQ_3D_FUSION_STARTED": ("3D FUSION ✓",       "#1ec882", "ZED Fusion + tracking"),
    "CLARQ_3D_FUSION_STOPPED": ("FUSION STOPPED",    "#646478", "Recording saved"),
    "CLARQ_PHOTO_STARTED":     ("PHOTO MODE ✓",      "#1ec882", "Capturing images"),
    "CLARQ_VIDEO_STARTED":     ("VIDEO MODE ✓",      "#1ec882", "Recording video"),
    "CLARQ_CAPTURE_STOPPED":   ("CAPTURE STOPPED",   "#646478", "Files saved"),
    "CLARQ_GIMBAL_SET":        ("GIMBAL MOVED",      "#1ec882", "Angle updated"),
    "CLARQ_END_SAVED":         ("END SAVED ✓",       "#1ec882", "scan_end_pos.json saved"),
    "CLARQ_GOING_HOME":        ("RETURNING HOME...", "#dc6438", "Phase 1 — navigating"),
    "CLARQ_LAUNCHED":          ("LAUNCHED ✓",        "#1ec882", "All systems starting"),
    "CLARQ_LAUNCHED_SIM":      ("LAUNCHED SIM ✓",    "#1ec882", "Sim stack starting"),
    "CLARQ_ALL_STOPPED":       ("STOPPED",           "#646478", "All processes stopped"),
    "CLARQ_TAG_RUNNING":       ("TAG RUNNING ✓",     "#1ec882", "AprilTag active"),
    "CLARQ_TAG_STOPPED":       ("TAG STOPPED",       "#646478", "AprilTag stopped"),
}

# ── RX response codes from Arduino ────────────────────────────
# Arduino sends back: TX_OK:<id>, TX_FAIL:<id>, RX:<id>
ARDUINO_RX_MAP = {
    CMD_ID_PING:            "CLARQ_PONG",
    CMD_ID_LAND:            "CLARQ_LAND_ACK",
    CMD_ID_START_TAG:       "CLARQ_TAG_RUNNING",
    CMD_ID_STOP_TAG:        "CLARQ_TAG_STOPPED",
    CMD_ID_STOP_ALL:        "CLARQ_ALL_STOPPED",
    CMD_ID_LAUNCH:          "CLARQ_LAUNCHED",
    CMD_ID_LAUNCH_SIM:      "CLARQ_LAUNCHED_SIM",
    CMD_ID_SET_HOME:        "CLARQ_HOME_SET",
    CMD_ID_START_SCAN:      "CLARQ_SCAN_STARTED",
    CMD_ID_START_3D_FUSION: "CLARQ_3D_FUSION_STARTED",
    CMD_ID_STOP_3D_FUSION:  "CLARQ_3D_FUSION_STOPPED",
    CMD_ID_START_PHOTO:     "CLARQ_PHOTO_STARTED",
    CMD_ID_START_VIDEO:     "CLARQ_VIDEO_STARTED",
    CMD_ID_STOP_CAPTURE:    "CLARQ_CAPTURE_STOPPED",
    CMD_ID_SET_GIMBAL:      "CLARQ_GIMBAL_SET",
    CMD_ID_SAVE_END:        "CLARQ_END_SAVED",
    CMD_ID_GO_HOME:         "CLARQ_GOING_HOME",
}

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
        self.root.title("CLARQ Drone Control")
        self.root.geometry("1000x800")
        self.root.configure(bg=BG)

        # MAVLink (telemetry receive only)
        self.mav       = None
        self.mav_connected = False
        self.telem_on  = False

        # nRF Arduino serial bridge
        self.ser       = None
        self.rf_connected = False

        # Gimbal control (via RF to Jetson)
        self.current_gimbal_angle = 0

        # Capture quality settings
        self.capture_quality = "HIGH"  # LOW, MEDIUM, HIGH, ULTRA

        self.build_ui()

    # ═══════════════════════════════════════════════════════════
    # GIMBAL CONTROL FUNCTIONS
    # ═══════════════════════════════════════════════════════════
    def gimbal_send_cmd(self, cmd, data=bytes([])):
        """Send command to SimpleBGC gimbal controller."""
        size = len(data)
        header_checksum = (cmd + size) & 0xFF
        body_checksum = 0
        for b in data:
            body_checksum = (body_checksum + b) & 0xFF
        packet = bytes([0x3E, cmd, size, header_checksum]) + data + bytes([body_checksum])
        self.gimbal_ser.write(packet)
        time.sleep(0.1)
        response = self.gimbal_ser.read(100)
        return response

    def gimbal_set_pitch(self, angle):
        """Send gimbal angle command via RF to Jetson."""
        if angle not in VALID_GIMBAL_ANGLES:
            self.log(f"Invalid angle: {angle}° (not in valid range)", "error")
            return
        
        self.log(f"Sending gimbal command: {angle}°", "cmd")
        
        # Send via RF to Jetson (Jetson will control gimbal)
        # Encode angle in command: "CLARQ_SET_GIMBAL:angle"
        self.send_jetson_cmd(CMD_ID_SET_GIMBAL, f"{CMD_SET_GIMBAL}:{int(angle)}")
        
        # Update current angle display
        self.root.after(0, lambda: self.gimbal_current_lbl.config(
            text=f"Current: {angle}°", fg=ACCENT))

    # ═══════════════════════════════════════════════════════════
    # UI BUILD
    # ═══════════════════════════════════════════════════════════
    def build_ui(self):
        title = tk.Frame(self.root, bg=PANEL, pady=12)
        title.pack(fill=tk.X)

        tk.Label(
            title,
            text="CLARQ DRONE NAVIGATION CONTROL",
            font=("Consolas", 16, "bold"),
            bg=PANEL, fg=ACCENT
        ).pack(side=tk.LEFT, padx=20)

        # Status indicators (right side of title bar)
        status_frame = tk.Frame(title, bg=PANEL)
        status_frame.pack(side=tk.RIGHT, padx=20)

        self.gimbal_label = tk.Label(
            status_frame, text="● GIMBAL DISCONNECTED",
            font=("Consolas", 11),
            bg=PANEL, fg=WARN)
        self.gimbal_label.pack(side=tk.RIGHT, padx=(10, 0))

        self.rf_label = tk.Label(
            status_frame, text="● RF DISCONNECTED",
            font=("Consolas", 11),
            bg=PANEL, fg=WARN)
        self.rf_label.pack(side=tk.RIGHT, padx=(10, 0))

        self.conn_label = tk.Label(
            status_frame, text="● MAV DISCONNECTED",
            font=("Consolas", 11),
            bg=PANEL, fg=WARN)
        self.conn_label.pack(side=tk.RIGHT)

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

        # ── nRF / Arduino serial connection ───────────────────
        section("NRF RADIO (COMMAND LINK)")

        port_frame = tk.Frame(parent, bg=PANEL)
        port_frame.pack(fill=tk.X, padx=15, pady=4)

        tk.Label(
            port_frame, text="Port:",
            font=("Consolas", 11),
            bg=PANEL, fg=TEXT
        ).pack(side=tk.LEFT)

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            port_frame,
            textvariable=self.port_var,
            font=("Consolas", 11),
            width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)

        tk.Button(
            port_frame, text="↻",
            font=("Consolas", 12),
            bg="#2a2a3a", fg=ACCENT,
            relief=tk.FLAT, cursor="hand2",
            command=self.refresh_ports
        ).pack(side=tk.LEFT)

        self.refresh_ports()

        btn("CONNECT RF",    BLUE, self.connect_rf)
        btn("DISCONNECT RF", RED,  self.disconnect_rf)

        # ── Gimbal control ────────────────────────────────────
        section("GIMBAL CONTROL")
        
        tk.Label(
            parent, text="Remote control via RF",
            font=("Consolas", 9, "italic"),
            bg=PANEL, fg=DIM
        ).pack(anchor=tk.W, padx=15, pady=(0, 6))

        # Gimbal angle input
        angle_frame = tk.Frame(parent, bg=PANEL)
        angle_frame.pack(fill=tk.X, padx=15, pady=4)

        tk.Label(
            angle_frame, text="Angle:",
            font=("Consolas", 11),
            bg=PANEL, fg=TEXT
        ).pack(side=tk.LEFT)

        self.gimbal_angle_entry = tk.Entry(
            angle_frame, font=("Consolas", 11),
            bg="#2a2a3a", fg=TEXT,
            insertbackground=TEXT,
            relief=tk.FLAT, width=10)
        self.gimbal_angle_entry.insert(0, "0")
        self.gimbal_angle_entry.pack(side=tk.LEFT, padx=5)
        
        # Bind Enter key to set angle
        self.gimbal_angle_entry.bind('<Return>', lambda e: self.set_gimbal_angle())

        tk.Button(
            angle_frame, text="SET",
            font=("Consolas", 11, "bold"),
            bg=BLUE, fg=TEXT,
            relief=tk.FLAT, cursor="hand2",
            command=self.set_gimbal_angle
        ).pack(side=tk.LEFT)

        # Current angle display
        self.gimbal_current_lbl = tk.Label(
            parent, text="Current: 0°",
            font=("Consolas", 11, "bold"),
            bg=PANEL, fg=DIM)
        self.gimbal_current_lbl.pack(
            anchor=tk.W, padx=15, pady=(2, 6))

        # Quick angle buttons
        quick_angles_frame = tk.Frame(parent, bg=PANEL)
        quick_angles_frame.pack(fill=tk.X, padx=15, pady=4)

        for angle in [0, 30, 45, 60, 90]:
            tk.Button(
                quick_angles_frame, text=f"{angle}°",
                font=("Consolas", 10),
                bg="#2a2a3a", fg=TEXT,
                relief=tk.FLAT, cursor="hand2",
                command=lambda a=angle: self.quick_set_angle(a)
            ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        quick_angles_frame2 = tk.Frame(parent, bg=PANEL)
        quick_angles_frame2.pack(fill=tk.X, padx=15, pady=(0, 4))

        for angle in [-30, -45, -60, -90]:
            tk.Button(
                quick_angles_frame2, text=f"{angle}°",
                font=("Consolas", 10),
                bg="#2a2a3a", fg=TEXT,
                relief=tk.FLAT, cursor="hand2",
                command=lambda a=angle: self.quick_set_angle(a)
            ).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        # ── MAVLink telemetry connection ───────────────────────
        section("MAVLINK (TELEMETRY ONLY)")

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
            relief=tk.FLAT, width=22)
        self.conn_entry.insert(0, MAVLINK_CONNECTION)
        self.conn_entry.pack(side=tk.LEFT, padx=5)

        btn("CONNECT MAV",    BLUE, self.connect_mav)
        btn("DISCONNECT MAV", RED,  self.disconnect_mav)

        # ── Camera navigation ─────────────────────────────────
        section("CAMERA NAVIGATION")

        self.jetson_status_lbl = tk.Label(
            parent, text="Jetson: IDLE",
            font=("Consolas", 11, "bold"),
            bg=PANEL, fg=DIM)
        self.jetson_status_lbl.pack(
            anchor=tk.W, padx=15, pady=(2, 0))

        self.jetson_phase_lbl = tk.Label(
            parent, text="Phase: --",
            font=("Consolas", 10),
            bg=PANEL, fg=DIM)
        self.jetson_phase_lbl.pack(
            anchor=tk.W, padx=15, pady=(0, 6))

        self.nav_btns = []
        b = btn("1. SET HOME POINT",     GREEN,  self.set_home_point)
        self.nav_btns.append(b)
        
        # Step 2: 3D Fusion Scan with START/STOP control
        tk.Label(
            parent, text="2. 3D FUSION SCAN",
            font=("Consolas", 11, "bold"),
            bg=PANEL, fg=TEXT
        ).pack(anchor=tk.W, padx=15, pady=(8, 4))
        
        tk.Label(
            parent, text="ZED Fusion + Position Tracking",
            font=("Consolas", 9),
            bg=PANEL, fg=DIM
        ).pack(anchor=tk.W, padx=15, pady=(0, 4))
        
        # 3D Fusion control buttons
        fusion_frame = tk.Frame(parent, bg=PANEL)
        fusion_frame.pack(fill=tk.X, padx=15, pady=(0, 4))
        
        tk.Button(
            fusion_frame, text="START 3D FUSION",
            font=("Consolas", 11, "bold"),
            bg="#1a6b5a", fg=TEXT,
            activebackground="#1a6b5a",
            relief=tk.FLAT, cursor="hand2",
            pady=10, command=self.start_3d_fusion
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        
        tk.Button(
            fusion_frame, text="STOP FUSION",
            font=("Consolas", 11, "bold"),
            bg="#6b4a1a", fg=TEXT,
            activebackground="#6b4a1a",
            relief=tk.FLAT, cursor="hand2",
            pady=10, command=self.stop_3d_fusion
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))
        
        # Photo/Video Capture Section
        tk.Label(
            parent, text="PHOTO / VIDEO CAPTURE",
            font=("Consolas", 11, "bold"),
            bg=PANEL, fg=TEXT
        ).pack(anchor=tk.W, padx=15, pady=(12, 4))
        
        # Quality selector
        quality_frame = tk.Frame(parent, bg=PANEL)
        quality_frame.pack(fill=tk.X, padx=15, pady=(0, 4))
        
        tk.Label(
            quality_frame, text="Quality:",
            font=("Consolas", 10),
            bg=PANEL, fg=TEXT
        ).pack(side=tk.LEFT)
        
        self.quality_var = tk.StringVar(value="HIGH")
        quality_dropdown = ttk.Combobox(
            quality_frame,
            textvariable=self.quality_var,
            font=("Consolas", 10),
            width=10, state="readonly",
            values=["LOW", "MEDIUM", "HIGH", "ULTRA"])
        quality_dropdown.pack(side=tk.LEFT, padx=5)
        
        # Photo/Video control buttons
        capture_frame = tk.Frame(parent, bg=PANEL)
        capture_frame.pack(fill=tk.X, padx=15, pady=(0, 4))
        
        tk.Button(
            capture_frame, text="PHOTO MODE",
            font=("Consolas", 10, "bold"),
            bg="#4a1a6b", fg=TEXT,
            activebackground="#4a1a6b",
            relief=tk.FLAT, cursor="hand2",
            pady=8, command=self.start_photo_capture
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        
        tk.Button(
            capture_frame, text="VIDEO MODE",
            font=("Consolas", 10, "bold"),
            bg="#1a4a6b", fg=TEXT,
            activebackground="#1a4a6b",
            relief=tk.FLAT, cursor="hand2",
            pady=8, command=self.start_video_capture
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))
        
        tk.Button(
            parent, text="STOP CAPTURE",
            font=("Consolas", 10, "bold"),
            bg="#6b3a1a", fg=TEXT,
            activebackground="#6b3a1a",
            relief=tk.FLAT, cursor="hand2",
            pady=8, command=self.stop_capture
        ).pack(fill=tk.X, padx=15, pady=(0, 4))
        
        b = btn("3. SAVE END POINT",     PURPLE, self.save_end_point)
        self.nav_btns.append(b)
        b = btn("4. GO HOME + LAND",     WARN,   self.go_home_and_land)
        self.nav_btns.append(b)

        # ── Jetson ────────────────────────────────────────────
        section("JETSON")
        btn("LAUNCH JETSON",     GREEN, self.launch_jetson)
        btn("LAUNCH JETSON SIM", BLUE,  self.launch_jetson_sim)
        btn("KILL JETSON",       RED,   self.kill_jetson)
        btn("PING JETSON",       BLUE,  self.ping_jetson)

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

        self.arm_btn       = btn("ARM",       GREEN, self.arm,       tk.DISABLED)
        self.disarm_btn    = btn("DISARM",    RED,   self.disarm,    tk.DISABLED)
        self.force_arm_btn = btn("FORCE ARM", WARN,  self.force_arm, tk.DISABLED)

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
        self.output.tag_config("nav",    foreground="#50c8c8")
        self.output.tag_config("normal", foreground=TEXT)
        self.output.tag_config("dim",    foreground=DIM)

    # ═══════════════════════════════════════════════════════════
    # LOGGING
    # ═══════════════════════════════════════════════════════════
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

    def update_jetson_status(self, text):
        if text in JETSON_STATUS_MAP:
            status, color, phase = JETSON_STATUS_MAP[text]
            def _u():
                self.jetson_status_lbl.config(
                    text=f"Jetson: {status}", fg=color)
                self.jetson_phase_lbl.config(
                    text=f"Phase: {phase}", fg=color)
            self.root.after(0, _u)

    def enable_mav_buttons(self):
        for b in (self.flight_btns + self.cmd_btns):
            b.config(state=tk.NORMAL)
        self.arm_btn.config(state=tk.NORMAL)
        self.disarm_btn.config(state=tk.NORMAL)
        self.force_arm_btn.config(state=tk.NORMAL)

    # ═══════════════════════════════════════════════════════════
    # GIMBAL SERIAL CONNECTION
    # ═══════════════════════════════════════════════════════════
    def connect_gimbal(self):
        def _connect():
            port = self.gimbal_port_var.get()
            if not port:
                self.log("No gimbal COM port selected", "error")
                return
            try:
                self.log(f"Connecting to gimbal on {port}...", "info")
                self.gimbal_ser = serial.Serial(port, GIMBAL_BAUD, timeout=1)
                time.sleep(2)  # wait for controller to be ready

                # Turn motors on (CMD 77)
                self.log("Turning gimbal motors on...", "dim")
                self.gimbal_send_cmd(77)
                time.sleep(1)

                # Set to 0 degrees initially
                self.gimbal_set_pitch(0)

                self.gimbal_connected = True
                self.log(f"Gimbal connected on {port} ✓", "info")
                self.root.after(
                    0, lambda: self.gimbal_label.config(
                        text="● GIMBAL CONNECTED", fg=ACCENT))

            except Exception as e:
                self.log(f"Gimbal connect failed: {e}", "error")
                if self.gimbal_ser:
                    try:
                        self.gimbal_ser.close()
                    except:
                        pass
                    self.gimbal_ser = None

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect_gimbal(self):
        self.gimbal_connected = False
        if self.gimbal_ser:
            try:
                self.gimbal_ser.close()
            except Exception:
                pass
            self.gimbal_ser = None
        self.log("Gimbal disconnected", "warn")
        self.root.after(
            0, lambda: self.gimbal_label.config(
                text="● GIMBAL DISCONNECTED", fg=WARN))

    def set_gimbal_angle(self):
        """Set gimbal angle from entry field."""
        try:
            angle = float(self.gimbal_angle_entry.get())
            
            if angle not in VALID_GIMBAL_ANGLES:
                self.log(f"Invalid angle: {angle}° (not in valid range)", "error")
                return
            
            self.gimbal_set_pitch(angle)
            
        except ValueError:
            self.log("Invalid angle input — enter a number", "error")

    def quick_set_angle(self, angle):
        """Quick set button handler."""
        self.gimbal_angle_entry.delete(0, tk.END)
        self.gimbal_angle_entry.insert(0, str(angle))
        self.gimbal_set_pitch(angle)

    # ═══════════════════════════════════════════════════════════
    # NRF / ARDUINO SERIAL CONNECTION
    # ═══════════════════════════════════════════════════════════
    def refresh_ports(self):
        """Scan for available COM ports and populate dropdown."""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        
        # Only update gimbal combo if it exists (it's created after RF section)
        if hasattr(self, 'gimbal_port_combo'):
            self.gimbal_port_combo['values'] = ports
        
        if ports:
            if not self.port_var.get():
                self.port_combo.set(ports[0])
            if hasattr(self, 'gimbal_port_combo') and not self.gimbal_port_var.get():
                # Try to set a different port for gimbal if available
                self.gimbal_port_combo.set(ports[1] if len(ports) > 1 else ports[0])
        else:
            self.port_combo.set("")
            if hasattr(self, 'gimbal_port_combo'):
                self.gimbal_port_combo.set("")
        
        self.log(f"Found ports: {ports if ports else 'none'}", "dim")

    def connect_rf(self):
        def _connect():
            port = self.port_var.get()
            if not port:
                self.log("No COM port selected", "error")
                return
            try:
                self.log(f"Connecting to Arduino on {port}...", "info")
                self.ser = serial.Serial(port, SERIAL_BAUD, timeout=2)
                time.sleep(2)  # wait for Arduino reset after serial open

                # Wait for CLARQ_RF_READY handshake from Arduino
                self.log("Waiting for Arduino ready signal...", "dim")
                deadline = time.time() + 5
                ready = False
                while time.time() < deadline:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode(
                            'utf-8', errors='ignore').strip()
                        if line == "CLARQ_RF_READY":
                            ready = True
                            break
                        self.log(f"Arduino: {line}", "dim")

                if ready:
                    self.rf_connected = True
                    self.log(
                        f"RF bridge connected on {port} ✓", "info")
                    self.root.after(
                        0, lambda: self.rf_label.config(
                            text="● RF CONNECTED", fg=ACCENT))
                    # Start background reader for Arduino responses
                    threading.Thread(
                        target=self.rf_read_loop,
                        daemon=True).start()
                else:
                    self.log(
                        "Arduino did not respond — check port/baud",
                        "error")
                    self.ser.close()
                    self.ser = None

            except Exception as e:
                self.log(f"RF connect failed: {e}", "error")

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect_rf(self):
        self.rf_connected = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.log("RF bridge disconnected", "warn")
        self.root.after(
            0, lambda: self.rf_label.config(
                text="● RF DISCONNECTED", fg=WARN))

    def rf_read_loop(self):
        """Background thread — reads responses from Arduino."""
        while self.rf_connected and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(
                        'utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    if line.startswith("TX_OK:"):
                        cmd_id = int(line.split(":")[1])
                        self.log(
                            f"RF TX OK — cmd {cmd_id} delivered ✓",
                            "info")

                    elif line.startswith("TX_FAIL:"):
                        cmd_id = int(line.split(":")[1])
                        self.log(
                            f"RF TX FAIL — cmd {cmd_id} not delivered",
                            "error")

                    elif line.startswith("RX:"):
                        # Jetson sent an ACK back over nRF
                        cmd_id = int(line.split(":")[1])
                        label = ARDUINO_RX_MAP.get(cmd_id, f"id={cmd_id}")
                        self.log(
                            f"JETSON ACK: {label}", "info")
                        self.update_jetson_status(label)

                    elif line.startswith("ERR:"):
                        self.log(f"Arduino error: {line}", "warn")

                    else:
                        self.log(f"Arduino: {line}", "dim")

                else:
                    time.sleep(0.05)

            except Exception as e:
                if self.rf_connected:
                    self.log(f"RF read error: {e}", "error")
                break

    # ═══════════════════════════════════════════════════════════
    # SEND COMMAND TO JETSON VIA NRF ARDUINO BRIDGE
    # ═══════════════════════════════════════════════════════════
    def send_jetson_cmd(self, cmd_id, text):
        """
        Send command to Jetson via Arduino nRF bridge.
        Writes 'CMD:<id>\n' over USB serial to Arduino.
        Arduino transmits over NRF24L01 to Jetson.
        """
        self.log(f"Sending to Jetson: {text} (id={cmd_id})", "cmd")

        if not self.rf_connected or not self.ser:
            self.log(
                "RF bridge not connected — cannot send command",
                "error")
            return

        try:
            packet = f"CMD:{cmd_id}\n"
            self.ser.write(packet.encode('utf-8'))
            self.log(
                f"→ Arduino: {packet.strip()}", "dim")
        except Exception as e:
            self.log(f"Serial write failed: {e}", "error")

    # ═══════════════════════════════════════════════════════════
    # MAVLINK TELEMETRY (RECEIVE ONLY — no send)
    # ═══════════════════════════════════════════════════════════
    def connect_mav(self):
        def _connect():
            try:
                conn = self.conn_entry.get()
                self.log(f"Connecting MAVLink to {conn}...", "info")
                self.mav = mavutil.mavlink_connection(conn)
                self.log("Waiting for heartbeat...", "dim")
                self.mav.wait_heartbeat(timeout=10)
                self.mav_connected = True
                self.log(
                    f"MAVLink connected — telemetry active ✓", "info")
                self.root.after(
                    0, lambda: self.conn_label.config(
                        text="● MAV CONNECTED", fg=ACCENT))
                self.root.after(0, self.enable_mav_buttons)
                self.start_telemetry()
            except Exception as e:
                self.log(
                    f"MAVLink connection failed: {e}", "error")

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect_mav(self):
        self.mav_connected = False
        self.telem_on = False
        self.log("MAVLink disconnected", "warn")
        self.root.after(
            0, lambda: self.conn_label.config(
                text="● MAV DISCONNECTED", fg=WARN))

    def start_telemetry(self):
        self.telem_on = True
        threading.Thread(
            target=self.telem_loop, daemon=True).start()

    def telem_loop(self):
        while self.telem_on and self.mav_connected:
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
                        self.log(f"JETSON: {text}", "info")
                        self.update_jetson_status(text)
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

    # ═══════════════════════════════════════════════════════════
    # MAVLINK FLIGHT COMMANDS (still sent via MAVLink)
    # ═══════════════════════════════════════════════════════════
    def set_mode(self, mode):
        if not self.mav_connected or not self.mav:
            self.log("MAVLink not connected!", "error")
            return
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
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num)
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num, 0, 0, 0, 0, 0)
        self.log(f"Mode {mode} ({mode_num}) sent", "cmd")

        def _watch_ack():
            try:
                ack = self.mav.recv_match(
                    type='COMMAND_ACK',
                    blocking=True, timeout=3)
                if ack and ack.result == 0:
                    self.log(f"Mode {mode} confirmed ✓", "info")
                elif ack:
                    self.log(
                        f"Mode {mode} rejected (result={ack.result})",
                        "warn")
                else:
                    self.log(f"Mode {mode} — no ACK", "dim")
            except Exception:
                pass
        threading.Thread(
            target=_watch_ack, daemon=True).start()

    def arm(self):
        if not self.mav_connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log("Arming...", "warn")
        self.mav.arducopter_arm()
        self.log("Arm command sent", "info")

    def disarm(self):
        if not self.mav_connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log("Disarming...", "warn")
        self.mav.arducopter_disarm()
        self.log("Disarm command sent", "info")

    def force_arm(self):
        if not self.mav_connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log("Force arming...", "warn")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0)
        self.log("Force arm sent!", "info")

    def takeoff(self):    self._takeoff(2.0)
    def takeoff_5(self):  self._takeoff(5.0)
    def takeoff_10(self): self._takeoff(10.0)

    def _takeoff(self, alt):
        if not self.mav_connected:
            self.log("MAVLink not connected!", "error")
            return
        self.log(f"Sending takeoff to {alt}m...", "cmd")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt)
        self.log(f"Takeoff {alt}m sent", "info")

    def rtl(self):
        if not self.mav_connected:
            self.log("MAVLink not connected!", "error")
            return
        self.set_mode("RTL")

    # ═══════════════════════════════════════════════════════════
    # CAMERA NAVIGATION
    # ═══════════════════════════════════════════════════════════
    def set_home_point(self):
        self.log("─" * 44, "nav")
        self.log("  STEP 1 — SET HOME POINT", "nav")
        self.log("  Saves ZED pose as origin_T0.json", "nav")
        self.log("  Gimbal tilts to 30° after save", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_SET_HOME, CMD_SET_HOME)

    def start_3d_fusion(self):
        """Start ZED Fusion 3D reconstruction + position tracking."""
        self.log("─" * 44, "nav")
        self.log("  STEP 2 — START 3D FUSION SCAN", "nav")
        self.log("  Jetson starting:", "nav")
        self.log("    • ZED Fusion (3D reconstruction)", "nav")
        self.log("    • Position tracking (save_position.py)", "nav")
        self.log("  Recording to .SVO file", "nav")
        self.log("  Fly mission area with Mission Planner", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_START_3D_FUSION, CMD_START_3D_FUSION)

    def stop_3d_fusion(self):
        """Stop ZED Fusion recording and save the 3D scan."""
        self.log("─" * 44, "nav")
        self.log("  STOPPING 3D FUSION", "nav")
        self.log("  Saving 3D reconstruction file", "nav")
        self.log("  Position data saved", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_STOP_3D_FUSION, CMD_STOP_3D_FUSION)

    def start_photo_capture(self):
        """Start photo capture mode with selected quality."""
        quality = self.quality_var.get()
        self.log("─" * 44, "nav")
        self.log("  PHOTO CAPTURE MODE", "nav")
        self.log(f"  Quality: {quality}", "nav")
        self.log("  Capturing images during flight", "nav")
        self.log("  Fly mission area with Mission Planner", "nav")
        self.log("─" * 44, "nav")
        
        # Encode quality in the command (will be parsed by Jetson)
        self.send_jetson_cmd(CMD_ID_START_PHOTO, f"{CMD_START_PHOTO}:{quality}")

    def start_video_capture(self):
        """Start video recording mode with selected quality."""
        quality = self.quality_var.get()
        self.log("─" * 44, "nav")
        self.log("  VIDEO RECORDING MODE", "nav")
        self.log(f"  Quality: {quality}", "nav")
        self.log("  Recording video during flight", "nav")
        self.log("  Fly mission area with Mission Planner", "nav")
        self.log("─" * 44, "nav")
        
        # Encode quality in the command (will be parsed by Jetson)
        self.send_jetson_cmd(CMD_ID_START_VIDEO, f"{CMD_START_VIDEO}:{quality}")

    def stop_capture(self):
        """Stop photo/video capture."""
        self.log("─" * 44, "nav")
        self.log("  STOPPING CAPTURE", "nav")
        self.log("  Saving files...", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_STOP_CAPTURE, CMD_STOP_CAPTURE)

    def save_end_point(self):
        self.log("─" * 44, "nav")
        self.log("  STEP 3 — SAVE END POINT", "nav")
        self.log("  Saves scan_end_pos.json on Jetson", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_SAVE_END, CMD_SAVE_END)

    def go_home_and_land(self):
        self.log("─" * 44, "nav")
        self.log("  STEP 4 — GO HOME + LAND", "nav")
        self.log("  Jetson runs return_land.py", "nav")
        self.log("  AprilTag activated automatically", "nav")
        self.log("  Phase 1 → ZED navigate to T0", "nav")
        self.log("  Phase 2 → AprilTag align at 90°", "nav")
        self.log("  Phase 3 → Land on rover", "nav")
        self.log("─" * 44, "nav")
        self.send_jetson_cmd(CMD_ID_GO_HOME, CMD_GO_HOME)
        if self.mav_connected:
            self.set_mode("GUIDED")

    # ── Jetson system commands ────────────────────────────────
    def ping_jetson(self):
        self.send_jetson_cmd(CMD_ID_PING, CMD_PING)

    def launch_jetson(self):
        self.send_jetson_cmd(CMD_ID_LAUNCH, CMD_LAUNCH)

    def launch_jetson_sim(self):
        self.send_jetson_cmd(CMD_ID_LAUNCH_SIM, CMD_LAUNCH_SIM)

    def kill_jetson(self):
        self.send_jetson_cmd(CMD_ID_STOP_ALL, "CLARQ_STOP_ALL")


# ── Run ───────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = DroneGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()