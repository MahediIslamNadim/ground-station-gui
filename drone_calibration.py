#!/usr/bin/env python3
"""
NexCore Ground Station # Production-quality ESP8266 Flight Controller GUI
ArduPilot/MissionPlanner-level ground station with full MAVLink support.
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import json
import csv
import time
import math
import threading
import struct
import os
import sys
from datetime import datetime
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# ### Theme ####################################################################

class Theme:
    BG_DARK = "#0a0e17"
    BG_PANEL = "#111827"
    BG_CARD = "#1a2332"
    BG_INPUT = "#0d1117"
    BORDER = "#1e293b"
    BLUE = "#3b82f6"
    GREEN = "#10b981"
    RED = "#ef4444"
    YELLOW = "#f59e0b"
    PURPLE = "#8b5cf6"
    CYAN = "#06b6d4"
    ORANGE = "#f97316"
    PINK = "#ec4899"
    TEXT = "#f1f5f9"
    TEXT2 = "#94a3b8"
    TEXT3 = "#64748b"
    TEXT4 = "#475569"


# ### ScrollFrame ##############################################################

class ScrollFrame(tk.Frame):
    def __init__(self, parent, **kw):
        super().__init__(parent, **kw)
        self.canvas = tk.Canvas(self, bg=Theme.BG_PANEL, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = tk.Frame(self.canvas, bg=Theme.BG_PANEL)
        self.inner.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.bind("<Enter>", lambda e: self.canvas.bind_all("<MouseWheel>", self._on_mousewheel))
        self.canvas.bind("<Leave>", lambda e: self.canvas.unbind_all("<MouseWheel>"))

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")


# ### MAVLink v2 Protocol #####################################################

class MAVLink:
    HEADER = 0xFE
    KNOWN_MSGS = {
        0: "HEARTBEAT", 1: "SYS_STATUS", 14: "BATTERY_STATUS",
        24: "GPS_RAW_INT", 27: "RAW_IMU", 30: "ATTITUDE",
        36: "SERVO_OUTPUT_RAW", 65: "RC_CHANNELS",
    }

    @classmethod
    def parse_frame(cls, data):
        if len(data) < 6:
            return None
        if data[0] != 0xFE:
            return None
        length = data[1]
        seq = data[2]
        sysid = data[3]
        compid = data[4]
        msgid = data[5]
        if len(data) < 6 + length + 1:
            return None
        payload = data[6:6 + length]
        checksum = data[6 + length]
        calc = 0
        for b in payload:
            calc ^= b
        if calc != checksum:
            return None
        return {"msgid": msgid, "sysid": sysid, "compid": compid,
                "payload": payload, "length": length, "seq": seq}

    @classmethod
    def decode_heartbeat(cls, p):
        if len(p) < 9:
            return {}
        type_, autopilot, base_mode = struct.unpack("<BBB", p[:3])
        custom_mode = struct.unpack("<I", p[3:7])[0]
        system_status = p[7]
        mavlink_version = p[8]
        return {"type": type_, "autopilot": autopilot, "base_mode": base_mode,
                "custom_mode": custom_mode, "system_status": system_status,
                "mavlink_version": mavlink_version}

    @classmethod
    def decode_sys_status(cls, p):
        if len(p) < 19:
            return {}
        vol = struct.unpack("<h", p[0:2])[0] / 1000.0
        cur = struct.unpack("<h", p[2:4])[0] / 100.0
        rem = p[4]
        armed = p[5]
        load = struct.unpack("<H", p[16:18])[0]
        failsafe = p[18]
        return {"voltage": vol, "current": cur, "remaining": rem,
                "armed": armed, "load": load, "failsafe": failsafe}

    @classmethod
    def decode_gps_raw_int(cls, p):
        if len(p) < 20:
            return {}
        lat = struct.unpack("<i", p[8:12])[0] / 1e7
        lon = struct.unpack("<i", p[12:16])[0] / 1e7
        alt = struct.unpack("<i", p[16:20])[0] / 1000.0
        fix = p[20] if len(p) > 20 else 0
        sats = p[21] if len(p) > 21 else 0
        return {"fix": fix, "sats": sats, "lat": lat, "lon": lon, "alt": alt}

    @classmethod
    def decode_raw_imu(cls, p):
        if len(p) < 26:
            return {}
        ax, ay, az = struct.unpack("<hhh", p[8:14])
        gx, gy, gz = struct.unpack("<hhh", p[14:20])
        mx, my, mz = struct.unpack("<hhh", p[20:26])
        return {"ax": ax / 1000.0, "ay": ay / 1000.0, "az": az / 1000.0,
                "gx": gx / 1000.0, "gy": gy / 1000.0, "gz": gz / 1000.0,
                "mx": mx / 1000.0, "my": my / 1000.0, "mz": mz / 1000.0}

    @classmethod
    def decode_attitude(cls, p):
        if len(p) < 28:
            return {}
        tms = struct.unpack("<I", p[0:4])[0]
        roll, pitch, yaw = struct.unpack("<fff", p[4:16])
        rollspeed, pitchspeed, yawspeed = struct.unpack("<fff", p[16:28])
        return {"time_boot_ms": tms, "roll": math.degrees(roll), "pitch": math.degrees(pitch),
                "yaw": math.degrees(yaw),
                "rollspeed": math.degrees(rollspeed), "pitchspeed": math.degrees(pitchspeed),
                "yawspeed": math.degrees(yawspeed)}

    @classmethod
    def decode_servo_output(cls, p):
        if len(p) < 17:
            return {}
        servos = struct.unpack("<8H", p[1:17])
        return {"servos": list(servos)}

    @classmethod
    def decode_rc_channels(cls, p):
        if len(p) < 21:
            return {}
        chs = struct.unpack("<8H", p[4:20])
        rssi = p[20]
        return {"channels": list(chs), "rssi": rssi}

    @classmethod
    def decode_param_value(cls, p):
        if len(p) < 24:
            return {}
        pid = p[0:16].split(b'\x00')[0].decode('ascii', errors='ignore')
        val = struct.unpack("<f", p[16:20])[0]
        ptype = p[20]
        return {"param_id": pid, "param_value": val, "param_type": ptype}

    @classmethod
    def decode_command_ack(cls, p):
        if len(p) < 4:
            return {}
        cmd, result = struct.unpack("<HH", p[:4])
        return {"command": cmd, "result": result}


# ### Parameter definitions ###################################################

PARAM_DEFS = [
    ("RATE_ROLL_P", 1.0, "float", 0.0, 5.0),
    ("RATE_ROLL_I", 0.1, "float", 0.0, 2.0),
    ("RATE_ROLL_D", 0.01, "float", 0.0, 1.0),
    ("RATE_PITCH_P", 1.0, "float", 0.0, 5.0),
    ("RATE_PITCH_I", 0.1, "float", 0.0, 2.0),
    ("RATE_PITCH_D", 0.01, "float", 0.0, 1.0),
    ("RATE_YAW_P", 1.0, "float", 0.0, 5.0),
    ("RATE_YAW_I", 0.1, "float", 0.0, 2.0),
    ("RATE_YAW_D", 0.01, "float", 0.0, 1.0),
    ("ANGLE_ROLL_P", 5.0, "float", 0.0, 20.0),
    ("ANGLE_PITCH_P", 5.0, "float", 0.0, 20.0),
    ("RC1_MIN", 1000, "int", 800, 1200),
    ("RC1_MAX", 2000, "int", 1800, 2200),
    ("RC1_EXPO", 0.0, "float", 0.0, 1.0),
    ("RC1_DEADBAND", 30, "int", 0, 100),
    ("RC2_MIN", 1000, "int", 800, 1200),
    ("RC2_MAX", 2000, "int", 1800, 2200),
    ("RC2_EXPO", 0.0, "float", 0.0, 1.0),
    ("RC2_DEADBAND", 30, "int", 0, 100),
    ("RC3_MIN", 1000, "int", 800, 1200),
    ("RC3_MAX", 2000, "int", 1800, 2200),
    ("RC3_EXPO", 0.0, "float", 0.0, 1.0),
    ("RC3_DEADBAND", 30, "int", 0, 100),
    ("RC4_MIN", 1000, "int", 800, 1200),
    ("RC4_MAX", 2000, "int", 1800, 2200),
    ("RC4_EXPO", 0.0, "float", 0.0, 1.0),
    ("RC4_DEADBAND", 30, "int", 0, 100),
    ("MIX_THR_CURVE", 1.0, "float", 0.0, 2.0),
    ("MIX_YAW_SCALE", 1.0, "float", 0.0, 2.0),
    ("FAILSAFE_TIMEOUT", 1.5, "float", 0.5, 10.0),
    ("BATT_WARN_VOLT", 10.5, "float", 8.0, 14.0),
    ("BATT_CRIT_VOLT", 9.5, "float", 7.0, 12.0),
    ("LOG_ENABLE", 1, "int", 0, 1),
    ("TELEM_RATE", 50, "int", 10, 200),
    ("MADGWICK_BETA", 0.04, "float", 0.001, 1.0),
    ("BATTERY_CAPACITY", 2200, "int", 500, 10000),
    ("FAILSAFE_ACTION", 1, "int", 0, 3),
    ("GEOFENCE_RADIUS", 100.0, "float", 0.0, 1000.0),
]


# ### Connection wrapper ######################################################

class SerialConn:
    def __init__(self):
        self.serial = None
        self.is_wifi = False

    def open_serial(self, port, baud):
        import time
        self.serial = serial.Serial(port, baud, timeout=0.005)
        self.serial.dtr = False
        self.serial.rts = False
        time.sleep(1.0)
        self.serial.reset_input_buffer()
        self.is_wifi = False

    def open_wifi(self, ip, port=23):
        import socket
        self.serial = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serial.connect((ip, port))
        self.serial.settimeout(0.01)
        self.is_wifi = True

    def read(self, n=1):
        if self.is_wifi:
            return self.serial.recv(n)
        return self.serial.read(n)

    def readline(self):
        if self.is_wifi:
            data = b""
            while True:
                try:
                    c = self.serial.recv(1)
                    if c == b"\n" or not c:
                        break
                    data += c
                except:
                    break
            return data + b"\n"
        return self.serial.readline()

    def write(self, data):
        if self.is_wifi:
            self.serial.sendall(data)
        else:
            self.serial.write(data)

    def close(self):
        try:
            if self.is_wifi:
                self.serial.close()
            elif self.serial and self.serial.is_open:
                self.serial.close()
        except:
            pass

    def is_open(self):
        if self.is_wifi:
            return self.serial is not None
        return self.serial is not None and self.serial.is_open


# ### Main Ground Station #####################################################

class GroundStation:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("NexCore Ground Station - ESP8266 Flight Controller")
        self.root.geometry("1800x1000")
        self.root.minsize(1400, 900)
        self.root.configure(bg=Theme.BG_DARK)

        self.conn = SerialConn()
        self.running = True
        self.reading = False
        self.connected = False
        self.connection_status = "IDLE"

        self.data = {
            "attitude": [0.0, 0.0, 0.0],
            "attitude_speed": [0.0, 0.0, 0.0],
            "accel": [0.0, 0.0, 0.0],
            "gyro": [0.0, 0.0, 0.0],
            "mag": [0.0, 0.0, 0.0],
            "gps": {"fix": 0, "sats": 0, "lat": 0.0, "lon": 0.0, "alt": 0.0},
            "battery": {"voltage": 0.0, "current": 0.0, "remaining": 100},
            "servos": [1000] * 8,
            "rc": [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500],
            "heartbeat": {"type": 0, "autopilot": 0, "base_mode": 0, "custom_mode": 0, "system_status": 0},
            "mode": "STABILIZE",
            "armed": False,
            "failsafe": False,
            "temp": 25.0,
            "pressure": 1013.25,
            "baro_alt": 0.0,
        }

        self._debug_lines = []
        self._history_len = 300
        self._last_heartbeat = 0
        self._graph_pending = False
        self._record_pending = False
        self._hb_warned = False
        self._graph_data = {
            "accel_x": [], "accel_y": [], "accel_z": [],
            "gyro_x": [], "gyro_y": [], "gyro_z": [],
            "mag_x": [], "mag_y": [], "mag_z": [],
            "roll": [], "pitch": [], "yaw": [],
        }
        self._sample_count = 0
        self._mavlink_count = 0
        self._text_count = 0
        self._last_rate_time = time.time()
        self._data_rate = 0.0
        self._rate_count = 0

        self.recording = False
        self.record_file = None
        self.record_writer = None

        self._cal_offsets = {"ax": 0.0, "ay": 0.0, "az": 0.0,
                             "gx": 0.0, "gy": 0.0, "gz": 0.0,
                             "mx": 0.0, "my": 0.0, "mz": 0.0}
        self._cal_scale = {"ax": 1.0, "ay": 1.0, "az": 1.0,
                           "mx": 1.0, "my": 1.0, "mz": 1.0}

        self._rc_cal = {
            "min": [1000] * 8, "max": [2000] * 8,
            "record_min": [2000] * 8, "record_max": [1000] * 8,
        }

        self.waypoints = []
        self.profiles = {}

        self._reader_thread = None

        self._setup_styles()
        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._scan_ports()
        self._load_profiles()
        self._update_hud()
        self._update_compass()
        self._update_level()
        self._update_graphs()
        self._update_telemetry()
        self._update_sensor_data()
        self._pulse_status()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=Theme.BG_PANEL)
        style.configure("TLabel", background=Theme.BG_PANEL, foreground=Theme.TEXT, font=("Segoe UI", 9))
        style.configure("TButton", background=Theme.BLUE, foreground="white", font=("Segoe UI", 9, "bold"),
                        borderwidth=0, padding=(8, 4))
        style.map("TButton", background=[("active", "#2563eb")])
        style.configure("Green.TButton", background=Theme.GREEN)
        style.map("Green.TButton", background=[("active", "#059669")])
        style.configure("Red.TButton", background=Theme.RED)
        style.map("Red.TButton", background=[("active", "#dc2626")])
        style.configure("TCombobox", fieldbackground=Theme.BG_INPUT, background=Theme.BG_INPUT,
                        foreground=Theme.TEXT, selectbackground=Theme.BLUE)
        style.configure("TEntry", fieldbackground=Theme.BG_INPUT, foreground=Theme.TEXT)
        style.configure("Treeview", background=Theme.BG_CARD, foreground=Theme.TEXT,
                        fieldbackground=Theme.BG_INPUT, borderwidth=0, font=("Consolas", 9))
        style.configure("Treeview.Heading", background=Theme.BG_PANEL, foreground=Theme.CYAN,
                        font=("Segoe UI", 9, "bold"))
        style.map("Treeview", background=[("selected", Theme.BLUE)])
        style.configure("Horizontal.TScale", background=Theme.BG_PANEL, troughcolor=Theme.BG_INPUT)
        style.configure("TNotebook", background=Theme.BG_DARK)
        style.configure("TNotebook.Tab", background=Theme.BG_CARD, foreground=Theme.TEXT2,
                        padding=(12, 6), font=("Segoe UI", 9))
        style.map("TNotebook.Tab",
                  background=[("selected", Theme.BLUE)],
                  foreground=[("selected", "white")])

    def _build_ui(self):
        self._build_top_bar()
        main = tk.Frame(self.root, bg=Theme.BG_DARK)
        main.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))

        self.left_panel = ScrollFrame(main, bg=Theme.BG_PANEL)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 2))
        self.left_panel.config(width=300)
        self.left_panel.pack_propagate(False)

        self.right_panel = ScrollFrame(main, bg=Theme.BG_PANEL)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(2, 0))
        self.right_panel.config(width=280)
        self.right_panel.pack_propagate(False)

        self.notebook = ttk.Notebook(main)
        self.notebook.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_connection_panel(self.left_panel.inner)
        self._build_parameter_panel(self.left_panel.inner)
        self._build_flight_config_panel(self.left_panel.inner)
        self._build_profiles_panel(self.left_panel.inner)
        self._build_record_panel(self.left_panel.inner)
        self._build_console_panel(self.left_panel.inner)

        self.hud_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_hud_tab(self.hud_frame)
        self.notebook.add(self.hud_frame, text=" HUD ")

        self.graphs_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_graphs_tab(self.graphs_frame)
        self.notebook.add(self.graphs_frame, text=" GRAPHS ")

        self.compass_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_compass_tab(self.compass_frame)
        self.notebook.add(self.compass_frame, text=" COMPASS ")

        self.level_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_level_tab(self.level_frame)
        self.notebook.add(self.level_frame, text=" LEVEL ")

        self.cal_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_cal_tab(self.cal_frame)
        self.notebook.add(self.cal_frame, text=" CAL ")

        self.rccal_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_rc_cal_tab(self.rccal_frame)
        self.notebook.add(self.rccal_frame, text=" RC CAL ")

        self.mission_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_mission_tab(self.mission_frame)
        self.notebook.add(self.mission_frame, text=" MISSION ")

        self.mixer_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_mixer_tab(self.mixer_frame)
        self.notebook.add(self.mixer_frame, text=" MIXER ")

        self.logs_frame = tk.Frame(self.notebook, bg=Theme.BG_DARK)
        self._build_logs_tab(self.logs_frame)
        self.notebook.add(self.logs_frame, text=" LOGS ")

        self._build_telemetry_panel(self.right_panel.inner)
        self._build_sensor_panel(self.right_panel.inner)

    def _build_top_bar(self):
        bar = tk.Frame(self.root, bg=Theme.BG_PANEL, height=36)
        bar.pack(fill=tk.X, padx=4, pady=4)
        bar.pack_propagate(False)

        tk.Label(bar, text="  NEXCORE GROUND STATION", bg=Theme.BG_PANEL,
                 fg=Theme.CYAN, font=("Consolas", 12, "bold")).pack(side=tk.LEFT)

        self.status_led = tk.Canvas(bar, width=16, height=16, bg=Theme.BG_PANEL, highlightthickness=0)
        self.status_led.pack(side=tk.LEFT, padx=(20, 6))
        self._led_oval = self.status_led.create_oval(2, 2, 14, 14, fill=Theme.RED, outline="")

        tk.Label(bar, text="Status:", bg=Theme.BG_PANEL, fg=Theme.TEXT2,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(10, 2))
        self.status_label = tk.Label(bar, text="IDLE", bg=Theme.BG_PANEL, fg=Theme.YELLOW,
                                     font=("Consolas", 9, "bold"))
        self.status_label.pack(side=tk.LEFT)

        tk.Label(bar, text="Rate:", bg=Theme.BG_PANEL, fg=Theme.TEXT2,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(30, 2))
        self.rate_label = tk.Label(bar, text="0 Hz", bg=Theme.BG_PANEL, fg=Theme.CYAN,
                                   font=("Consolas", 9, "bold"))
        self.rate_label.pack(side=tk.LEFT)

        tk.Label(bar, text="Samples:", bg=Theme.BG_PANEL, fg=Theme.TEXT2,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(30, 2))
        self.samples_label = tk.Label(bar, text="0", bg=Theme.BG_PANEL, fg=Theme.GREEN,
                                      font=("Consolas", 9, "bold"))
        self.samples_label.pack(side=tk.LEFT)

        self.record_indicator = tk.Label(bar, text="", bg=Theme.BG_PANEL, fg=Theme.RED,
                                         font=("Consolas", 10, "bold"))
        self.record_indicator.pack(side=tk.RIGHT, padx=10)

    # ### Connection Panel #################################################

    def _build_connection_panel(self, parent):
        f = tk.LabelFrame(parent, text=" CONNECTION ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=8, pady=6)
        f.pack(fill=tk.X, padx=6, pady=4)

        r1 = tk.Frame(f, bg=Theme.BG_CARD)
        r1.pack(fill=tk.X, pady=2)
        tk.Label(r1, text="Mode:", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=8, anchor="w").pack(side=tk.LEFT)
        self.conn_mode = ttk.Combobox(r1, values=["Serial", "WiFi"], width=10, state="readonly")
        self.conn_mode.set("Serial")
        self.conn_mode.pack(side=tk.LEFT, padx=4)
        self.conn_mode.bind("<<ComboboxSelected>>", self._on_conn_mode)

        r2 = tk.Frame(f, bg=Theme.BG_CARD)
        r2.pack(fill=tk.X, pady=2)
        tk.Label(r2, text="Port:", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=8, anchor="w").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(r2, width=12)
        self.port_combo.pack(side=tk.LEFT, padx=4)
        self.refresh_btn = ttk.Button(r2, text="#", width=3, command=self._scan_ports)
        self.refresh_btn.pack(side=tk.LEFT)

        r3 = tk.Frame(f, bg=Theme.BG_CARD)
        r3.pack(fill=tk.X, pady=2)
        tk.Label(r3, text="Baud:", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=8, anchor="w").pack(side=tk.LEFT)
        self.baud_combo = ttk.Combobox(r3, values=["9600", "19200", "38400", "57600", "115200",
                                                     "230400", "460800", "921600"], width=12)
        self.baud_combo.set("115200")
        self.baud_combo.pack(side=tk.LEFT, padx=4)

        self.wifi_frame = tk.Frame(f, bg=Theme.BG_CARD)
        tk.Label(self.wifi_frame, text="IP:", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=8, anchor="w").pack(side=tk.LEFT)
        self.wifi_ip = tk.Entry(self.wifi_frame, bg=Theme.BG_INPUT, fg=Theme.TEXT, insertbackground=Theme.TEXT,
                                width=14, font=("Consolas", 9))
        self.wifi_ip.insert(0, "192.168.4.1")
        self.wifi_ip.pack(side=tk.LEFT, padx=4)

        r4 = tk.Frame(f, bg=Theme.BG_CARD)
        r4.pack(fill=tk.X, pady=4)
        self.connect_btn = ttk.Button(r4, text="CONNECT", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self.disconnect_btn = ttk.Button(r4, text="DISCONNECT", command=self._disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)

    def _on_conn_mode(self, event=None):
        if self.conn_mode.get() == "WiFi":
            self.wifi_frame.pack(fill=tk.X, pady=2, after=self.conn_mode.master)
        else:
            self.wifi_frame.pack_forget()

    def _scan_ports(self):
        ports = []
        if HAS_SERIAL:
            for p in serial.tools.list_ports.comports():
                ports.append(p.device)
        if not ports:
            ports = [f"COM{i}" for i in range(1, 31)]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.set(ports[0])

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        self.connection_status = "CONNECTING"
        self._update_status()
        port = self.port_combo.get()
        try:
            mode = self.conn_mode.get()
            if mode == "Serial":
                baud = int(self.baud_combo.get())
                self.conn.open_serial(port, baud)
            else:
                ip = self.wifi_ip.get().strip()
                self.conn.open_wifi(ip)
            self.connected = True
            self.reading = True
            self.connection_status = "CONNECTED"
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self._start_reader()
            self._log("Connected via " + mode)
        except Exception as e:
            self.connection_status = "FAILED"
            err = str(e)
            if "Access is denied" in err or "PermissionError" in err:
                err = "Port busy - close Arduino Serial Monitor or other apps using " + port
            self._log("Connection failed: " + err)
        self._update_status()

    def _disconnect(self):
        self.reading = False
        self.connected = False
        self.conn.close()
        self.connection_status = "IDLE"
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self._update_status()
        self._log("Disconnected")

    def _update_status(self):
        colors = {"IDLE": Theme.YELLOW, "CONNECTING": Theme.ORANGE, "CONNECTED": Theme.GREEN,
                  "READING": Theme.GREEN, "FAILED": Theme.RED}
        color = colors.get(self.connection_status, Theme.TEXT3)
        self.status_label.config(text=self.connection_status, fg=color)
        self.status_led.itemconfig(self._led_oval, fill=color)

    # ### Reader Thread ####################################################

    def _start_reader(self):
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()

    def _reader(self):
        buf = b''
        while self.running:
            if self.reading and self.conn.is_open():
                try:
                    chunk = self.conn.read(1024)
                    if chunk:
                        buf += chunk
                        while buf:
                            if buf[0:1] == b'\xfe':
                                if len(buf) < 7:
                                    break
                                length = buf[1]
                                total = 6 + length + 1
                                if len(buf) < total:
                                    break
                                frame_data = buf[:total]
                                buf = buf[total:]
                                frame = MAVLink.parse_frame(frame_data)
                                if frame:
                                    self._parse_mavlink(frame)
                                    self._sample_count += 1
                                    self._rate_count += 1
                                    self._mavlink_count += 1
                                    name = MAVLink.KNOWN_MSGS.get(frame["msgid"], str(frame["msgid"]))
                                    if self._mavlink_count <= 3:
                                        self.root.after(0, lambda n=name, l=frame["length"]: self._log(f"MAV: {n} len={l}"))
                            elif buf[0:1] == b'\n':
                                buf = buf[1:]
                            elif buf[0] < 0x20 or buf[0] > 0x7e:
                                buf = buf[1:]
                            else:
                                nl = buf.find(b'\n')
                                if nl >= 0:
                                    line_bytes = buf[:nl + 1]
                                    buf = buf[nl + 1:]
                                    text = line_bytes.decode('utf-8', errors='ignore').strip()
                                    if text:
                                        self._parse_text(text)
                                        self._sample_count += 1
                                        self._rate_count += 1
                                        self._text_count += 1
                                else:
                                    break
                except Exception as e:
                    self.root.after(0, lambda e=e: self._log("ERR: " + str(e)))
            time.sleep(0.001)

    def _parse_mavlink(self, frame):
        msgid = frame["msgid"]
        p = frame["payload"]
        if msgid == 0:
            d = MAVLink.decode_heartbeat(p)
            if d:
                self.data["heartbeat"] = d
                self.data["armed"] = bool(d.get("base_mode", 0) & 0x80)
                modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED",
                         "LOITER", "RTL", "CIRCLE", "POSHOLD"]
                mode_id = d.get("custom_mode", 0)
                self.data["mode"] = modes[mode_id] if mode_id < len(modes) else "UNKNOWN"
                self._last_heartbeat = time.time()
        elif msgid == 1:
            d = MAVLink.decode_sys_status(p)
            if d:
                self.data["battery"] = d
                if "failsafe" in d:
                    self.data["failsafe"] = bool(d["failsafe"])
                if "armed" in d:
                    self.data["armed"] = bool(d["armed"])
        elif msgid == 24:
            d = MAVLink.decode_gps_raw_int(p)
            if d:
                self.data["gps"] = d
        elif msgid == 27:
            d = MAVLink.decode_raw_imu(p)
            if d:
                self.data["accel"] = [d["ax"], d["ay"], d["az"]]
                self.data["gyro"] = [d["gx"], d["gy"], d["gz"]]
                self.data["mag"] = [d["mx"], d["my"], d["mz"]]
        elif msgid == 30:
            d = MAVLink.decode_attitude(p)
            if d:
                self.data["attitude"] = [d["roll"], d["pitch"], d["yaw"]]
                self.data["attitude_speed"] = [d.get("rollspeed", 0), d.get("pitchspeed", 0), d.get("yawspeed", 0)]
        elif msgid == 36:
            d = MAVLink.decode_servo_output(p)
            if d:
                self.data["servos"] = d["servos"]
        elif msgid == 65:
            d = MAVLink.decode_rc_channels(p)
            if d:
                self.data["rc"] = d["channels"]
                self.data["rssi"] = d.get("rssi", 0)
        elif msgid == 22:
            d = MAVLink.decode_param_value(p)
            if d:
                self._log(f"PARAM: {d['param_id']} = {d['param_value']:.4f}")
        elif msgid == 76:
            d = MAVLink.decode_command_ack(p)
            if d:
                self._log(f"CMD_ACK: cmd={d['command']} result={d['result']}")

        if not getattr(self, '_graph_pending', False):
            self._graph_pending = True
            self.root.after(0, lambda: self._throttled_graph_update())
        if not getattr(self, '_record_pending', False):
            self._record_pending = True
            self.root.after(0, lambda: self._throttled_record())

    def _throttled_graph_update(self):
        self._graph_pending = False
        self._update_graph_data()

    def _throttled_record(self):
        self._record_pending = False
        self._record_data_point()

    def _parse_text(self, line):
        self.root.after(0, lambda: self._log("RX: " + line))
        if line.startswith("{"):
            try:
                d = json.loads(line)
                if "attitude" in d:
                    self.data["attitude"] = d["attitude"]
                if "accel" in d:
                    self.data["accel"] = d["accel"]
                if "gyro" in d:
                    self.data["gyro"] = d["gyro"]
                if "mag" in d:
                    self.data["mag"] = d["mag"]
                if "gps" in d:
                    self.data["gps"] = d["gps"]
                if "battery" in d:
                    self.data["battery"] = d["battery"]
                if "servos" in d:
                    self.data["servos"] = d["servos"]
                if "rc" in d:
                    self.data["rc"] = d["rc"]
                if "mode" in d:
                    self.data["mode"] = d["mode"]
                if "armed" in d:
                    self.data["armed"] = d["armed"]
                if "temp" in d:
                    self.data["temp"] = d["temp"]
                if "pressure" in d:
                    self.data["pressure"] = d["pressure"]
                if "baro_alt" in d:
                    self.data["baro_alt"] = d["baro_alt"]
            except json.JSONDecodeError:
                pass

    def _send_command(self, cmd):
        if self.connected:
            try:
                self.conn.write((cmd + "\n").encode('utf-8'))
                self._log("TX: " + cmd)
            except Exception as e:
                self._log("Send error: " + str(e))

    def _send_mavlink_cmd(self, cmd_id, params=None):
        if params is None:
            params = [0] * 7
        cmd_map = {
            22: lambda p: self._send_command(f"PARAM_SET {self._param_id_to_name(int(p[0]))} {p[1]:.4f}"),
            23: lambda p: self._send_command(f"PARAM_GET {self._param_id_to_name(int(p[0]))}"),
            520: lambda p: self._send_command("ARM"),
            521: lambda p: self._send_command("DISARM"),
        }
        if cmd_id in cmd_map:
            try:
                cmd_map[cmd_id](params)
            except Exception as e:
                self._log(f"CMD error: {e}")
        else:
            self._log(f"Unknown MAVLink cmd: {cmd_id}")

    def _update_graph_data(self):
        a = self.data["accel"]
        g = self.data["gyro"]
        m = self.data["mag"]
        att = self.data["attitude"]
        g_deg = [math.degrees(g[0]), math.degrees(g[1]), math.degrees(g[2])]
        for k, v in [("accel_x", a[0]), ("accel_y", a[1]), ("accel_z", a[2]),
                      ("gyro_x", g_deg[0]), ("gyro_y", g_deg[1]), ("gyro_z", g_deg[2]),
                      ("mag_x", m[0]), ("mag_y", m[1]), ("mag_z", m[2]),
                      ("roll", att[0]), ("pitch", att[1]), ("yaw", att[2])]:
            self._graph_data[k].append(v)
            if len(self._graph_data[k]) > self._history_len:
                self._graph_data[k] = self._graph_data[k][-self._history_len:]

    def _record_data_point(self):
        if self.recording and self.record_writer:
            row = {
                "time": time.time(),
                "roll": self.data["attitude"][0], "pitch": self.data["attitude"][1],
                "yaw": self.data["attitude"][2],
                "ax": self.data["accel"][0], "ay": self.data["accel"][1], "az": self.data["accel"][2],
                "gx": self.data["gyro"][0], "gy": self.data["gyro"][1], "gz": self.data["gyro"][2],
                "mx": self.data["mag"][0], "my": self.data["mag"][1], "mz": self.data["mag"][2],
                "voltage": self.data["battery"].get("voltage", 0),
                "current": self.data["battery"].get("current", 0),
                "lat": self.data["gps"].get("lat", 0), "lon": self.data["gps"].get("lon", 0),
                "alt": self.data["gps"].get("alt", 0),
            }
            try:
                self.record_writer.writerow(row)
            except:
                pass

    # ### Console Panel ####################################################

    def _build_console_panel(self, parent):
        f = tk.LabelFrame(parent, text=" CONSOLE ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=6, pady=4)
        f.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        self.debug_text = tk.Text(f, bg=Theme.BG_INPUT, fg=Theme.GREEN, font=("Consolas", 8),
                                  height=8, wrap=tk.WORD, state=tk.DISABLED,
                                  insertbackground=Theme.GREEN, borderwidth=0)
        self.debug_text.pack(fill=tk.BOTH, expand=True)

        btn_row = tk.Frame(f, bg=Theme.BG_CARD)
        btn_row.pack(fill=tk.X, pady=(4, 0))
        for txt, cmd in [("HELP", "HELP"), ("STATUS", "STATUS"), ("I2C", "I2C_SCAN"),
                          ("RAW", "RAW_IMU"), ("ARM", "ARM"), ("DISARM", "DISARM")]:
            ttk.Button(btn_row, text=txt, width=7,
                       command=lambda c=cmd: self._send_command(c)).pack(side=tk.LEFT, padx=1)

        entry_row = tk.Frame(f, bg=Theme.BG_CARD)
        entry_row.pack(fill=tk.X, pady=(4, 0))
        self.cmd_entry = tk.Entry(entry_row, bg=Theme.BG_INPUT, fg=Theme.TEXT,
                                  insertbackground=Theme.TEXT, font=("Consolas", 9), borderwidth=0)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self.cmd_entry.bind("<Return>", lambda e: self._send_cmd_entry())
        ttk.Button(entry_row, text="SEND", command=self._send_cmd_entry).pack(side=tk.RIGHT)

    def _send_cmd_entry(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self._send_command(cmd)
            self.cmd_entry.delete(0, tk.END)

    def _log(self, msg):
        self.root.after(0, lambda: self.log_debug(msg))

    def log_debug(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] {msg}"
        self._debug_lines.append(line)
        if len(self._debug_lines) > 200:
            self._debug_lines = self._debug_lines[-200:]
        self.debug_text.config(state=tk.NORMAL)
        self.debug_text.insert(tk.END, line + "\n")
        total_lines = int(self.debug_text.index("end-1c").split(".")[0])
        if total_lines > 250:
            self.debug_text.delete("1.0", f"{total_lines - 200}.0")
        self.debug_text.see(tk.END)
        self.debug_text.config(state=tk.DISABLED)

    # ### Parameter Panel ##################################################

    def _build_parameter_panel(self, parent):
        f = tk.LabelFrame(parent, text=" PARAMETERS ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=6, pady=4)
        f.pack(fill=tk.X, padx=6, pady=4)

        tree_frame = tk.Frame(f, bg=Theme.BG_CARD)
        tree_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        self.param_tree = ttk.Treeview(tree_frame, columns=("name", "value"), show="headings", height=10)
        self.param_tree.heading("name", text="Parameter")
        self.param_tree.heading("value", text="Value")
        self.param_tree.column("name", width=120)
        self.param_tree.column("value", width=70)
        self.param_tree.pack(fill=tk.BOTH, expand=True)

        for name, val, *_ in PARAM_DEFS:
            self.param_tree.insert("", tk.END, iid=name, values=(name, f"{val:.4f}"))

        edit_row = tk.Frame(f, bg=Theme.BG_CARD)
        edit_row.pack(fill=tk.X, pady=(4, 0))
        tk.Label(edit_row, text="Value:", bg=Theme.BG_CARD, fg=Theme.TEXT2).pack(side=tk.LEFT)
        self.param_val_entry = tk.Entry(edit_row, bg=Theme.BG_INPUT, fg=Theme.TEXT,
                                        insertbackground=Theme.TEXT, width=10, font=("Consolas", 9))
        self.param_val_entry.pack(side=tk.LEFT, padx=4)
        ttk.Button(edit_row, text="SET", command=self._param_set).pack(side=tk.LEFT, padx=2)

        btn_row = tk.Frame(f, bg=Theme.BG_CARD)
        btn_row.pack(fill=tk.X, pady=(4, 0))
        ttk.Button(btn_row, text="GET ALL", command=self._param_get_all).pack(side=tk.LEFT, padx=1, fill=tk.X, expand=True)
        ttk.Button(btn_row, text="SAVE", command=self._param_save).pack(side=tk.LEFT, padx=1, fill=tk.X, expand=True)
        ttk.Button(btn_row, text="RESET", command=self._param_reset).pack(side=tk.LEFT, padx=1, fill=tk.X, expand=True)

    def _param_set(self):
        sel = self.param_tree.selection()
        if not sel:
            return
        name = sel[0]
        val_str = self.param_val_entry.get().strip()
        try:
            val = float(val_str)
        except ValueError:
            self._log("Invalid value")
            return
        self.param_tree.item(name, values=(name, f"{val:.4f}"))
        self._send_command(f"PARAM_SET {name} {val}")
        self._log(f"Set {name} = {val}")

    def _param_get_all(self):
        self._send_command("PARAM_GET ALL")
        self._log("Requesting all parameters...")

    def _param_save(self):
        self._send_command("PARAM_SAVE")
        self._log("Parameters saved to EEPROM")

    def _param_reset(self):
        if messagebox.askyesno("Reset Parameters", "Reset all parameters to defaults?"):
            for name, val, *_ in PARAM_DEFS:
                self.param_tree.item(name, values=(name, f"{val:.4f}"))
            self._send_command("PARAM RESET")
            self._log("Parameters reset to defaults")

    # ### Flight Config Panel ##############################################

    def _build_flight_config_panel(self, parent):
        f = tk.LabelFrame(parent, text=" FLIGHT CONFIG ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=6, pady=4)
        f.pack(fill=tk.X, padx=6, pady=4)

        modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE", "POSHOLD"]
        for i, mode in enumerate(modes[:6]):
            r = tk.Frame(f, bg=Theme.BG_CARD)
            r.pack(fill=tk.X, pady=1)
            tk.Label(r, text=f"Ch5 {1000+i*200}-{1199+i*200}:", bg=Theme.BG_CARD,
                     fg=Theme.TEXT3, font=("Segoe UI", 8), width=14, anchor="w").pack(side=tk.LEFT)
            cb = ttk.Combobox(r, values=modes, width=10, state="readonly")
            cb.set(mode)
            cb.pack(side=tk.LEFT, padx=4)

        tk.Frame(f, height=4, bg=Theme.BG_CARD).pack()

        r = tk.Frame(f, bg=Theme.BG_CARD)
        r.pack(fill=tk.X, pady=1)
        tk.Label(r, text="Failsafe (s):", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=14, anchor="w").pack(side=tk.LEFT)
        self.fs_timeout = tk.Entry(r, bg=Theme.BG_INPUT, fg=Theme.TEXT, insertbackground=Theme.TEXT,
                                   width=8, font=("Consolas", 9))
        self.fs_timeout.insert(0, "1.5")
        self.fs_timeout.pack(side=tk.LEFT, padx=4)

        r = tk.Frame(f, bg=Theme.BG_CARD)
        r.pack(fill=tk.X, pady=1)
        tk.Label(r, text="Batt Warn (V):", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=14, anchor="w").pack(side=tk.LEFT)
        self.batt_warn = tk.Entry(r, bg=Theme.BG_INPUT, fg=Theme.TEXT, insertbackground=Theme.TEXT,
                                  width=8, font=("Consolas", 9))
        self.batt_warn.insert(0, "10.5")
        self.batt_warn.pack(side=tk.LEFT, padx=4)

        r = tk.Frame(f, bg=Theme.BG_CARD)
        r.pack(fill=tk.X, pady=1)
        tk.Label(r, text="Batt Crit (V):", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=14, anchor="w").pack(side=tk.LEFT)
        self.batt_crit = tk.Entry(r, bg=Theme.BG_INPUT, fg=Theme.TEXT, insertbackground=Theme.TEXT,
                                  width=8, font=("Consolas", 9))
        self.batt_crit.insert(0, "9.5")
        self.batt_crit.pack(side=tk.LEFT, padx=4)

        r = tk.Frame(f, bg=Theme.BG_CARD)
        r.pack(fill=tk.X, pady=1)
        tk.Label(r, text="Geofence (m):", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=14, anchor="w").pack(side=tk.LEFT)
        self.geofence = tk.Entry(r, bg=Theme.BG_INPUT, fg=Theme.TEXT, insertbackground=Theme.TEXT,
                                 width=8, font=("Consolas", 9))
        self.geofence.insert(0, "100")
        self.geofence.pack(side=tk.LEFT, padx=4)

        r = tk.Frame(f, bg=Theme.BG_CARD)
        r.pack(fill=tk.X, pady=1)
        tk.Label(r, text="Telem Rate (Hz):", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=14, anchor="w").pack(side=tk.LEFT)
        self.telem_rate = tk.Entry(r, bg=Theme.BG_INPUT, fg=Theme.TEXT, insertbackground=Theme.TEXT,
                                   width=8, font=("Consolas", 9))
        self.telem_rate.insert(0, "50")
        self.telem_rate.pack(side=tk.LEFT, padx=4)

        r = tk.Frame(f, bg=Theme.BG_CARD)
        r.pack(fill=tk.X, pady=1)
        tk.Label(r, text="Failsafe Action:", bg=Theme.BG_CARD, fg=Theme.TEXT2, width=14, anchor="w").pack(side=tk.LEFT)
        self.failsafe_action = ttk.Combobox(r, values=["NONE", "HOLD", "LAND", "RTL"], width=8, state="readonly")
        self.failsafe_action.set("HOLD")
        self.failsafe_action.pack(side=tk.LEFT, padx=4)

        ttk.Button(f, text="APPLY CONFIG", command=self._apply_flight_config).pack(fill=tk.X, pady=(6, 0))

    def _apply_flight_config(self):
        self._send_command(f"PARAM_SET BAT_VOLT_MIN {self.batt_crit.get()}")
        self._send_command(f"PARAM_SET GEOFENCE_RADIUS {self.geofence.get()}")
        fs_map = {"NONE": 0, "HOLD": 1, "LAND": 2, "RTL": 3}
        fs_val = fs_map.get(self.failsafe_action.get(), 1)
        self._send_command(f"PARAM_SET FAILSAFE_ACTION {fs_val}")
        self._log("Flight config applied")

    # ### Profiles Panel ###################################################

    def _build_profiles_panel(self, parent):
        f = tk.LabelFrame(parent, text=" PROFILES ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=6, pady=4)
        f.pack(fill=tk.X, padx=6, pady=4)

        self.profile_listbox = tk.Listbox(f, bg=Theme.BG_INPUT, fg=Theme.TEXT, font=("Consolas", 9),
                                          height=4, selectbackground=Theme.BLUE, borderwidth=0)
        self.profile_listbox.pack(fill=tk.X)

        row = tk.Frame(f, bg=Theme.BG_CARD)
        row.pack(fill=tk.X, pady=(4, 0))
        self.profile_name = tk.Entry(row, bg=Theme.BG_INPUT, fg=Theme.TEXT, insertbackground=Theme.TEXT,
                                     width=14, font=("Consolas", 9))
        self.profile_name.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(row, text="SAVE", command=self._profile_save).pack(side=tk.LEFT, padx=1)
        ttk.Button(row, text="LOAD", command=self._profile_load).pack(side=tk.LEFT, padx=1)
        ttk.Button(row, text="DEL", command=self._profile_delete).pack(side=tk.LEFT, padx=1)

    def _profile_save(self):
        name = self.profile_name.get().strip()
        if not name:
            return
        profile = {}
        for item in self.param_tree.get_children():
            vals = self.param_tree.item(item, "values")
            profile[vals[0]] = float(vals[1])
        self.profiles[name] = profile
        self._save_profiles()
        self._refresh_profile_list()
        self._log(f"Profile '{name}' saved")

    def _profile_load(self):
        sel = self.profile_listbox.curselection()
        if not sel:
            return
        name = self.profile_listbox.get(sel[0])
        if name in self.profiles:
            for pname, pval in self.profiles[name].items():
                if pname in [p[0] for p in PARAM_DEFS]:
                    self.param_tree.item(pname, values=(pname, f"{pval:.4f}"))
            self._log(f"Profile '{name}' loaded")

    def _profile_delete(self):
        sel = self.profile_listbox.curselection()
        if not sel:
            return
        name = self.profile_listbox.get(sel[0])
        if name in self.profiles:
            del self.profiles[name]
            self._save_profiles()
            self._refresh_profile_list()
            self._log(f"Profile '{name}' deleted")

    def _refresh_profile_list(self):
        self.profile_listbox.delete(0, tk.END)
        for name in sorted(self.profiles.keys()):
            self.profile_listbox.insert(tk.END, name)

    def _save_profiles(self):
        try:
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles.json")
            with open(path, "w") as f:
                json.dump(self.profiles, f, indent=2)
        except Exception as e:
            self._log("Profile save error: " + str(e))

    def _load_profiles(self):
        try:
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles.json")
            if os.path.exists(path):
                with open(path, "r") as f:
                    self.profiles = json.load(f)
                self._refresh_profile_list()
        except:
            self.profiles = {}

    # ### Record Panel #####################################################

    def _build_record_panel(self, parent):
        f = tk.LabelFrame(parent, text=" RECORDING ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=6, pady=4)
        f.pack(fill=tk.X, padx=6, pady=4)

        self.record_btn = ttk.Button(f, text="START RECORD", command=self._toggle_record)
        self.record_btn.pack(fill=tk.X)

        self.record_status = tk.Label(f, text="Stopped", bg=Theme.BG_CARD, fg=Theme.TEXT3,
                                      font=("Segoe UI", 8))
        self.record_status.pack(pady=2)

    def _toggle_record(self):
        if self.recording:
            self.recording = False
            if self.record_file:
                self.record_file.close()
                self.record_file = None
                self.record_writer = None
            self.record_btn.config(text="START RECORD")
            self.record_status.config(text="Stopped", fg=Theme.TEXT3)
            self.record_indicator.config(text="")
            self._log("Recording stopped")
        else:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"flight_log_{ts}.csv"
            try:
                self.record_file = open(filename, "w", newline="")
                self.record_writer = csv.DictWriter(self.record_file, fieldnames=[
                    "time", "roll", "pitch", "yaw", "ax", "ay", "az",
                    "gx", "gy", "gz", "mx", "my", "mz",
                    "voltage", "current", "lat", "lon", "alt"
                ])
                self.record_writer.writeheader()
                self.recording = True
                self.record_btn.config(text="STOP RECORD")
                self.record_status.config(text=f"Recording: {filename}", fg=Theme.RED)
                self.record_indicator.config(text="# REC")
                self._log(f"Recording to {filename}")
            except Exception as e:
                self._log("Record error: " + str(e))

    # ### Telemetry Panel ##################################################

    def _build_telemetry_panel(self, parent):
        f = tk.LabelFrame(parent, text=" TELEMETRY ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=6, pady=4)
        f.pack(fill=tk.X, padx=6, pady=4)

        self.batt_bar_canvas = tk.Canvas(f, height=14, bg=Theme.BG_INPUT, highlightthickness=0)
        self.batt_bar_canvas.pack(fill=tk.X, pady=2)
        self.batt_bar = self.batt_bar_canvas.create_rectangle(0, 0, 0, 14, fill=Theme.GREEN, outline="")

        self.batt_info = tk.Label(f, text="0.0V  0.0A  100%", bg=Theme.BG_CARD, fg=Theme.GREEN,
                                  font=("Consolas", 9, "bold"))
        self.batt_info.pack(pady=2)

        self.batt_time = tk.Label(f, text="Est: --:--", bg=Theme.BG_CARD, fg=Theme.TEXT3,
                                  font=("Consolas", 8))
        self.batt_time.pack(pady=1)

        tk.Frame(f, height=6, bg=Theme.BG_CARD).pack()

        self.gps_label = tk.Label(f, text="GPS: No Fix  Sats: 0", bg=Theme.BG_CARD,
                                  fg=Theme.YELLOW, font=("Consolas", 9, "bold"))
        self.gps_label.pack(anchor="w", pady=1)

        self.gps_coords = tk.Label(f, text="0.000000, 0.000000", bg=Theme.BG_CARD,
                                   fg=Theme.TEXT2, font=("Consolas", 8))
        self.gps_coords.pack(anchor="w", pady=1)

        self.gps_alt = tk.Label(f, text="Alt: 0.0 m", bg=Theme.BG_CARD,
                                fg=Theme.TEXT2, font=("Consolas", 8))
        self.gps_alt.pack(anchor="w", pady=1)

        tk.Frame(f, height=6, bg=Theme.BG_CARD).pack()

        self.rc_canvas = tk.Canvas(f, height=80, bg=Theme.BG_INPUT, highlightthickness=0)
        self.rc_canvas.pack(fill=tk.X, pady=2)
        self.rc_bars = []
        for i in range(8):
            x0 = i * 34 + 4
            self.rc_bars.append(self.rc_canvas.create_rectangle(x0, 60, x0 + 30, 78, fill=Theme.BLUE, outline=""))
            self.rc_canvas.create_text(x0 + 15, 74, text=str(i + 1), fill=Theme.TEXT4, font=("Consolas", 7))

        tk.Frame(f, height=6, bg=Theme.BG_CARD).pack()

        self.mode_label = tk.Label(f, text="Mode: STABILIZE", bg=Theme.BG_CARD, fg=Theme.GREEN,
                                   font=("Consolas", 10, "bold"))
        self.mode_label.pack(anchor="w", pady=1)

        self.armed_label = tk.Label(f, text="Disarmed", bg=Theme.BG_CARD, fg=Theme.RED,
                                    font=("Consolas", 9, "bold"))
        self.armed_label.pack(anchor="w", pady=1)

        self.fs_label = tk.Label(f, text="Failsafe: OFF", bg=Theme.BG_CARD, fg=Theme.TEXT3,
                                 font=("Consolas", 8))
        self.fs_label.pack(anchor="w", pady=1)

        self.log_status = tk.Label(f, text="Log: OFF", bg=Theme.BG_CARD, fg=Theme.TEXT3,
                                   font=("Consolas", 8))
        self.log_status.pack(anchor="w", pady=1)

        self.cpu_label = tk.Label(f, text="CPU: --%", bg=Theme.BG_CARD, fg=Theme.TEXT3,
                                  font=("Consolas", 8))
        self.cpu_label.pack(anchor="w", pady=1)

    # ### Sensor Panel #####################################################

    def _build_sensor_panel(self, parent):
        f = tk.LabelFrame(parent, text=" SENSOR DATA ", bg=Theme.BG_CARD, fg=Theme.CYAN,
                          font=("Segoe UI", 9, "bold"), bd=1, relief=tk.GROOVE, padx=6, pady=4)
        f.pack(fill=tk.X, padx=6, pady=4)

        self.sensor_accel = tk.Label(f, text="Accel: 0.00  0.00  0.00 m/s²", bg=Theme.BG_CARD,
                                     fg=Theme.TEXT, font=("Consolas", 8))
        self.sensor_accel.pack(anchor="w", pady=1)

        self.sensor_gyro = tk.Label(f, text="Gyro:  0.00  0.00  0.00 °/s", bg=Theme.BG_CARD,
                                    fg=Theme.TEXT, font=("Consolas", 8))
        self.sensor_gyro.pack(anchor="w", pady=1)

        self.sensor_mag = tk.Label(f, text="Mag:   0.0  0.0  0.0 µT", bg=Theme.BG_CARD,
                                   fg=Theme.TEXT, font=("Consolas", 8))
        self.sensor_mag.pack(anchor="w", pady=1)

        tk.Frame(f, height=4, bg=Theme.BG_CARD).pack()

        self.sensor_temp = tk.Label(f, text="Temp: 25.0 °C", bg=Theme.BG_CARD,
                                    fg=Theme.TEXT2, font=("Consolas", 8))
        self.sensor_temp.pack(anchor="w", pady=1)

        self.sensor_pres = tk.Label(f, text="Pres: 1013.25 hPa", bg=Theme.BG_CARD,
                                    fg=Theme.TEXT2, font=("Consolas", 8))
        self.sensor_pres.pack(anchor="w", pady=1)

        self.sensor_baro_alt = tk.Label(f, text="Baro: 0.0 m", bg=Theme.BG_CARD,
                                        fg=Theme.TEXT2, font=("Consolas", 8))
        self.sensor_baro_alt.pack(anchor="w", pady=1)

        tk.Frame(f, height=4, bg=Theme.BG_CARD).pack()

        self.sensor_orientation = tk.Label(f, text="Roll: 0.0°  Pitch: 0.0°  Yaw: 0.0°",
                                           bg=Theme.BG_CARD, fg=Theme.PURPLE, font=("Consolas", 9, "bold"))
        self.sensor_orientation.pack(anchor="w", pady=1)

        tk.Frame(f, height=4, bg=Theme.BG_CARD).pack()

        tk.Label(f, text="Cal Offsets:", bg=Theme.BG_CARD, fg=Theme.TEXT3,
                 font=("Segoe UI", 8, "bold"), anchor="w").pack(anchor="w")
        self.cal_offsets_label = tk.Label(f, text="AX:0.00 AY:0.00 AZ:0.00", bg=Theme.BG_CARD,
                                          fg=Theme.TEXT3, font=("Consolas", 8))
        self.cal_offsets_label.pack(anchor="w")
        self.cal_offsets_label2 = tk.Label(f, text="GX:0.00 GY:0.00 GZ:0.00", bg=Theme.BG_CARD,
                                           fg=Theme.TEXT3, font=("Consolas", 8))
        self.cal_offsets_label2.pack(anchor="w")
        self.cal_offsets_label3 = tk.Label(f, text="MX:0.0  MY:0.0  MZ:0.0", bg=Theme.BG_CARD,
                                           fg=Theme.TEXT3, font=("Consolas", 8))
        self.cal_offsets_label3.pack(anchor="w")

        tk.Frame(f, height=4, bg=Theme.BG_CARD).pack()

        self.stats_label = tk.Label(f, text="Samples: 0  Rate: 0 Hz", bg=Theme.BG_CARD,
                                    fg=Theme.TEXT3, font=("Consolas", 8))
        self.stats_label.pack(anchor="w", pady=1)

    # ### HUD Tab ##########################################################

    def _build_hud_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)
        self.hud_canvas = tk.Canvas(parent, width=500, height=400, bg=Theme.BG_DARK, highlightthickness=0)
        self.hud_canvas.pack(expand=True)

    def _update_hud(self):
        c = self.hud_canvas
        c.delete("all")
        W, H = 500, 400
        cx, cy = W // 2, H // 2

        roll = math.radians(self.data["attitude"][0])
        pitch = self.data["attitude"][1]
        heading = self.data["attitude"][2]
        if heading < 0:
            heading += 360

        pitch_offset = pitch * 3

        sky_top = -100 + pitch_offset
        ground_top = cy + pitch_offset

        c.create_rectangle(0, 0, W, max(0, ground_top), fill='#1a3a6c', outline='')
        c.create_rectangle(0, ground_top, W, H, fill='#5c3a1a', outline='')

        c.create_line(0, ground_top, W, ground_top, fill='white', width=2)

        for deg in [-30, -20, -10, 10, 20, 30]:
            y = ground_top - deg * 3
            if 10 < y < H - 10:
                half = 30 if abs(deg) > 10 else 20
                c.create_line(cx - half, y, cx + half, y, fill='white', width=1)
                c.create_text(cx + half + 12, y, text=str(deg), fill='white',
                              font=('Consolas', 7), anchor="w")

        c.create_line(cx - 50, cy, cx - 15, cy, fill='yellow', width=2)
        c.create_line(cx + 15, cy, cx + 50, cy, fill='yellow', width=2)
        c.create_line(cx - 15, cy, cx - 15, cy + 8, fill='yellow', width=2)
        c.create_line(cx + 15, cy, cx + 15, cy + 8, fill='yellow', width=2)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill='yellow', outline='')

        arc_r = 50
        for angle in range(-60, 61, 10):
            rad = math.radians(angle - 90)
            x1 = cx + (arc_r - 5) * math.cos(rad)
            y1 = 20 + (arc_r - 5) * math.sin(rad)
            x2 = cx + arc_r * math.cos(rad)
            y2 = 20 + arc_r * math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill='white', width=1)
        c.create_arc(cx - arc_r, 20 - arc_r, cx + arc_r, 20 + arc_r,
                     start=30, extent=300, style=tk.ARC, outline=Theme.TEXT3, width=1)
        roll_rad = math.radians(-roll)
        pointer_x = cx + (arc_r + 8) * math.sin(roll_rad)
        pointer_y = 20 - (arc_r + 8) * math.cos(roll_rad)
        c.create_polygon(pointer_x - 4, pointer_y + 6, pointer_x, pointer_y - 4,
                         pointer_x + 4, pointer_y + 6, fill='orange', outline='white')

        tape_h = 30
        tape_y = 30
        heading_int = int(heading)
        for i in range(-4, 5):
            h = (heading_int + i * 10) % 360
            x_off = cx + (i * 10 - (heading_int % 10)) * 3
            if 0 < x_off < W:
                if h % 30 == 0:
                    labels = {0: "N", 90: "E", 180: "S", 270: "W"}
                    txt = labels.get(h, str(h))
                    c.create_line(x_off, tape_y, x_off, tape_y + 10, fill='white', width=1)
                    c.create_text(x_off, tape_y - 6, text=txt, fill='white', font=('Consolas', 8))
                elif h % 10 == 0:
                    c.create_line(x_off, tape_y, x_off, tape_y + 6, fill=Theme.TEXT3, width=1)
                    c.create_text(x_off, tape_y - 6, text=str(h), fill=Theme.TEXT3, font=('Consolas', 7))
        c.create_rectangle(cx - 1, tape_y - 2, cx + 1, tape_y + 14, fill='orange', outline='')

        alt = self.data["gps"].get("alt", 0)
        if alt == 0:
            alt = self.data["baro_alt"]
        alt_str = f"{alt:.0f}m"
        c.create_rectangle(W - 60, cy - 30, W - 5, cy + 30, fill=Theme.BG_INPUT, outline=Theme.BORDER)
        c.create_text(W - 32, cy - 10, text=alt_str, fill='white', font=('Consolas', 10, 'bold'))
        c.create_text(W - 32, cy + 12, text="ALT", fill=Theme.TEXT3, font=('Consolas', 7))

        spd = abs(self.data["gyro"][2]) * 0.5
        spd_str = f"{spd:.0f}"
        c.create_rectangle(5, cy - 30, 60, cy + 30, fill=Theme.BG_INPUT, outline=Theme.BORDER)
        c.create_text(32, cy - 10, text=spd_str, fill='white', font=('Consolas', 10, 'bold'))
        c.create_text(32, cy + 12, text="SPD", fill=Theme.TEXT3, font=('Consolas', 7))

        mode = self.data.get("mode", "STABILIZE")
        c.create_rectangle(cx - 35, 8, cx + 35, 22, fill=Theme.BLUE, outline='')
        c.create_text(cx, 15, text=mode, fill='white', font=('Consolas', 8, 'bold'))

        thr = int((self.data["rc"][2] - 1000) / 10)
        thr = max(0, min(100, thr))
        c.create_rectangle(cx - 20, H - 55, cx + 20, H - 10, fill=Theme.BG_INPUT, outline=Theme.BORDER)
        thr_h = int(thr * 0.4)
        c.create_rectangle(cx - 18, H - 13 - thr_h, cx + 18, H - 12, fill=Theme.ORANGE, outline='')
        c.create_text(cx, H - 35, text=f"{thr}%", fill='white', font=('Consolas', 8))

        ax, ay, az = self.data["accel"]
        g_force = math.sqrt(ax * ax + ay * ay + az * az)
        c.create_text(10, H - 10, text=f"{g_force:.1f}G", fill=Theme.CYAN, font=('Consolas', 9, 'bold'),
                      anchor="sw")

        armed = self.data.get("armed", False)
        arm_text = "ARMED" if armed else "DISARMED"
        arm_color = Theme.RED if armed else Theme.GREEN
        c.create_text(W - 10, H - 10, text=arm_text, fill=arm_color, font=('Consolas', 9, 'bold'),
                      anchor="se")

        self.root.after(50, self._update_hud)

    # ### Telemetry Update #################################################

    def _update_telemetry(self):
        b = self.data["battery"]
        vol = b.get("voltage", 0)
        cur = b.get("current", 0)
        rem = b.get("remaining", 100)

        bar_w = int(rem / 100.0 * 260) if vol > 0 else 0
        bar_w = max(0, min(260, bar_w))
        color = Theme.GREEN if vol > 11.0 else (Theme.YELLOW if vol > 10.0 else Theme.RED)
        self.batt_bar_canvas.coords(self.batt_bar, 0, 0, bar_w, 14)
        self.batt_bar_canvas.itemconfig(self.batt_bar, fill=color)
        self.batt_info.config(text=f"{vol:.1f}V  {cur:.1f}A  {rem:.0f}%", fg=color)

        if cur > 0 and vol > 0:
            cap_mah = 2200
            rem_mah = cap_mah * rem / 100
            est_min = rem_mah / cur * 60 if cur > 0 else 0
            mins = int(est_min)
            secs = int((est_min - mins) * 60)
            self.batt_time.config(text=f"Est: {mins:02d}:{secs:02d}", fg=Theme.TEXT2)
        else:
            self.batt_time.config(text="Est: --:--", fg=Theme.TEXT3)

        gps = self.data["gps"]
        fix_types = ["No Fix", "2D", "3D", "DGPS", "RTK Float", "RTK Fixed"]
        fix_str = fix_types[min(gps.get("fix", 0), 5)]
        sats = gps.get("sats", 0)
        gps_color = Theme.GREEN if gps.get("fix", 0) >= 3 else (Theme.YELLOW if gps.get("fix", 0) >= 2 else Theme.RED)
        self.gps_label.config(text=f"GPS: {fix_str}  Sats: {sats}", fg=gps_color)
        self.gps_coords.config(text=f"{gps.get('lat', 0):.6f}, {gps.get('lon', 0):.6f}")
        self.gps_alt.config(text=f"Alt: {gps.get('alt', 0):.1f} m")

        rc = self.data["rc"]
        for i in range(8):
            val = rc[i] if i < len(rc) else 1500
            norm = (val - 1000) / 1000.0
            norm = max(0, min(1, norm))
            h = norm * 55
            x0 = i * 34 + 4
            bar_color = Theme.GREEN if abs(norm - 0.5) < 0.1 else Theme.BLUE
            self.rc_canvas.coords(self.rc_bars[i], x0, 60 - h, x0 + 30, 60)
            self.rc_canvas.itemconfig(self.rc_bars[i], fill=bar_color)

        self.mode_label.config(text=f"Mode: {self.data.get('mode', 'STABILIZE')}")
        armed = self.data.get("armed", False)
        self.armed_label.config(text="ARMED" if armed else "Disarmed",
                                fg=Theme.RED if armed else Theme.GREEN)
        self.fs_label.config(text=f"Failsafe: {'ON' if self.data.get('failsafe', False) else 'OFF'}")
        self.log_status.config(text=f"Log: {'ON' if self.recording else 'OFF'}",
                               fg=Theme.RED if self.recording else Theme.TEXT3)

        self.root.after(50, self._update_telemetry)

    def _update_sensor_data(self):
        a = self.data["accel"]
        g = self.data["gyro"]
        m = self.data["mag"]
        att = self.data["attitude"]

        self.sensor_accel.config(text=f"Accel: {a[0]*9.81:+.2f}  {a[1]*9.81:+.2f}  {a[2]*9.81:+.2f} m/s²")
        self.sensor_gyro.config(text=f"Gyro:  {math.degrees(g[0]):+.1f}  {math.degrees(g[1]):+.1f}  {math.degrees(g[2]):+.1f} °/s")
        self.sensor_mag.config(text=f"Mag:   {m[0]:+.1f}  {m[1]:+.1f}  {m[2]:+.1f} µT")
        self.sensor_orientation.config(text=f"Roll: {att[0]:+.1f}°  Pitch: {att[1]:+.1f}°  Yaw: {att[2]:+.1f}°")
        self.sensor_temp.config(text=f"Temp: {self.data['temp']:.1f} °C")
        self.sensor_pres.config(text=f"Pres: {self.data['pressure']:.2f} hPa")
        self.sensor_baro_alt.config(text=f"Baro: {self.data['baro_alt']:.1f} m")

        co = self._cal_offsets
        cs = self._cal_scale
        self.cal_offsets_label.config(text=f"AX:{co['ax']:+.2f} AY:{co['ay']:+.2f} AZ:{co['az']:+.2f}")
        self.cal_offsets_label2.config(text=f"GX:{co['gx']:+.2f} GY:{co['gy']:+.2f} GZ:{co['gz']:+.2f}")
        self.cal_offsets_label3.config(text=f"MX:{co['mx']:+.1f} MY:{co['my']:+.1f} MZ:{co['mz']:+.1f}")

        self.stats_label.config(text=f"Samples: {self._sample_count}  MAV: {self._mavlink_count}  TXT: {self._text_count}  Rate: {self._data_rate:.0f} Hz")

        self.root.after(50, self._update_sensor_data)

    def _pulse_status(self):
        now = time.time()
        dt = now - self._last_rate_time
        if dt >= 1.0:
            self._data_rate = self._rate_count / dt
            self._rate_count = 0
            self._last_rate_time = now
            self.rate_label.config(text=f"{self._data_rate:.0f} Hz")
            self.samples_label.config(text=str(self._sample_count))
        if self.connected and self._last_heartbeat > 0 and (now - self._last_heartbeat > 5.0):
            if not getattr(self, '_hb_warned', False):
                self._log("WARN: Heartbeat timeout (no FC heartbeat for 5s)")
                self._hb_warned = True
        else:
            self._hb_warned = False
        self.root.after(500, self._pulse_status)

    # ### Graphs Tab #######################################################

    def _build_graphs_tab(self, parent):
        if not HAS_MATPLOTLIB:
            tk.Label(parent, text="matplotlib not installed", bg=Theme.BG_DARK,
                     fg=Theme.RED, font=("Consolas", 12)).pack(expand=True)
            return

        self.graph_fig = Figure(figsize=(8, 6), dpi=80, facecolor=Theme.BG_DARK)
        self.graph_axes = []
        labels = ["Accel (G)", "Gyro (°/s)", "Mag (raw)", "Attitude (°)"]
        colors_list = [
            [(Theme.RED, 'X'), (Theme.GREEN, 'Y'), (Theme.BLUE, 'Z')],
            [(Theme.RED, 'X'), (Theme.GREEN, 'Y'), (Theme.BLUE, 'Z')],
            [(Theme.RED, 'X'), (Theme.GREEN, 'Y'), (Theme.BLUE, 'Z')],
            [(Theme.RED, 'Roll'), (Theme.GREEN, 'Pitch'), (Theme.BLUE, 'Yaw')],
        ]
        data_keys = [
            ["accel_x", "accel_y", "accel_z"],
            ["gyro_x", "gyro_y", "gyro_z"],
            ["mag_x", "mag_y", "mag_z"],
            ["roll", "pitch", "yaw"],
        ]

        for i in range(4):
            ax = self.graph_fig.add_subplot(4, 1, i + 1)
            ax.set_facecolor(Theme.BG_CARD)
            ax.tick_params(colors=Theme.TEXT3, labelsize=7)
            ax.set_ylabel(labels[i], color=Theme.TEXT2, fontsize=8)
            for spine in ax.spines.values():
                spine.set_color(Theme.BORDER)
            ax.grid(True, alpha=0.3, color=Theme.TEXT4)
            self.graph_axes.append({"ax": ax, "keys": data_keys[i], "colors": colors_list[i], "lines": []})
            for j, (color, lbl) in enumerate(colors_list[i]):
                line, = ax.plot([], [], color=color, linewidth=1, label=lbl)
                self.graph_axes[-1]["lines"].append(line)
            ax.legend(loc="upper right", fontsize=7, facecolor=Theme.BG_CARD, edgecolor=Theme.BORDER,
                      labelcolor=Theme.TEXT2)

        self.graph_fig.tight_layout(pad=1.5)
        self.graph_canvas = FigureCanvasTkAgg(self.graph_fig, master=parent)
        self.graph_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _update_graphs(self):
        if HAS_MATPLOTLIB and hasattr(self, 'graph_axes'):
            for ax_info in self.graph_axes:
                ax = ax_info["ax"]
                for j, key in enumerate(ax_info["keys"]):
                    data = self._graph_data.get(key, [])
                    if data:
                        ax_info["lines"][j].set_data(range(len(data)), data)
                ax.relim()
                ax.autoscale_view()
            self.graph_fig.canvas.draw_idle()
        self.root.after(200, self._update_graphs)

    # ### Compass Tab ######################################################

    def _build_compass_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)
        self.compass_canvas = tk.Canvas(parent, width=300, height=300, bg=Theme.BG_DARK, highlightthickness=0)
        self.compass_canvas.pack(expand=True)

    def _update_compass(self):
        c = self.compass_canvas
        c.delete("all")
        cx, cy, r = 150, 150, 120

        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=Theme.BORDER, width=2)
        c.create_oval(cx - r + 20, cy - r + 20, cx + r - 20, cy + r - 20, outline=Theme.BORDER, width=1)

        heading = self.data["attitude"][2]
        if heading < 0:
            heading += 360

        labels = {0: "N", 90: "E", 180: "S", 270: "W"}
        for deg in range(0, 360, 30):
            rad = math.radians(deg - 90)
            x1 = cx + (r - 15) * math.cos(rad)
            y1 = cy + (r - 15) * math.sin(rad)
            x2 = cx + (r - 5) * math.cos(rad)
            y2 = cy + (r - 5) * math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill=Theme.TEXT3, width=1)
            lbl = labels.get(deg, str(deg))
            tx = cx + (r - 30) * math.cos(rad)
            ty = cy + (r - 30) * math.sin(rad)
            c.create_text(tx, ty, text=lbl, fill=Theme.TEXT2, font=("Consolas", 10, "bold"))

        for deg in range(0, 360, 10):
            if deg % 30 != 0:
                rad = math.radians(deg - 90)
                x1 = cx + (r - 10) * math.cos(rad)
                y1 = cy + (r - 10) * math.sin(rad)
                x2 = cx + (r - 5) * math.cos(rad)
                y2 = cy + (r - 5) * math.sin(rad)
                c.create_line(x1, y1, x2, y2, fill=Theme.TEXT4, width=1)

        needle_rad = math.radians(heading - 90)
        nx = cx + (r - 25) * math.cos(needle_rad)
        ny = cy + (r - 25) * math.sin(needle_rad)
        c.create_line(cx, cy, nx, ny, fill=Theme.PINK, width=3)
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=Theme.PINK, outline='')

        tail_x = cx - (r - 60) * math.cos(needle_rad)
        tail_y = cy - (r - 60) * math.sin(needle_rad)
        c.create_line(cx, cy, tail_x, tail_y, fill=Theme.TEXT4, width=2)

        c.create_text(cx, cy - r - 15, text=f"Heading: {heading:.0f}°", fill=Theme.CYAN,
                      font=("Consolas", 11, "bold"))

        self.root.after(100, self._update_compass)

    # ### Level Tab ########################################################

    def _build_level_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)
        self.level_canvas = tk.Canvas(parent, width=300, height=300, bg=Theme.BG_DARK, highlightthickness=0)
        self.level_canvas.pack(expand=True)

    def _update_level(self):
        c = self.level_canvas
        c.delete("all")
        cx, cy, r = 150, 150, 120

        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=Theme.BORDER, width=2)
        c.create_oval(cx - r // 2, cy - r // 2, cx + r // 2, cy + r // 2, outline=Theme.BORDER, width=1)

        c.create_line(cx - r, cy, cx + r, cy, fill=Theme.TEXT4, width=1)
        c.create_line(cx, cy - r, cx, cy + r, fill=Theme.TEXT4, width=1)

        c.create_line(cx - 8, cy, cx + 8, cy, fill=Theme.GREEN, width=2)
        c.create_line(cx, cy - 8, cx, cy + 8, fill=Theme.GREEN, width=2)

        ax, ay, az = self.data["accel"]
        g_total = math.sqrt(ax * ax + ay * ay + az * az)
        if g_total > 0.1:
            norm_x = ax / g_total
            norm_y = ay / g_total
        else:
            norm_x, norm_y = 0, 0

        dot_x = cx + norm_x * r
        dot_y = cy - norm_y * r

        in_range = math.sqrt(norm_x ** 2 + norm_y ** 2) < 0.3
        dot_color = Theme.GREEN if in_range else Theme.RED

        c.create_oval(dot_x - 8, dot_y - 8, dot_x + 8, dot_y + 8, fill=dot_color, outline='white', width=2)

        c.create_text(cx, cy + r + 20, text=f"X: {ax:.2f}  Y: {ay:.2f}  Z: {az:.2f} G",
                      fill=Theme.TEXT2, font=("Consolas", 9))
        c.create_text(cx, cy + r + 36, text=f"Level: {'YES' if in_range else 'NO'}",
                      fill=Theme.GREEN if in_range else Theme.RED,
                      font=("Consolas", 10, "bold"))

        self.root.after(100, self._update_level)

    # ### Calibration Wizard Tab ###########################################

    def _build_cal_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)

        tk.Label(parent, text="Calibration Wizard", bg=Theme.BG_DARK, fg=Theme.CYAN,
                 font=("Consolas", 14, "bold")).pack(pady=10)

        cal_frame = tk.Frame(parent, bg=Theme.BG_PANEL, padx=10, pady=10)
        cal_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        self.cal_status = tk.Label(cal_frame, text="Ready", bg=Theme.BG_PANEL,
                                   fg=Theme.TEXT, font=("Consolas", 10))
        self.cal_status.pack(pady=5)

        self.cal_progress = ttk.Progressbar(cal_frame, length=300, mode='determinate')
        self.cal_progress.pack(pady=5)

        self.cal_sample_label = tk.Label(cal_frame, text="Samples: 0 / 0", bg=Theme.BG_PANEL,
                                         fg=Theme.TEXT2, font=("Consolas", 9))
        self.cal_sample_label.pack(pady=5)

        self.cal_offsets_display = tk.Label(cal_frame, text="", bg=Theme.BG_PANEL,
                                            fg=Theme.PURPLE, font=("Consolas", 9, "bold"),
                                            justify=tk.LEFT)
        self.cal_offsets_display.pack(pady=10)

        btn_frame = tk.Frame(cal_frame, bg=Theme.BG_PANEL)
        btn_frame.pack(pady=10)

        ttk.Button(btn_frame, text="Accel Cal (6-pos)", command=self._start_accel_cal).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Gyro Cal (Still)", command=self._start_gyro_cal).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Mag Cal (Circle)", command=self._start_mag_cal).pack(side=tk.LEFT, padx=5)

        self._cal_running = False
        self._cal_type = None
        self._cal_samples = []
        self._cal_target = 600

    def _start_accel_cal(self):
        self._cal_running = True
        self._cal_type = "accel"
        self._cal_samples = []
        self._cal_target = 600
        self.cal_status.config(text="Place flat and press SET", fg=Theme.YELLOW)
        self.cal_progress["value"] = 0
        self._log("Starting accel calibration")
        self._send_command("CAL ACC")

    def _start_gyro_cal(self):
        self._cal_running = True
        self._cal_type = "gyro"
        self._cal_samples = []
        self._cal_target = 300
        self.cal_status.config(text="Keep still...", fg=Theme.YELLOW)
        self.cal_progress["value"] = 0
        self._log("Starting gyro calibration")
        self._send_command("CAL GYRO")

    def _start_mag_cal(self):
        self._cal_running = True
        self._cal_type = "mag"
        self._cal_samples = []
        self._cal_target = 900
        self.cal_status.config(text="Move in circles...", fg=Theme.YELLOW)
        self.cal_progress["value"] = 0
        self._log("Starting mag calibration")
        self._send_command("CAL MAG")

    # ### RC Calibration Tab ##############################################

    def _build_rc_cal_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)

        tk.Label(parent, text="RC Calibration", bg=Theme.BG_DARK, fg=Theme.CYAN,
                 font=("Consolas", 14, "bold")).pack(pady=10)

        rc_frame = tk.Frame(parent, bg=Theme.BG_PANEL, padx=10, pady=10)
        rc_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        self.rc_cal_bars = []
        self.rc_cal_min_labels = []
        self.rc_cal_max_labels = []
        self.rc_cal_val_labels = []

        for i in range(8):
            row = tk.Frame(rc_frame, bg=Theme.BG_PANEL)
            row.pack(fill=tk.X, pady=2)

            tk.Label(row, text=f"CH{i + 1}", bg=Theme.BG_PANEL, fg=Theme.TEXT2,
                     width=4, font=("Consolas", 9)).pack(side=tk.LEFT)

            bar = tk.Canvas(row, width=200, height=16, bg=Theme.BG_INPUT, highlightthickness=0)
            bar.pack(side=tk.LEFT, padx=4)
            self.rc_cal_bars.append(bar)

            val_lbl = tk.Label(row, text="1500", bg=Theme.BG_PANEL, fg=Theme.TEXT,
                               width=5, font=("Consolas", 9))
            val_lbl.pack(side=tk.LEFT, padx=2)
            self.rc_cal_val_labels.append(val_lbl)

            min_lbl = tk.Label(row, text="Min:1000", bg=Theme.BG_PANEL, fg=Theme.GREEN,
                               width=9, font=("Consolas", 8))
            min_lbl.pack(side=tk.LEFT, padx=2)
            self.rc_cal_min_labels.append(min_lbl)

            max_lbl = tk.Label(row, text="Max:2000", bg=Theme.BG_PANEL, fg=Theme.RED,
                               width=9, font=("Consolas", 8))
            max_lbl.pack(side=tk.LEFT, padx=2)
            self.rc_cal_max_labels.append(max_lbl)

        tk.Frame(rc_frame, height=10, bg=Theme.BG_PANEL).pack()

        btn_frame = tk.Frame(rc_frame, bg=Theme.BG_PANEL)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="Record Min/Max", command=self._rc_record).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Reset", command=self._rc_reset).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Apply & Save", command=self._rc_apply).pack(side=tk.LEFT, padx=5)

        stick_frame = tk.Frame(rc_frame, bg=Theme.BG_PANEL)
        stick_frame.pack(pady=10)
        self.stick_canvas = tk.Canvas(stick_frame, width=200, height=100, bg=Theme.BG_INPUT,
                                      highlightthickness=0)
        self.stick_canvas.pack(side=tk.LEFT, padx=10)
        self.stick2_canvas = tk.Canvas(stick_frame, width=200, height=100, bg=Theme.BG_INPUT,
                                       highlightthickness=0)
        self.stick2_canvas.pack(side=tk.LEFT, padx=10)

        self._update_rc_cal_display()

    def _update_rc_cal_display(self):
        rc = self.data["rc"]
        for i in range(8):
            val = rc[i] if i < len(rc) else 1500
            norm = (val - 1000) / 1000.0
            norm = max(0, min(1, norm))
            w = 200
            self.rc_cal_bars[i].delete("all")
            bar_color = Theme.BLUE
            self.rc_cal_bars[i].create_rectangle(0, 0, int(norm * w), 16, fill=bar_color, outline="")
            self.rc_cal_val_labels[i].config(text=str(val))

        if hasattr(self, 'stick_canvas'):
            sc = self.stick_canvas
            sc.delete("all")
            sx = 100 + (rc[0] - 1500) / 500 * 80
            sy = 50 - (rc[1] - 1500) / 500 * 40
            sc.create_oval(sx - 6, sy - 6, sx + 6, sy + 6, fill=Theme.ORANGE, outline="white")

            sc2 = self.stick2_canvas
            sc2.delete("all")
            sx2 = 100 + (rc[3] - 1500) / 500 * 80
            sy2 = 50 - (rc[2] - 1500) / 500 * 40
            sc2.create_oval(sx2 - 6, sy2 - 6, sx2 + 6, sy2 + 6, fill=Theme.PINK, outline="white")

        self.root.after(100, self._update_rc_cal_display)

    def _rc_record(self):
        rc = self.data["rc"]
        for i in range(8):
            val = rc[i] if i < len(rc) else 1500
            if val < self._rc_cal["record_min"][i]:
                self._rc_cal["record_min"][i] = val
            if val > self._rc_cal["record_max"][i]:
                self._rc_cal["record_max"][i] = val
            self.rc_cal_min_labels[i].config(text=f"Min:{self._rc_cal['record_min'][i]}")
            self.rc_cal_max_labels[i].config(text=f"Max:{self._rc_cal['record_max'][i]}")
        self._log("RC min/max recorded")

    def _rc_reset(self):
        self._rc_cal["record_min"] = [2000] * 8
        self._rc_cal["record_max"] = [1000] * 8
        for i in range(8):
            self.rc_cal_min_labels[i].config(text=f"Min:{self._rc_cal['record_min'][i]}")
            self.rc_cal_max_labels[i].config(text=f"Max:{self._rc_cal['record_max'][i]}")
        self._log("RC cal reset")

    def _rc_apply(self):
        for i in range(8):
            self._send_command(f"RC{i + 1}_MIN {self._rc_cal['record_min'][i]}")
            self._send_command(f"RC{i + 1}_MAX {self._rc_cal['record_max'][i]}")
        self._log("RC calibration applied and saved")

    # ### Mission Tab ######################################################

    def _build_mission_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)

        tk.Label(parent, text="Mission Planner", bg=Theme.BG_DARK, fg=Theme.CYAN,
                 font=("Consolas", 14, "bold")).pack(pady=10)

        tree_frame = tk.Frame(parent, bg=Theme.BG_PANEL, padx=10, pady=10)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)

        self.mission_tree = ttk.Treeview(tree_frame, columns=("idx", "type", "lat", "lon", "alt"),
                                          show="headings", height=12)
        self.mission_tree.heading("idx", text="#")
        self.mission_tree.heading("type", text="Type")
        self.mission_tree.heading("lat", text="Latitude")
        self.mission_tree.heading("lon", text="Longitude")
        self.mission_tree.heading("alt", text="Alt (m)")
        self.mission_tree.column("idx", width=30)
        self.mission_tree.column("type", width=90)
        self.mission_tree.column("lat", width=100)
        self.mission_tree.column("lon", width=100)
        self.mission_tree.column("alt", width=60)
        self.mission_tree.pack(fill=tk.BOTH, expand=True)

        btn_frame = tk.Frame(parent, bg=Theme.BG_DARK)
        btn_frame.pack(fill=tk.X, padx=20, pady=5)

        ttk.Button(btn_frame, text="Add WP", command=self._mission_add).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Remove", command=self._mission_remove).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Move Up", command=self._mission_up).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Move Down", command=self._mission_down).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Export", command=self._mission_export).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Import", command=self._mission_import).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Upload", command=self._mission_upload).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="Clear", command=self._mission_clear).pack(side=tk.LEFT, padx=2)

    def _mission_add(self):
        idx = len(self.waypoints) + 1
        wp = {"type": "WAYPOINT", "lat": 0.0, "lon": 0.0, "alt": 10.0}
        self.waypoints.append(wp)
        self.mission_tree.insert("", tk.END, iid=str(idx),
                                  values=(idx, wp["type"], f"{wp['lat']:.6f}",
                                          f"{wp['lon']:.6f}", wp["alt"]))

    def _mission_remove(self):
        sel = self.mission_tree.selection()
        if sel:
            idx = int(sel[0]) - 1
            if 0 <= idx < len(self.waypoints):
                self.waypoints.pop(idx)
                self._refresh_mission_tree()

    def _mission_up(self):
        sel = self.mission_tree.selection()
        if sel:
            idx = int(sel[0]) - 1
            if idx > 0:
                self.waypoints[idx], self.waypoints[idx - 1] = self.waypoints[idx - 1], self.waypoints[idx]
                self._refresh_mission_tree()

    def _mission_down(self):
        sel = self.mission_tree.selection()
        if sel:
            idx = int(sel[0]) - 1
            if idx < len(self.waypoints) - 1:
                self.waypoints[idx], self.waypoints[idx + 1] = self.waypoints[idx + 1], self.waypoints[idx]
                self._refresh_mission_tree()

    def _refresh_mission_tree(self):
        for item in self.mission_tree.get_children():
            self.mission_tree.delete(item)
        for i, wp in enumerate(self.waypoints):
            self.mission_tree.insert("", tk.END, iid=str(i + 1),
                                      values=(i + 1, wp["type"], f"{wp['lat']:.6f}",
                                              f"{wp['lon']:.6f}", wp["alt"]))

    def _mission_export(self):
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                             filetypes=[("JSON", "*.json")])
        if path:
            with open(path, "w") as f:
                json.dump(self.waypoints, f, indent=2)
            self._log(f"Mission exported to {path}")

    def _mission_import(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            with open(path, "r") as f:
                self.waypoints = json.load(f)
            self._refresh_mission_tree()
            self._log(f"Mission imported from {path}")

    def _mission_upload(self):
        for i, wp in enumerate(self.waypoints):
            self._send_command(f"WP SET {i} {wp['type']} {wp['lat']} {wp['lon']} {wp['alt']}")
        self._log(f"Uploaded {len(self.waypoints)} waypoints")

    def _mission_clear(self):
        self.waypoints = []
        self._refresh_mission_tree()
        self._log("Mission cleared")

    # ### Mixer Tab ########################################################

    def _build_mixer_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)

        tk.Label(parent, text="Actuator Mixer", bg=Theme.BG_DARK, fg=Theme.CYAN,
                 font=("Consolas", 14, "bold")).pack(pady=10)

        self.mixer_canvas = tk.Canvas(parent, width=400, height=350, bg=Theme.BG_CARD, highlightthickness=0)
        self.mixer_canvas.pack(expand=True)

        ctrl_frame = tk.Frame(parent, bg=Theme.BG_DARK)
        ctrl_frame.pack(fill=tk.X, padx=20, pady=5)

        self.mix_roll = tk.Scale(ctrl_frame, from_=-100, to=100, orient=tk.HORIZONTAL,
                                  bg=Theme.BG_PANEL, fg=Theme.TEXT, troughcolor=Theme.BG_INPUT,
                                  label="Roll", length=120)
        self.mix_roll.pack(side=tk.LEFT, padx=5)
        self.mix_pitch = tk.Scale(ctrl_frame, from_=-100, to=100, orient=tk.HORIZONTAL,
                                   bg=Theme.BG_PANEL, fg=Theme.TEXT, troughcolor=Theme.BG_INPUT,
                                   label="Pitch", length=120)
        self.mix_pitch.pack(side=tk.LEFT, padx=5)
        self.mix_yaw = tk.Scale(ctrl_frame, from_=-100, to=100, orient=tk.HORIZONTAL,
                                 bg=Theme.BG_PANEL, fg=Theme.TEXT, troughcolor=Theme.BG_INPUT,
                                 label="Yaw", length=120)
        self.mix_yaw.pack(side=tk.LEFT, padx=5)
        self.mix_thr = tk.Scale(ctrl_frame, from_=0, to=100, orient=tk.HORIZONTAL,
                                 bg=Theme.BG_PANEL, fg=Theme.TEXT, troughcolor=Theme.BG_INPUT,
                                 label="Throttle", length=120)
        self.mix_thr.pack(side=tk.LEFT, padx=5)

        self._update_mixer()

    def _update_mixer(self):
        c = self.mixer_canvas
        c.delete("all")
        cx, cy = 200, 175

        c.create_line(cx - 80, cy - 80, cx + 80, cy + 80, fill=Theme.TEXT4, width=2)
        c.create_line(cx + 80, cy - 80, cx - 80, cy + 80, fill=Theme.TEXT4, width=2)

        roll = self.mix_roll.get()
        pitch = self.mix_pitch.get()
        yaw = self.mix_yaw.get()
        thr = self.mix_thr.get()

        m1 = max(0, thr - roll - pitch + yaw) / 100
        m2 = max(0, thr + roll - pitch - yaw) / 100
        m3 = max(0, thr + roll + pitch + yaw) / 100
        m4 = max(0, thr - roll + pitch - yaw) / 100

        motors = [(cx - 80, cy - 80, m1, "M1"), (cx + 80, cy - 80, m2, "M2"),
                  (cx + 80, cy + 80, m3, "M3"), (cx - 80, cy + 80, m4, "M4")]

        for mx, my, power, label in motors:
            bar_h = power * 60
            bar_color = Theme.GREEN if power > 0.3 else (Theme.YELLOW if power > 0.1 else Theme.RED)
            c.create_rectangle(mx - 12, my + 15, mx + 12, my + 15 + bar_h, fill=bar_color, outline="")
            c.create_oval(mx - 15, my - 15, mx + 15, my + 15, outline=bar_color, width=2)
            c.create_text(mx, my, text=label, fill=bar_color, font=("Consolas", 9, "bold"))

            arrow_len = power * 30
            if arrow_len > 5:
                c.create_line(mx, my, mx, my - arrow_len, fill=bar_color, width=2, arrow=tk.LAST)

        c.create_text(cx, cy, text="X", fill=Theme.TEXT3, font=("Consolas", 14, "bold"))

        c.create_text(cx, 320, text=f"R:{roll:+d}  P:{pitch:+d}  Y:{yaw:+d}  T:{thr}",
                       fill=Theme.TEXT2, font=("Consolas", 9))

        self.root.after(100, self._update_mixer)

    # ### Log Analyzer Tab #################################################

    def _build_logs_tab(self, parent):
        parent.configure(bg=Theme.BG_DARK)

        tk.Label(parent, text="Log Analyzer", bg=Theme.BG_DARK, fg=Theme.CYAN,
                 font=("Consolas", 14, "bold")).pack(pady=10)

        btn_frame = tk.Frame(parent, bg=Theme.BG_DARK)
        btn_frame.pack(fill=tk.X, padx=20, pady=5)

        ttk.Button(btn_frame, text="Open .bin Log", command=self._log_open).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Export CSV", command=self._log_export_csv).pack(side=tk.LEFT, padx=5)

        self.log_info = tk.Label(parent, text="No log loaded", bg=Theme.BG_DARK,
                                 fg=Theme.TEXT2, font=("Consolas", 9))
        self.log_info.pack(pady=5)

        self.log_channel_var = tk.StringVar(value="accel_x")
        channels = ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
                     "mag_x", "mag_y", "mag_z", "roll", "pitch", "yaw",
                     "voltage", "current", "rc1", "rc2", "rc3", "rc4",
                     "m1", "m2", "m3", "m4"]
        ttk.OptionMenu(parent, self.log_channel_var, channels[0], *channels).pack(pady=5)

        if HAS_MATPLOTLIB:
            self.log_fig = Figure(figsize=(8, 4), dpi=80, facecolor=Theme.BG_DARK)
            self.log_ax = self.log_fig.add_subplot(1, 1, 1)
            self.log_ax.set_facecolor(Theme.BG_CARD)
            self.log_ax.tick_params(colors=Theme.TEXT3, labelsize=7)
            self.log_ax.grid(True, alpha=0.3)
            for spine in self.log_ax.spines.values():
                spine.set_color(Theme.BORDER)
            self.log_canvas = FigureCanvasTkAgg(self.log_fig, master=parent)
            self.log_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        else:
            tk.Label(parent, text="matplotlib not available", bg=Theme.BG_DARK,
                     fg=Theme.RED).pack(expand=True)

        self._log_data = {}

    def _log_open(self):
        path = filedialog.askopenfilename(filetypes=[("Binary Log", "*.bin"), ("All", "*.*")])
        if not path:
            return
        try:
            with open(path, "rb") as f:
                raw = f.read()

            records = []
            i = 0
            while i < len(raw) - 1:
                if raw[i] == 0xA3 and raw[i + 1] == 0x95:
                    if i + 5 <= len(raw):
                        ts = struct.unpack("<I", raw[i + 2:i + 6])[0]
                        rec_len = 63
                        if i + 6 + rec_len <= len(raw):
                            rec_data = raw[i + 6:i + 6 + rec_len]
                            records.append((ts, rec_data))
                            i += 6 + rec_len
                        else:
                            break
                    else:
                        break
                else:
                    i += 1

            self._log_data = {
                "time_ms": [],
                "accel_x": [], "accel_y": [], "accel_z": [],
                "gyro_x": [], "gyro_y": [], "gyro_z": [],
                "mag_x": [], "mag_y": [], "mag_z": [],
                "roll": [], "pitch": [], "yaw": [],
                "rc1": [], "rc2": [], "rc3": [], "rc4": [],
                "m1": [], "m2": [], "m3": [], "m4": [],
                "voltage": [], "current": [],
            }

            for ts, rec in records:
                self._log_data["time_ms"].append(ts)
                off = 0
                ax, ay, az = struct.unpack("<hhh", rec[off:off+6]); off += 6
                gx, gy, gz = struct.unpack("<hhh", rec[off:off+6]); off += 6
                mx, my, mz = struct.unpack("<hhh", rec[off:off+6]); off += 6
                self._log_data["accel_x"].append(ax / 1000.0)
                self._log_data["accel_y"].append(ay / 1000.0)
                self._log_data["accel_z"].append(az / 1000.0)
                self._log_data["gyro_x"].append(math.degrees(gx / 1000.0))
                self._log_data["gyro_y"].append(math.degrees(gy / 1000.0))
                self._log_data["gyro_z"].append(math.degrees(gz / 1000.0))
                self._log_data["mag_x"].append(mx / 1000.0)
                self._log_data["mag_y"].append(my / 1000.0)
                self._log_data["mag_z"].append(mz / 1000.0)
                r, p, y = struct.unpack("<fff", rec[off:off+12]); off += 12
                self._log_data["roll"].append(math.degrees(r))
                self._log_data["pitch"].append(math.degrees(p))
                self._log_data["yaw"].append(math.degrees(y))
                for ch_idx in range(4):
                    ch_val = struct.unpack("<H", rec[off:off+2])[0]; off += 2
                    self._log_data[f"rc{ch_idx+1}"].append(ch_val)
                for m_idx in range(4):
                    m_val = struct.unpack("<H", rec[off:off+2])[0]; off += 2
                    self._log_data[f"m{m_idx+1}"].append(m_val)
                v, c = struct.unpack("<hh", rec[off:off+4]); off += 4
                self._log_data["voltage"].append(v / 100.0)
                self._log_data["current"].append(c / 100.0)

            total = len(self._log_data.get("time_ms", []))
            self.log_info.config(text=f"Loaded: {path} - {total} records")
            self._log(f"Log loaded: {total} records")
            self._plot_log_channel()

        except Exception as e:
            self._log(f"Log open error: {e}")

    def _plot_log_channel(self):
        if not HAS_MATPLOTLIB:
            return
        ch = self.log_channel_var.get()
        data = self._log_data.get(ch, [])
        time_ms = self._log_data.get("time_ms", [])
        self.log_ax.clear()
        self.log_ax.set_facecolor(Theme.BG_CARD)
        self.log_ax.tick_params(colors=Theme.TEXT3, labelsize=7)
        self.log_ax.grid(True, alpha=0.3)
        for spine in self.log_ax.spines.values():
            spine.set_color(Theme.BORDER)
        if data:
            if time_ms and len(time_ms) == len(data):
                time_s = [(t - time_ms[0]) / 1000.0 for t in time_ms]
                self.log_ax.plot(time_s, data, color=Theme.CYAN, linewidth=0.8)
                self.log_ax.set_xlabel("Time (s)", color=Theme.TEXT2, fontsize=8)
            else:
                self.log_ax.plot(data, color=Theme.CYAN, linewidth=0.8)
            self.log_ax.set_title(ch, color=Theme.TEXT, fontsize=10)
        self.log_fig.canvas.draw_idle()

    def _log_export_csv(self):
        if not self._log_data:
            self._log("No log data to export")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                             filetypes=[("CSV", "*.csv")])
        if path:
            try:
                max_len = max((len(v) for v in self._log_data.values()), default=0)
                with open(path, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=list(self._log_data.keys()))
                    writer.writeheader()
                    for i in range(max_len):
                        row = {}
                        for k, v in self._log_data.items():
                            row[k] = v[i] if i < len(v) else ""
                        writer.writerow(row)
                self._log(f"Exported to {path}")
            except Exception as e:
                self._log(f"Export error: {e}")

    # ### Close ############################################################

    def _on_close(self):
        self.running = False
        self.reading = False
        self.conn.close()
        if self.recording and self.record_file:
            self.record_file.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ### Entry Point ##############################################################

if __name__ == "__main__":
    app = GroundStation()
    app.run()

