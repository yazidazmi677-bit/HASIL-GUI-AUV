import sys
import math
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, 
    QHBoxLayout, QLabel, QFrame, QGridLayout, QProgressBar
)
from PyQt5.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF, 
    QPainterPath, QLinearGradient, QRadialGradient
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QPointF, QRectF, QTimer
from pymavlink import mavutil


# =========================================================
# 1. THREAD TELEMETRI (MAVLink COM19)
# =========================================================
class FullMavlinkWorker(QThread):
    telemetry_updated = pyqtSignal(dict)
    status_signal = pyqtSignal(str, bool)

    def __init__(self, port='COM19', baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.is_running = True
        
        self.data = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'alt_relative': 0.0,
            'alt_msl': 0.0,
            'climb_rate': 0.0,
            'groundspeed': 0.0,
            'airspeed': 0.0,
            'heading': 0,
            'throttle': 0,
            'lat': 0.0,
            'lon': 0.0,
            'satellites': 0,
            'gps_fix': "NO FIX",
            'voltage': 0.0,
            'current': 0.0,
            'battery_remaining': 0,
            'flight_mode': "MANUAL",
            'armed': False,
            'cpu_load': 0.0,
            'msg_rate': 0
        }

    def run(self):
        self.status_signal.emit(f"Connecting to {self.port}...", False)
        try:
            conn = mavutil.mavlink_connection(self.port, baud=self.baud, autoreconnect=True)
        except Exception as e:
            self.status_signal.emit(f"Serial Error: {e}", False)
            return

        connected = False
        while self.is_running and not connected:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.8)
            if msg:
                connected = True
                self.status_signal.emit(f"ONLINE | LINK SYS {conn.target_system}", True)
                break

        if not self.is_running:
            return

        for s in [
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION
        ]:
            conn.mav.request_data_stream_send(conn.target_system, conn.target_component, s, 20, 1)

        last_hb = time.time()
        pkt_count = 0
        last_rate_calc = time.time()

        while self.is_running:
            now = time.time()
            if now - last_hb > 1.0:
                conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now

            if now - last_rate_calc >= 1.0:
                self.data['msg_rate'] = pkt_count
                pkt_count = 0
                last_rate_calc = now

            msg = conn.recv_match(blocking=True, timeout=0.1)
            if not msg:
                continue

            pkt_count += 1
            mtype = msg.get_type()

            if mtype == 'HEARTBEAT':
                try:
                    self.data['flight_mode'] = mavutil.mode_string_v10(msg)
                except Exception:
                    self.data['flight_mode'] = f"MODE {msg.custom_mode}"
                self.data['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            elif mtype == 'ATTITUDE':
                self.data['roll'] = math.degrees(msg.roll)
                self.data['pitch'] = math.degrees(msg.pitch)
                yaw = math.degrees(msg.yaw)
                self.data['yaw'] = yaw if yaw >= 0 else yaw + 360

            elif mtype == 'VFR_HUD':
                self.data['airspeed'] = msg.airspeed
                self.data['groundspeed'] = msg.groundspeed
                self.data['heading'] = msg.heading
                self.data['throttle'] = msg.throttle
                self.data['alt_msl'] = msg.alt
                self.data['climb_rate'] = msg.climb

            elif mtype == 'GLOBAL_POSITION_INT':
                self.data['lat'] = msg.lat / 1e7
                self.data['lon'] = msg.lon / 1e7
                self.data['alt_relative'] = msg.relative_alt / 1000.0

            elif mtype == 'GPS_RAW_INT':
                self.data['satellites'] = msg.satellites_visible
                fixes = {0: "NO FIX", 1: "NO FIX", 2: "2D FIX", 3: "3D FIX", 4: "DGPS", 5: "RTK FLT", 6: "RTK FIX"}
                self.data['gps_fix'] = fixes.get(msg.fix_type, "GPS")

            elif mtype == 'SYS_STATUS':
                self.data['voltage'] = msg.voltage_battery / 1000.0
                self.data['current'] = msg.current_battery / 100.0
                self.data['battery_remaining'] = msg.battery_remaining
                self.data['cpu_load'] = msg.load / 10.0

            self.telemetry_updated.emit(self.data)

    def stop(self):
        self.is_running = False
        self.wait()


# =========================================================
# 2. WIDGET: PRIMARY FLIGHT DISPLAY (PFD + TAPES)
# =========================================================
class TacticalPFDWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.alt = 0.0
        self.climb = 0.0
        self.setMinimumSize(480, 360)

    def update_pfd(self, roll, pitch, yaw, speed, alt, climb):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.speed = speed
        self.alt = alt
        self.climb = climb
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0

        # Background Tactical Dark Frame
        p.fillRect(0, 0, w, h, QColor(8, 12, 19))

        # --- 1. HORIZON BOLA TENGAH ---
        pfd_w, pfd_h = w - 160, h - 50
        pfd_x, pfd_y = 80, 40
        horizon_rect = QRectF(pfd_x, pfd_y, pfd_w, pfd_h)

        p.save()
        clip = QPainterPath()
        clip.addRoundedRect(horizon_rect, 10, 10)
        p.setClipPath(clip)

        p.translate(cx, cy + 10)
        p.rotate(self.roll)  # Sumbu X sudah sinkron
        p.translate(0, self.pitch * 3.8)

        # Sky Gradient & Ground
        grad_sky = QLinearGradient(0, -pfd_h * 2, 0, 0)
        grad_sky.setColorAt(0, QColor(2, 60, 110))
        grad_sky.setColorAt(1, QColor(0, 140, 200))
        p.fillRect(int(-w * 2), int(-h * 3), int(w * 4), int(h * 3), grad_sky)

        grad_gnd = QLinearGradient(0, 0, 0, pfd_h * 2)
        grad_gnd.setColorAt(0, QColor(105, 50, 15))
        grad_gnd.setColorAt(1, QColor(45, 20, 5))
        p.fillRect(int(-w * 2), 0, int(w * 4), int(h * 3), grad_gnd)

        # Garis Horizon
        p.setPen(QPen(QColor(255, 255, 255), 2.5))
        p.drawLine(int(-w * 2), 0, int(w * 2), 0)

        # Pitch Ladder Baris
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        for deg in range(-50, 60, 10):
            if deg == 0:
                continue
            y_pos = -deg * 3.8
            span = 45 if abs(deg) % 20 == 0 else 25
            p.setPen(QPen(QColor(255, 255, 255, 230), 1.8))
            p.drawLine(int(-span), int(y_pos), int(-span + 12), int(y_pos))
            p.drawLine(int(span - 12), int(y_pos), int(span), int(y_pos))
            p.drawText(int(span + 6), int(y_pos + 4), f"{abs(deg)}")
            p.drawText(int(-span - 26), int(y_pos + 4), f"{abs(deg)}")

        p.restore()

        # Border PFD Center
        p.setPen(QPen(QColor(30, 41, 59), 2))
        p.drawRoundedRect(horizon_rect, 10, 10)

        # --- 2. RETICLE PESAWAT (MIL-SPEC HUD CROSSHAIR) ---
        p.setPen(QPen(QColor(250, 204, 21), 2.5))
        p.drawLine(int(cx - 45), int(cy + 10), int(cx - 15), int(cy + 10))
        p.drawLine(int(cx + 15), int(cy + 10), int(cx + 45), int(cy + 10))
        p.drawLine(int(cx - 15), int(cy + 10), int(cx - 15), int(cy + 18))
        p.drawLine(int(cx + 15), int(cy + 10), int(cx + 15), int(cy + 18))
        p.setBrush(QColor(250, 204, 21))
        p.drawEllipse(QPointF(cx, cy + 10), 3, 3)

        # --- 3. AIRSPEED TAPE (KIRI) ---
        tape_w = 70
        p.fillRect(5, 40, tape_w, int(pfd_h), QColor(15, 23, 42, 220))
        p.setPen(QPen(QColor(51, 65, 85), 1.5))
        p.drawRoundedRect(5, 40, tape_w, int(pfd_h), 6, 6)

        p.save()
        p.setClipRect(5, 40, tape_w, int(pfd_h))
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        for s in range(0, 150, 5):
            dy = (cy + 10) - (s - self.speed) * 4.0
            if 30 <= dy <= h - 10:
                p.setPen(QPen(QColor(148, 163, 184), 1.2))
                p.drawLine(tape_w - 5, int(dy), tape_w + 3, int(dy))
                if s % 10 == 0:
                    p.drawText(15, int(dy + 4), f"{s:02d}")
        p.restore()

        # Airspeed Digital Box (Tengah Tape Kiri)
        p.fillRect(5, int(cy - 4), tape_w + 5, 28, QColor(2, 132, 199))
        p.setPen(Qt.white)
        p.setFont(QFont("Consolas", 11, QFont.Bold))
        p.drawText(10, int(cy + 15), f"{self.speed:04.1f}m/s")

        # --- 4. ALTITUDE TAPE (KANAN) ---
        alt_x = w - tape_w - 5
        p.fillRect(int(alt_x), 40, tape_w, int(pfd_h), QColor(15, 23, 42, 220))
        p.setPen(QPen(QColor(51, 65, 85), 1.5))
        p.drawRoundedRect(int(alt_x), 40, tape_w, int(pfd_h), 6, 6)

        p.save()
        p.setClipRect(int(alt_x), 40, tape_w, int(pfd_h))
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        for a in range(-50, 500, 5):
            dy = (cy + 10) - (a - self.alt) * 3.5
            if 30 <= dy <= h - 10:
                p.setPen(QPen(QColor(148, 163, 184), 1.2))
                p.drawLine(int(alt_x + 2), int(dy), int(alt_x + 10), int(dy))
                if a % 20 == 0:
                    p.drawText(int(alt_x + 15), int(dy + 4), f"{a:03d}m")
        p.restore()

        # Altitude Digital Box (Tengah Tape Kanan)
        p.fillRect(int(alt_x - 5), int(cy - 4), tape_w + 8, 28, QColor(16, 185, 129))
        p.setPen(Qt.white)
        p.setFont(QFont("Consolas", 11, QFont.Bold))
        p.drawText(int(alt_x), int(cy + 15), f"{self.alt:05.1f}m")

        # --- 5. HEADING TAPE (PITA DERAJAT ATAS) ---
        p.fillRect(80, 5, int(pfd_w), 30, QColor(15, 23, 42, 240))
        p.setPen(QPen(QColor(51, 65, 85), 1))
        p.drawRoundedRect(80, 5, int(pfd_w), 30, 4, 4)

        p.save()
        p.setClipRect(80, 5, int(pfd_w), 30)
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
        for d in range(-360, 720, 10):
            norm_deg = d % 360
            dx = cx + (norm_deg - self.yaw) * 4.0
            if 70 <= dx <= w - 70:
                p.setPen(QPen(QColor(239, 68, 68) if norm_deg == 0 else QColor(148, 163, 184), 1.2))
                p.drawLine(int(dx), 5, int(dx), 13)
                txt = cardinals.get(norm_deg, str(norm_deg // 10))
                p.drawText(int(dx - 5), 25, txt)
        p.restore()

        # Center Heading Marker (Segitiga Kuning)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(250, 204, 21))
        p.drawPolygon(QPolygonF([QPointF(cx, 34), QPointF(cx - 5, 27), QPointF(cx + 5, 27)]))

        # --- 6. VERTICAL SPEED (VSI) BAR DI SEBELAH TAPE KANAN ---
        vsi_x = w - 10
        climb_clamped = max(-5.0, min(5.0, self.climb))
        vsi_h = -climb_clamped * 10
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(56, 189, 248))
        p.drawRect(int(vsi_x - 3), int(cy + 10), 4, int(vsi_h))


# =========================================================
# 3. WIDGET: TACTICAL RADAR COMPASS 360
# =========================================================
class TacticalCompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.setMinimumSize(220, 220)

    def set_yaw(self, yaw):
        self.yaw = yaw
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        r = min(w, h) / 2.0 - 15

        # Tactical Radar Sweep Gradient Background
        rad_grad = QRadialGradient(cx, cy, r)
        rad_grad.setColorAt(0, QColor(15, 23, 42))
        rad_grad.setColorAt(0.85, QColor(10, 15, 26))
        rad_grad.setColorAt(1, QColor(2, 6, 23))
        p.setBrush(QBrush(rad_grad))
        p.setPen(QPen(QColor(30, 41, 59), 2))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Radar Grid Ring
        p.setPen(QPen(QColor(30, 58, 100), 1, Qt.DashLine))
        p.drawEllipse(QPointF(cx, cy), r * 0.65, r * 0.65)

        p.save()
        p.translate(cx, cy)
        p.rotate(-self.yaw)

        cardinals = {0: "N", 90: "E", 180: "S", 270: "W"}
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        for d in range(0, 360, 15):
            p.save()
            p.rotate(d)
            if d in cardinals:
                p.setPen(QPen(QColor(239, 68, 68) if d == 0 else Qt.white, 2))
                p.drawLine(0, int(-r), 0, int(-r + 10))
                p.drawText(-5, int(-r + 22), cardinals[d])
            else:
                p.setPen(QPen(QColor(71, 85, 105), 1))
                p.drawLine(0, int(-r), 0, int(-r + 6))
            p.restore()

        # Needle Drone Aircraft Top-Down
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(239, 68, 68))
        needle = QPolygonF([QPointF(0, -r + 28), QPointF(-6, -r + 45), QPointF(6, -r + 45)])
        p.drawPolygon(needle)
        p.restore()

        # Reticle Drone Tengah
        p.setPen(QPen(QColor(250, 204, 21), 2))
        p.drawEllipse(QPointF(cx, cy), 5, 5)
        p.drawLine(int(cx - 15), int(cy), int(cx + 15), int(cy))
        p.drawLine(int(cx), int(cy - 15), int(cx), int(cy + 15))

        # Digital Yaw Box
        p.fillRect(int(cx - 32), int(cy + r - 22), 64, 20, QColor(15, 23, 42, 230))
        p.setPen(QPen(QColor(56, 189, 248), 1))
        p.drawRect(int(cx - 32), int(cy + r - 22), 64, 20)
        p.setPen(Qt.white)
        p.setFont(QFont("Consolas", 10, QFont.Bold))
        p.drawText(int(cx - 24), int(cy + r - 8), f"{self.yaw:05.1f}°")


# =========================================================
# 4. MAIN AVIONICS COMMAND GCS
# =========================================================
class TacticalHUDWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PIXHAWK 6C // TACTICAL MISSION AVIONICS - COM19")
        self.resize(1280, 780)
        self.setStyleSheet("background-color: #020617;")

        self.start_time = time.time()

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(12, 10, 12, 10)
        root_layout.setSpacing(10)

        # ---------------- TOP TACTICAL BAR ----------------
        top_bar = QFrame()
        top_bar.setStyleSheet("""
            QFrame {
                background-color: #0f172a;
                border: 1px solid #1e293b;
                border-radius: 8px;
                padding: 6px 12px;
            }
        """)
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(8, 4, 8, 4)

        logo = QLabel("⚡ GCS TACTICAL AVIONICS [PX4/APM]")
        logo.setStyleSheet("color: #38bdf8; font-family: Consolas; font-size: 14px; font-weight: bold;")

        self.lbl_arm = QLabel("DISARMED")
        self.lbl_arm.setStyleSheet("background-color: #450a0a; color: #ef4444; border: 1px solid #dc2626; border-radius: 4px; padding: 2px 10px; font-weight: bold; font-family: Consolas;")

        self.lbl_mode = QLabel("MODE: MANUAL")
        self.lbl_mode.setStyleSheet("background-color: #1e293b; color: #facc15; border: 1px solid #ca8a04; border-radius: 4px; padding: 2px 10px; font-weight: bold; font-family: Consolas;")

        self.lbl_timer = QLabel("FLT TIME: 00:00")
        self.lbl_timer.setStyleSheet("color: #cbd5e1; font-family: Consolas; font-size: 12px;")

        self.lbl_rate = QLabel("RATE: 00 Hz")
        self.lbl_rate.setStyleSheet("color: #38bdf8; font-family: Consolas; font-size: 12px;")

        self.lbl_status = QLabel("SEARCHING COM19...")
        self.lbl_status.setStyleSheet("color: #f59e0b; font-weight: bold; font-size: 12px;")

        top_layout.addWidget(logo)
        top_layout.addSpacing(20)
        top_layout.addWidget(self.lbl_arm)
        top_layout.addWidget(self.lbl_mode)
        top_layout.addStretch()
        top_layout.addWidget(self.lbl_timer)
        top_layout.addSpacing(15)
        top_layout.addWidget(self.lbl_rate)
        top_layout.addSpacing(15)
        top_layout.addWidget(self.lbl_status)
        root_layout.addWidget(top_bar)

        # ------------- CENTER INSTRUMENT SUITE -------------
        center_layout = QHBoxLayout()
        center_layout.setSpacing(10)

        # Left / Center: Tactical HUD PFD
        self.pfd = TacticalPFDWidget()
        center_layout.addWidget(self.pfd, stretch=3)

        # Right: Compass + Quick Gauges Panel
        right_panel = QFrame()
        right_panel.setStyleSheet("background-color: #0b1120; border: 1px solid #1e293b; border-radius: 8px; padding: 10px;")
        rp_layout = QVBoxLayout(right_panel)
        rp_layout.setSpacing(10)

        rp_title = QLabel("TACTICAL SITUATION // RADAR")
        rp_title.setStyleSheet("color: #64748b; font-size: 11px; font-weight: bold; font-family: Consolas;")
        rp_layout.addWidget(rp_title, alignment=Qt.AlignCenter)

        self.compass = TacticalCompassWidget()
        rp_layout.addWidget(self.compass, alignment=Qt.AlignCenter)

        # System Bar Gauges (Battery, Throttle, CPU)
        self.bar_batt = self._create_prog_bar("BATTERY FUEL", "#10b981", rp_layout)
        self.bar_thr = self._create_prog_bar("THROTTLE", "#38bdf8", rp_layout)
        self.bar_cpu = self._create_prog_bar("PIXHAWK CPU LOAD", "#f59e0b", rp_layout)

        center_layout.addWidget(right_panel, stretch=1)
        root_layout.addLayout(center_layout)

        # ----------- BOTTOM TELEMETRY MATRIX CARDS ----------
        matrix_frame = QFrame()
        matrix_frame.setStyleSheet("background-color: #0b1120; border: 1px solid #1e293b; border-radius: 8px; padding: 8px;")
        grid = QGridLayout(matrix_frame)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        self.cards = {}
        telemetry_defs = [
            ("ROLL ANGLE", "0.0°", 0, 0),
            ("PITCH ANGLE", "0.0°", 0, 1),
            ("YAW HEADING", "000.0°", 0, 2),
            ("VERT SPEED", "0.00 m/s", 0, 3),
            ("VOLTAGE", "0.00 V", 0, 4),
            ("CURRENT", "0.00 A", 0, 5),

            ("GROUND SPEED", "0.00 m/s", 1, 0),
            ("AIR SPEED", "0.00 m/s", 1, 1),
            ("REL ALTITUDE", "0.0 m", 1, 2),
            ("BARO ALT MSL", "0.0 m", 1, 3),
            ("GPS FIX STAT", "NO FIX", 1, 4),
            ("SATELLITES", "0 SATS", 1, 5),
        ]

        for title, val, r, c in telemetry_defs:
            self.cards[title] = self._create_matrix_cell(title, val, grid, r, c)

        root_layout.addWidget(matrix_frame)
        self.setCentralWidget(root)

        # ---------------- TIMERS & WORKER -----------------
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)

        self.worker = FullMavlinkWorker(port='COM19', baud=115200)
        self.worker.telemetry_updated.connect(self.update_telemetry)
        self.worker.status_signal.connect(self.update_status)
        self.worker.start()

    def _create_prog_bar(self, label_text, color_hex, parent_layout):
        container = QWidget()
        lay = QVBoxLayout(container)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setSpacing(2)

        lbl = QLabel(f"{label_text}: 0%")
        lbl.setStyleSheet("color: #94a3b8; font-size: 10px; font-weight: bold; font-family: Consolas;")
        lay.addWidget(lbl)

        pbar = QProgressBar()
        pbar.setRange(0, 100)
        pbar.setValue(0)
        pbar.setTextVisible(False)
        pbar.setFixedHeight(8)
        pbar.setStyleSheet(f"""
            QProgressBar {{
                background-color: #1e293b;
                border: none;
                border-radius: 4px;
            }}
            QProgressBar::chunk {{
                background-color: {color_hex};
                border-radius: 4px;
            }}
        """)
        lay.addWidget(pbar)
        parent_layout.addWidget(container)
        return (pbar, lbl, label_text)

    def _create_matrix_cell(self, title, def_val, grid, r, c):
        f = QFrame()
        f.setStyleSheet("""
            QFrame {
                background-color: #0f172a;
                border: 1px solid #1e293b;
                border-radius: 6px;
                padding: 4px;
            }
        """)
        vb = QVBoxLayout(f)
        vb.setContentsMargins(6, 4, 6, 4)
        vb.setSpacing(2)
        t = QLabel(title)
        t.setStyleSheet("color: #64748b; font-size: 9px; font-weight: bold; font-family: Consolas;")
        v = QLabel(def_val)
        v.setStyleSheet("color: #38bdf8; font-size: 15px; font-weight: bold; font-family: Consolas;")
        vb.addWidget(t, alignment=Qt.AlignCenter)
        vb.addWidget(v, alignment=Qt.AlignCenter)
        grid.addWidget(f, r, c)
        return v

    def _update_clock(self):
        elapsed = int(time.time() - self.start_time)
        m, s = divmod(elapsed, 60)
        self.lbl_timer.setText(f"FLT TIME: {m:02d}:{s:02d}")

    def update_telemetry(self, data):
        # Update PFD & Compass
        self.pfd.update_pfd(
            data['roll'], data['pitch'], data['yaw'], 
            data['groundspeed'], data['alt_relative'], data['climb_rate']
        )
        self.compass.set_yaw(data['yaw'])

        # Header Status
        if data['armed']:
            self.lbl_arm.setText("ARMED")
            self.lbl_arm.setStyleSheet("background-color: #064e3b; color: #34d399; border: 1px solid #059669; border-radius: 4px; padding: 2px 10px; font-weight: bold; font-family: Consolas;")
        else:
            self.lbl_arm.setText("DISARMED")
            self.lbl_arm.setStyleSheet("background-color: #450a0a; color: #ef4444; border: 1px solid #dc2626; border-radius: 4px; padding: 2px 10px; font-weight: bold; font-family: Consolas;")

        self.lbl_mode.setText(f"MODE: {data['flight_mode'].upper()}")
        self.lbl_rate.setText(f"RATE: {data['msg_rate']:02d} Hz")

        # Update System Progress Bars
        batt_pct = int(max(0, min(100, data['battery_remaining'])))
        self.bar_batt[0].setValue(batt_pct)
        self.bar_batt[1].setText(f"{self.bar_batt[2]}: {batt_pct}%")

        thr_pct = int(max(0, min(100, data['throttle'])))
        self.bar_thr[0].setValue(thr_pct)
        self.bar_thr[1].setText(f"{self.bar_thr[2]}: {thr_pct}%")

        cpu_pct = int(max(0, min(100, data['cpu_load'])))
        self.bar_cpu[0].setValue(cpu_pct)
        self.bar_cpu[1].setText(f"{self.bar_cpu[2]}: {cpu_pct}%")

        # Update Matrix Readouts
        self.cards["ROLL ANGLE"].setText(f"{data['roll']:+06.1f}°")
        self.cards["PITCH ANGLE"].setText(f"{data['pitch']:+06.1f}°")
        self.cards["YAW HEADING"].setText(f"{data['yaw']:05.1f}°")
        self.cards["VERT SPEED"].setText(f"{data['climb_rate']:+.2f} m/s")
        self.cards["VOLTAGE"].setText(f"{data['voltage']:.2f} V")
        self.cards["CURRENT"].setText(f"{data['current']:.2f} A")

        self.cards["GROUND SPEED"].setText(f"{data['groundspeed']:.2f} m/s")
        self.cards["AIR SPEED"].setText(f"{data['airspeed']:.2f} m/s")
        self.cards["REL ALTITUDE"].setText(f"{data['alt_relative']:.2f} m")
        self.cards["BARO ALT MSL"].setText(f"{data['alt_msl']:.2f} m")
        self.cards["GPS FIX STAT"].setText(str(data['gps_fix']))
        self.cards["SATELLITES"].setText(f"{data['satellites']} SATS")

    def update_status(self, text, is_conn):
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet("color: #34d399; font-weight: bold;" if is_conn else "color: #f59e0b; font-weight: bold;")

    def closeEvent(self, event):
        self.worker.stop()
        event.accept()


# =========================================================
# 5. ENTRY POINT
# =========================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TacticalHUDWindow()
    win.show()
    sys.exit(app.exec_())