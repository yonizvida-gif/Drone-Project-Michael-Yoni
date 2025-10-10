# gui.py
import sys
import json
import math
import asyncio
from time import strftime

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QProgressBar, QFrame, QPushButton, QLineEdit, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal, QObject, QRectF, QPointF
from PyQt5.QtGui import QPixmap, QPainter, QPen, QBrush, QColor, QTransform, QPolygonF

import websockets


# ========================= חלון התחברות =========================
class ConnectWindow(QWidget):
    connected = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Connect")
        self.setGeometry(300, 300, 320, 120)
        self.setStyleSheet("""
            QWidget { background-color: #0a1a2f; color: #aef1ff; font-family: Arial; font-size: 14px; }
            QLabel  { font-size: 16px; padding-bottom: 6px; }
            QLineEdit { background-color: #1e1e1e; border: 2px solid #1e90ff; border-radius: 6px; padding: 6px; color: #aef1ff; }
            QPushButton { background-color: #1e90ff; color: white; border-radius: 8px; padding: 8px; font-size: 16px; font-weight: bold; }
        """)
        layout = QVBoxLayout()
        self.label = QLabel("Enter IP address:")
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("e.g. 192.168.1.111")
        self.connect_btn = QPushButton("Connect")
        layout.addWidget(self.label)
        layout.addWidget(self.ip_input)
        layout.addWidget(self.connect_btn)
        self.setLayout(layout)
        self.connect_btn.clicked.connect(self.on_connect)

    def on_connect(self):
        ip = self.ip_input.text().strip()
        if ip.count('.') != 3:
            QMessageBox.warning(self, "Invalid IP", "Please enter a valid IP (e.g. 192.168.1.111)")
            return
        self.connected.emit(f"ws://{ip}:80")
        self.close()


# ========================= Worker ל-WebSocket =========================
class WebSocketWorker(QObject):
    data_received = pyqtSignal(dict)
    connection_closed = pyqtSignal()
    connection_error = pyqtSignal(str)
    connection_opened = pyqtSignal()

    def __init__(self, uri):
        super().__init__()
        self.uri = uri
        self._running = True
        self.websocket = None

    async def listen(self):
        try:
            async with websockets.connect(self.uri) as websocket:
                self.websocket = websocket
                self.connection_opened.emit()
                while self._running:
                    msg = await websocket.recv()

                    # 1) JSON
                    try:
                        data = json.loads(msg)
                        if isinstance(data, dict):
                            self.data_received.emit(data)
                            continue
                    except Exception:
                        pass

                    # 2) CSV "throttle,x,y[,temp][,battery]"
                    parts = [p.strip() for p in msg.split(",")]
                    if len(parts) >= 3:
                        payload = {}
                        try:
                            payload["throttle"] = int(float(parts[0]))
                        except Exception:
                            pass
                        try:
                            payload["x"] = int(float(parts[1]))
                            payload["y"] = int(float(parts[2]))
                        except Exception:
                            pass
                        if len(parts) >= 4:
                            try:
                                payload["temp"] = float(parts[3])
                            except Exception:
                                pass
                        if len(parts) >= 5:
                            try:
                                payload["battery"] = int(round(float(parts[4])))
                            except Exception:
                                pass

                        if payload:
                            self.data_received.emit(payload)
                            continue

                    # 3) רק מספר = throttle
                    try:
                        t = int(msg.strip())
                        self.data_received.emit({"throttle": t})
                    except Exception:
                        pass
        except Exception as e:
            self.connection_error.emit(str(e))
        finally:
            self.connection_closed.emit()

    def stop(self):
        self._running = False

    def run(self):
        asyncio.run(self.listen())

    def send_command(self, command: dict):
        async def _send():
            if self.websocket:
                try:
                    await self.websocket.send(json.dumps(command))
                except Exception:
                    pass
        asyncio.run(_send())


# ========================= רכיב אנימציה (רחפן X-Frame) =========================
class FancyDroneView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(260)
        self.setStyleSheet("background-color: #081522;")

        # מצב
        self.roll_deg = 0.0     # x
        self.pitch_deg = 0.0    # y
        self.throttle_pct = 0   # 0..100
        self._prop_angle = 0.0  # אנימציית פרופים

        # שיפוע תצוגה קבוע קדימה (תחושה 3D גם ב-0,0)
        self.DISPLAY_TILT_PITCH = 10.0

        # טיימר פריימים
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._tick)
        self.spin_timer.start(16)  # ~60fps

        # צבעים/עטים
        self.grid_color = QColor("#0f2c4a")
        self.grid_line = QPen(QColor("#134266")); self.grid_line.setWidth(1)
        self.horizon_color = QColor("#0e2036")
        self.drone_outline = QPen(QColor("#7fd3ff")); self.drone_outline.setWidth(2)
        self.body_color = QColor("#1aa3ff")
        self.arm_color = QColor("#b5f0ff")
        self.prop_color = QColor("#00e0c6")
        self.shadow_color = QColor(0, 0, 0, 60)

        # מקדמי השפעה (כיוונון)
        self.K_PITCH = 1.2      # פיקסל למעלת pitch (לשינוי גבהי פרופים)
        self.K_ROLL  = 1.2      # פיקסל למעלת roll
        self.K_BODY_PITCH = 0.6 # עיוות גוף
        self.K_BODY_ROLL  = 0.6 # עיוות גוף

    def _tick(self):
        if self.throttle_pct > 0:
            speed = 2.0 + 10.0 * ((self.throttle_pct / 100.0) ** 1.2)
            self._prop_angle = (self._prop_angle + speed) % 360.0
        self.update()

    def setAngles(self, roll_deg: float, pitch_deg: float):
        self.roll_deg  = max(-50.0, min(50.0, float(roll_deg)))
        self.pitch_deg = max(-50.0, min(50.0, float(pitch_deg)))

    def setThrottle(self, pct: int):
        self.throttle_pct = max(0, min(100, int(pct)))

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHints(QPainter.Antialiasing | QPainter.HighQualityAntialiasing)
        w, h = self.width(), self.height()
        cx, cy = w * 0.5, h * 0.6
        p.translate(cx, cy)
        self._draw_floor(p, w, h)
        self._draw_drone_xframe(p)

    def _draw_floor(self, p: QPainter, w: int, h: int):
        """ציור רשת קרקע תלת-ממדית."""
        p.save()
        # pitch "יעיל" לרצפה (כולל הטיה קבועה קדימה)
        eff_pitch = self.pitch_deg + self.DISPLAY_TILT_PITCH
        persp = QTransform()
        persp.shear(0.0, -eff_pitch * 0.004)
        persp.scale(1.0, 0.6)
        p.setTransform(persp, True)
        p.setPen(self.grid_line)

        # קווים אופקיים
        y = 0
        spacing = 10
        for _ in range(18):
            p.drawLine(int(-w), int(y), int(w), int(y))
            y += spacing
            spacing = int(spacing * 1.18)
            if y > h:
                break
        p.restore()

    def _draw_body_polygon(self, p: QPainter, bw=92, bh=40):
        """גוף כמצולע שמתעוות לפי roll/pitch (חזית = למעלה)."""
        half_w, half_h = bw / 2, bh / 2

        eff_pitch = self.pitch_deg + self.DISPLAY_TILT_PITCH
        dp = self.K_BODY_PITCH * eff_pitch
        dr = self.K_BODY_ROLL  * self.roll_deg

        y_top    = -half_h + dp        # חזית יורדת כש pitch>0 (כולל bias)
        y_bottom =  half_h - dp        # אחור עולה כש pitch>0
        y_left_shift  = -dr            # roll>0 => שמאל עולה
        y_right_shift =  dr            # roll>0 => ימין יורד

        tl = QPointF(-half_w, y_top    + y_left_shift)
        tr = QPointF( half_w, y_top    + y_right_shift)
        br = QPointF( half_w, y_bottom + y_right_shift)
        bl = QPointF(-half_w, y_bottom + y_left_shift)

        p.setPen(self.drone_outline)
        p.setBrush(QBrush(self.body_color))
        p.drawPolygon(QPolygonF([tl, tr, br, bl]))

        # חץ קדימה
        p.setBrush(QBrush(QColor("#ffd24d"))); p.setPen(Qt.NoPen)
        arrow = QPolygonF([QPointF(0, y_top - 12),
                           QPointF(-12, y_top + 8),
                           QPointF(12,  y_top + 8)])
        p.drawPolygon(arrow)

    def _draw_arm(self, p: QPainter, x: float, y: float, width: float, color: QColor):
        angle = math.degrees(math.atan2(y, x))
        length = math.hypot(x, y)
        p.save()
        p.rotate(angle)
        p.setBrush(QBrush(color))
        p.setPen(self.drone_outline)
        p.drawRoundedRect(QRectF(0, -width/2, length, width), 6, 6)
        p.restore()

    def _draw_prop(self, p: QPainter, pos: QPointF, prop_r: float, hub_r: float):
        p.save(); p.translate(pos)
        # צל
        p.setBrush(QBrush(QColor(0, 0, 0, 55))); p.setPen(Qt.NoPen)
        p.drawEllipse(QRectF(-prop_r, prop_r*0.65, prop_r*2, prop_r*0.65))
        # blur אם יש מהירות
        if self.throttle_pct > 0:
            blur_alpha = min(140, 50 + int(self.throttle_pct * 1.3))
            p.setBrush(QBrush(QColor(0, 255, 208, blur_alpha)))
            p.drawEllipse(QRectF(-prop_r*1.25, -prop_r*0.55, prop_r*2.5, prop_r*1.1))
        # להבים
        p.rotate(self._prop_angle)
        p.setBrush(QBrush(self.prop_color))
        blade_len = prop_r * 1.35; blade_w = 4
        for k in range(8):
            p.save(); p.rotate(k * 45)
            p.drawRoundedRect(QRectF(-blade_w/2, -blade_len, blade_w, blade_len), 2, 2)
            p.restore()
        # Hub
        p.setBrush(QBrush(QColor("#e6fbff")))
        p.drawEllipse(QRectF(-hub_r, -hub_r, hub_r*2, hub_r*2))
        p.restore()

    def _draw_drone_xframe(self, p: QPainter):
        """תצורת X: קדמי-שמאלי, קדמי-ימני, אחורי-שמאלי, אחורי-ימני."""
        p.save()

        arm_len = 74
        arm_w   = 12
        prop_r  = 16
        hub_r   = 4

        # בסיס אלכסוני
        d = int(arm_len * 0.75)

        # נקודות בסיס (x,y): קדמי= y שלילי (למעלה)
        base_FL = QPointF(-d, -d)   # Front-Left
        base_FR = QPointF( d, -d)   # Front-Right
        base_BL = QPointF(-d,  d)   # Back-Left
        base_BR = QPointF( d,  d)   # Back-Right

        # pitch "יעיל" עבור פרופים/זרועות
        eff_pitch = self.pitch_deg + self.DISPLAY_TILT_PITCH

        def apply_offsets(base_pt: QPointF, front: int, right: int) -> QPointF:
            # הזחה על ציר Y כדי להמחיש "מי נמוך יותר" + bias קדימה
            y_off = self.K_PITCH * eff_pitch * front + self.K_ROLL * self.roll_deg * right
            return QPointF(base_pt.x(), base_pt.y() + y_off)

        # pitch: front=+1, back=-1 | roll: right=+1, left=-1
        FL = apply_offsets(base_FL, front=+1, right=-1)
        FR = apply_offsets(base_FR, front=+1, right=+1)
        BL = apply_offsets(base_BL, front=-1, right=-1)
        BR = apply_offsets(base_BR, front=-1, right=+1)

        # צל גוף
        p.setPen(Qt.NoPen); p.setBrush(QBrush(self.shadow_color))
        p.drawEllipse(QRectF(-34, 18, 68, 14))

        # גוף (מצולע מתעוות לפי eff_pitch)
        self._draw_body_polygon(p, bw=92, bh=40)

        # זרועות אלכסוניות
        self._draw_arm(p, FL.x(), FL.y(), arm_w, self.arm_color)
        self._draw_arm(p, FR.x(), FR.y(), arm_w, self.arm_color)
        self._draw_arm(p, BL.x(), BL.y(), arm_w, self.arm_color)
        self._draw_arm(p, BR.x(), BR.y(), arm_w, self.arm_color)

        # פרופים
        self._draw_prop(p, FL, prop_r, hub_r)
        self._draw_prop(p, FR, prop_r, hub_r)
        self._draw_prop(p, BL, prop_r, hub_r)
        self._draw_prop(p, BR, prop_r, hub_r)

        p.restore()


# ========================= המסך הראשי =========================
class DroneCockpit(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Cockpit")
        self.setGeometry(100, 100, 640, 440)
        self.setStyleSheet("""
            QWidget { background-color: #0a1a2f; color: #aef1ff; font-family: Arial; }
            QLabel  { font-size: 14px; padding: 4px; }
            QPushButton { background-color: #1e90ff; color: white; border-radius: 8px; padding: 6px; font-size: 14px; font-weight: bold; }
        """)
        self.ws_worker = None
        self.main_app = None
        self._build_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_clock)
        self.timer.start(1000)

    def set_ws_worker(self, worker):
        self.ws_worker = worker

    def set_main_app(self, app):
        self.main_app = app

    def _box(self, title, widget, show=True):
        f = QFrame(); f.setStyleSheet("border: 1px solid #1e90ff; border-radius: 6px;")
        v = QVBoxLayout()
        if show:
            l = QLabel(title); l.setAlignment(Qt.AlignCenter)
            v.addWidget(l)
        v.addWidget(widget)
        f.setLayout(v)
        return f

    def _set_light_color(self, color_name: str):
        # red / yellow / green
        self.status_light.setStyleSheet(f"background-color: {color_name}; border-radius: 10px;")

    def _build_throttle_box(self) -> QFrame:
        self.throttle_percent_label = QLabel("Throttle: 0%")
        self.throttle_percent_label.setAlignment(Qt.AlignCenter)

        self.throttle_bar = QProgressBar()
        self.throttle_bar.setOrientation(Qt.Vertical)
        self.throttle_bar.setRange(0, 100)
        self.throttle_bar.setValue(0)
        self.throttle_bar.setTextVisible(False)
        self.throttle_bar.setStyleSheet("""
            QProgressBar {border: 1px solid #555; border-radius: 6px; background-color: #1e1e1e;}
            QProgressBar::chunk {background-color: #ff6600;}
        """)

        self.throttle_value_label = QLabel("1000")
        self.throttle_value_label.setAlignment(Qt.AlignCenter)

        box = QFrame()
        v = QVBoxLayout()
        v.addWidget(self.throttle_percent_label)
        v.addWidget(self.throttle_bar, alignment=Qt.AlignHCenter)
        v.addWidget(self.throttle_value_label, alignment=Qt.AlignHCenter)
        box.setLayout(v)
        box.setStyleSheet("border: 1px solid #1e90ff; border-radius: 6px;")
        return box

    def _build_ui(self):
        layout = QVBoxLayout()

        # שורת מצב חיבור עם נורה
        top_status_layout = QHBoxLayout()
        self.status_light = QLabel()
        self.status_light.setFixedSize(20, 20)
        self._set_light_color("red")
        self.status_label = QLabel("Disconnected")
        self.reconnect_button = QPushButton("Reconnect")
        self.reconnect_button.setFixedWidth(100)
        self.reconnect_button.clicked.connect(self.handle_reconnect)
        self.reconnect_button.setStyleSheet("""
            QPushButton { background-color: #444; color: #aef1ff; border: 1px solid #1e90ff; border-radius: 6px; padding: 4px; font-size: 13px; }
            QPushButton:hover { background-color: #1e90ff; color: white; }
        """)
        top_status_layout.addWidget(self.status_light)
        top_status_layout.addWidget(self.status_label)
        top_status_layout.addStretch()
        top_status_layout.addWidget(self.reconnect_button)
        layout.addLayout(top_status_layout)

        # שורה עליונה — Temp + Roll/Pitch + כפתור
        row1 = QHBoxLayout()
        self.temp_label = QLabel("--.- °C")
        self.angles_label = QLabel("roll: 0.0°\npitch: 0.0°")
        self.button_status = QPushButton("CAM")
        self.button_status.clicked.connect(self.send_button_command)
        row1.addWidget(self._box("Temp", self.temp_label))
        row1.addWidget(self._box("Desired Angles", self.angles_label))
        row1.addWidget(self._box("", self.button_status, False))

        # שורה שניה: אנימציה + מד טרוטל
        row2 = QHBoxLayout()
        self.drone_view = FancyDroneView()
        throttle_widget = self._build_throttle_box()
        frame_mid = QFrame()
        frame_mid.setStyleSheet("border: 1px solid #1e90ff; border-radius: 6px;")
        mid_layout = QHBoxLayout()
        mid_layout.addWidget(self.drone_view, stretch=1)
        mid_layout.addWidget(throttle_widget)
        frame_mid.setLayout(mid_layout)
        row2.addWidget(frame_mid)

        # שורה שלישית: בטרייה + שעה
        row3 = QHBoxLayout()
        self.battery = QProgressBar()
        self.battery.setValue(0)
        self.battery.setFormat("Battery 0%")
        self.battery_img = QLabel()
        pm = QPixmap("BATTERY25.png")
        if not pm.isNull():
            self.battery_img.setPixmap(pm.scaled(60, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.battery_img.setAlignment(Qt.AlignCenter)
        self.time_label = QLabel("--:--:--")
        row3.addWidget(self._box("BATTERY", self.battery))
        row3.addWidget(self.battery_img)
        row3.addWidget(self._box("TIME", self.time_label))

        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addLayout(row3)
        self.setLayout(layout)

    def update_clock(self):
        self.time_label.setText(strftime("%H:%M:%S"))

    def send_button_command(self):
        if self.ws_worker:
            self.ws_worker.send_command({"command": "TOGGLE_BUTTON"})

    def handle_reconnect(self):
        if self.main_app and self.main_app.last_ws_url:
            self.status_label.setText("Reconnecting...")
            self._set_light_color("yellow")
            self.main_app.restart_connection()

    def update_from_data(self, data: dict):
        # קיבלנו דאטה ⇒ יש חיבור
        self.status_label.setText("Connected")
        self._set_light_color("green")

        # זוויות (x,y מגיעים 1000..2000 => המרה ל-°)
        x_raw = data.get("x")
        y_raw = data.get("y")
        if x_raw is not None and y_raw is not None:
            try:
                x_int = int(float(str(x_raw).strip()))
                y_int = int(float(str(y_raw).strip()))
                roll_deg  = 0.10 * (x_int - 1500)
                pitch_deg = 0.10 * (y_int - 1500)
                self.angles_label.setText(f"roll: {roll_deg:.1f}°\npitch: {pitch_deg:.1f}°")
                self.drone_view.setAngles(roll_deg, pitch_deg)
            except Exception:
                pass

        # טרוטל
        t = data.get("throttle")
        if t is not None:
            try:
                t_int = int(float(str(t).strip()))
                t_int = max(1000, min(2000, t_int))
                percent = int(round((t_int - 1000) / 10.0))  # 1000->0, 2000->100
                self.throttle_bar.setValue(percent)
                self.throttle_percent_label.setText(f"Throttle: {percent}%")
                self.throttle_value_label.setText(str(t_int))
                self.drone_view.setThrottle(percent)
            except Exception:
                pass

        # בטרייה
        battery = data.get("battery")
        if battery is not None:
            try:
                b = max(0, min(100, int(float(battery))))
                self.battery.setValue(b)
                self.battery.setFormat(f"Battery {b}%")
                img_name = "BATTERY25.png"
                if b >= 90: img_name = "BATTERY100.png"
                elif b >= 65: img_name = "BATTERY75.png"
                elif b >= 40: img_name = "BATTERY50.png"
                pm = QPixmap(img_name)
                if not pm.isNull():
                    self.battery_img.setPixmap(pm.scaled(60, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            except Exception:
                pass

        # טמפ'
        temp_val = data.get("temp")
        if temp_val is not None:
            try:
                self.temp_label.setText(f"{float(temp_val):.1f} °C")
            except Exception:
                pass


# ========================= Main App =========================
class MainApp:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.connect_window = ConnectWindow()
        self.connect_window.connected.connect(self.start_connection)
        self.connect_window.show()
        self.cockpit_window = None
        self.ws_thread = None
        self.ws_worker = None
        self.last_ws_url = None

    def start_connection(self, ws_url):
        self.last_ws_url = ws_url

        if self.cockpit_window is None:
            self.cockpit_window = DroneCockpit()
            self.cockpit_window.set_main_app(self)
            self.cockpit_window.show()

        self.ws_worker = WebSocketWorker(ws_url)
        self.cockpit_window.set_ws_worker(self.ws_worker)

        self.ws_thread = QThread()
        self.ws_worker.moveToThread(self.ws_thread)
        self.ws_thread.started.connect(self.ws_worker.run)

        # אותות מצב חיבור → נורית
        self.ws_worker.connection_opened.connect(lambda: (self.cockpit_window._set_light_color("green"),
                                                          self.cockpit_window.status_label.setText("Connected")))
        self.ws_worker.connection_closed.connect(lambda: (self.cockpit_window._set_light_color("red"),
                                                          self.cockpit_window.status_label.setText("Disconnected")))
        self.ws_worker.connection_error.connect(lambda e: (self.cockpit_window._set_light_color("red"),
                                                           self.cockpit_window.status_label.setText("Disconnected")))

        self.ws_worker.data_received.connect(self.cockpit_window.update_from_data)

        self.ws_thread.start()

    def restart_connection(self):
        if self.ws_worker:
            self.ws_worker.stop()
        if self.ws_thread:
            self.ws_thread.quit()
            self.ws_thread.wait()

        # צהוב בזמן ניסיון חיבור מחדש
        self.cockpit_window._set_light_color("yellow")
        self.cockpit_window.status_label.setText("Reconnecting...")

        # להתחבר לאותה כתובת מחדש
        self.start_connection(self.last_ws_url)

    def run(self):
        sys.exit(self.app.exec_())


if __name__ == "__main__":
    app = MainApp()
    app.run()