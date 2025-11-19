#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Turtlebot Control GUI (PyQt5 + WebSocket)
- Integrated team design
- WebSocket communication
- Multi-robot support
- Navigation2, QR, Collision Avoidance
- NO EMOJIS
"""

import sys
import json
import os
import datetime
import math

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QDialog,
    QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QGroupBox, QGridLayout
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

import websocket

try:
    from shared.config import ROBOTS, SERVER_IP, SERVER_PORT
except ImportError:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from shared.config import ROBOTS, SERVER_IP, SERVER_PORT

from gui.ui_main_window import Ui_video_label

class NavigationDialog(QDialog):
    """Dialog for Navigation Coordinates Input"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Move to Coordinate")
        self.setModal(True)
        self.setMinimumWidth(350)

        layout = QVBoxLayout()

        # Coordinate Input
        coord_group = QGroupBox("Target Coordinates")
        coord_layout = QGridLayout()

        coord_layout.addWidget(QLabel("X (m):"), 0, 0)
        self.x_input = QLineEdit("2.0")
        coord_layout.addWidget(self.x_input, 0, 1)

        coord_layout.addWidget(QLabel("Y (m):"), 1, 0)
        self.y_input = QLineEdit("1.0")
        coord_layout.addWidget(self.y_input, 1, 1)

        coord_layout.addWidget(QLabel("Yaw (deg):"), 2, 0)
        self.yaw_input = QLineEdit("0")
        coord_layout.addWidget(self.yaw_input, 2, 1)

        coord_group.setLayout(coord_layout)
        layout.addWidget(coord_group)

        # Buttons
        button_layout = QHBoxLayout()

        self.btn_move = QPushButton("Start")
        self.btn_move.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.btn_move.clicked.connect(self.accept)

        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.setStyleSheet("background-color: #f44336; color: white; padding: 10px;")
        self.btn_cancel.clicked.connect(self.reject)

        button_layout.addWidget(self.btn_move)
        button_layout.addWidget(self.btn_cancel)

        layout.addLayout(button_layout)
        self.setLayout(layout)

    def get_coordinates(self):
        """Returns entered coordinates"""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            yaw_deg = float(self.yaw_input.text())
            yaw_rad = yaw_deg * math.pi / 180.0
            return (x, y, yaw_rad)
        except ValueError:
            return None

class WebSocketThread(QThread):
    """WebSocket Communication Thread"""

    message_received = pyqtSignal(dict)
    connection_status = pyqtSignal(bool)
    # Signal now includes robot_id: (robot_id, image)
    camera_image_received = pyqtSignal(int, object)

    def __init__(self):
        super().__init__()
        self.running = False
        self.ws = None

    def run(self):
        uri = f"ws://{SERVER_IP}:{SERVER_PORT}/ws"

        self.ws = websocket.WebSocketApp(
            uri,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        self.running = True
        self.ws.run_forever()

    def on_open(self, ws):
        print("[WebSocket] Connected to Server")
        self.connection_status.emit(True)

    def on_message(self, ws, message):
        try:
            msg_dict = json.loads(message)
            msg_type = msg_dict.get('type')

            # Camera Image Handling
            if msg_type == 'camera_image':
                img_data = msg_dict.get('image_data')
                robot_id = msg_dict.get('robot_id')

                if img_data and robot_id is not None:
                    import numpy as np
                    import cv2

                    np_arr = np.frombuffer(bytes.fromhex(img_data), np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    # Emit robot_id along with image for filtering
                    self.camera_image_received.emit(robot_id, cv_image)
                return

            # General Message
            self.message_received.emit(msg_dict)

        except json.JSONDecodeError:
            print(f"[WebSocket] JSON Error: {message[:50]}...")
        except Exception as e:
            print(f"[WebSocket] Processing Error: {e}")

    def on_error(self, ws, error):
        print(f"[WebSocket] Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print("[WebSocket] Disconnected")
        self.connection_status.emit(False)
        self.running = False

    def send(self, message: dict):
        if self.ws and self.running:
            json_msg = json.dumps(message)
            self.ws.send(json_msg)

    def stop(self):
        self.running = False
        if self.ws:
            self.ws.close()

class RobotControlWindow(QMainWindow, Ui_video_label):
    """Main GUI Window"""

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Robot Setup
        self.robot_map = {}
        for robot_id, info in ROBOTS.items():
            display_name = f"{info['name']} (ID: {robot_id})"
            self.robot_map[display_name] = robot_id

        self.display_names = list(self.robot_map.keys())
        self.current_robot_id = list(ROBOTS.keys())[0] if ROBOTS else 1

        # Status Variables
        self.connected = False
        self.emergency_stop_active = False
        self.manual_mode_active = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.origin_offset_x = 0.0
        self.origin_offset_y = 0.0
        self.current_yaw = 0.0
        self.front_distance = float('inf')
        self.current_battery_voltage = 0.0
        self.max_linear_limit = 0.15

        # Velocity Control
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        # Trip Info
        self.total_distance = 0.0
        self.prev_x = None
        self.prev_y = None
        self.start_time = datetime.datetime.now()

        # States
        self.qr_active = False
        self.lidar_active = False
        self.is_navigating = False

        # Camera Frame
        self.current_cv_frame = None

        # Init
        self.init_ui()
        self.connect_to_server()

        # Timer (10Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(100)

        print("=" * 60)
        print("GUI Started")
        print(f"Server: {SERVER_IP}:{SERVER_PORT}")
        print("=" * 60)

    def init_ui(self):
        # Robot Selector
        if hasattr(self, 'combo_robot_select'):
            self.combo_robot_select.clear()
            self.combo_robot_select.addItems(self.display_names)
            self.combo_robot_select.activated[str].connect(self.on_robot_changed)

        # Direction Buttons
        self.btn_forward.pressed.connect(lambda: self.set_velocity(1.0, 0.0))
        self.btn_backward.pressed.connect(lambda: self.set_velocity(-1.0, 0.0))
        self.btn_left.pressed.connect(lambda: self.set_velocity(0.0, 0.8))
        self.btn_right.pressed.connect(lambda: self.set_velocity(0.0, -0.8))
        self.btn_stop.clicked.connect(self.set_stop_velocity)

        for btn in [self.btn_forward, self.btn_backward, self.btn_left, self.btn_right]:
            btn.released.connect(self.set_stop_velocity)

        # Mode Buttons
        self.btn_manual_mode.toggled.connect(self.on_manual_mode_toggled)
        self.btn_emergency_stop.toggled.connect(self.on_emergency_stop_toggled)

        # Feature Buttons
        if hasattr(self, 'btn_set_origin'):
            self.btn_set_origin.clicked.connect(self.on_set_origin_clicked)
        if hasattr(self, 'btn_return_origin'):
            self.btn_return_origin.clicked.connect(self.on_return_to_origin_clicked)
            self.btn_return_origin.setEnabled(False)
        if hasattr(self, 'btn_screenshot'):
            self.btn_screenshot.clicked.connect(self.on_screenshot_clicked)
        if hasattr(self, 'btn_reset_trip'):
            self.btn_reset_trip.clicked.connect(self.on_reset_trip)

        # Slider
        if hasattr(self, 'slider_speed'):
            self.slider_speed.valueChanged.connect(self.update_max_speed)
            self.update_max_speed(self.slider_speed.value())

        self.add_advanced_controls()

        # Init State
        self.btn_manual_mode.setChecked(False)
        self.on_manual_mode_toggled(False)

    def add_advanced_controls(self):
        if hasattr(self, 'centralwidget'):
            main_layout = self.centralwidget.layout()

            advanced_group = QGroupBox("Advanced Features")
            advanced_layout = QHBoxLayout()

            # Navigation
            self.btn_navigate = QPushButton("Move to Coord")
            self.btn_navigate.setMinimumHeight(35)
            self.btn_navigate.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
            self.btn_navigate.clicked.connect(self.on_navigate_clicked)
            advanced_layout.addWidget(self.btn_navigate)

            self.btn_cancel_nav = QPushButton("Cancel Move")
            self.btn_cancel_nav.setMinimumHeight(35)
            self.btn_cancel_nav.setStyleSheet("background-color: #FF9800; color: white;")
            self.btn_cancel_nav.clicked.connect(self.on_cancel_nav_clicked)
            advanced_layout.addWidget(self.btn_cancel_nav)

            # QR
            self.btn_qr_toggle = QPushButton("Start QR Scan")
            self.btn_qr_toggle.setMinimumHeight(35)
            self.btn_qr_toggle.setCheckable(True)
            self.btn_qr_toggle.toggled.connect(self.on_qr_toggled)
            advanced_layout.addWidget(self.btn_qr_toggle)

            # LiDAR
            self.btn_lidar_toggle = QPushButton("Start Collision Check")
            self.btn_lidar_toggle.setMinimumHeight(35)
            self.btn_lidar_toggle.setCheckable(True)
            self.btn_lidar_toggle.toggled.connect(self.on_lidar_toggled)
            advanced_layout.addWidget(self.btn_lidar_toggle)

            advanced_group.setLayout(advanced_layout)

            if main_layout:
                main_layout.addWidget(advanced_group, main_layout.rowCount(), 0, 1, 2)

        if hasattr(self, 'statusbar'):
            self.nav_status_label = QLabel("Nav: Idle")
            self.nav_status_label.setStyleSheet("color: white; padding: 5px;")
            self.statusbar.addPermanentWidget(self.nav_status_label)

            self.qr_status_label = QLabel("QR: OFF")
            self.qr_status_label.setStyleSheet("color: gray; padding: 5px;")
            self.statusbar.addPermanentWidget(self.qr_status_label)

            self.lidar_status_label = QLabel("Collision: OFF")
            self.lidar_status_label.setStyleSheet("color: gray; padding: 5px;")
            self.statusbar.addPermanentWidget(self.lidar_status_label)

    def connect_to_server(self):
        self.ws_thread = WebSocketThread()
        self.ws_thread.message_received.connect(self.on_websocket_message)
        self.ws_thread.connection_status.connect(self.on_connection_changed)
        self.ws_thread.camera_image_received.connect(self.update_camera_image)
        self.ws_thread.start()

    def on_robot_changed(self, display_name):
        robot_id = self.robot_map.get(display_name)
        if robot_id is None:
            return

        self.current_robot_id = robot_id

        # Request camera stream for the new robot
        if self.connected:
            self.start_camera_stream()

        self.on_reset_trip()
        self.set_stop_velocity()
        print(f"[Robot Switched] {display_name}")

    def start_camera_stream(self):
        if not self.connected:
            return

        self.ws_thread.send({
            "command": "camera_start",
            "robot_id": self.current_robot_id
        })

    def on_manual_mode_toggled(self, checked):
        if self.emergency_stop_active:
            self.btn_manual_mode.setChecked(not checked)
            return

        self.manual_mode_active = checked

        if checked:
            self.btn_manual_mode.setText('Manual Mode ON')
            self.set_buttons_enabled(True)
            if hasattr(self, 'btn_return_origin'): self.btn_return_origin.setEnabled(False)
            if hasattr(self, 'btn_navigate'): self.btn_navigate.setEnabled(False)
            self.setFocus()
        else:
            self.btn_manual_mode.setText('Auto Mode ON')
            self.set_buttons_enabled(False)
            if hasattr(self, 'btn_return_origin'): self.btn_return_origin.setEnabled(True)
            if hasattr(self, 'btn_navigate'): self.btn_navigate.setEnabled(True)

        self.set_stop_velocity()

    def on_emergency_stop_toggled(self, checked):
        self.emergency_stop_active = checked

        if checked:
            self.btn_emergency_stop.setText('Click to Release')
            self.set_buttons_enabled(False)
            self.btn_manual_mode.setEnabled(False)

            if hasattr(self, 'btn_return_origin'): self.btn_return_origin.setEnabled(False)
            if hasattr(self, 'btn_navigate'): self.btn_navigate.setEnabled(False)

            if hasattr(self, 'label_status_display'):
                self.label_status_display.setStyleSheet(
                    "background-color: #444; padding: 5px; border-radius: 3px; color: red; font-weight: bold;"
                )
        else:
            self.btn_emergency_stop.setText('Emergency Stop')
            self.btn_manual_mode.setEnabled(True)
            self.set_buttons_enabled(self.manual_mode_active)

            if hasattr(self, 'btn_return_origin'):
                self.btn_return_origin.setEnabled(not self.manual_mode_active)
            if hasattr(self, 'btn_navigate'):
                self.btn_navigate.setEnabled(not self.manual_mode_active)

            if hasattr(self, 'label_status_display'):
                self.label_status_display.setStyleSheet(
                    "background-color: #444; padding: 5px; border-radius: 3px; color: white;"
                )

        self.set_stop_velocity()

    def set_buttons_enabled(self, enabled):
        self.btn_forward.setEnabled(enabled)
        self.btn_backward.setEnabled(enabled)
        self.btn_left.setEnabled(enabled)
        self.btn_right.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)

    def set_velocity(self, linear_scale, angular):
        if not self.manual_mode_active or self.emergency_stop_active:
            return
        self.target_linear_vel = linear_scale * self.max_linear_limit
        self.target_angular_vel = angular

    def set_stop_velocity(self):
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

    def update_max_speed(self, value):
        self.max_linear_limit = value / 100.0
        if hasattr(self, 'label_max_speed_display'):
            self.label_max_speed_display.setText(f"{self.max_linear_limit:.2f} m/s")

    def update_loop(self):
        # Send velocity
        if self.connected and (self.manual_mode_active or self.emergency_stop_active):
            linear = 0.0 if self.emergency_stop_active else self.target_linear_vel
            angular = 0.0 if self.emergency_stop_active else self.target_angular_vel

            self.ws_thread.send({
                "command": "move",
                "robot_id": self.current_robot_id,
                "linear": float(linear),
                "angular": float(angular)
            })

        self.update_gui_labels()

    def update_gui_labels(self):
        if hasattr(self, 'label_linear_vel'):
            self.label_linear_vel.setText(f"Linear: {self.target_linear_vel:.2f} m/s")
        if hasattr(self, 'label_angular_vel'):
            self.label_angular_vel.setText(f"Angular: {self.target_angular_vel:.2f} rad/s")

        if hasattr(self, 'label_status_display'):
            dx = self.current_x - self.origin_offset_x
            dy = self.current_y - self.origin_offset_y
            self.label_status_display.setText(f"Pos: X: {dx:.2f} m | Y: {dy:.2f} m")

        if hasattr(self, 'label_trip_info'):
            elapsed = datetime.datetime.now() - self.start_time
            elapsed_str = str(elapsed).split('.')[0]
            self.label_trip_info.setText(f"Time {elapsed_str} | Dist {self.total_distance:.2f} m")

        # Battery
        if hasattr(self, 'label_battery'):
            v = self.current_battery_voltage
            if v > 0:
                MAX_V, MIN_V = 12.6, 11.0
                percentage = int((v - MIN_V) / (MAX_V - MIN_V) * 100)
                percentage = max(0, min(100, percentage))
                self.label_battery.setText(f"Battery: {percentage}% ({v:.1f}V)")

                color = "green" if percentage > 70 else "orange" if percentage > 30 else "red"
                self.label_battery.setStyleSheet(
                    f"background-color: #222; color: {color}; font-weight: bold; "
                    "border-radius: 10px; border: 1px solid #444; padding: 10px; font-size: 16pt;"
                )
            else:
                self.label_battery.setText("Battery: Waiting...")

        # Collision
        if hasattr(self, 'label_collision_warning'):
            if self.front_distance < 0.35:
                self.label_collision_warning.setText("!!! COLLISION WARNING !!!")
                self.label_collision_warning.setStyleSheet(
                    "background-color: #ffcccc; border: 1px solid red; color: red; font-weight: bold; padding: 5px;"
                )
            elif self.front_distance < 1.0:
                self.label_collision_warning.setText(f"Front Dist: {self.front_distance:.2f} m")
                self.label_collision_warning.setStyleSheet(
                    "background-color: #fff5cc; border: 1px solid orange; color: orange; font-weight: bold; padding: 5px;"
                )
            else:
                self.label_collision_warning.setText("Front Clear")
                self.label_collision_warning.setStyleSheet(
                    "background-color: #2E8B57; color: white; padding: 5px;"
                )

    def update_camera_image(self, robot_id, cv_image):
        """Updates camera image ONLY if robot_id matches selected robot"""

        # CRITICAL FIX: Filter images by robot_id
        if robot_id != self.current_robot_id:
            return

        import cv2
        self.current_cv_frame = cv_image

        if hasattr(self, 'label_camera_feed') and self.label_camera_feed.width() > 0:
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            scaled = pixmap.scaled(self.label_camera_feed.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.label_camera_feed.setPixmap(scaled)

    def on_websocket_message(self, msg: dict):
        msg_type = msg.get('type')
        robot_id = msg.get('robot_id')

        if robot_id and robot_id != self.current_robot_id:
            return

        if msg_type == 'position':
            data = msg.get('position', {})
            self.current_x = data.get('x', 0.0)
            self.current_y = data.get('y', 0.0)
            qz = data.get('qz', 0.0)
            qw = data.get('qw', 1.0)
            self.current_yaw = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)

            if self.prev_x is not None:
                dx = self.current_x - self.prev_x
                dy = self.current_y - self.prev_y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0.001:
                    self.total_distance += dist
            self.prev_x = self.current_x
            self.prev_y = self.current_y

        elif msg_type == 'scan':
            ranges = msg.get('ranges', [])
            if ranges:
                self.front_distance = ranges[0] if ranges[0] > 0 else float('inf')

        elif msg_type == 'battery':
            self.current_battery_voltage = msg.get('voltage', 0.0)

        elif msg_type == 'qr_detected':
            qr_text = msg.get('qr_text', '')
            if hasattr(self, 'qr_status_label'):
                self.qr_status_label.setText(f"QR: {qr_text}")
                self.qr_status_label.setStyleSheet("color: blue; font-weight: bold;")

        elif msg_type == 'nav_feedback':
            x = msg.get('x', 0.0)
            y = msg.get('y', 0.0)
            if hasattr(self, 'nav_status_label'):
                self.nav_status_label.setText(f"Nav: Moving ({x:.1f}, {y:.1f})")
                self.nav_status_label.setStyleSheet("color: yellow; font-weight: bold;")

        elif msg_type == 'nav_complete':
            self.is_navigating = False
            if hasattr(self, 'nav_status_label'):
                self.nav_status_label.setText("Nav: Reached!")
                self.nav_status_label.setStyleSheet("color: green; font-weight: bold;")

        elif msg_type == 'nav_failed':
            reason = msg.get('reason', '')
            self.is_navigating = False
            if hasattr(self, 'nav_status_label'):
                self.nav_status_label.setText(f"Nav: Failed ({reason})")
                self.nav_status_label.setStyleSheet("color: red; font-weight: bold;")

        elif msg_type == 'collision_warning':
            print(f"[Warning] {msg.get('message', '')}")

    def on_connection_changed(self, connected: bool):
        self.connected = connected
        if connected:
            print("[Conn] Connected")
            self.start_camera_stream()
            if hasattr(self, 'statusbar'):
                self.statusbar.showMessage("Connected", 3000)
        else:
            print("[Conn] Disconnected")
            if hasattr(self, 'statusbar'):
                self.statusbar.showMessage("Disconnected", 5000)

    def on_set_origin_clicked(self):
        self.origin_offset_x = self.current_x
        self.origin_offset_y = self.current_y
        if hasattr(self, 'label_status_display'):
            self.label_status_display.setText("Status: Origin Reset (0, 0)")
        if hasattr(self, 'statusbar'):
            self.statusbar.showMessage("Origin Set", 2000)

    def on_return_to_origin_clicked(self):
        if not self.connected:
            QMessageBox.warning(self, "Error", "Not connected!")
            return
        self.ws_thread.send({
            "command": "navigate_to",
            "robot_id": self.current_robot_id,
            "x": self.origin_offset_x,
            "y": self.origin_offset_y,
            "yaw": 0.0
        })
        self.is_navigating = True
        if hasattr(self, 'nav_status_label'):
            self.nav_status_label.setText("Nav: Returning Home...")
            self.nav_status_label.setStyleSheet("color: cyan; font-weight: bold;")

    def on_navigate_clicked(self):
        if not self.connected:
            QMessageBox.warning(self, "Error", "Not connected!")
            return
        dialog = NavigationDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            coords = dialog.get_coordinates()
            if coords:
                x, y, yaw = coords
                self.ws_thread.send({
                    "command": "navigate_to",
                    "robot_id": self.current_robot_id,
                    "x": x,
                    "y": y,
                    "yaw": yaw
                })
                self.is_navigating = True
                if hasattr(self, 'nav_status_label'):
                    self.nav_status_label.setText(f"Nav: Moving to ({x:.1f}, {y:.1f})")
                    self.nav_status_label.setStyleSheet("color: yellow; font-weight: bold;")
            else:
                QMessageBox.warning(self, "Error", "Invalid Input")

    def on_cancel_nav_clicked(self):
        if not self.connected: return
        self.ws_thread.send({
            "command": "cancel_navigation",
            "robot_id": self.current_robot_id
        })
        self.is_navigating = False
        if hasattr(self, 'nav_status_label'):
            self.nav_status_label.setText("Nav: Canceled")
            self.nav_status_label.setStyleSheet("color: orange;")

    def on_qr_toggled(self, checked):
        if not self.connected:
            self.btn_qr_toggle.setChecked(False)
            return
        if checked:
            self.ws_thread.send({"command": "qr_detect", "robot_id": self.current_robot_id})
            self.qr_active = True
            self.btn_qr_toggle.setText("Stop QR Scan")
            if hasattr(self, 'qr_status_label'):
                self.qr_status_label.setText("QR: Running")
                self.qr_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.ws_thread.send({"command": "qr_stop", "robot_id": self.current_robot_id})
            self.qr_active = False
            self.btn_qr_toggle.setText("Start QR Scan")
            if hasattr(self, 'qr_status_label'):
                self.qr_status_label.setText("QR: OFF")
                self.qr_status_label.setStyleSheet("color: gray;")

    def on_lidar_toggled(self, checked):
        if not self.connected:
            self.btn_lidar_toggle.setChecked(False)
            return
        if checked:
            self.ws_thread.send({"command": "lidar_check", "robot_id": self.current_robot_id})
            self.lidar_active = True
            self.btn_lidar_toggle.setText("Stop Collision Check")
            if hasattr(self, 'lidar_status_label'):
                self.lidar_status_label.setText("Collision: Running")
                self.lidar_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.ws_thread.send({"command": "lidar_stop", "robot_id": self.current_robot_id})
            self.lidar_active = False
            self.btn_lidar_toggle.setText("Start Collision Check")
            if hasattr(self, 'lidar_status_label'):
                self.lidar_status_label.setText("Collision: OFF")
                self.lidar_status_label.setStyleSheet("color: gray;")

    def on_screenshot_clicked(self):
        if self.current_cv_frame is not None:
            import cv2
            fname = f"capture_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            cv2.imwrite(fname, self.current_cv_frame)
            if hasattr(self, 'statusbar'):
                self.statusbar.showMessage(f"Saved: {fname}", 3000)
        else:
            QMessageBox.warning(self, "Error", "No Camera Feed")

    def on_reset_trip(self):
        self.total_distance = 0.0
        self.start_time = datetime.datetime.now()
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        if hasattr(self, 'statusbar'):
            self.statusbar.showMessage("Trip Reset", 2000)

    def keyPressEvent(self, event):
        if not self.manual_mode_active or self.emergency_stop_active: return
        if event.isAutoRepeat(): return
        key = event.key()
        if key == Qt.Key_W:
            self.set_velocity(1.0, 0.0)
            self.btn_forward.setDown(True)
        elif key == Qt.Key_S:
            self.set_velocity(-1.0, 0.0)
            self.btn_backward.setDown(True)
        elif key == Qt.Key_A:
            self.set_velocity(0.0, 0.8)
            self.btn_left.setDown(True)
        elif key == Qt.Key_D:
            self.set_velocity(0.0, -0.8)
            self.btn_right.setDown(True)
        elif key in [Qt.Key_Space, Qt.Key_X]:
            self.set_stop_velocity()
            self.btn_stop.setDown(True)

    def keyReleaseEvent(self, event):
        if not self.manual_mode_active or self.emergency_stop_active: return
        if event.isAutoRepeat(): return
        self.set_stop_velocity()
        self.btn_forward.setDown(False)
        self.btn_backward.setDown(False)
        self.btn_left.setDown(False)
        self.btn_right.setDown(False)
        self.btn_stop.setDown(False)

    def closeEvent(self, event):
        if hasattr(self, 'ws_thread'):
            self.ws_thread.stop()
            self.ws_thread.wait(2000)
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = RobotControlWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
