#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Turtlebot Control GUI - Demo Version
Simplified for presentation
"""

import sys
import json
import os
import datetime
import math
import numpy as np
import cv2

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QComboBox
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont

import websocket

# 서버 설정
SERVER_IP = "127.0.0.1"
SERVER_PORT = 8080

ROBOTS = {
    1: {"name": "터틀봇1", "ip": "192.168.30.10"},
    2: {"name": "터틀봇2", "ip": "192.168.30.2"},
    3: {"name": "터틀봇3", "ip": "192.168.30.9"}
}

class WebSocketThread(QThread):
    message_received = pyqtSignal(dict)
    connection_status = pyqtSignal(bool)
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
        self.connection_status.emit(True)

    def on_message(self, ws, message):
        try:
            msg_dict = json.loads(message)
            msg_type = msg_dict.get('type')

            if msg_type == 'camera_image':
                img_data = msg_dict.get('image_data')
                robot_id = msg_dict.get('robot_id')
                if img_data and robot_id is not None:
                    np_arr = np.frombuffer(bytes.fromhex(img_data), np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    self.camera_image_received.emit(robot_id, cv_image)
                return
            self.message_received.emit(msg_dict)
        except Exception as e:
            print(f"WS Error: {e}")

    def on_error(self, ws, error):
        print(f"WS Error: {error}")

    def on_close(self, ws, *args):
        self.connection_status.emit(False)
        self.running = False

    def send(self, message: dict):
        if self.ws and self.running:
            self.ws.send(json.dumps(message))

    def stop(self):
        self.running = False
        if self.ws:
            self.ws.close()

class RobotControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("터틀봇3 관제 시스템 - 시연 데모")
        self.setGeometry(100, 100, 1200, 700)

        # 다크 테마 스타일
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QGroupBox {
                border: 2px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                font-size: 14px;
                font-weight: bold;
                color: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QLabel { color: #ffffff; font-size: 12px; }
            QPushButton {
                background-color: #3a3a3a;
                color: white;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #4a4a4a; }
            QPushButton:pressed { background-color: #2a2a2a; }
            QComboBox {
                background-color: #3a3a3a;
                color: white;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 5px;
                min-height: 25px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox::down-arrow { image: none; border: none; }
        """)

        # 로봇 설정
        self.current_robot_id = 1
        self.connected = False
        self.emergency_stop_active = False
        self.manual_mode_active = False

        # 상태 변수
        self.current_x = 0.0
        self.current_y = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.battery_voltage = 0.0
        self.front_distance = float('inf')
        self.target_linear = 0.0
        self.target_angular = 0.0

        # QR/충돌방지 상태
        self.qr_active = False
        self.lidar_active = False

        self.init_ui()
        self.connect_to_server()

        # 주기적 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(100)

    def init_ui(self):
        central = QGroupBox()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout()
        central.setLayout(main_layout)

        # ========== 왼쪽: 카메라 ==========
        left_panel = QVBoxLayout()

        # 카메라 그룹
        camera_group = QGroupBox("카메라 모니터링")
        camera_layout = QVBoxLayout()

        self.camera_label = QLabel("카메라 연결 대기중...")
        self.camera_label.setFixedSize(640, 480)
        self.camera_label.setStyleSheet("""
            background-color: #1a1a1a;
            border: 2px solid #444;
            border-radius: 5px;
        """)
        self.camera_label.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.camera_label)

        camera_group.setLayout(camera_layout)
        left_panel.addWidget(camera_group)

        main_layout.addLayout(left_panel, 2)

        # ========== 오른쪽: 제어 패널 ==========
        right_panel = QVBoxLayout()

        # 1. 로봇 선택 및 상태
        status_group = QGroupBox("로봇 상태")
        status_layout = QGridLayout()

        # 로봇 선택
        status_layout.addWidget(QLabel("로봇 선택:"), 0, 0)
        self.robot_combo = QComboBox()
        for rid, info in ROBOTS.items():
            self.robot_combo.addItem(f"{info['name']} ({info['ip']})", rid)
        self.robot_combo.currentIndexChanged.connect(self.on_robot_changed)
        status_layout.addWidget(self.robot_combo, 0, 1)

        # 연결 상태
        self.connection_label = QLabel("연결 상태: 대기중")
        self.connection_label.setStyleSheet("color: yellow;")
        status_layout.addWidget(self.connection_label, 1, 0, 1, 2)

        # 배터리
        self.battery_label = QLabel("배터리: -- V")
        status_layout.addWidget(self.battery_label, 2, 0, 1, 2)

        # 위치
        self.position_label = QLabel("위치: X: 0.00 Y: 0.00")
        status_layout.addWidget(self.position_label, 3, 0, 1, 2)

        # 전방 거리
        self.distance_label = QLabel("전방 거리: -- m")
        status_layout.addWidget(self.distance_label, 4, 0, 1, 2)

        status_group.setLayout(status_layout)
        right_panel.addWidget(status_group)

        # 2. 수동 제어
        control_group = QGroupBox("수동 제어")
        control_layout = QGridLayout()

        # 수동 모드 토글
        self.manual_btn = QPushButton("수동 모드 OFF")
        self.manual_btn.setCheckable(True)
        self.manual_btn.toggled.connect(self.toggle_manual_mode)
        self.manual_btn.setStyleSheet("""
            QPushButton { background-color: #555; }
            QPushButton:checked { background-color: #2a7f2a; }
        """)
        control_layout.addWidget(self.manual_btn, 0, 0, 1, 3)

        # 방향 버튼
        self.btn_forward = QPushButton("전진 (W)")
        self.btn_forward.pressed.connect(lambda: self.set_velocity(0.15, 0))
        self.btn_forward.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_forward, 1, 1)

        self.btn_left = QPushButton("좌 (A)")
        self.btn_left.pressed.connect(lambda: self.set_velocity(0, 0.5))
        self.btn_left.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_left, 2, 0)

        self.btn_stop = QPushButton("정지")
        self.btn_stop.clicked.connect(self.stop_robot)
        self.btn_stop.setStyleSheet("background-color: #7f2a2a;")
        control_layout.addWidget(self.btn_stop, 2, 1)

        self.btn_right = QPushButton("우 (D)")
        self.btn_right.pressed.connect(lambda: self.set_velocity(0, -0.5))
        self.btn_right.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_right, 2, 2)

        self.btn_backward = QPushButton("후진 (S)")
        self.btn_backward.pressed.connect(lambda: self.set_velocity(-0.15, 0))
        self.btn_backward.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_backward, 3, 1)

        # 초기 상태: 비활성화
        self.set_control_enabled(False)

        control_group.setLayout(control_layout)
        right_panel.addWidget(control_group)

        # 3. 기능 제어
        feature_group = QGroupBox("기능 제어")
        feature_layout = QVBoxLayout()

        # QR 인식
        self.qr_btn = QPushButton("QR 인식 시작")
        self.qr_btn.setCheckable(True)
        self.qr_btn.toggled.connect(self.toggle_qr)
        feature_layout.addWidget(self.qr_btn)

        # 충돌 방지
        self.lidar_btn = QPushButton("충돌 방지 시작")
        self.lidar_btn.setCheckable(True)
        self.lidar_btn.toggled.connect(self.toggle_lidar)
        feature_layout.addWidget(self.lidar_btn)

        # 원점 복귀
        self.origin_btn = QPushButton("원점 복귀")
        self.origin_btn.clicked.connect(self.return_to_origin)
        feature_layout.addWidget(self.origin_btn)

        # 비상 정지
        self.emergency_btn = QPushButton("비상 정지")
        self.emergency_btn.setCheckable(True)
        self.emergency_btn.toggled.connect(self.toggle_emergency)
        self.emergency_btn.setStyleSheet("""
            QPushButton { background-color: #7f2a2a; color: white; font-weight: bold; }
            QPushButton:checked { background-color: #ff0000; }
        """)
        feature_layout.addWidget(self.emergency_btn)

        feature_group.setLayout(feature_layout)
        right_panel.addWidget(feature_group)

        # QR 상태 표시
        self.qr_status_label = QLabel("QR: 대기중")
        self.qr_status_label.setStyleSheet("padding: 5px; background-color: #3a3a3a; border-radius: 3px;")
        right_panel.addWidget(self.qr_status_label)

        right_panel.addStretch()
        main_layout.addLayout(right_panel, 1)

    def connect_to_server(self):
        self.ws_thread = WebSocketThread()
        self.ws_thread.message_received.connect(self.on_message)
        self.ws_thread.connection_status.connect(self.on_connection_changed)
        self.ws_thread.camera_image_received.connect(self.update_camera)
        self.ws_thread.start()

    def on_robot_changed(self, index):
        self.current_robot_id = self.robot_combo.currentData()
        self.stop_robot()
        if self.connected:
            self.ws_thread.send({"command": "camera_start", "robot_id": self.current_robot_id})

    def toggle_manual_mode(self, checked):
        if self.emergency_stop_active:
            self.manual_btn.setChecked(False)
            return

        self.manual_mode_active = checked
        self.manual_btn.setText("수동 모드 ON" if checked else "수동 모드 OFF")
        self.set_control_enabled(checked)

        if not checked:
            self.stop_robot()

    def set_control_enabled(self, enabled):
        self.btn_forward.setEnabled(enabled)
        self.btn_backward.setEnabled(enabled)
        self.btn_left.setEnabled(enabled)
        self.btn_right.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)

    def toggle_qr(self, checked):
        if not self.connected:
            self.qr_btn.setChecked(False)
            return

        self.qr_active = checked
        cmd = "qr_detect" if checked else "qr_stop"
        self.ws_thread.send({"command": cmd, "robot_id": self.current_robot_id})
        self.qr_btn.setText("QR 인식 중지" if checked else "QR 인식 시작")

        if checked:
            self.qr_btn.setStyleSheet("background-color: #2a5f2a;")
        else:
            self.qr_btn.setStyleSheet("")

    def toggle_lidar(self, checked):
        if not self.connected:
            self.lidar_btn.setChecked(False)
            return

        self.lidar_active = checked
        cmd = "lidar_check" if checked else "lidar_stop"
        self.ws_thread.send({"command": cmd, "robot_id": self.current_robot_id})
        self.lidar_btn.setText("충돌 방지 중지" if checked else "충돌 방지 시작")

        if checked:
            self.lidar_btn.setStyleSheet("background-color: #2a5f2a;")
        else:
            self.lidar_btn.setStyleSheet("")

    def toggle_emergency(self, checked):
        self.emergency_stop_active = checked

        if checked:
            self.emergency_btn.setText("비상 정지 해제")
            self.ws_thread.send({"command": "emergency_stop", "robot_id": self.current_robot_id})
            self.set_control_enabled(False)
            self.manual_btn.setEnabled(False)
        else:
            self.emergency_btn.setText("비상 정지")
            self.ws_thread.send({"command": "release_emergency", "robot_id": self.current_robot_id})
            self.manual_btn.setEnabled(True)
            if self.manual_mode_active:
                self.set_control_enabled(True)

    def return_to_origin(self):
        if not self.connected:
            return
        self.ws_thread.send({
            "command": "navigate_to",
            "robot_id": self.current_robot_id,
            "x": self.origin_x,
            "y": self.origin_y,
            "yaw": 0.0
        })
        QMessageBox.information(self, "원점 복귀", "원점으로 복귀를 시작합니다.")

    def set_velocity(self, linear, angular):
        if not self.manual_mode_active or self.emergency_stop_active:
            return
        self.target_linear = linear
        self.target_angular = angular

    def stop_robot(self):
        self.target_linear = 0.0
        self.target_angular = 0.0

    def update_loop(self):
        if self.connected and self.manual_mode_active:
            self.ws_thread.send({
                "command": "move",
                "robot_id": self.current_robot_id,
                "linear": self.target_linear,
                "angular": self.target_angular
            })

    def update_camera(self, robot_id, cv_image):
        if robot_id != self.current_robot_id:
            return

        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        scaled = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_label.setPixmap(scaled)

    def on_message(self, msg):
        msg_type = msg.get('type')
        robot_id = msg.get('robot_id')

        if robot_id and robot_id != self.current_robot_id:
            return

        if msg_type == 'position':
            pos = msg.get('position', {})
            self.current_x = pos.get('x', 0)
            self.current_y = pos.get('y', 0)
            self.position_label.setText(f"위치: X: {self.current_x:.2f} Y: {self.current_y:.2f}")

        elif msg_type == 'battery':
            self.battery_voltage = msg.get('voltage', 0)
            self.battery_label.setText(f"배터리: {self.battery_voltage:.1f} V")

        elif msg_type == 'scan':
            ranges = msg.get('ranges', [])
            if ranges and ranges[0] > 0:
                self.front_distance = ranges[0]
                self.distance_label.setText(f"전방 거리: {self.front_distance:.2f} m")

                # 충돌 경고
                if self.front_distance < 0.35:
                    self.distance_label.setStyleSheet("color: red; font-weight: bold;")
                else:
                    self.distance_label.setStyleSheet("color: white;")

        elif msg_type == 'qr_detected':
            qr_text = msg.get('qr_text', '')
            self.qr_status_label.setText(f"QR 감지: {qr_text}")
            self.qr_status_label.setStyleSheet("color: #00ff00; font-weight: bold; padding: 5px; background-color: #3a3a3a; border-radius: 3px;")

            # 3초 후 초기화
            QTimer.singleShot(3000, lambda: self.qr_status_label.setText("QR: 대기중"))

    def on_connection_changed(self, connected):
        self.connected = connected
        if connected:
            self.connection_label.setText("연결 상태: 연결됨")
            self.connection_label.setStyleSheet("color: #00ff00;")
            self.ws_thread.send({"command": "camera_start", "robot_id": self.current_robot_id})
        else:
            self.connection_label.setText("연결 상태: 연결 끊김")
            self.connection_label.setStyleSheet("color: red;")

    def keyPressEvent(self, event):
        if not self.manual_mode_active or self.emergency_stop_active:
            return

        key = event.key()
        if event.isAutoRepeat():
            return

        if key == Qt.Key_W:
            self.set_velocity(0.15, 0)
        elif key == Qt.Key_S:
            self.set_velocity(-0.15, 0)
        elif key == Qt.Key_A:
            self.set_velocity(0, 0.5)
        elif key == Qt.Key_D:
            self.set_velocity(0, -0.5)
        elif key == Qt.Key_Space:
            self.stop_robot()

    def keyReleaseEvent(self, event):
        if not self.manual_mode_active or event.isAutoRepeat():
            return
        self.stop_robot()

    def closeEvent(self, event):
        if hasattr(self, 'ws_thread'):
            self.ws_thread.stop()
            self.ws_thread.wait(2000)
        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 모던한 스타일
    window = RobotControlWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
