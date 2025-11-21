#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Turtlebot Control GUI - Demo Version v12
- Changed Text Color of Velocity Values and System Log to White (#ffffff)
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
    QGroupBox, QGridLayout, QRadioButton, QButtonGroup, QWidget,
    QPlainTextEdit
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
        try:
            self.ws = websocket.WebSocketApp(
                uri,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )
            self.running = True
            self.ws.run_forever()
        except Exception as e:
            print(f"Connection Error: {e}")

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
            try:
                self.ws.send(json.dumps(message))
            except Exception as e:
                print(f"Send Error: {e}")

    def stop(self):
        self.running = False
        if self.ws:
            self.ws.close()

class RobotControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("터틀봇 GUI")
        # 창 크기 재조정: 높이를 560px로 줄여서 작은 화면에서도 보이도록 함
        self.setGeometry(100, 100, 1000, 560)

        # 다크 테마 스타일
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QGroupBox {
                border: 2px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                font-size: 13px;
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
                padding: 6px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #4a4a4a; }
            QPushButton:pressed { background-color: #2a2a2a; }
            QRadioButton {
                color: white;
                font-size: 12px;
                padding: 4px;
                spacing: 6px;
            }
            QRadioButton::indicator {
                width: 14px;
                height: 14px;
                border-radius: 7px;
                border: 2px solid #aaa;
            }
            QRadioButton::indicator:checked {
                background-color: #00aaff;
                border: 2px solid #00aaff;
            }
            /* 속도 값 표시 라벨 스타일 */
            QLabel#SpeedValue {
                background-color: #222;
                border: 1px solid #555;
                border-radius: 3px;
                color: #ffffff; /* 흰색으로 변경 */
                font-weight: bold;
                padding: 2px;
                min-width: 50px;
            }
            /* 로그 터미널 스타일 */
            QPlainTextEdit {
                background-color: #1a1a1a;
                color: #ffffff; /* 흰색으로 변경 */
                border: 1px solid #555;
                font-family: Consolas, monospace;
                font-size: 11px;
                padding: 5px;
            }
        """)

        # 로봇 설정
        self.current_robot_id = 1
        self.robot_group = None

        self.connected = False
        self.emergency_stop_active = False
        self.manual_mode_active = False

        # 상태 변수
        self.current_x = 0.0
        self.current_y = 0.0
        self.battery_voltage = 0.0
        self.front_distance = float('inf')
        self.target_linear = 0.0
        self.target_angular = 0.0

        # 속도 설정 변수
        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.5

        self.LIMIT_LINEAR_MIN = 0.01
        self.LIMIT_LINEAR_MAX = 0.50
        self.LIMIT_ANGULAR_MIN = 0.1
        self.LIMIT_ANGULAR_MAX = 2.0

        # 기능 상태
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

        # ========== 왼쪽 패널 (카메라 + 로그) ==========
        left_panel = QVBoxLayout()

        # 1. 카메라 모니터링
        camera_group = QGroupBox("카메라 모니터링")
        camera_layout = QVBoxLayout()

        self.camera_label = QLabel("카메라 연결 대기중...")
        # 480x360 크기 유지
        self.camera_label.setFixedSize(480, 360)
        self.camera_label.setStyleSheet("""
            background-color: #1a1a1a;
            border: 2px solid #444;
            border-radius: 5px;
        """)
        self.camera_label.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.camera_label, 0, Qt.AlignCenter)
        camera_group.setLayout(camera_layout)

        left_panel.addWidget(camera_group)

        # 2. 시스템 로그 (왼쪽으로 이동)
        log_group = QGroupBox("시스템 로그")
        log_layout = QVBoxLayout()

        self.log_terminal = QPlainTextEdit()
        self.log_terminal.setReadOnly(True)
        self.log_terminal.setMaximumBlockCount(100)
        # 로그 창 높이: 남은 공간에 맞게 적절히 조정 (약 100px)
        self.log_terminal.setFixedHeight(100)
        log_layout.addWidget(self.log_terminal)

        log_group.setLayout(log_layout)
        left_panel.addWidget(log_group)

        main_layout.addLayout(left_panel, 6) # 비율 6

        # ========== 오른쪽 패널 (제어) ==========
        right_panel = QVBoxLayout()

        # 3. 로봇 선택
        status_group = QGroupBox("로봇 선택 및 상태")
        status_layout = QGridLayout()

        status_layout.addWidget(QLabel("제어할 로봇 선택:"), 0, 0, 1, 2)

        self.robot_group = QButtonGroup(self)
        self.robot_group.buttonClicked[int].connect(self.on_robot_selected)

        radio_layout = QHBoxLayout()
        radio_layout.setAlignment(Qt.AlignLeft)

        for rid, info in ROBOTS.items():
            rb = QRadioButton(f"{info['name']}")
            self.robot_group.addButton(rb, rid)
            if rid == self.current_robot_id:
                rb.setChecked(True)
            radio_layout.addWidget(rb)

        status_layout.addLayout(radio_layout, 1, 0, 1, 2)

        self.connection_label = QLabel("연결 상태: 대기중")
        self.connection_label.setStyleSheet("color: yellow;")
        status_layout.addWidget(self.connection_label, 2, 0, 1, 2)

        self.battery_label = QLabel("배터리: --% (-- V)")
        status_layout.addWidget(self.battery_label, 3, 0, 1, 2)

        self.position_label = QLabel("위치: X: 0.00 Y: 0.00")
        status_layout.addWidget(self.position_label, 4, 0, 1, 2)

        self.distance_label = QLabel("전방 거리: -- m")
        status_layout.addWidget(self.distance_label, 5, 0, 1, 2)

        status_group.setLayout(status_layout)
        right_panel.addWidget(status_group)

        # 4. 수동 제어
        control_group = QGroupBox("수동 제어")
        control_layout = QGridLayout()

        # 초기 텍스트를 "자동 모드"로 변경
        self.manual_btn = QPushButton("자동 모드")
        self.manual_btn.setCheckable(True)
        self.manual_btn.toggled.connect(self.toggle_manual_mode)
        self.manual_btn.setStyleSheet("""
            QPushButton { background-color: #555; }
            QPushButton:checked { background-color: #2a7f2a; }
        """)
        control_layout.addWidget(self.manual_btn, 0, 0, 1, 3)

        # 속도 조절
        linear_layout = QHBoxLayout()
        linear_layout.addWidget(QLabel("선속도:"))
        self.btn_linear_down = QPushButton("-")
        self.btn_linear_down.setFixedWidth(30)
        self.btn_linear_down.setAutoRepeat(True)
        self.btn_linear_down.clicked.connect(lambda: self.change_linear_speed(-0.01))
        self.lbl_linear_val = QLabel(f"{self.max_linear_speed:.2f}")
        self.lbl_linear_val.setObjectName("SpeedValue")
        self.lbl_linear_val.setAlignment(Qt.AlignCenter)
        self.btn_linear_up = QPushButton("+")
        self.btn_linear_up.setFixedWidth(30)
        self.btn_linear_up.setAutoRepeat(True)
        self.btn_linear_up.clicked.connect(lambda: self.change_linear_speed(0.01))
        linear_layout.addWidget(self.btn_linear_down)
        linear_layout.addWidget(self.lbl_linear_val)
        linear_layout.addWidget(self.btn_linear_up)
        control_layout.addLayout(linear_layout, 1, 0, 1, 3)

        angular_layout = QHBoxLayout()
        angular_layout.addWidget(QLabel("각속도:"))
        self.btn_angular_down = QPushButton("-")
        self.btn_angular_down.setFixedWidth(30)
        self.btn_angular_down.setAutoRepeat(True)
        self.btn_angular_down.clicked.connect(lambda: self.change_angular_speed(-0.1))
        self.lbl_angular_val = QLabel(f"{self.max_angular_speed:.1f}")
        self.lbl_angular_val.setObjectName("SpeedValue")
        self.lbl_angular_val.setAlignment(Qt.AlignCenter)
        self.btn_angular_up = QPushButton("+")
        self.btn_angular_up.setFixedWidth(30)
        self.btn_angular_up.setAutoRepeat(True)
        self.btn_angular_up.clicked.connect(lambda: self.change_angular_speed(0.1))
        angular_layout.addWidget(self.btn_angular_down)
        angular_layout.addWidget(self.lbl_angular_val)
        angular_layout.addWidget(self.btn_angular_up)
        control_layout.addLayout(angular_layout, 2, 0, 1, 3)

        # 방향 버튼
        self.btn_forward = QPushButton("전진 (W)")
        self.btn_forward.pressed.connect(lambda: self.set_velocity(self.max_linear_speed, 0))
        self.btn_forward.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_forward, 3, 1)

        self.btn_left = QPushButton("좌 (A)")
        self.btn_left.pressed.connect(lambda: self.set_velocity(0, self.max_angular_speed))
        self.btn_left.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_left, 4, 0)

        self.btn_stop = QPushButton("정지")
        self.btn_stop.clicked.connect(self.stop_robot)
        self.btn_stop.setStyleSheet("background-color: #7f2a2a;")
        control_layout.addWidget(self.btn_stop, 4, 1)

        self.btn_right = QPushButton("우 (D)")
        self.btn_right.pressed.connect(lambda: self.set_velocity(0, -self.max_angular_speed))
        self.btn_right.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_right, 4, 2)

        self.btn_backward = QPushButton("후진 (S)")
        self.btn_backward.pressed.connect(lambda: self.set_velocity(-self.max_linear_speed, 0))
        self.btn_backward.released.connect(self.stop_robot)
        control_layout.addWidget(self.btn_backward, 5, 1)

        self.set_control_enabled(False)
        control_group.setLayout(control_layout)
        right_panel.addWidget(control_group)

        # 5. 기능 제어
        feature_group = QGroupBox("기능 제어")
        feature_layout = QVBoxLayout()

        self.qr_btn = QPushButton("QR 인식 시작")
        self.qr_btn.setCheckable(True)
        self.qr_btn.toggled.connect(self.toggle_qr)
        feature_layout.addWidget(self.qr_btn)

        self.lidar_btn = QPushButton("충돌 방지 시작")
        self.lidar_btn.setCheckable(True)
        self.lidar_btn.toggled.connect(self.toggle_lidar)
        feature_layout.addWidget(self.lidar_btn)

        self.origin_btn = QPushButton("원점 복귀")
        self.origin_btn.clicked.connect(self.return_to_origin)
        feature_layout.addWidget(self.origin_btn)

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

        self.qr_status_label = QLabel("QR: 대기중")
        self.qr_status_label.setStyleSheet("padding: 5px; background-color: #3a3a3a; border-radius: 3px;")
        right_panel.addWidget(self.qr_status_label)

        main_layout.addLayout(right_panel, 4) # 비율 4

        # 초기화 로그
        self.log_message("시스템 초기화 완료.")

    def log_message(self, message):
        """터미널에 메시지를 출력하고 스크롤을 내립니다."""
        time_str = datetime.datetime.now().strftime("%H:%M:%S")
        log_text = f"[{time_str}] {message}"
        self.log_terminal.appendPlainText(log_text)
        self.log_terminal.verticalScrollBar().setValue(
            self.log_terminal.verticalScrollBar().maximum()
        )

    def connect_to_server(self):
        self.ws_thread = WebSocketThread()
        self.ws_thread.message_received.connect(self.on_message)
        self.ws_thread.connection_status.connect(self.on_connection_changed)
        self.ws_thread.camera_image_received.connect(self.update_camera)
        self.ws_thread.start()

    def on_robot_selected(self, robot_id):
        if self.current_robot_id == robot_id:
            return
        if self.manual_mode_active:
             self.stop_robot()

        prev_name = ROBOTS[self.current_robot_id]['name']
        self.current_robot_id = robot_id
        curr_name = ROBOTS[self.current_robot_id]['name']

        self.log_message(f"로봇 변경: {prev_name} -> {curr_name}")

        self.battery_label.setText("배터리: --% (-- V)")
        self.position_label.setText("위치: X: 0.00 Y: 0.00")
        self.distance_label.setText("전방 거리: -- m")

        if self.connected:
            self.ws_thread.send({"command": "camera_start", "robot_id": self.current_robot_id})
            if self.qr_active:
                self.ws_thread.send({"command": "qr_detect", "robot_id": self.current_robot_id})
            if self.lidar_active:
                self.ws_thread.send({"command": "lidar_check", "robot_id": self.current_robot_id})

    def toggle_manual_mode(self, checked):
        if self.emergency_stop_active:
            self.manual_btn.setChecked(False)
            self.log_message("[경고] 비상 정지 중에는 수동 모드를 켤 수 없습니다.")
            return

        self.manual_mode_active = checked
        # 체크되면 "수동 모드", 해제되면 "자동 모드"
        self.manual_btn.setText("수동 모드" if checked else "자동 모드")
        self.set_control_enabled(checked)
        if not checked:
            self.stop_robot()

        status = "ON (수동)" if checked else "OFF (자동)"
        self.log_message(f"모드 변경: {status}")

    def set_control_enabled(self, enabled):
        self.btn_forward.setEnabled(enabled)
        self.btn_backward.setEnabled(enabled)
        self.btn_left.setEnabled(enabled)
        self.btn_right.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)
        self.btn_linear_up.setEnabled(enabled)
        self.btn_linear_down.setEnabled(enabled)
        self.btn_angular_up.setEnabled(enabled)
        self.btn_angular_down.setEnabled(enabled)

    def change_linear_speed(self, delta):
        new_speed = self.max_linear_speed + delta
        new_speed = max(self.LIMIT_LINEAR_MIN, min(new_speed, self.LIMIT_LINEAR_MAX))
        self.max_linear_speed = new_speed
        self.lbl_linear_val.setText(f"{self.max_linear_speed:.2f}")
        self.log_message(f"선속도 변경: {self.max_linear_speed:.2f} m/s")

    def change_angular_speed(self, delta):
        new_speed = self.max_angular_speed + delta
        new_speed = max(self.LIMIT_ANGULAR_MIN, min(new_speed, self.LIMIT_ANGULAR_MAX))
        self.max_angular_speed = new_speed
        self.lbl_angular_val.setText(f"{self.max_angular_speed:.1f}")
        self.log_message(f"각속도 변경: {self.max_angular_speed:.1f} rad/s")

    def toggle_qr(self, checked):
        if not self.connected:
             self.qr_btn.setChecked(False)
             self.log_message("[오류] 서버 연결 끊김")
             return

        self.qr_active = checked
        cmd = "qr_detect" if checked else "qr_stop"
        self.ws_thread.send({"command": cmd, "robot_id": self.current_robot_id})

        self.qr_btn.setText("QR 인식 중지" if checked else "QR 인식 시작")
        self.qr_btn.setStyleSheet("background-color: #2a5f2a;" if checked else "")

        status = "시작" if checked else "중지"
        self.log_message(f"QR 인식 기능 {status}")

    def toggle_lidar(self, checked):
        if not self.connected:
             self.lidar_btn.setChecked(False)
             self.log_message("[오류] 서버 연결 끊김")
             return

        self.lidar_active = checked
        cmd = "lidar_check" if checked else "lidar_stop"
        self.ws_thread.send({"command": cmd, "robot_id": self.current_robot_id})

        self.lidar_btn.setText("충돌 방지 중지" if checked else "충돌 방지 시작")
        self.lidar_btn.setStyleSheet("background-color: #2a5f2a;" if checked else "")

        status = "시작" if checked else "중지"
        self.log_message(f"충돌 방지 기능 {status}")

    def toggle_emergency(self, checked):
        self.emergency_stop_active = checked
        cmd = "emergency_stop" if checked else "release_emergency"

        self.ws_thread.send({"command": cmd, "robot_id": self.current_robot_id})

        if checked:
            self.emergency_btn.setText("비상 정지 해제")
            self.set_control_enabled(False)
            self.manual_btn.setEnabled(False)
            self.log_message("[!!!] 비상 정지 활성화됨")
        else:
            self.emergency_btn.setText("비상 정지")
            self.manual_btn.setEnabled(True)
            if self.manual_mode_active:
                self.set_control_enabled(True)
            self.log_message("비상 정지 해제됨")

    def return_to_origin(self):
        if not self.connected:
            self.log_message("[오류] 서버 연결 끊김")
            return

        reply = QMessageBox.question(self, "확인", "정말로 원점으로 복귀하시겠습니까?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.ws_thread.send({
                "command": "navigate_to",
                "robot_id": self.current_robot_id,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0
            })
            self.log_message(f"{ROBOTS[self.current_robot_id]['name']} 원점 복귀 명령 전송")
        else:
            self.log_message("원점 복귀 취소됨")

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
            BAT_MIN = 11.0
            BAT_MAX = 12.6
            percent = int(((self.battery_voltage - BAT_MIN) / (BAT_MAX - BAT_MIN)) * 100)
            percent = max(0, min(percent, 100))
            self.battery_label.setText(f"배터리: {percent}% ({self.battery_voltage:.1f} V)")

        elif msg_type == 'scan':
            ranges = msg.get('ranges', [])
            if ranges and ranges[0] > 0:
                self.front_distance = ranges[0]
                self.distance_label.setText(f"전방 거리: {self.front_distance:.2f} m")
                if self.front_distance < 0.35:
                    self.distance_label.setStyleSheet("color: red; font-weight: bold;")
                else:
                    self.distance_label.setStyleSheet("color: white;")

        elif msg_type == 'qr_detected':
            qr_text = msg.get('qr_text', '')
            self.qr_status_label.setText(f"QR 감지: {qr_text}")
            self.qr_status_label.setStyleSheet("color: #00ff00; font-weight: bold; padding: 5px; background-color: #3a3a3a; border-radius: 3px;")
            self.log_message(f"QR 코드 감지됨: {qr_text}")
            QTimer.singleShot(3000, lambda: self.qr_status_label.setText("QR: 대기중"))

    def on_connection_changed(self, connected):
        self.connected = connected
        if connected:
            self.connection_label.setText("연결 상태: 연결됨")
            self.connection_label.setStyleSheet("color: #00ff00;")
            self.log_message("서버에 연결되었습니다.")
            self.ws_thread.send({"command": "camera_start", "robot_id": self.current_robot_id})
        else:
            self.connection_label.setText("연결 상태: 연결 끊김")
            self.connection_label.setStyleSheet("color: red;")
            self.log_message("서버 연결이 끊어졌습니다.")

    def keyPressEvent(self, event):
        if not self.manual_mode_active or self.emergency_stop_active:
            return
        key = event.key()
        if event.isAutoRepeat():
            return
        if key == Qt.Key_W:
            self.set_velocity(self.max_linear_speed, 0)
        elif key == Qt.Key_S:
            self.set_velocity(-self.max_linear_speed, 0)
        elif key == Qt.Key_A:
            self.set_velocity(0, self.max_angular_speed)
        elif key == Qt.Key_D:
            self.set_velocity(0, -self.max_angular_speed)
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
    app.setStyle('Fusion')
    window = RobotControlWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
