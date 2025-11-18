"""
터틀봇 제어 GUI (PyQt5) + Navigation2 + 카메라 영상
- WebSocket으로 서버와 실시간 통신
- 로봇 제어 및 상태 모니터링
- Navigation 좌표 이동 기능
- 카메라 영상 실시간 표시
"""

import sys
import json
import os

if sys.platform == 'win32':
    import locale
    locale.setlocale(locale.LC_ALL, '')

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel, QGroupBox, QComboBox, QGridLayout,
    QLineEdit
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QImage, QPixmap

import websocket
import cv2
import numpy as np

try:
    from shared.config import ROBOTS, SERVER_IP, SERVER_PORT
except ImportError:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from shared.config import ROBOTS, SERVER_IP, SERVER_PORT

class WebSocketThread(QThread):
    """WebSocket 통신 스레드"""

    message_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool)
    camera_image_received = pyqtSignal(np.ndarray)

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
        self.message_received.emit("서버 연결 성공!")
        self.connection_status.emit(True)

    def on_message(self, ws, message):
        # JSON 메시지 처리
        try:
            msg_dict = json.loads(message)
            msg_type = msg_dict.get('type')

            # 카메라 이미지는 별도 시그널로 전달
            if msg_type == 'camera_image':
                img_data = msg_dict.get('image_data')
                if img_data:
                    np_arr = np.frombuffer(bytes.fromhex(img_data), np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    self.camera_image_received.emit(cv_image)
                return

        except json.JSONDecodeError:
            pass

        self.message_received.emit(message)

    def on_error(self, ws, error):
        self.message_received.emit(f"에러: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.message_received.emit("연결 종료됨")
        self.connection_status.emit(False)
        self.running = False

    def send(self, message: dict):
        """메시지 전송"""
        if self.ws and self.running:
            json_msg = json.dumps(message)
            self.ws.send(json_msg)
            print(f"[전송] {json_msg}")

    def stop(self):
        self.running = False
        if self.ws:
            self.ws.close()

class TurtlebotGUI(QMainWindow):
    """터틀봇 제어 GUI 메인 창"""

    def __init__(self):
        super().__init__()

        self.selected_robot = 1
        self.connected = False
        self.qr_active = False
        self.lidar_active = False

        self.init_ui()
        self.connect_to_server()

    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("터틀봇 제어 GUI v2.2 (카메라 포함)")
        self.setGeometry(100, 100, 1200, 900)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # 좌측: 제어 패널
        left_frame = QWidget()
        left_layout = QVBoxLayout(left_frame)
        left_layout.addWidget(self.create_robot_selector())
        left_layout.addWidget(self.create_control_group())
        left_layout.addWidget(self.create_navigation_group())
        left_layout.addWidget(self.create_mode_group())
        left_layout.addWidget(self.create_status_group())

        # 우측: 카메라 영상
        right_frame = QWidget()
        right_layout = QVBoxLayout(right_frame)
        right_layout.addWidget(self.create_camera_group())

        main_layout.addWidget(left_frame, stretch=1)
        main_layout.addWidget(right_frame, stretch=1)

    def create_robot_selector(self):
        """로봇 선택 그룹"""
        group = QGroupBox("로봇 선택")
        layout = QHBoxLayout()

        self.robot_combo = QComboBox()
        for robot_id, info in ROBOTS.items():
            self.robot_combo.addItem(
                f"{info['name']} (ID: {robot_id})",
                robot_id
            )
        self.robot_combo.currentIndexChanged.connect(self.on_robot_changed)

        layout.addWidget(QLabel("제어할 로봇:"))
        layout.addWidget(self.robot_combo)
        layout.addStretch()

        group.setLayout(layout)
        return group

    def create_control_group(self):
        """방향 제어 그룹"""
        group = QGroupBox("수동 제어 (방향키)")
        layout = QVBoxLayout()
        grid = QGridLayout()

        # 전진
        self.btn_forward = QPushButton("전진\n↑")
        self.btn_forward.setMinimumSize(120, 80)
        self.btn_forward.pressed.connect(lambda: self.send_move(0.2, 0.0))
        self.btn_forward.released.connect(self.send_stop)
        grid.addWidget(self.btn_forward, 0, 1)

        # 좌회전
        self.btn_left = QPushButton("좌회전\n←")
        self.btn_left.setMinimumSize(120, 80)
        self.btn_left.pressed.connect(lambda: self.send_move(0.0, 0.5))
        self.btn_left.released.connect(self.send_stop)
        grid.addWidget(self.btn_left, 1, 0)

        # 정지
        self.btn_stop = QPushButton("정지\n■")
        self.btn_stop.setMinimumSize(120, 80)
        self.btn_stop.setStyleSheet(
            "background-color: #ff4444; color: white; "
            "font-size: 14px; font-weight: bold;"
        )
        self.btn_stop.clicked.connect(self.send_stop)
        grid.addWidget(self.btn_stop, 1, 1)

        # 우회전
        self.btn_right = QPushButton("우회전\n→")
        self.btn_right.setMinimumSize(120, 80)
        self.btn_right.pressed.connect(lambda: self.send_move(0.0, -0.5))
        self.btn_right.released.connect(self.send_stop)
        grid.addWidget(self.btn_right, 1, 2)

        # 후진
        self.btn_backward = QPushButton("후진\n↓")
        self.btn_backward.setMinimumSize(120, 80)
        self.btn_backward.pressed.connect(lambda: self.send_move(-0.2, 0.0))
        self.btn_backward.released.connect(self.send_stop)
        grid.addWidget(self.btn_backward, 2, 1)

        layout.addLayout(grid)
        group.setLayout(layout)
        return group

    def create_navigation_group(self):
        """Navigation 제어 그룹"""
        group = QGroupBox("Navigation (자율주행)")
        layout = QVBoxLayout()

        # 좌표 입력
        coord_layout = QHBoxLayout()

        coord_layout.addWidget(QLabel("목표 좌표:"))

        coord_layout.addWidget(QLabel("X:"))
        self.nav_x_input = QLineEdit("2.0")
        self.nav_x_input.setMaximumWidth(80)
        coord_layout.addWidget(self.nav_x_input)

        coord_layout.addWidget(QLabel("Y:"))
        self.nav_y_input = QLineEdit("1.0")
        self.nav_y_input.setMaximumWidth(80)
        coord_layout.addWidget(self.nav_y_input)

        coord_layout.addWidget(QLabel("Yaw(°):"))
        self.nav_yaw_input = QLineEdit("0")
        self.nav_yaw_input.setMaximumWidth(80)
        coord_layout.addWidget(self.nav_yaw_input)

        coord_layout.addStretch()

        # 버튼
        button_layout = QHBoxLayout()

        self.btn_navigate = QPushButton("좌표로 이동")
        self.btn_navigate.setMinimumHeight(40)
        self.btn_navigate.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.btn_navigate.clicked.connect(self.navigate_to_goal)

        self.btn_cancel_nav = QPushButton("이동 취소")
        self.btn_cancel_nav.setMinimumHeight(40)
        self.btn_cancel_nav.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.btn_cancel_nav.clicked.connect(self.cancel_navigation)

        button_layout.addWidget(self.btn_navigate)
        button_layout.addWidget(self.btn_cancel_nav)

        # 상태 표시
        self.nav_status_label = QLabel("Navigation: 대기 중")
        self.nav_status_label.setStyleSheet("color: gray; font-size: 12px;")

        layout.addLayout(coord_layout)
        layout.addLayout(button_layout)
        layout.addWidget(self.nav_status_label)

        group.setLayout(layout)
        return group

    def create_mode_group(self):
        """모드 제어 그룹"""
        group = QGroupBox("센서 모드")
        layout = QVBoxLayout()

        # QR 인식
        qr_layout = QHBoxLayout()
        self.btn_qr_start = QPushButton("QR 인식 시작")
        self.btn_qr_start.clicked.connect(self.start_qr_detection)
        self.btn_qr_stop = QPushButton("QR 중지")
        self.btn_qr_stop.clicked.connect(self.stop_qr_detection)
        self.btn_qr_stop.setEnabled(False)
        qr_layout.addWidget(self.btn_qr_start)
        qr_layout.addWidget(self.btn_qr_stop)

        # LiDAR 충돌 체크
        lidar_layout = QHBoxLayout()
        self.btn_lidar_start = QPushButton("충돌 체크 시작")
        self.btn_lidar_start.clicked.connect(self.start_lidar_check)
        self.btn_lidar_stop = QPushButton("충돌 체크 중지")
        self.btn_lidar_stop.clicked.connect(self.stop_lidar_check)
        self.btn_lidar_stop.setEnabled(False)
        lidar_layout.addWidget(self.btn_lidar_start)
        lidar_layout.addWidget(self.btn_lidar_stop)

        # 상태 라벨
        self.qr_status_label = QLabel("QR: 대기 중")
        self.qr_status_label.setStyleSheet("color: gray;")
        self.lidar_status_label = QLabel("충돌 체크: 대기 중")
        self.lidar_status_label.setStyleSheet("color: gray;")

        layout.addLayout(qr_layout)
        layout.addWidget(self.qr_status_label)
        layout.addLayout(lidar_layout)
        layout.addWidget(self.lidar_status_label)

        group.setLayout(layout)
        return group

    def create_camera_group(self):
        """카메라 영상 표시 그룹"""
        group = QGroupBox("카메라 영상")
        layout = QVBoxLayout()

        self.camera_label = QLabel("영상 수신 대기 중...")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setStyleSheet("background-color: black; color: white;")

        layout.addWidget(self.camera_label)
        group.setLayout(layout)
        return group

    def create_status_group(self):
        """상태 표시 그룹"""
        group = QGroupBox("상태")
        layout = QVBoxLayout()

        self.status_label = QLabel("서버 연결 대기 중...")
        self.status_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(self.status_label)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(150)
        layout.addWidget(self.log)

        group.setLayout(layout)
        return group

    def connect_to_server(self):
        """서버 연결"""
        self.ws_thread = WebSocketThread()
        self.ws_thread.message_received.connect(self.on_message)
        self.ws_thread.connection_status.connect(self.on_connection_changed)
        self.ws_thread.camera_image_received.connect(self.update_camera_image)
        self.ws_thread.start()

    def on_robot_changed(self, index):
        """로봇 선택 변경"""
        self.selected_robot = self.robot_combo.currentData()
        robot_name = ROBOTS[self.selected_robot]['name']
        self.add_log(f"선택: {robot_name}")

    def send_move(self, linear: float, angular: float):
        """이동 명령"""
        if not self.connected:
            self.add_log("에러: 서버 연결 안됨")
            return

        message = {
            "command": "move",
            "robot_id": self.selected_robot,
            "linear": linear,
            "angular": angular
        }
        self.ws_thread.send(message)

    def send_stop(self):
        """정지 명령"""
        if not self.connected:
            return

        message = {
            "command": "stop",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)
        self.add_log("정지")

    def navigate_to_goal(self):
        """Navigation: 목표 좌표로 이동"""
        if not self.connected:
            self.add_log("에러: 서버 연결 안됨")
            return

        try:
            x = float(self.nav_x_input.text())
            y = float(self.nav_y_input.text())
            yaw_deg = float(self.nav_yaw_input.text())
            yaw_rad = yaw_deg * 3.14159 / 180.0

            message = {
                "command": "navigate_to",
                "robot_id": self.selected_robot,
                "x": x,
                "y": y,
                "yaw": yaw_rad
            }
            self.ws_thread.send(message)

            self.nav_status_label.setText(f"Navigation: ({x:.2f}, {y:.2f})로 이동 중")
            self.nav_status_label.setStyleSheet("color: blue; font-weight: bold;")
            self.add_log(f"[Nav] 목표: x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°")

        except ValueError:
            self.add_log("에러: 좌표 입력값이 잘못되었습니다")

    def cancel_navigation(self):
        """Navigation: 이동 취소"""
        if not self.connected:
            return

        message = {
            "command": "cancel_navigation",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        self.nav_status_label.setText("Navigation: 취소됨")
        self.nav_status_label.setStyleSheet("color: orange;")
        self.add_log("[Nav] 이동 취소")

    def start_qr_detection(self):
        """QR 인식 시작"""
        if not self.connected:
            self.add_log("에러: 서버 연결 안됨")
            return

        message = {
            "command": "qr_detect",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        self.qr_active = True
        self.btn_qr_start.setEnabled(False)
        self.btn_qr_stop.setEnabled(True)
        self.qr_status_label.setText("QR: 실행 중")
        self.qr_status_label.setStyleSheet("color: green;")
        self.add_log("QR 인식 시작")

    def stop_qr_detection(self):
        """QR 인식 중지"""
        message = {
            "command": "qr_stop",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        self.qr_active = False
        self.btn_qr_start.setEnabled(True)
        self.btn_qr_stop.setEnabled(False)
        self.qr_status_label.setText("QR: 대기 중")
        self.qr_status_label.setStyleSheet("color: gray;")
        self.add_log("QR 인식 중지")

    def start_lidar_check(self):
        """LiDAR 충돌 체크 시작"""
        if not self.connected:
            self.add_log("에러: 서버 연결 안됨")
            return

        message = {
            "command": "lidar_check",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        self.lidar_active = True
        self.btn_lidar_start.setEnabled(False)
        self.btn_lidar_stop.setEnabled(True)
        self.lidar_status_label.setText("충돌 체크: 실행 중")
        self.lidar_status_label.setStyleSheet("color: green;")
        self.add_log("충돌 체크 시작")

    def stop_lidar_check(self):
        """LiDAR 충돌 체크 중지"""
        message = {
            "command": "lidar_stop",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        self.lidar_active = False
        self.btn_lidar_start.setEnabled(True)
        self.btn_lidar_stop.setEnabled(False)
        self.lidar_status_label.setText("충돌 체크: 대기 중")
        self.lidar_status_label.setStyleSheet("color: gray;")
        self.add_log("충돌 체크 중지")

    def update_camera_image(self, cv_image):
        """카메라 영상 업데이트"""
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w

        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)

        scaled_pixmap = pixmap.scaled(
            self.camera_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )

        self.camera_label.setPixmap(scaled_pixmap)

    def on_message(self, msg):
        """메시지 수신 처리"""
        self.log.append(msg)

        try:
            data = json.loads(msg)
            msg_type = data.get('type')

            if msg_type == 'qr_detected':
                qr_text = data.get('qr_text', '')
                robot_id = data.get('robot_id', '')
                self.add_log(f"[로봇{robot_id}] QR 인식: {qr_text}")
                self.qr_status_label.setText(f"QR: 인식됨 [{qr_text}]")
                self.qr_status_label.setStyleSheet("color: blue;")

            elif msg_type == 'collision_warning':
                robot_id = data.get('robot_id', '')
                warning_msg = data.get('message', '')
                self.add_log(f"[경고] [로봇{robot_id}] {warning_msg}")
                self.lidar_status_label.setText("충돌 체크: 위험 감지!")
                self.lidar_status_label.setStyleSheet("color: red;")

            elif msg_type == 'nav_complete':
                robot_id = data.get('robot_id', '')
                self.add_log(f"[로봇{robot_id}] 목표 도착 완료!")
                self.nav_status_label.setText("Navigation: 도착 완료")
                self.nav_status_label.setStyleSheet("color: green; font-weight: bold;")

            elif msg_type == 'nav_failed':
                robot_id = data.get('robot_id', '')
                reason = data.get('reason', 'unknown')
                self.add_log(f"[로봇{robot_id}] 이동 실패: {reason}")
                self.nav_status_label.setText(f"Navigation: 실패 ({reason})")
                self.nav_status_label.setStyleSheet("color: red; font-weight: bold;")

        except json.JSONDecodeError:
            pass

    def on_connection_changed(self, connected: bool):
        """연결 상태 변경"""
        self.connected = connected

        if connected:
            self.status_label.setText("서버 연결됨")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.enable_controls(True)
        else:
            self.status_label.setText("서버 연결 안됨")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.enable_controls(False)

    def enable_controls(self, enabled: bool):
        """제어 버튼 활성화/비활성화"""
        self.btn_forward.setEnabled(enabled)
        self.btn_backward.setEnabled(enabled)
        self.btn_left.setEnabled(enabled)
        self.btn_right.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)

        self.btn_navigate.setEnabled(enabled)
        self.btn_cancel_nav.setEnabled(enabled)

        if enabled:
            self.btn_qr_start.setEnabled(not self.qr_active)
            self.btn_qr_stop.setEnabled(self.qr_active)
            self.btn_lidar_start.setEnabled(not self.lidar_active)
            self.btn_lidar_stop.setEnabled(self.lidar_active)
        else:
            self.btn_qr_start.setEnabled(False)
            self.btn_qr_stop.setEnabled(False)
            self.btn_lidar_start.setEnabled(False)
            self.btn_lidar_stop.setEnabled(False)

    def add_log(self, message: str):
        """로그 추가"""
        self.log.append(message)
        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        """종료 처리"""
        print("GUI 종료 중")
        if hasattr(self, 'ws_thread'):
            self.ws_thread.stop()
            self.ws_thread.wait(2000)
        event.accept()
        print("종료 완료")

def main():
    """GUI 메인 함수"""
    app = QApplication(sys.argv)
    window = TurtlebotGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    print("=" * 60)
    print("터틀봇 제어 GUI 시작 (Navigation + 카메라)")
    print("=" * 60)
    print(f"서버 주소: {SERVER_IP}:{SERVER_PORT}")
    print("=" * 60)
    main()
