"""
=================================================================
터틀봇 제어 GUI (PyQt5)
=================================================================
[목적]
- PyQt5로 그래픽 인터페이스 제공
- WebSocket으로 서버와 실시간 통신
- 로봇 제어 및 상태 모니터링

[실행 방법]
$ cd ~/작업공간/turtle_project
$ python3 gui/gui.py

또는
$ python3 -m gui.gui

[기능]
1. 로봇 선택
2. 방향 제어 (전진/후진/좌우회전/정지)
3. QR 인식 시작/중지
4. LiDAR 충돌 체크 시작/중지
5. 서버 메시지 수신 및 표시

[수정 방법]
- create_control_group(): 제어 버튼 추가/변경
- create_mode_group(): 새 기능 버튼 추가
- on_message(): 서버 메시지 처리 로직 수정

[Windows에서도 실행 가능]
같은 WiFi 연결되어 있으면 Windows PC에서도 사용 가능
=================================================================
"""

import sys
import json
import os

# ======================================================================
# Windows 한글 깨짐 방지
# ======================================================================
# [설명]
# Windows에서 콘솔 출력 시 한글이 깨지는 문제 해결
# Linux/Mac에서는 영향 없음
if sys.platform == 'win32':
    import locale
    locale.setlocale(locale.LC_ALL, '')

# ======================================================================
# PyQt5 import
# ======================================================================
from PyQt5.QtWidgets import (
    QApplication,       # 앱 객체
    QMainWindow,        # 메인 창
    QWidget,            # 기본 위젯
    QVBoxLayout,        # 세로 레이아웃
    QHBoxLayout,        # 가로 레이아웃
    QPushButton,        # 버튼
    QTextEdit,          # 텍스트 박스 (로그)
    QLabel,             # 라벨 (텍스트 표시)
    QGroupBox,          # 그룹 박스 (테두리)
    QComboBox,          # 콤보 박스 (드롭다운)
    QGridLayout         # 그리드 레이아웃
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont

# WebSocket 클라이언트
import websocket

# ======================================================================
# 설정 파일 import
# ======================================================================
# [경로 처리]
# gui/ 폴더에서 실행해도
# 프로젝트 루트에서 실행해도 작동하도록
try:
    from shared.config import ROBOTS, SERVER_IP, SERVER_PORT
except ImportError:
    # import 실패 시 경로 추가
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from shared.config import ROBOTS, SERVER_IP, SERVER_PORT

# ======================================================================
# WebSocket 통신 스레드
# ======================================================================
class WebSocketThread(QThread):
    """
    서버와 WebSocket 통신하는 백그라운드 스레드

    [역할]
    1. 서버 연결 유지
    2. 메시지 수신
    3. 시그널로 메인 스레드에 전달

    [시그널]
    - message_received: 메시지 받음 (str)
    - connection_status: 연결 상태 변경 (bool)

    [왜 스레드?]
    WebSocket 통신은 블로킹 작업
    메인 스레드에서 하면 GUI가 멈춤
    별도 스레드에서 실행하면 GUI는 계속 반응
    """

    # ========================================
    # 시그널 정의
    # ========================================
    # [설명]
    # pyqtSignal: 스레드 간 안전한 통신
    # 이 스레드에서 emit()하면 메인 스레드에서 받음
    message_received = pyqtSignal(str)      # 메시지 수신 시그널
    connection_status = pyqtSignal(bool)    # 연결 상태 시그널

    def __init__(self):
        """
        스레드 초기화

        [내부 변수]
        - running: 스레드 실행 플래그
        - ws: WebSocket 객체
        """
        super().__init__()
        self.running = False
        self.ws = None

    def run(self):
        """
        스레드 메인 함수

        [호출]
        start() 호출 시 자동으로 실행됨
        (직접 호출 금지!)

        [작동]
        1. WebSocket 연결
        2. 무한 루프로 메시지 수신
        3. 연결 끊기면 종료

        [종료]
        stop() 호출 시 running=False로 설정되어 종료
        """

        # WebSocket 주소
        uri = f"ws://{SERVER_IP}:{SERVER_PORT}/ws"

        try:
            # ========================================
            # WebSocket 앱 생성
            # ========================================
            # [콜백 설명]
            # - on_open: 연결 성공 시
            # - on_message: 메시지 수신 시
            # - on_error: 에러 발생 시
            # - on_close: 연결 종료 시
            self.ws = websocket.WebSocketApp(
                uri,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )

            # ========================================
            # 연결 및 수신 루프 시작
            # ========================================
            self.running = True
            # run_forever(): 무한 루프 (블로킹)
            # 메시지가 올 때마다 on_message() 호출
            self.ws.run_forever()

        except Exception as e:
            # 연결 실패
            self.message_received.emit(f"연결 실패: {e}")
            self.connection_status.emit(False)

    # ========================================
    # WebSocket 콜백 함수들
    # ========================================

    def on_open(self, ws):
        """
        연결 성공 콜백

        [호출 시점]
        서버와 WebSocket 연결 성공 시

        [처리]
        1. 메인 스레드에 성공 메시지 전송
        2. 연결 상태 True로 변경
        """
        self.message_received.emit("서버 연결 성공!")
        self.connection_status.emit(True)

    def on_message(self, ws, message):
        """
        메시지 수신 콜백

        [호출 시점]
        서버에서 메시지 보낼 때마다

        [입력]
        message: JSON 문자열
        예: '{"type": "qr_detected", "qr_text": "forward"}'

        [처리]
        메인 스레드로 메시지 전달 (시그널)
        """
        self.message_received.emit(message)

    def on_error(self, ws, error):
        """
        에러 발생 콜백

        [호출 시점]
        통신 중 에러 발생 시

        [처리]
        에러 메시지를 메인 스레드로 전달
        """
        self.message_received.emit(f"에러: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        """
        연결 종료 콜백

        [호출 시점]
        - 서버 종료
        - 네트워크 끊김
        - stop() 호출

        [처리]
        1. 종료 메시지 전송
        2. 연결 상태 False로 변경
        3. 실행 플래그 False
        """
        self.message_received.emit("연결 종료됨")
        self.connection_status.emit(False)
        self.running = False

    # ========================================
    # 메시지 전송
    # ========================================

    def send(self, message: dict):
        """
        서버로 메시지 전송

        [입력]
        message: 명령 딕셔너리
        {
            "command": "move",
            "robot_id": 1,
            "linear": 0.2,
            "angular": 0.0
        }

        [처리]
        1. 딕셔너리 → JSON 문자열 변환
        2. WebSocket으로 전송
        3. 실패 시 에러 메시지

        [사용 예시]
        self.ws_thread.send({
            "command": "move",
            "robot_id": 1,
            "linear": 0.2,
            "angular": 0.0
        })
        """
        if self.ws and self.running:
            try:
                # 딕셔너리 → JSON 문자열
                json_msg = json.dumps(message)

                # 전송
                self.ws.send(json_msg)

                # 로그 (디버깅용)
                print(f"[전송] {json_msg}")

            except Exception as e:
                self.message_received.emit(f"전송 실패: {e}")

    # ========================================
    # 스레드 종료
    # ========================================

    def stop(self):
        """
        스레드 종료

        [호출]
        GUI 종료 시 (closeEvent)

        [처리]
        1. 실행 플래그 False
        2. WebSocket 닫기
        3. 스레드 자동 종료
        """
        self.running = False
        if self.ws:
            self.ws.close()

# ======================================================================
# 메인 GUI 클래스
# ======================================================================
class TurtlebotGUI(QMainWindow):
    """
    터틀봇 제어 GUI 메인 창

    [구조]
    - 상단: 로봇 선택
    - 중단: 방향 제어 버튼
    - 하단: 모드 제어 (QR, LiDAR)
    - 최하단: 상태 표시 및 로그

    [수정 포인트]
    - init_ui(): UI 레이아웃 변경
    - create_control_group(): 제어 버튼 추가/변경
    - create_mode_group(): 새 기능 버튼 추가
    - on_message(): 서버 메시지 처리 로직
    """

    def __init__(self):
        """
        GUI 초기화

        [초기화 순서]
        1. 내부 변수 설정
        2. UI 생성
        3. 서버 연결
        """
        super().__init__()

        # ========================================
        # 내부 변수 초기화
        # ========================================
        self.selected_robot = 1         # 현재 선택된 로봇 (기본값: 1)
        self.connected = False          # 서버 연결 상태

        # 모드 활성화 상태
        self.qr_active = False          # QR 인식 중인지
        self.lidar_active = False       # LiDAR 체크 중인지

        # ========================================
        # UI 생성
        # ========================================
        self.init_ui()

        # ========================================
        # 서버 연결
        # ========================================
        self.connect_to_server()

    def init_ui(self):
        """
        UI 초기화

        [레이아웃 구조]
        QMainWindow
          └─ QWidget (central)
               └─ QVBoxLayout (main_layout)
                    ├─ 로봇 선택 (QGroupBox)
                    ├─ 방향 제어 (QGroupBox)
                    ├─ 모드 제어 (QGroupBox)
                    └─ 상태 표시 (QGroupBox)

        [수정 방법]
        그룹 순서 바꾸기:
        main_layout.addWidget(status_group)  # 이 줄들의 순서 변경
        main_layout.addWidget(control_group)
        """

        # ========================================
        # 윈도우 기본 설정
        # ========================================
        self.setWindowTitle("터틀봇 제어 GUI v2.0")
        self.setGeometry(100, 100, 700, 800)  # (x, y, width, height)

        # ========================================
        # 중앙 위젯 및 레이아웃
        # ========================================
        # QMainWindow는 중앙 위젯이 필요
        central = QWidget()
        self.setCentralWidget(central)

        # 세로 레이아웃 (위에서 아래로)
        main_layout = QVBoxLayout(central)

        # ========================================
        # 1. 로봇 선택 그룹
        # ========================================
        robot_group = self.create_robot_selector()
        main_layout.addWidget(robot_group)

        # ========================================
        # 2. 방향 제어 그룹
        # ========================================
        control_group = self.create_control_group()
        main_layout.addWidget(control_group)

        # ========================================
        # 3. 모드 제어 그룹 (QR, LiDAR)
        # ========================================
        mode_group = self.create_mode_group()
        main_layout.addWidget(mode_group)

        # ========================================
        # 4. 상태 표시 그룹
        # ========================================
        status_group = self.create_status_group()
        main_layout.addWidget(status_group)

    # ======================================================================
    # UI 구성 요소 생성 함수들
    # ======================================================================

    def create_robot_selector(self):
        """
        로봇 선택 그룹 생성

        [구성]
        - 라벨: "제어할 로봇:"
        - 콤보박스: 로봇 목록

        [자동 업데이트]
        config.py의 ROBOTS 딕셔너리에서 자동으로 로봇 목록 생성
        로봇 추가 시 이 함수 수정 불필요

        [반환]
        QGroupBox 객체
        """

        # ========================================
        # 그룹 박스 생성
        # ========================================
        group = QGroupBox("로봇 선택")

        # 가로 레이아웃
        layout = QHBoxLayout()

        # ========================================
        # 콤보 박스 생성
        # ========================================
        self.robot_combo = QComboBox()

        # ROBOTS 딕셔너리에서 로봇 추가
        for robot_id, info in ROBOTS.items():
            # 표시 텍스트: "터틀봇1 (ID: 1)"
            # 데이터: robot_id (1, 2, 3)
            self.robot_combo.addItem(
                f"{info['name']} (ID: {robot_id})",  # 표시 텍스트
                robot_id                              # 저장 데이터
            )

        # 선택 변경 시 콜백 연결
        self.robot_combo.currentIndexChanged.connect(self.on_robot_changed)

        # ========================================
        # 레이아웃 구성
        # ========================================
        layout.addWidget(QLabel("제어할 로봇:"))
        layout.addWidget(self.robot_combo)
        layout.addStretch()  # 오른쪽 여백

        group.setLayout(layout)
        return group

    def create_control_group(self):
        """
        방향 제어 그룹 생성

        [구성]
        3x3 그리드:
        [  ] [위] [  ]
        [좌] [정지][우]
        [  ] [아래][  ]

        [버튼 동작]
        - pressed: 버튼 누르는 순간
        - released: 버튼 떼는 순간
        - clicked: 누르고 떼기 (1번 실행)

        [수정 방법]
        1. 새 버튼 추가:
           btn_new = QPushButton("새 기능")
           btn_new.clicked.connect(self.new_function)
           grid.addWidget(btn_new, 행, 열)

        2. 속도 변경:
           self.send_move(0.3, 0.0)  # 더 빠르게

        3. 버튼 크기 변경:
           btn.setMinimumSize(150, 100)

        [반환]
        QGroupBox 객체
        """

        # ========================================
        # 그룹 박스 생성
        # ========================================
        group = QGroupBox("방향 제어")
        layout = QVBoxLayout()

        # ========================================
        # 그리드 레이아웃 (3x3)
        # ========================================
        # [행, 열] 인덱스:
        # [0,0] [0,1] [0,2]
        # [1,0] [1,1] [1,2]
        # [2,0] [2,1] [2,2]
        grid = QGridLayout()

        # ========================================
        # 전진 버튼 (상단 중앙)
        # ========================================
        self.btn_forward = QPushButton("전진\n↑")
        self.btn_forward.setMinimumSize(120, 80)

        # pressed: 버튼 누르는 순간 → 이동 시작
        self.btn_forward.pressed.connect(lambda: self.send_move(0.2, 0.0))
        # released: 버튼 떼는 순간 → 정지
        self.btn_forward.released.connect(self.send_stop)

        # 그리드 [0행, 1열]에 배치
        grid.addWidget(self.btn_forward, 0, 1)

        # ========================================
        # 좌회전 버튼 (중단 좌측)
        # ========================================
        self.btn_left = QPushButton("좌회전\n←")
        self.btn_left.setMinimumSize(120, 80)
        self.btn_left.pressed.connect(lambda: self.send_move(0.0, 0.5))
        self.btn_left.released.connect(self.send_stop)
        grid.addWidget(self.btn_left, 1, 0)

        # ========================================
        # 정지 버튼 (중앙)
        # ========================================
        self.btn_stop = QPushButton("정지\n■")
        self.btn_stop.setMinimumSize(120, 80)

        # 스타일 시트 (CSS 비슷)
        # background-color: 배경색
        # color: 글자색
        # font-size: 글자 크기
        # font-weight: 글자 굵기
        self.btn_stop.setStyleSheet(
            "background-color: #ff4444; "
            "color: white; "
            "font-size: 14px; "
            "font-weight: bold;"
        )

        # clicked: 클릭 시 (누르고 떼기)
        self.btn_stop.clicked.connect(self.send_stop)
        grid.addWidget(self.btn_stop, 1, 1)

        # ========================================
        # 우회전 버튼 (중단 우측)
        # ========================================
        self.btn_right = QPushButton("우회전\n→")
        self.btn_right.setMinimumSize(120, 80)
        self.btn_right.pressed.connect(lambda: self.send_move(0.0, -0.5))
        self.btn_right.released.connect(self.send_stop)
        grid.addWidget(self.btn_right, 1, 2)

        # ========================================
        # 후진 버튼 (하단 중앙)
        # ========================================
        self.btn_backward = QPushButton("후진\n↓")
        self.btn_backward.setMinimumSize(120, 80)
        self.btn_backward.pressed.connect(lambda: self.send_move(-0.2, 0.0))
        self.btn_backward.released.connect(self.send_stop)
        grid.addWidget(self.btn_backward, 2, 1)

        # ========================================
        # 레이아웃 구성
        # ========================================
        layout.addLayout(grid)
        group.setLayout(layout)
        return group

    def create_mode_group(self):
        """
        모드 제어 그룹 생성 (QR, LiDAR 등)

        [구성]
        - QR 인식: 시작/중지 버튼 + 상태 라벨
        - LiDAR 충돌 체크: 시작/중지 버튼 + 상태 라벨

        [새 기능 추가 방법]
        1. 버튼 생성:
           btn_new = QPushButton("새 기능")
           btn_new.clicked.connect(self.new_function)

        2. 레이아웃 추가:
           new_layout = QHBoxLayout()
           new_layout.addWidget(btn_new)
           layout.addLayout(new_layout)

        3. 함수 구현:
           def new_function(self):
               # 처리 로직
               pass

        [반환]
        QGroupBox 객체
        """

        # ========================================
        # 그룹 박스 생성
        # ========================================
        group = QGroupBox("모드 제어")
        layout = QVBoxLayout()

        # ========================================
        # QR 인식 버튼
        # ========================================
        qr_layout = QHBoxLayout()

        # 시작 버튼
        self.btn_qr_start = QPushButton("QR 인식 시작")
        self.btn_qr_start.clicked.connect(self.start_qr_detection)

        # 중지 버튼 (처음엔 비활성화)
        self.btn_qr_stop = QPushButton("QR 중지")
        self.btn_qr_stop.clicked.connect(self.stop_qr_detection)
        self.btn_qr_stop.setEnabled(False)  # 비활성화

        # 레이아웃에 추가
        qr_layout.addWidget(self.btn_qr_start)
        qr_layout.addWidget(self.btn_qr_stop)

        # ========================================
        # LiDAR 충돌 체크 버튼
        # ========================================
        lidar_layout = QHBoxLayout()

        # 시작 버튼
        self.btn_lidar_start = QPushButton("충돌 체크 시작")
        self.btn_lidar_start.clicked.connect(self.start_lidar_check)

        # 중지 버튼 (처음엔 비활성화)
        self.btn_lidar_stop = QPushButton("충돌 체크 중지")
        self.btn_lidar_stop.clicked.connect(self.stop_lidar_check)
        self.btn_lidar_stop.setEnabled(False)

        # 레이아웃에 추가
        lidar_layout.addWidget(self.btn_lidar_start)
        lidar_layout.addWidget(self.btn_lidar_stop)

        # ========================================
        # 상태 라벨
        # ========================================
        # QR 상태 라벨
        self.qr_status_label = QLabel("QR: 대기 중")
        self.qr_status_label.setStyleSheet("color: gray;")

        # LiDAR 상태 라벨
        self.lidar_status_label = QLabel("충돌 체크: 대기 중")
        self.lidar_status_label.setStyleSheet("color: gray;")

        # ========================================
        # 레이아웃 구성
        # ========================================
        # 순서:
        # 1. QR 버튼들
        # 2. QR 상태 라벨
        # 3. LiDAR 버튼들
        # 4. LiDAR 상태 라벨
        layout.addLayout(qr_layout)
        layout.addWidget(self.qr_status_label)
        layout.addLayout(lidar_layout)
        layout.addWidget(self.lidar_status_label)

        group.setLayout(layout)
        return group

    def create_status_group(self):
        """
        상태 표시 그룹 생성

        [구성]
        - 연결 상태 라벨 (상단)
        - 로그 텍스트 박스 (하단, 스크롤 가능)

        [로그 사용]
        self.add_log("메시지")로 로그 추가
        자동으로 최신 로그가 하단에 표시됨

        [반환]
        QGroupBox 객체
        """

        # ========================================
        # 그룹 박스 생성
        # ========================================
        group = QGroupBox("상태")
        layout = QVBoxLayout()

        # ========================================
        # 연결 상태 라벨
        # ========================================
        self.status_label = QLabel("서버 연결 대기 중...")
        # 폰트 설정: (폰트명, 크기, 굵기)
        self.status_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(self.status_label)

        # ========================================
        # 로그 텍스트 박스
        # ========================================
        self.log = QTextEdit()
        self.log.setReadOnly(True)          # 읽기 전용
        self.log.setMaximumHeight(200)      # 최대 높이 200픽셀
        layout.addWidget(self.log)

        group.setLayout(layout)
        return group

    # ======================================================================
    # 서버 통신 함수들
    # ======================================================================

    def connect_to_server(self):
        """
        서버 연결 시작

        [작동]
        1. WebSocketThread 생성
        2. 시그널 연결
           - message_received → on_message
           - connection_status → on_connection_changed
        3. 스레드 시작

        [비동기]
        스레드가 백그라운드에서 실행되므로
        GUI는 계속 반응함 (멈추지 않음)
        """

        # ========================================
        # WebSocket 스레드 생성
        # ========================================
        self.ws_thread = WebSocketThread()

        # ========================================
        # 시그널 연결
        # ========================================
        # message_received 시그널이 발생하면
        # self.on_message() 함수 호출
        self.ws_thread.message_received.connect(self.on_message)

        # connection_status 시그널이 발생하면
        # self.on_connection_changed() 함수 호출
        self.ws_thread.connection_status.connect(self.on_connection_changed)

        # ========================================
        # 스레드 시작
        # ========================================
        # start() → run() 함수가 별도 스레드에서 실행됨
        self.ws_thread.start()

    def on_robot_changed(self, index):
        """
        로봇 선택 변경 콜백

        [호출 시점]
        콤보박스에서 다른 로봇 선택 시

        [처리]
        1. 선택된 로봇 ID 저장
        2. 로그에 기록

        [입력]
        index: 콤보박스 인덱스 (0, 1, 2, ...)
        (실제로는 currentData()로 robot_id 가져옴)
        """
        # 콤보박스의 현재 데이터 (robot_id) 가져오기
        self.selected_robot = self.robot_combo.currentData()

        # 로봇 이름 가져오기
        robot_name = ROBOTS[self.selected_robot]['name']

        # 로그 추가
        self.add_log(f"선택: {robot_name}")

    # ======================================================================
    # 로봇 제어 함수들
    # ======================================================================

    def send_move(self, linear: float, angular: float):
        """
        이동 명령 전송

        [입력]
        - linear: 직진 속도 (m/s)
          양수: 전진
          음수: 후진
          범위: -0.22 ~ 0.22

        - angular: 회전 속도 (rad/s)
          양수: 좌회전
          음수: 우회전
          범위: -2.84 ~ 2.84

        [메시지 형식]
        {
            "command": "move",
            "robot_id": 1,
            "linear": 0.2,
            "angular": 0.0
        }

        [사용 예시]
        self.send_move(0.2, 0.0)   # 전진
        self.send_move(-0.15, 0.0) # 후진
        self.send_move(0.0, 0.5)   # 좌회전
        """

        # ========================================
        # 연결 확인
        # ========================================
        if not self.connected:
            self.add_log("에러: 서버 연결 안됨")
            return

        # ========================================
        # 메시지 생성
        # ========================================
        message = {
            "command": "move",
            "robot_id": self.selected_robot,
            "linear": linear,
            "angular": angular
        }

        # ========================================
        # 전송
        # ========================================
        self.ws_thread.send(message)

    def send_stop(self):
        """
        정지 명령 전송

        [메시지 형식]
        {
            "command": "stop",
            "robot_id": 1
        }

        [호출]
        - 정지 버튼 클릭 시
        - 방향 버튼 떼는 순간 (released)
        """

        # ========================================
        # 연결 확인
        # ========================================
        if not self.connected:
            return

        # ========================================
        # 메시지 생성 및 전송
        # ========================================
        message = {
            "command": "stop",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        # 로그 추가
        self.add_log("정지")

    # ======================================================================
    # QR 제어 함수들
    # ======================================================================

    def start_qr_detection(self):
        """
        QR 인식 시작

        [작동]
        1. 서버에 "qr_detect" 명령 전송
        2. 버튼 상태 변경
           - 시작 버튼: 비활성화
           - 중지 버튼: 활성화
        3. 상태 라벨 업데이트

        [서버 동작]
        서버가 카메라 이미지를 QR 모듈로 전달 시작
        QR 인식되면 서버에서 메시지 전송 → on_message()에서 처리
        """

        # ========================================
        # 연결 확인
        # ========================================
        if not self.connected:
            self.add_log("에러: 서버 연결 안됨")
            return

        # ========================================
        # 명령 전송
        # ========================================
        message = {
            "command": "qr_detect",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        # ========================================
        # UI 업데이트
        # ========================================
        self.qr_active = True
        self.btn_qr_start.setEnabled(False)     # 시작 버튼 비활성화
        self.btn_qr_stop.setEnabled(True)       # 중지 버튼 활성화

        # 상태 라벨 업데이트
        self.qr_status_label.setText("QR: 실행 중")
        self.qr_status_label.setStyleSheet("color: green;")

        # 로그 추가
        self.add_log("QR 인식 시작")

    def stop_qr_detection(self):
        """
        QR 인식 중지

        [작동]
        1. 서버에 "qr_stop" 명령 전송
        2. 버튼 상태 변경
           - 시작 버튼: 활성화
           - 중지 버튼: 비활성화
        3. 상태 라벨 업데이트

        [서버 동작]
        서버가 카메라 이미지를 QR 모듈로 전달 중지
        """

        # ========================================
        # 명령 전송
        # ========================================
        message = {
            "command": "qr_stop",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        # ========================================
        # UI 업데이트
        # ========================================
        self.qr_active = False
        self.btn_qr_start.setEnabled(True)      # 시작 버튼 활성화
        self.btn_qr_stop.setEnabled(False)      # 중지 버튼 비활성화

        # 상태 라벨 업데이트
        self.qr_status_label.setText("QR: 대기 중")
        self.qr_status_label.setStyleSheet("color: gray;")

        # 로그 추가
        self.add_log("QR 인식 중지")

    # ======================================================================
    # LiDAR 제어 함수들
    # ======================================================================

    def start_lidar_check(self):
        """
        LiDAR 충돌 체크 시작

        [작동]
        1. 서버에 "lidar_check" 명령 전송
        2. 버튼 상태 변경
        3. 상태 라벨 업데이트

        [서버 동작]
        - 서버가 LiDAR 데이터를 LiDAR 모듈로 전달 시작
        - 충돌 위험 감지 시:
          1. 로봇 자동 정지
          2. GUI로 경고 메시지 전송 → on_message()에서 처리
        """

        # ========================================
        # 연결 확인
        # ========================================
        if not self.connected:
            self.add_log("에러: 서버 연결 안됨")
            return

        # ========================================
        # 명령 전송
        # ========================================
        message = {
            "command": "lidar_check",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        # ========================================
        # UI 업데이트
        # ========================================
        self.lidar_active = True
        self.btn_lidar_start.setEnabled(False)
        self.btn_lidar_stop.setEnabled(True)

        # 상태 라벨 업데이트
        self.lidar_status_label.setText("충돌 체크: 실행 중")
        self.lidar_status_label.setStyleSheet("color: green;")

        # 로그 추가
        self.add_log("충돌 체크 시작")

    def stop_lidar_check(self):
        """
        LiDAR 충돌 체크 중지

        [작동]
        1. 서버에 "lidar_stop" 명령 전송
        2. 버튼 상태 변경
        3. 상태 라벨 업데이트

        [서버 동작]
        서버가 LiDAR 데이터를 LiDAR 모듈로 전달 중지
        """

        # ========================================
        # 명령 전송
        # ========================================
        message = {
            "command": "lidar_stop",
            "robot_id": self.selected_robot
        }
        self.ws_thread.send(message)

        # ========================================
        # UI 업데이트
        # ========================================
        self.lidar_active = False
        self.btn_lidar_start.setEnabled(True)
        self.btn_lidar_stop.setEnabled(False)

        # 상태 라벨 업데이트
        self.lidar_status_label.setText("충돌 체크: 대기 중")
        self.lidar_status_label.setStyleSheet("color: gray;")

        # 로그 추가
        self.add_log("충돌 체크 중지")

    # ======================================================================
    # 메시지 처리
    # ======================================================================

    def on_message(self, msg):
        """
        서버 메시지 수신 처리

        [입력]
        msg: JSON 문자열 또는 일반 텍스트

        [메시지 타입]
        1. 일반 텍스트: 로그에만 추가
           예: "서버 연결 성공!"

        2. JSON 메시지: 타입별 처리
           예: {"type": "qr_detected", "qr_text": "forward"}

        [JSON 메시지 타입]
        - qr_detected: QR 인식 결과
        - collision_warning: 충돌 경고
        - position: 위치 정보 (추후 사용)
        - init: 초기 정보 (연결 직후)

        [수정 방법]
        새 메시지 타입 추가:
        elif msg_type == 'new_type':
            # 처리 로직
            pass
        """

        # ========================================
        # 로그에 추가 (모든 메시지)
        # ========================================
        self.log.append(msg)

        # ========================================
        # JSON 파싱 시도
        # ========================================
        try:
            # JSON 문자열 → 딕셔너리
            data = json.loads(msg)

            # 메시지 타입 추출
            msg_type = data.get('type')

            # ========================================
            # 타입별 처리
            # ========================================

            if msg_type == 'qr_detected':
                """
                QR 인식 결과 처리

                [데이터 형식]
                {
                    "type": "qr_detected",
                    "robot_id": 1,
                    "qr_text": "forward"
                }

                [처리]
                1. QR 텍스트 추출
                2. 로그에 강조 표시
                3. 상태 라벨 업데이트
                """
                qr_text = data.get('qr_text', '')
                robot_id = data.get('robot_id', '')

                # 로그 추가 (강조)
                self.add_log(f"[로봇{robot_id}] QR 인식: {qr_text}")

                # 상태 라벨 업데이트
                self.qr_status_label.setText(f"QR: 인식됨 [{qr_text}]")
                self.qr_status_label.setStyleSheet("color: blue;")

            elif msg_type == 'collision_warning':
                """
                충돌 경고 처리

                [데이터 형식]
                {
                    "type": "collision_warning",
                    "robot_id": 1,
                    "message": "충돌 위험 감지! 정지함"
                }

                [처리]
                1. 경고 메시지 로그에 추가
                2. 상태 라벨을 빨간색으로 표시
                """
                robot_id = data.get('robot_id', '')
                warning_msg = data.get('message', '')

                # 로그 추가 (경고)
                self.add_log(f"[경고] [로봇{robot_id}] {warning_msg}")

                # 상태 라벨 업데이트
                self.lidar_status_label.setText("충돌 체크: 위험 감지!")
                self.lidar_status_label.setStyleSheet("color: red;")

            elif msg_type == 'position':
                """
                위치 정보 처리 (추후 구현)

                [데이터 형식]
                {
                    "type": "position",
                    "robot_id": 1,
                    "position": {"x": 1.2, "y": 0.5}
                }

                [용도]
                Navigation 연동 시 지도에 로봇 위치 표시

                [현재]
                아무 처리 안 함 (나중에 구현)
                """
                pass  # 추후 지도 표시 등에 활용

            elif msg_type == 'init':
                """
                초기 정보 처리

                [데이터 형식]
                {
                    "type": "init",
                    "robots": [1, 2, 3],
                    "modules": {
                        "qr": true,
                        "lidar": true
                    }
                }

                [처리]
                연결된 로봇 목록 로그에 표시
                """
                robots = data.get('robots', [])
                self.add_log(f"연결된 로봇: {robots}")

        except json.JSONDecodeError:
            # JSON 아니면 무시 (일반 텍스트)
            pass
        except Exception as e:
            # 기타 에러
            self.add_log(f"메시지 처리 에러: {e}")

    # ======================================================================
    # 연결 상태 처리
    # ======================================================================

    def on_connection_changed(self, connected: bool):
        """
        서버 연결 상태 변경 처리

        [입력]
        connected: True (연결됨) / False (연결 끊김)

        [호출 시점]
        WebSocketThread에서 연결 상태 변경 시
        - on_open(): connected=True
        - on_close(): connected=False

        [처리]
        1. 내부 상태 변수 업데이트
        2. 상태 라벨 업데이트
           - 연결됨: 녹색
           - 끊김: 빨간색
        3. 버튼 활성화/비활성화
        """

        # ========================================
        # 내부 상태 저장
        # ========================================
        self.connected = connected

        # ========================================
        # UI 업데이트
        # ========================================
        if connected:
            # 연결됨
            self.status_label.setText("서버 연결됨")
            self.status_label.setStyleSheet(
                "color: green; "
                "font-weight: bold;"
            )

            # 모든 버튼 활성화
            self.enable_controls(True)

        else:
            # 연결 끊김
            self.status_label.setText("서버 연결 안됨")
            self.status_label.setStyleSheet(
                "color: red; "
                "font-weight: bold;"
            )

            # 모든 버튼 비활성화
            self.enable_controls(False)

    def enable_controls(self, enabled: bool):
        """
        제어 버튼들 활성화/비활성화

        [입력]
        enabled: True (활성화) / False (비활성화)

        [처리]
        모든 제어 버튼의 setEnabled() 호출

        [용도]
        - 서버 연결 끊김 시: 모든 버튼 비활성화
        - 서버 연결됨 시: 모든 버튼 활성화
        """

        # ========================================
        # 방향 제어 버튼들
        # ========================================
        self.btn_forward.setEnabled(enabled)
        self.btn_backward.setEnabled(enabled)
        self.btn_left.setEnabled(enabled)
        self.btn_right.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)

        # ========================================
        # QR 제어 버튼
        # ========================================
        # QR이 실행 중이 아닐 때만 시작 버튼 활성화
        if enabled:
            self.btn_qr_start.setEnabled(not self.qr_active)
            self.btn_qr_stop.setEnabled(self.qr_active)
        else:
            self.btn_qr_start.setEnabled(False)
            self.btn_qr_stop.setEnabled(False)

        # ========================================
        # LiDAR 제어 버튼
        # ========================================
        # LiDAR이 실행 중이 아닐 때만 시작 버튼 활성화
        if enabled:
            self.btn_lidar_start.setEnabled(not self.lidar_active)
            self.btn_lidar_stop.setEnabled(self.lidar_active)
        else:
            self.btn_lidar_start.setEnabled(False)
            self.btn_lidar_stop.setEnabled(False)

    # ======================================================================
    # 유틸리티 함수들
    # ======================================================================

    def add_log(self, message: str):
        """
        로그 추가

        [입력]
        message: 로그 메시지 (문자열)

        [처리]
        1. 로그 텍스트 박스에 추가
        2. 자동으로 최신 로그로 스크롤

        [사용 예시]
        self.add_log("전진 시작")
        self.add_log("[에러] 연결 실패")

        [참고]
        append(): 줄 끝에 자동으로 개행 추가
        """
        self.log.append(message)

        # 자동 스크롤 (최신 로그로)
        # verticalScrollBar(): 세로 스크롤바 객체
        # setMaximum(): 스크롤바를 끝으로 이동
        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    # ======================================================================
    # 종료 처리
    # ======================================================================

    def closeEvent(self, event):
        """
        창 닫기 이벤트 처리

        [호출 시점]
        - X 버튼 클릭 시
        - Alt+F4 눌렀을 때
        - 프로그램 종료 시

        [처리]
        1. WebSocket 스레드 종료
        2. 서버 연결 닫기
        3. 리소스 정리

        [중요]
        이 함수가 없으면:
        - 창은 닫히지만 스레드가 계속 실행됨
        - 프로세스가 백그라운드에 남음
        - 메모리 누수

        [입력]
        event: QCloseEvent 객체
          - event.accept(): 종료 허용
          - event.ignore(): 종료 취소
        """

        print("GUI 종료 중")

        # ========================================
        # WebSocket 스레드 종료
        # ========================================
        if hasattr(self, 'ws_thread'):
            # stop(): 스레드에 종료 신호 전송
            self.ws_thread.stop()

            # wait(): 스레드가 완전히 종료될 때까지 대기
            # 최대 2초 대기
            self.ws_thread.wait(2000)  # 2000ms = 2초

        # ========================================
        # 종료 허용
        # ========================================
        event.accept()

        print("종료 완료")

# ======================================================================
# 메인 실행
# ======================================================================

def main():
    """
    GUI 앱 메인 함수

    [실행 순서]
    1. QApplication 생성
    2. GUI 윈도우 생성
    3. 윈도우 표시
    4. 이벤트 루프 시작

    [이벤트 루프]
    app.exec_():
    - 무한 루프
    - 마우스 클릭, 키보드 입력 등 모든 이벤트 처리
    - 창 닫으면 종료

    [반환값]
    종료 코드:
    - 0: 정상 종료
    - 기타: 에러 발생
    """

    # ========================================
    # QApplication 생성
    # ========================================
    # [설명]
    # PyQt5 앱을 실행하려면 QApplication이 필수
    # 프로그램당 1개만 생성
    #
    # [인자]
    # sys.argv: 커맨드 라인 인자
    #   예: python gui.py --debug
    #       sys.argv = ['gui.py', '--debug']
    app = QApplication(sys.argv)

    # ========================================
    # GUI 윈도우 생성
    # ========================================
    window = TurtlebotGUI()

    # ========================================
    # 윈도우 표시
    # ========================================
    # show(): 윈도우를 화면에 표시
    # (생성만 하고 show() 안 하면 화면에 안 보임)
    window.show()

    # ========================================
    # 이벤트 루프 시작
    # ========================================
    # exec_(): 이벤트 루프 실행 (블로킹)
    # 창이 닫힐 때까지 계속 실행됨
    #
    # [이벤트 루프가 하는 일]
    # - 마우스 클릭 감지 → 버튼의 clicked 시그널 발생
    # - 키보드 입력 감지 → keyPressEvent() 호출
    # - 타이머 만료 → 타이머 콜백 호출
    # - 시그널 발생 → 연결된 슬롯 호출
    #
    # [종료]
    # - 창 닫기 → closeEvent() 호출 → 이벤트 루프 종료
    # - exec_() 반환 → sys.exit()로 프로그램 종료
    sys.exit(app.exec_())

# ======================================================================
# 직접 실행 시 (python gui/gui.py)
# ======================================================================
if __name__ == "__main__":
    """
    이 파일을 직접 실행했을 때만 실행되는 코드

    [실행 방법]
    $ python3 gui/gui.py

    또는
    $ python3 -m gui.gui

    [import 시]
    다른 파일에서 import할 때는 실행 안 됨
    예: from gui.gui import TurtlebotGUI
        (main() 실행 안 됨)
    """

    print("=" * 60)
    print("터틀봇 제어 GUI 시작")
    print("=" * 60)
    print(f"서버 주소: {SERVER_IP}:{SERVER_PORT}")
    print("=" * 60)

    # GUI 실행
    main()
