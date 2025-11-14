"""
=================================================================
FastAPI 서버 + 멀티 로봇 통합 제어
=================================================================
[목적]
- FastAPI로 WebSocket 서버 실행
- MultiRobotManager로 로봇들 제어
- QR/LiDAR 모듈 연동
- GUI 클라이언트와 실시간 통신

[실행 방법]
$ cd ~/작업공간/turtle_project
$ python3 server/server.py

또는
$ python3 -m server.server

[접속 주소]
- HTTP: http://서버IP:8080
- WebSocket: ws://서버IP:8080/ws

[구조]
서버 (FastAPI)
  ├─ MultiRobotManager (로봇 제어)
  │    ├─ 브릿지 프로세스 1
  │    ├─ 브릿지 프로세스 2
  │    └─ 브릿지 프로세스 3
  ├─ QR 모듈 (QR 인식)
  ├─ LiDAR 모듈 (충돌 회피)
  └─ WebSocket (GUI 통신)

[데이터 흐름]
GUI → WebSocket → 서버 → 브릿지 → 터틀봇
터틀봇 → 브릿지 → 서버 → 모듈 → WebSocket → GUI
=================================================================
"""

import json
import asyncio
import time
import cv2
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# 프로젝트 모듈 import
from server.ros_bridge import MultiRobotManager
from shared.config import ROBOTS, SERVER_IP, SERVER_PORT

# QR 모듈 import (선택적)
try:
    from modules.qr_module import QRModule
    QR_AVAILABLE = True
except Exception as e:
    print(f"[경고] QR 모듈 로드 실패: {e}")
    print("  QR 기능 없이 실행됩니다")
    QR_AVAILABLE = False

# LiDAR 모듈 import (선택적)
try:
    from modules.lidar_module import LidarModule
    LIDAR_AVAILABLE = True
except Exception as e:
    print(f"[경고] LiDAR 모듈 로드 실패: {e}")
    print("  LiDAR 기능 없이 실행됩니다")
    LIDAR_AVAILABLE = False

# ======================================================================
# FastAPI 앱 생성
# ======================================================================
app = FastAPI(
    title="터틀봇 제어 서버",
    description="멀티 터틀봇 제어 및 센서 데이터 처리",
    version="2.0"
)

# ======================================================================
# CORS 설정 (다른 도메인에서 접속 허용)
# ======================================================================
# [설명]
# CORS: Cross-Origin Resource Sharing
# 웹 브라우저 보안 정책으로 다른 도메인 접근 제한
# 개발/테스트 시에는 모두 허용
#
# [주의]
# 실제 배포 시에는 allow_origins를 특정 도메인으로 제한
# 예: allow_origins=["http://192.168.30.5:8080"]
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],        # 모든 도메인 허용 (개발용)
    allow_credentials=True,     # 쿠키 허용
    allow_methods=["*"],        # 모든 HTTP 메서드 허용
    allow_headers=["*"],        # 모든 헤더 허용
)

# ======================================================================
# 전역 변수
# ======================================================================

# [1] 로봇 매니저 생성
# MultiRobotManager: 모든 터틀봇 제어
robot_manager = MultiRobotManager()

# [2] WebSocket 클라이언트 목록
# 연결된 GUI 클라이언트들 저장
# 여러 GUI가 동시에 연결 가능
clients = []

# ======================================================================
# 로봇 컨트롤러 래퍼 (QR 모듈에 전달)
# ======================================================================
class RobotControllerWrapper:
    """
    QR 모듈이 사용할 로봇 제어 인터페이스

    [목적]
    QR 모듈이 직접 로봇을 제어할 수 있도록
    간단한 인터페이스 제공

    [제공 함수]
    - move(robot_id, linear, angular): 로봇 이동
    - stop(robot_id): 로봇 정지
    - sleep(seconds): 대기
    - navigate_to(robot_id, x, y): Navigation (추후 구현)

    [특징]
    - 동기식 함수 (time.sleep 사용)
    - QR 모듈이 쉽게 사용 가능
    - 실제 제어는 robot_manager에게 위임
    """

    def __init__(self, robot_manager):
        """
        초기화

        [입력]
        - robot_manager: MultiRobotManager 인스턴스
        """
        self.robot_manager = robot_manager

    def move(self, robot_id: int, linear: float, angular: float):
        """
        로봇 이동

        [입력]
        - robot_id: 로봇 번호 (1, 2, 3)
        - linear: 직진 속도 (m/s)
        - angular: 회전 속도 (rad/s)

        [출력]
        - True: 성공
        - False: 실패

        [사용 예시]
        self.robot_controller.move(1, 0.2, 0.0)  # 로봇1 전진
        """
        success = self.robot_manager.move_robot(robot_id, linear, angular)
        if success:
            print(f"[제어] 로봇{robot_id} 이동: L={linear:.2f}, A={angular:.2f}")
        return success

    def stop(self, robot_id: int):
        """
        로봇 정지

        [입력]
        - robot_id: 로봇 번호

        [출력]
        - True: 성공
        - False: 실패

        [사용 예시]
        self.robot_controller.stop(1)  # 로봇1 정지
        """
        success = self.robot_manager.stop_robot(robot_id)
        if success:
            print(f"[제어] 로봇{robot_id} 정지")
        return success

    def sleep(self, seconds: float):
        """
        대기

        [입력]
        - seconds: 대기 시간 (초)

        [주의]
        동기 함수이므로 이 시간 동안 서버 블로킹
        짧은 시간만 사용 권장 (최대 2초)

        [사용 예시]
        self.robot_controller.sleep(1.0)  # 1초 대기
        """
        time.sleep(seconds)

    def navigate_to(self, robot_id: int, x: float, y: float):
        """
        Navigation 목표 설정 (추후 구현)

        [입력]
        - robot_id: 로봇 번호
        - x: 목표 x 좌표 (미터)
        - y: 목표 y 좌표 (미터)

        [상태]
        현재는 로그만 출력
        Navigation2 연동 시 구현 예정

        [사용 예시]
        self.robot_controller.navigate_to(1, 2.0, 3.0)
        """
        print(f"[Navigation] 로봇{robot_id} → ({x}, {y})")
        # TODO: Navigation2 연동
        # nav2 goal 퍼블리시
        pass

# ======================================================================
# 모듈 초기화
# ======================================================================

# [3] 로봇 컨트롤러 래퍼 생성
robot_controller = RobotControllerWrapper(robot_manager)

# [4] QR 모듈 생성 (로봇 컨트롤러 전달)
if QR_AVAILABLE:
    qr_module = QRModule(robot_controller=robot_controller)
else:
    qr_module = None

# [5] LiDAR 모듈 생성
if LIDAR_AVAILABLE:
    lidar_module = LidarModule()
else:
    lidar_module = None

# [6] 모듈 활성화 상태
# {robot_id: True/False}
# GUI에서 "QR 시작" 버튼 누르면 True
qr_enabled = {}      # QR 인식 활성화 여부
lidar_enabled = {}   # LiDAR 체크 활성화 여부

# ======================================================================
# 서버 시작/종료 이벤트
# ======================================================================

@app.on_event("startup")
async def startup_event():
    """
    서버 시작 시 실행

    [작업]
    1. 터틀봇들 연결
    2. 모듈 초기화 확인
    3. 백그라운드 작업 시작

    [호출 시점]
    uvicorn.run() 실행 직후 자동 호출
    """

    print("=" * 60)
    print("터틀봇 제어 서버 시작")
    print("=" * 60)

    # ========================================
    # 1단계: 터틀봇 연결
    # ========================================
    print("\n[1단계] 터틀봇 연결 중")

    # config.py의 ROBOTS 딕셔너리에서 로봇 정보 읽기
    for robot_id, info in ROBOTS.items():
        print(f"  연결 중: {info['name']} (ID:{robot_id}, Domain:{info['domain_id']})")
        robot_manager.add_robot(robot_id, info['domain_id'])

    # ========================================
    # 2단계: 모듈 상태 초기화
    # ========================================
    print("\n[2단계] 모듈 초기화")

    # 모든 로봇의 모듈 비활성화 상태로 시작
    for robot_id in ROBOTS.keys():
        qr_enabled[robot_id] = False
        lidar_enabled[robot_id] = False

    # 로드된 모듈 표시
    if QR_AVAILABLE:
        print("  QR 모듈 로드됨")
    else:
        print("  QR 모듈 없음 (선택 기능)")

    if LIDAR_AVAILABLE:
        print("  LiDAR 모듈 로드됨")
    else:
        print("  LiDAR 모듈 없음 (선택 기능)")

    # ========================================
    # 3단계: 백그라운드 작업 시작
    # ========================================
    print("\n[3단계] 백그라운드 작업 시작")

    # process_robot_status() 함수를 백그라운드에서 실행
    # asyncio.create_task(): 비동기 작업 생성
    asyncio.create_task(process_robot_status())

    print("\n" + "=" * 60)
    print("서버 준비 완료")
    print(f"HTTP: http://{SERVER_IP}:{SERVER_PORT}")
    print(f"WebSocket: ws://{SERVER_IP}:{SERVER_PORT}/ws")
    print("=" * 60)

@app.on_event("shutdown")
async def shutdown_event():
    """
    서버 종료 시 실행

    [작업]
    1. 모든 로봇 정지
    2. 브릿지 프로세스 종료
    3. 리소스 정리

    [호출 시점]
    Ctrl+C 또는 서버 종료 시 자동 호출
    """
    print("\n서버 종료 중")

    # 모든 로봇 정지
    print("  모든 로봇 정지")
    for robot_id in ROBOTS.keys():
        robot_manager.stop_robot(robot_id)

    # 브릿지 프로세스 종료
    print("  브릿지 프로세스 종료")
    robot_manager.shutdown()

    print("종료 완료")

# ======================================================================
# HTTP 엔드포인트
# ======================================================================

@app.get("/")
def home():
    """
    서버 상태 확인

    [접속]
    브라우저에서 http://서버IP:8080 열기

    [응답]
    {
        "message": "터틀봇 제어 서버 작동 중",
        "connected_robots": [1, 2, 3],
        "modules": {
            "qr": true,
            "lidar": true
        }
    }

    [용도]
    - 서버 작동 확인
    - 연결된 로봇 확인
    - 모듈 로드 상태 확인
    """
    return {
        "message": "터틀봇 제어 서버 작동 중",
        "connected_robots": list(robot_manager.processes.keys()),
        "modules": {
            "qr": QR_AVAILABLE,
            "lidar": LIDAR_AVAILABLE
        }
    }

@app.get("/robots")
def get_robots():
    """
    연결된 로봇 목록

    [접속]
    http://서버IP:8080/robots

    [응답]
    {
        "robots": [
            {
                "id": 1,
                "name": "터틀봇1",
                "domain_id": 17,
                "connected": true
            },
            ...
        ]
    }

    [용도]
    - GUI 초기화 시 로봇 목록 가져오기
    - 연결 상태 확인
    """
    return {
        "robots": [
            {
                "id": rid,
                "name": ROBOTS[rid]['name'],
                "domain_id": ROBOTS[rid]['domain_id'],
                "connected": rid in robot_manager.processes
            }
            for rid in ROBOTS.keys()
        ]
    }

# ======================================================================
# WebSocket 엔드포인트
# ======================================================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket 연결 처리

    [접속]
    GUI에서 ws://서버IP:8080/ws 로 연결

    [통신 방식]
    양방향 실시간 통신
    - GUI → 서버: 명령 전송
    - 서버 → GUI: 상태 전송

    [메시지 형식]
    JSON 문자열
    {"command": "move", "robot_id": 1, ...}

    [연결 유지]
    GUI가 종료되거나 네트워크 끊기면 자동 종료
    """

    # ========================================
    # 연결 수락
    # ========================================
    await websocket.accept()
    clients.append(websocket)
    print(f"[WebSocket] 클라이언트 연결 (총 {len(clients)}개)")

    # ========================================
    # 초기 정보 전송
    # ========================================
    # GUI가 연결되자마자 서버 상태 전송
    await websocket.send_text(json.dumps({
        "type": "init",
        "robots": list(robot_manager.processes.keys()),
        "modules": {
            "qr": QR_AVAILABLE,
            "lidar": LIDAR_AVAILABLE
        }
    }))

    try:
        # ========================================
        # 메시지 수신 루프
        # ========================================
        while True:
            # 메시지 대기 (블로킹)
            # GUI에서 메시지 보낼 때까지 대기
            data = await websocket.receive_text()

            try:
                # JSON 파싱
                msg = json.loads(data)

                # 명령 처리
                response = await handle_command(msg)

                # 응답 전송
                await websocket.send_text(json.dumps(response))

            except json.JSONDecodeError:
                # JSON 형식 오류
                await websocket.send_text(json.dumps({
                    "status": "error",
                    "message": "JSON 형식 오류"
                }))

            except Exception as e:
                # 기타 에러
                await websocket.send_text(json.dumps({
                    "status": "error",
                    "message": str(e)
                }))

    except WebSocketDisconnect:
        # 연결 끊김 (정상 종료)
        print(f"[WebSocket] 클라이언트 연결 끊김")
        if websocket in clients:
            clients.remove(websocket)

async def handle_command(msg: dict) -> dict:
    """
    GUI에서 받은 명령 처리

    [입력]
    msg: 명령 딕셔너리
    {
        "command": "move",
        "robot_id": 1,
        "linear": 0.2,
        "angular": 0.0
    }

    [출력]
    응답 딕셔너리
    {
        "status": "ok" 또는 "error",
        "message": "..."
    }

    [지원 명령]
    - move: 로봇 이동
    - stop: 로봇 정지
    - qr_detect: QR 인식 시작
    - qr_stop: QR 인식 중지
    - lidar_check: 충돌 체크 시작
    - lidar_stop: 충돌 체크 중지

    [확장 방법]
    새 명령 추가:
    elif command == 'new_command':
        # 처리 로직
        return {"status": "ok", ...}
    """

    # 명령 타입과 로봇 ID 추출
    command = msg.get('command')
    robot_id = msg.get('robot_id', 1)

    # ========================================
    # 로봇 존재 확인
    # ========================================
    if robot_id not in robot_manager.processes:
        return {
            "status": "error",
            "message": f"로봇 {robot_id}이 연결되지 않음"
        }

    # ========================================
    # 기본 이동 명령
    # ========================================

    if command == 'move':
        """
        로봇 이동 명령

        [필수 파라미터]
        - linear: 직진 속도
        - angular: 회전 속도
        """
        linear = msg.get('linear', 0.0)
        angular = msg.get('angular', 0.0)

        success = robot_manager.move_robot(robot_id, linear, angular)

        if success:
            return {
                "status": "ok",
                "command": "move",
                "robot_id": robot_id
            }
        else:
            return {
                "status": "error",
                "message": "명령 전송 실패"
            }

    elif command == 'stop':
        """
        로봇 정지 명령
        """
        success = robot_manager.stop_robot(robot_id)

        if success:
            return {
                "status": "ok",
                "command": "stop",
                "robot_id": robot_id
            }
        else:
            return {
                "status": "error",
                "message": "명령 전송 실패"
            }

    # ========================================
    # QR 인식 제어
    # ========================================

    elif command == 'qr_detect':
        """
        QR 인식 시작

        [작동]
        1. qr_enabled[robot_id] = True 설정
        2. 백그라운드 작업에서 카메라 데이터 처리 시작
        3. QR 인식되면 GUI로 전송
        """
        if not QR_AVAILABLE:
            return {
                "status": "error",
                "message": "QR 모듈 없음"
            }

        qr_enabled[robot_id] = True
        return {
            "status": "ok",
            "message": f"로봇 {robot_id} QR 인식 시작"
        }

    elif command == 'qr_stop':
        """
        QR 인식 중지
        """
        qr_enabled[robot_id] = False
        return {
            "status": "ok",
            "message": f"로봇 {robot_id} QR 인식 중지"
        }

    # ========================================
    # LiDAR 충돌 체크 제어
    # ========================================

    elif command == 'lidar_check':
        """
        LiDAR 충돌 체크 시작

        [작동]
        1. lidar_enabled[robot_id] = True 설정
        2. 백그라운드 작업에서 LiDAR 데이터 처리 시작
        3. 충돌 위험 감지 시 자동 정지 + GUI 알림
        """
        if not LIDAR_AVAILABLE:
            return {
                "status": "error",
                "message": "LiDAR 모듈 없음"
            }

        lidar_enabled[robot_id] = True
        return {
            "status": "ok",
            "message": f"로봇 {robot_id} 충돌 체크 시작"
        }

    elif command == 'lidar_stop':
        """
        LiDAR 충돌 체크 중지
        """
        lidar_enabled[robot_id] = False
        return {
            "status": "ok",
            "message": f"로봇 {robot_id} 충돌 체크 중지"
        }

    # ========================================
    # 알 수 없는 명령
    # ========================================

    else:
        return {
            "status": "error",
            "message": f"알 수 없는 명령: {command}"
        }

# ======================================================================
# 백그라운드 작업
# ======================================================================

async def process_robot_status():
    """
    로봇 상태 처리 (백그라운드)

    [작동]
    무한 루프로 계속 실행되며:
    1. 브릿지 프로세스에서 센서 데이터 가져오기
    2. QR/LiDAR 모듈로 처리
    3. 결과를 GUI로 브로드캐스트

    [주기]
    0.1초마다 체크 (10Hz)
    - 너무 빠르면: CPU 부하
    - 너무 느리면: 반응 지연

    [비동기]
    asyncio.sleep() 사용으로 다른 작업 블로킹 안 함
    """

    print("[백그라운드] 상태 처리 시작")

    while True:
        try:
            # ========================================
            # 모든 로봇 상태 가져오기
            # ========================================
            # robot_manager.get_all_status():
            # {
            #     1: [status1, status2, ...],
            #     2: [...],
            #     ...
            # }
            all_status = robot_manager.get_all_status()

            # ========================================
            # 각 로봇의 상태 처리
            # ========================================
            for robot_id, statuses in all_status.items():
                for status in statuses:
                    # 개별 상태 처리
                    await process_status(robot_id, status)

            # ========================================
            # 0.1초 대기
            # ========================================
            # await: 다른 비동기 작업 실행 가능
            await asyncio.sleep(0.1)

        except Exception as e:
            print(f"[백그라운드 에러] {e}")
            # 에러 발생 시 1초 대기 후 재시도
            await asyncio.sleep(1)

async def process_status(robot_id: int, status: dict):
    """
    개별 상태 처리

    [입력]
    status: 상태 딕셔너리
    {
        'type': 'odom' | 'scan' | 'camera',
        'robot_id': 1,
        'data': {...}
    }

    [처리]
    1. 타입별로 적절한 모듈에 전달
    2. 모듈 처리 결과를 GUI로 전송

    [타입별 처리]
    - odom: GUI로 위치 정보 전송
    - scan: LiDAR 모듈로 충돌 체크
    - camera: QR 모듈로 QR 인식
    """

    status_type = status.get('type')
    data = status.get('data')

    # ========================================
    # 카메라 → QR 인식
    # ========================================
    if status_type == 'camera' and qr_enabled.get(robot_id, False):
        """
        카메라 이미지 처리

        [조건]
        1. 타입이 'camera'
        2. 해당 로봇의 QR 인식이 활성화됨
        3. QR 모듈이 로드됨

        [처리]
        1. 압축 이미지 디코딩
        2. QR 모듈로 인식
        3. 결과를 GUI로 전송

        [데이터 형식]
        data = {
            'data': bytes(...),  # 압축된 이미지
            'format': 'jpeg'
        }
        """
        if QR_AVAILABLE and qr_module:
            try:
                # ========================================
                # 이미지 디코딩
                # ========================================
                # bytes → numpy 배열 → OpenCV 이미지
                np_arr = np.frombuffer(data['data'], np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                # ========================================
                # QR 인식
                # ========================================
                # qr_module.process_image()가:
                # 1. QR 인식
                # 2. on_qr_detected() 호출 (로봇 제어)
                # 3. QR 텍스트 반환
                qr_text = qr_module.process_image(image, time.time(), robot_id)

                # ========================================
                # 결과 전송
                # ========================================
                if qr_text:
                    # QR이 인식되었으면 GUI로 알림
                    await broadcast({
                        "type": "qr_detected",
                        "robot_id": robot_id,
                        "qr_text": qr_text
                    })

            except Exception as e:
                print(f"[QR 처리 에러] {e}")

    # ========================================
    # LiDAR → 충돌 체크
    # ========================================
    elif status_type == 'scan' and lidar_enabled.get(robot_id, False):
        """
        LiDAR 데이터 처리

        [조건]
        1. 타입이 'scan'
        2. 해당 로봇의 충돌 체크가 활성화됨
        3. LiDAR 모듈이 로드됨

        [처리]
        1. ranges 데이터 추출
        2. LiDAR 모듈로 충돌 체크
        3. 위험 감지 시:
           - 로봇 긴급 정지
           - GUI로 경고 전송

        [데이터 형식]
        data = {
            'ranges': [1.2, 1.3, ..., 0.5],  # 360개
            'angle_min': -3.14,
            'angle_max': 3.14
        }
        """
        if LIDAR_AVAILABLE and lidar_module:
            try:
                # ========================================
                # 거리 데이터 추출
                # ========================================
                ranges = data['ranges']

                # ========================================
                # 충돌 체크
                # ========================================
                # moving_forward=True: 전진 중 (전방 체크)
                # 후진 구현 시: GUI에서 방향 정보 전달 필요
                collision = lidar_module.check_collision(ranges, moving_forward=True)

                # ========================================
                # 충돌 위험 처리
                # ========================================
                if collision:
                    # 긴급 정지
                    robot_manager.stop_robot(robot_id)

                    # GUI로 경고 전송
                    await broadcast({
                        "type": "collision_warning",
                        "robot_id": robot_id,
                        "message": "충돌 위험 감지! 정지함"
                    })

            except Exception as e:
                print(f"[LiDAR 처리 에러] {e}")

    # ========================================
    # 위치 정보 → GUI 전송
    # ========================================
    elif status_type == 'odom':
        """
        Odometry 데이터 전송

        [처리]
        위치 정보를 GUI로 전송
        (Navigation 연동 시 지도에 표시)

        [데이터 형식]
        data = {
            'x': 1.2,
            'y': 0.5,
            'z': 0.0
        }
        """
        await broadcast({
            "type": "position",
            "robot_id": robot_id,
            "position": data
        })

async def broadcast(message: dict):
    """
    모든 클라이언트에게 메시지 전송

    [입력]
    message: 전송할 딕셔너리
    {
        "type": "qr_detected",
        "robot_id": 1,
        "qr_text": "forward"
    }

    [작동]
    1. 딕셔너리 → JSON 문자열 변환
    2. 연결된 모든 GUI에 전송
    3. 연결 끊긴 클라이언트는 목록에서 제거

    [용도]
    - QR 인식 결과 전송
    - 충돌 경고 전송
    - 위치 정보 전송
    """

    # 클라이언트 없으면 리턴
    if not clients:
        return

    # JSON 변환
    json_msg = json.dumps(message)

    # 모든 클라이언트에게 전송
    # clients[:]: 복사본으로 순회 (순회 중 수정 안전)
    for client in clients[:]:
        try:
            await client.send_text(json_msg)
        except:
            # 전송 실패 (연결 끊김)
            # 목록에서 제거
            if client in clients:
                clients.remove(client)

# ======================================================================
# 서버 실행
# ======================================================================

if __name__ == "__main__":
    """
    서버 메인 실행

    [실행 방법]
    $ python3 server/server.py

    또는
    $ python3 -m server.server

    [종료]
    Ctrl+C

    [포트 변경]
    shared/config.py에서 SERVER_PORT 수정

    [IP 변경]
    shared/config.py에서 SERVER_IP 수정
    """

    print("=" * 60)
    print("터틀봇 멀티 제어 서버")
    print("=" * 60)
    print(f"HTTP: http://{SERVER_IP}:{SERVER_PORT}")
    print(f"WebSocket: ws://{SERVER_IP}:{SERVER_PORT}/ws")
    print("\n시작 중...")
    print("=" * 60)

    # uvicorn 실행
    # [파라미터]
    # - app: FastAPI 앱 객체
    # - host: 0.0.0.0 (모든 네트워크 인터페이스)
    # - port: 포트 번호 (config.py에서)
    # - log_level: 로그 레벨 (info, debug, warning, error)
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=SERVER_PORT,
        log_level="info"
    )
