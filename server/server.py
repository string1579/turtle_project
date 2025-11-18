"""
FastAPI 서버 + 멀티 로봇 통합 제어 + Navigation2 + 카메라 전송
"""

import json
import asyncio
import time
import cv2
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import rclpy

from server.ros_bridge import MultiRobotManager
from shared.config import ROBOTS, ACTIVE_ROBOTS, SERVER_IP, SERVER_PORT

# 모듈 import
try:
    from modules.qr_module import QRModule
    QR_AVAILABLE = True
except Exception as e:
    print(f"[경고] QR 모듈 로드 실패: {e}")
    QR_AVAILABLE = False

try:
    from modules.lidar_module import LidarModule
    LIDAR_AVAILABLE = True
except Exception as e:
    print(f"[경고] LiDAR 모듈 로드 실패: {e}")
    LIDAR_AVAILABLE = False

app = FastAPI(title="터틀봇 제어 서버", version="2.2")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

robot_manager = MultiRobotManager()
clients = []

class RobotControllerWrapper:
    """QR 모듈용 로봇 제어 인터페이스"""

    def __init__(self, robot_manager):
        self.robot_manager = robot_manager

    def move(self, robot_id: int, linear: float, angular: float):
        success = self.robot_manager.move_robot(robot_id, linear, angular)
        if success:
            print(f"[제어] 로봇{robot_id} 이동: L={linear:.2f}, A={angular:.2f}")
        return success

    def stop(self, robot_id: int):
        success = self.robot_manager.stop_robot(robot_id)
        if success:
            print(f"[제어] 로봇{robot_id} 정지")
        return success

    def sleep(self, seconds: float):
        time.sleep(seconds)

robot_controller = None
qr_module = None
lidar_module = None

qr_enabled = {}
lidar_enabled = {}
camera_enabled = {}

@app.on_event("startup")
async def startup_event():
    """서버 시작"""
    global robot_controller, qr_module, lidar_module

    print("=" * 60)
    print("서버 시작")
    print("=" * 60)

    # [1] 터틀봇 연결
    print("\n[1] 터틀봇 연결")
    for robot_id in ACTIVE_ROBOTS:
        if robot_id in ROBOTS:
            info = ROBOTS[robot_id]
            robot_manager.add_robot(robot_id, info['domain_id'])
        else:
            print(f"[경고] 로봇 {robot_id}은 ROBOTS에 정의되지 않음")

    # [2] 제어 래퍼
    print("\n[2] 제어 래퍼 생성")
    robot_controller = RobotControllerWrapper(robot_manager)

    # [3] 모듈 초기화
    print("\n[3] 모듈 초기화")
    for robot_id in ROBOTS.keys():
        qr_enabled[robot_id] = False
        lidar_enabled[robot_id] = False
        camera_enabled[robot_id] = False

    if QR_AVAILABLE:
        qr_module = QRModule(robot_controller=robot_controller)

    if LIDAR_AVAILABLE:
        lidar_module = LidarModule()

    # [4] 백그라운드
    print("\n[4] 백그라운드 작업 시작")
    asyncio.create_task(process_robot_status())

    print("\n서버 준비 완료!")

@app.on_event("shutdown")
async def shutdown_event():
    """서버 종료"""
    print("\n서버 종료 중")

    for robot_id in ROBOTS.keys():
        robot_manager.stop_robot(robot_id)

    robot_manager.shutdown()

    if rclpy.ok():
        rclpy.shutdown()

    print("종료 완료")

@app.get("/")
def home():
    """서버 상태 확인"""
    return {
        "message": "터틀봇 제어 서버 작동 중",
        "connected_robots": list(robot_manager.processes.keys()),
        "modules": {
            "qr": QR_AVAILABLE,
            "lidar": LIDAR_AVAILABLE
        }
    }

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 연결 처리"""
    await websocket.accept()
    clients.append(websocket)
    print(f"[WebSocket] 클라이언트 연결 (총 {len(clients)}개)")

    # 초기 정보 전송
    await websocket.send_text(json.dumps({
        "type": "init",
        "robots": list(robot_manager.processes.keys()),
        "modules": {
            "qr": QR_AVAILABLE,
            "lidar": LIDAR_AVAILABLE
        }
    }))

    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            response = await handle_command(msg)
            await websocket.send_text(json.dumps(response))

    except WebSocketDisconnect:
        print(f"[WebSocket] 클라이언트 연결 끊김")
        if websocket in clients:
            clients.remove(websocket)

async def handle_command(msg: dict) -> dict:
    """GUI 명령 처리"""
    command = msg.get('command')
    robot_id = msg.get('robot_id', 1)

    if robot_id not in robot_manager.processes:
        return {
            "status": "error",
            "message": f"로봇 {robot_id}이 연결되지 않음"
        }

    # 이동 명령
    if command == 'move':
        linear = msg.get('linear', 0.0)
        angular = msg.get('angular', 0.0)
        success = robot_manager.move_robot(robot_id, linear, angular)

        if success:
            return {"status": "ok", "command": "move", "robot_id": robot_id}
        else:
            return {"status": "error", "message": "명령 전송 실패"}

    elif command == 'stop':
        success = robot_manager.stop_robot(robot_id)
        if success:
            return {"status": "ok", "command": "stop", "robot_id": robot_id}
        else:
            return {"status": "error", "message": "명령 전송 실패"}

    # QR 제어
    elif command == 'qr_detect':
        if not QR_AVAILABLE:
            return {"status": "error", "message": "QR 모듈 없음"}
        qr_enabled[robot_id] = True
        return {"status": "ok", "message": f"로봇 {robot_id} QR 인식 시작"}

    elif command == 'qr_stop':
        qr_enabled[robot_id] = False
        return {"status": "ok", "message": f"로봇 {robot_id} QR 인식 중지"}

    # LiDAR 제어
    elif command == 'lidar_check':
        if not LIDAR_AVAILABLE:
            return {"status": "error", "message": "LiDAR 모듈 없음"}
        lidar_enabled[robot_id] = True
        return {"status": "ok", "message": f"로봇 {robot_id} 충돌 체크 시작"}

    elif command == 'lidar_stop':
        lidar_enabled[robot_id] = False
        return {"status": "ok", "message": f"로봇 {robot_id} 충돌 체크 중지"}

    # 카메라 제어
    elif command == 'camera_start':
        camera_enabled[robot_id] = True
        return {"status": "ok", "message": f"로봇 {robot_id} 카메라 전송 시작"}

    elif command == 'camera_stop':
        camera_enabled[robot_id] = False
        return {"status": "ok", "message": f"로봇 {robot_id} 카메라 전송 중지"}

    # Navigation 명령
    elif command == 'navigate_to':
        x = msg.get('x', 0.0)
        y = msg.get('y', 0.0)
        yaw = msg.get('yaw', 0.0)

        success = robot_manager.navigate_to(robot_id, x, y, yaw)

        if success:
            return {
                "status": "ok",
                "message": f"로봇 {robot_id} 이동 시작",
                "target": {"x": x, "y": y, "yaw": yaw}
            }
        else:
            return {"status": "error", "message": "이동 명령 실패"}

    elif command == 'cancel_navigation':
        success = robot_manager.cancel_navigation(robot_id)

        if success:
            return {"status": "ok", "message": f"로봇 {robot_id} 취소"}
        else:
            return {"status": "error", "message": "취소 실패"}

async def process_robot_status():
    """백그라운드 상태 처리 (10Hz)"""
    print("[백그라운드] 상태 처리 시작")

    while True:
        all_status = robot_manager.get_all_status()

        for robot_id, statuses in all_status.items():
            for status in statuses:
                await process_status(robot_id, status)

        await asyncio.sleep(0.1)

async def process_status(robot_id: int, status: dict):
    """개별 상태 처리"""
    status_type = status.get('type')
    data = status.get('data')

    # 카메라 → GUI 전송
    if status_type == 'camera' and camera_enabled.get(robot_id, False):
        np_arr = np.frombuffer(data['data'], np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # JPEG 인코딩 후 Hex 문자열로 전송
        _, buffer = cv2.imencode('.jpg', image)
        img_hex = buffer.tobytes().hex()

        await broadcast({
            "type": "camera_image",
            "robot_id": robot_id,
            "image_data": img_hex
        })

    # 카메라 → QR 인식
    if status_type == 'camera' and qr_enabled.get(robot_id, False):
        if QR_AVAILABLE and qr_module:
            np_arr = np.frombuffer(data['data'], np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            qr_text = qr_module.process_image(image, time.time(), robot_id)

            if qr_text:
                await broadcast({
                    "type": "qr_detected",
                    "robot_id": robot_id,
                    "qr_text": qr_text
                })

    # LiDAR → 충돌 체크
    elif status_type == 'scan' and lidar_enabled.get(robot_id, False):
        if LIDAR_AVAILABLE and lidar_module:
            ranges = data['ranges']
            collision = lidar_module.check_collision(ranges, moving_forward=True)

            if collision:
                # 탈출 방향 계산
                angular = lidar_module.find_best_escape_direction(ranges)

                # 탈출 회전 명령 전송
                robot_manager.escape_turn(robot_id, angular)

                await broadcast({
                    "type": "collision_warning",
                    "robot_id": robot_id,
                    "message": f"충돌 위험! 탈출 회전 ({angular:.2f} rad/s)"
                })

    # AMCL 위치 정보
    elif status_type == 'amcl_pose':
        await broadcast({
            "type": "position",
            "robot_id": robot_id,
            "position": data
        })

    # Navigation 상태
    elif status_type == 'nav_status':
        result = status.get('result')
        reason = status.get('reason', '')

        if result == 'complete':
            await broadcast({
                "type": "nav_complete",
                "robot_id": robot_id
            })
        elif result == 'failed':
            await broadcast({
                "type": "nav_failed",
                "robot_id": robot_id,
                "reason": reason
            })

async def broadcast(message: dict):
    """모든 클라이언트에게 메시지 전송"""
    if not clients:
        return

    json_msg = json.dumps(message)
    for client in clients[:]:
        try:
            await client.send_text(json_msg)
        except:
            if client in clients:
                clients.remove(client)

if __name__ == "__main__":
    print("=" * 60)
    print("터틀봇 멀티 제어 서버 (v2.2)")
    print("=" * 60)
    print(f"HTTP: http://{SERVER_IP}:{SERVER_PORT}")
    print(f"WebSocket: ws://{SERVER_IP}:{SERVER_PORT}/ws")
    print("\n시작 중...")
    print("=" * 60)

    uvicorn.run(app, host="0.0.0.0", port=SERVER_PORT, log_level="info")
