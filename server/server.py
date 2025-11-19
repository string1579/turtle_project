"""
FastAPI Server + Multi-Robot Control + Navigation2
- Integrated logic for QR events
- Server-side processing emphasis
- NO EMOJIS
"""

import json
import asyncio
import time
import cv2
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import traceback

from server.ros_bridge import MultiRobotManager
from shared.config import ROBOTS, SERVER_IP, SERVER_PORT

# ==========================================
# [설정] 디버그 모드 (True: 상세 로그 출력, False: 숨김)
DEBUG = True
# ==========================================

# Optional Modules
QR_AVAILABLE = False
try:
    from modules.qr_module import QRModule
    QR_AVAILABLE = True
    print("[INFO] QR Module Loaded")
except Exception as e:
    print(f"[WARN] QR Module Failed: {e}")

LIDAR_AVAILABLE = False
try:
    from modules.lidar_module import LidarModule
    LIDAR_AVAILABLE = True
    print("[INFO] LiDAR Module Loaded")
except Exception as e:
    print(f"[WARN] LiDAR Module Failed: {e}")

app = FastAPI(title="Turtlebot Control Server", version="3.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global Managers
robot_manager = MultiRobotManager()
clients = []  # WebSocket Clients

# State Flags
qr_enabled = {}
lidar_enabled = {}

# --- Controller Wrapper for Modules ---
class RobotControllerWrapper:
    def __init__(self, manager):
        self.manager = manager

    def move(self, robot_id: int, linear: float, angular: float):
        return self.manager.move_robot(robot_id, linear, angular)

    def stop(self, robot_id: int):
        return self.manager.stop_robot(robot_id)

    def sleep(self, seconds: float):
        time.sleep(seconds)

robot_controller = RobotControllerWrapper(robot_manager)

# Initialize Modules
qr_module = QRModule(robot_controller=robot_controller) if QR_AVAILABLE else None
lidar_module = LidarModule() if LIDAR_AVAILABLE else None

# --- Startup/Shutdown Events ---

@app.on_event("startup")
async def startup_event():
    print("=" * 60)
    print("SERVER STARTUP")
    print("=" * 60)

    # 1. Connect Robots
    print("[1] Connecting to Robots...")
    for robot_id, info in ROBOTS.items():
        print(f"    Target: {info['name']} (ID:{robot_id}, Domain:{info['domain_id']})")
        robot_manager.add_robot(robot_id, info['domain_id'])

        # Init Flags
        qr_enabled[robot_id] = False
        lidar_enabled[robot_id] = False

    # 2. Start Background Loop
    print("[2] Starting Background Task...")
    asyncio.create_task(process_robot_status())

    print(f"Server Ready at http://{SERVER_IP}:{SERVER_PORT}")

@app.on_event("shutdown")
async def shutdown_event():
    print("SERVER SHUTDOWN")
    for robot_id in ROBOTS.keys():
        robot_manager.stop_robot(robot_id)
    robot_manager.shutdown()

# --- WebSocket Handler ---

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.append(websocket)

    # Send Init Info
    await websocket.send_text(json.dumps({
        "type": "init",
        "robots": list(robot_manager.processes.keys()),
        "modules": {"qr": QR_AVAILABLE, "lidar": LIDAR_AVAILABLE}
    }))

    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
                response = await handle_command(msg)
                await websocket.send_text(json.dumps(response))
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        if websocket in clients:
            clients.remove(websocket)

async def handle_command(msg: dict) -> dict:
    command = msg.get('command')
    robot_id = msg.get('robot_id', 1)

    if DEBUG:
        print(f"[DEBUG-SERVER] WS Command received: {command} for Robot {robot_id}")

    if robot_id not in robot_manager.processes:
        return {"status": "error", "message": f"Robot {robot_id} not connected"}

    # 1. Movement
    if command == 'move':
        success = robot_manager.move_robot(
            robot_id,
            float(msg.get('linear', 0.0)),
            float(msg.get('angular', 0.0))
        )
        return {"status": "ok"} if success else {"status": "error"}

    elif command == 'stop':
        robot_manager.stop_robot(robot_id)
        return {"status": "ok"}

    # 2. Navigation
    elif command == 'navigate_to':
        success = robot_manager.navigate_to_pose(
            robot_id,
            float(msg.get('x', 0.0)),
            float(msg.get('y', 0.0)),
            float(msg.get('yaw', 0.0))
        )
        return {"status": "ok"} if success else {"status": "error"}

    elif command == 'cancel_navigation':
        robot_manager.cancel_navigation(robot_id)
        return {"status": "ok"}

    # 3. Module Toggles
    elif command == 'qr_detect':
        if QR_AVAILABLE:
            qr_enabled[robot_id] = True
            if DEBUG:
                print(f"[DEBUG-SERVER] QR Detection ENABLED for Robot {robot_id}")
            return {"status": "ok", "message": "QR Started"}
        return {"status": "error", "message": "No QR Module"}

    elif command == 'qr_stop':
        qr_enabled[robot_id] = False
        if DEBUG:
            print(f"[DEBUG-SERVER] QR Detection DISABLED for Robot {robot_id}")
        return {"status": "ok", "message": "QR Stopped"}

    elif command == 'lidar_check':
        if LIDAR_AVAILABLE:
            lidar_enabled[robot_id] = True
            return {"status": "ok", "message": "LiDAR Started"}
        return {"status": "error", "message": "No LiDAR Module"}

    elif command == 'lidar_stop':
        lidar_enabled[robot_id] = False
        return {"status": "ok", "message": "LiDAR Stopped"}

    elif command == 'camera_start':
        # Just a placeholder, camera is always streaming if robot is connected
        return {"status": "ok"}

    return {"status": "error", "message": "Unknown Command"}

# --- Background Processing ---

async def process_robot_status():
    print("[Task] Status Loop Started")
    while True:
        try:
            all_status = robot_manager.get_all_status()
            for robot_id, statuses in all_status.items():
                for status in statuses:
                    await handle_single_status(robot_id, status)

            await asyncio.sleep(0.1) # 10Hz Loop
        except Exception as e:
            print(f"[Error] Loop Error: {e}")
            await asyncio.sleep(1.0)

async def handle_single_status(robot_id: int, status: dict):
    msg_type = status.get('type')
    data = status.get('data')

    # 1. Camera & QR Logic
    if msg_type == 'camera':
        # A. Always broadcast image to GUI
        if data:
            # Convert raw bytes to hex for JSON
            # NOTE: For performance, in production we might use binary websocket
            # but here we stick to JSON for compatibility with existing GUI
            try:
                np_arr = np.frombuffer(data['data'], np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                # Broadcast Image
                _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
                img_hex = buffer.tobytes().hex()
                await broadcast({
                    "type": "camera_image",
                    "robot_id": robot_id,
                    "image_data": img_hex
                })

                # B. QR Processing (if enabled)
                if qr_enabled.get(robot_id, False) and QR_AVAILABLE:
                    event_str = qr_module.process_image(image, time.time(), robot_id)

                    if event_str:
                        # Notify GUI
                        await broadcast({
                            "type": "qr_detected",
                            "robot_id": robot_id,
                            "qr_text": event_str
                        })

                        # LOGIC: Translate Event -> Action
                        print(f"[QR Action] Robot {robot_id}: {event_str}")

                        if event_str == "STOP":
                            if DEBUG: print(f"[DEBUG-SERVER] Action: STOP triggered")
                            robot_manager.stop_robot(robot_id)

                        elif event_str == "CANCEL":
                            if DEBUG: print(f"[DEBUG-SERVER] Action: CANCEL NAV triggered")
                            robot_manager.cancel_navigation(robot_id)

                        elif event_str == "MARK":
                            # Start Visual Servoing
                            if DEBUG: print(f"[DEBUG-SERVER] Action: MARK ALIGNMENT triggered")
                            robot_manager.send_command(robot_id, {'command': 'start_mark_alignment'})

                        elif event_str.startswith("NAV_GOAL:"):
                            # NAV_GOAL:name,x,y,yaw
                            try:
                                _, content = event_str.split(':', 1)
                                _, x, y, yaw = content.split(',')
                                if DEBUG: print(f"[DEBUG-SERVER] Action: NAV_GOAL to ({x}, {y}, {yaw})")
                                robot_manager.navigate_to_pose(robot_id, float(x), float(y), float(yaw))
                            except Exception as ex:
                                print(f"[Error] Bad Nav Goal: {event_str} -> {ex}")

            except Exception as e:
                print(f"[Camera Error] {e}")

    # 2. LiDAR Logic
    elif msg_type == 'scan':
        if lidar_enabled.get(robot_id, False) and LIDAR_AVAILABLE:
            ranges = data.get('ranges', [])
            if lidar_module.check_collision(ranges):
                robot_manager.stop_robot(robot_id)
                await broadcast({
                    "type": "collision_warning",
                    "robot_id": robot_id,
                    "message": "COLLISION DETECTED"
                })

    # 3. Navigation Feedback & Result
    elif msg_type == 'nav_result':
        if DEBUG:
            print(f"[DEBUG-SERVER] Nav Result: {status}")
        await broadcast({
            "type": "nav_failed" if not status.get('success') else "nav_complete",
            "robot_id": robot_id,
            "reason": status.get('reason')
        })

    elif msg_type == 'mark_complete':
        if DEBUG:
            print(f"[DEBUG-SERVER] Mark Complete Event")
        await broadcast({
            "type": "qr_detected",
            "robot_id": robot_id,
            "qr_text": "Precision Alignment Done"
        })

    # 4. Odometry
    elif msg_type == 'odom':
        await broadcast({
            "type": "position",
            "robot_id": robot_id,
            "position": data
        })

async def broadcast(message: dict):
    if not clients: return
    txt = json.dumps(message)
    for client in clients[:]:
        try:
            await client.send_text(txt)
        except:
            if client in clients: clients.remove(client)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=SERVER_PORT, log_level="info")
