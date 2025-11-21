#!/usr/bin/env python3
"""
Simple Turtlebot Server - Multi-Domain Fixed Version
- Fix: Robot 1 moving forward issue
- Fix: Robot 2, 3 control issue
"""

import asyncio
import json
import time
import cv2
import math
import numpy as np
from threading import Thread
import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage, BatteryState

SERVER_IP = "0.0.0.0"
SERVER_PORT = 8080

# [설정] 로봇별 도메인 ID 확인 필수!
ROBOT_DOMAINS = {
    1: 3,  # Robot 1 -> Domain 3
    2: 5,  # Robot 2 -> Domain 5
    3: 17  # Robot 3 -> Domain 17
}

class DomainBridge(Node):
    def __init__(self, robot_id, context, loop, manager):
        super().__init__(f'bridge_node_{robot_id}', context=context)

        self.robot_id = robot_id
        self.loop = loop
        self.manager = manager

        self.last_qr_cmd = ""        # [추가] 마지막 수행한 QR 명령 저장
        self.qr_debounce_time = 0    # [추가] 중복 방지 타이머

        self.state = {
            'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'battery': 0.0,
            'front_distance': 99.9, # 초기값 충분히 크게
            'emergency': False,
            'mode': 'idle',
            'qr_enabled': False,
            'lidar_enabled': False,
            'is_navigating': False,
            'mark_aligning': False,
            'target_x': 0.0, 'target_y': 0.0
        }

        self.qr_detector = cv2.QRCodeDetector()
        self.last_qr_center = None
        self.target_qr_width = 150
        self.last_qr_width = 0
        self.last_print_time = 0

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=1)
        cmd_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.camera_callback, sensor_qos)
        # 기존: cmd_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        # 수정: 아래와 같이 변경
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 변경!
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_timer(0.1, self.control_loop)
        print(f"[Init] Bridge for Robot {robot_id} initialized.")

    def control_loop(self):
        if self.state['mark_aligning'] and self.last_qr_center:
            self.process_mark_alignment()
        elif self.state['is_navigating']:
            self.process_custom_navigation()

    def process_mark_alignment(self):
        image_center_x = 160
        cx, cy = self.last_qr_center
        error_x = cx - image_center_x
        error_size = self.target_qr_width - self.last_qr_width

        if abs(error_x) < 20 and abs(error_size) < 20:
            self.stop()
            self.state['mark_aligning'] = False
            self.broadcast({'type': 'qr_detected', 'robot_id': self.robot_id, 'qr_text': 'MARK - 정렬 완료'})
            return

        angular = -error_x * 0.003
        linear = error_size * 0.001
        self.move(linear, angular)

    def process_custom_navigation(self):
        dx = self.state['target_x'] - self.state['x']
        dy = self.state['target_y'] - self.state['y']
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.state['yaw'])

        if distance < 0.10:
            self.stop()
            self.state['is_navigating'] = False
            print(f"[Robot {self.robot_id}] 도착 완료!")
            return

        # 자동 주행 중 장애물 회피 (0.35m)
        if self.state['front_distance'] < 0.35:
            self.move(0.0, 0.5)
            return

        if abs(yaw_error) > 0.3:
            angular = yaw_error * 1.5
            self.move(0.0, angular)
        else:
            linear = min(0.18, distance)
            angular = yaw_error * 2.0
            self.move(linear, angular)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg):
        cur = time.time()
        # 디버깅: 5초마다 수신 확인
        if cur - self.last_print_time > 5.0:
            print(f"[Robot {self.robot_id}] Odom receiving...")
            self.last_print_time = cur

        self.state['x'] = msg.pose.pose.position.x
        self.state['y'] = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.state['yaw'] = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)

        self.broadcast({
            'type': 'position',
            'robot_id': self.robot_id,
            'position': {'x': self.state['x'], 'y': self.state['y']}
        })

    def scan_callback(self, msg):
        ranges = [r for r in msg.ranges if r > 0.05]
        min_dist = min(ranges) if ranges else 99.9
        self.state['front_distance'] = min_dist

        # [수정] 수동 모드에서는 충돌 방지 '경고'만 보내고 정지는 안 시킴 (데모용)
        # 만약 정지시키고 싶다면 아래 주석 해제
        # if not self.state['is_navigating'] and self.state['mode'] == 'manual':
        #      if self.state['lidar_enabled'] and min_dist < 0.2:
        #         self.stop()

        if min_dist < 0.35:
             self.broadcast({'type': 'collision_warning', 'robot_id': self.robot_id, 'message': f'Obstacle: {min_dist:.2f}m'})

    def camera_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: return

            if self.state['qr_enabled']:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                data, points, _ = self.qr_detector.detectAndDecode(gray)
                if data:
                    if points is not None:
                        points = points.reshape(-1, 2)
                        self.last_qr_center = (np.mean(points[:, 0]), np.mean(points[:, 1]))
                        self.last_qr_width = np.linalg.norm(points[0] - points[1])
                    self.handle_qr_command(data)
                    self.broadcast({'type': 'qr_detected', 'robot_id': self.robot_id, 'qr_text': data})

            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 50])
            self.broadcast({
                'type': 'camera_image',
                'robot_id': self.robot_id,
                'image_data': buffer.tobytes().hex()
            })
        except: pass

    def battery_callback(self, msg):
        self.broadcast({'type': 'battery', 'robot_id': self.robot_id, 'voltage': msg.voltage})

    def handle_qr_command(self, data):
        cmd = data.strip().upper()
        current_time = time.time()

        # [핵심] 중복 명령 방지 (3초 쿨타임 or 같은 명령 무시)
        if cmd == self.last_qr_cmd and (current_time - self.qr_debounce_time < 5.0):
            return

        # 명령 수락
        self.last_qr_cmd = cmd
        self.qr_debounce_time = current_time

        print(f"[QR NEW] {cmd}") # 디버깅 로그

        if cmd == "STOP":
            self.stop()
            self.state['is_navigating'] = False
            self.state['mark_aligning'] = False
            # STOP은 언제든 먹어야 하므로 last_qr_cmd 초기화
            self.last_qr_cmd = ""

        elif cmd == "MARK":
            self.state['is_navigating'] = False
            self.state['mark_aligning'] = True
            self.state['is_setting_origin'] = False

        elif cmd.startswith("NAV_GOAL:"):
            try:
                content = cmd.split(':')[1]
                parts = content.split(',')

                # ORIGIN 체크
                if parts[0].strip() == "ORIGIN":
                    print(f"[Robot {self.robot_id}] QR ORIGIN -> Alignment")
                    self.state['is_navigating'] = False
                    self.state['mark_aligning'] = True
                    self.state['is_setting_origin'] = True
                    return

                # 좌표 파싱
                tx, ty = 0.0, 0.0
                if len(parts) == 2:
                    tx = float(parts[0])
                    ty = float(parts[1])
                elif len(parts) >= 3:
                    try:
                        tx = float(parts[1])
                        ty = float(parts[2])
                    except:
                        tx = float(parts[0])
                        ty = float(parts[1])

                self.state['target_x'] = tx
                self.state['target_y'] = ty
                self.state['is_navigating'] = True
                self.state['mark_aligning'] = False
                self.state['is_setting_origin'] = False
                print(f"[Robot {self.robot_id}] Nav -> ({tx}, {ty})")

            except Exception as e:
                print(f"[QR Parsing Error] {e}")
                self.last_qr_cmd = "" # 에러나면 다시 읽을 수 있게 초기화

    def move(self, linear, angular):
        if self.state['emergency']: return

        # [수정] 수동 모드 전진 차단 거리 완화 (0.3 -> 0.15)
        # 자동 주행 중에는 별도 로직이 있으므로 여기선 최소한의 안전만
        if not self.state['is_navigating'] and linear > 0 and self.state['front_distance'] < 0.15:
            linear = 0.0

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)
        self.state['mark_aligning'] = False

    def broadcast(self, message):
        self.manager.broadcast(message, self.loop)

class WebSocketManager:
    def __init__(self):
        self.clients = []
        self.bridges = {}

    def add_bridge(self, robot_id, bridge):
        self.bridges[robot_id] = bridge

    def get_bridge(self, robot_id):
        return self.bridges.get(robot_id)

    def broadcast(self, message, loop):
        if not self.clients: return
        txt = json.dumps(message)
        for ws in self.clients:
            try:
                asyncio.run_coroutine_threadsafe(ws.send_text(txt), loop)
            except: pass

manager = WebSocketManager()

# =============================================================
# FastAPI
# =============================================================
executors = []

def spin_executor(executor):
    try:
        executor.spin()
    except Exception as e:
        print(f"Executor Error: {e}")

@asynccontextmanager
async def lifespan(app: FastAPI):
    loop = asyncio.get_running_loop()
    print("=" * 60)
    print(f"MULTI-ROBOT SERVER START (0.0.0.0:8080)")
    print("=" * 60)

    for rid, domain in ROBOT_DOMAINS.items():
        context = rclpy.Context()
        context.init(domain_id=domain)

        bridge = DomainBridge(rid, context, loop, manager)
        manager.add_bridge(rid, bridge)

        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(bridge)
        executors.append(executor)

        t = Thread(target=spin_executor, args=(executor,), daemon=True)
        t.start()

    print("[Server] All Bridges Ready!")
    yield

app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    manager.clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            await handle_command(msg)
    except:
        manager.clients.remove(websocket)

async def handle_command(msg: dict):
    try:
        rid = int(msg.get('robot_id', 1))
        cmd = msg.get('command')

        bridge = manager.get_bridge(rid)
        if not bridge:
            print(f"Error: Bridge for Robot {rid} not found!")
            return

        # [디버깅] 로봇 3번 명령만 로그 출력
        if rid == 3:
            print(f"[Robot 3 CMD] {cmd}")

        if cmd == 'move':
            bridge.state['is_navigating'] = False
            bridge.move(msg.get('linear', 0), msg.get('angular', 0))
            bridge.state['mode'] = 'manual'
        elif cmd == 'stop':
            bridge.stop()
            bridge.state['mode'] = 'idle'
        elif cmd == 'emergency_stop':
            bridge.stop()
            bridge.state['emergency'] = True
            bridge.state['is_navigating'] = False
        elif cmd == 'release_emergency':
            bridge.state['emergency'] = False
        elif cmd == 'navigate_to':
            bridge.state['target_x'] = msg.get('x', 0)
            bridge.state['target_y'] = msg.get('y', 0)
            bridge.state['is_navigating'] = True
            bridge.state['mark_aligning'] = False
        elif cmd == 'qr_detect':
            bridge.state['qr_enabled'] = True
        elif cmd == 'qr_stop':
            bridge.state['qr_enabled'] = False
        elif cmd == 'lidar_check':
            bridge.state['lidar_enabled'] = True
        elif cmd == 'lidar_stop':
            bridge.state['lidar_enabled'] = False

    except Exception as e:
        print(f"Cmd Error: {e}")

if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_IP, port=SERVER_PORT, log_level="warning")
