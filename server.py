#!/usr/bin/env python3
"""
Simple Turtlebot Server - The Complete Final Version
- Team A/B/E: Thread-Safe Broadcast (영상 끊김 해결)
- Team D: Advanced Control & Debug Logs (주행 안정성)
- Team Last: Advanced LiDAR Box Filtering (정밀 장애물 감지)
"""

import asyncio
import json
import math
import time
import cv2
import numpy as np
from typing import Dict, Optional, List
import os
from threading import Thread

# ==========================================
# ROS Domain ID 설정
# ==========================================
os.environ['ROS_DOMAIN_ID'] = '17'  # [최종 확인] 로봇과 일치시킬 것

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage, BatteryState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# ==========================================
# 설정
# ==========================================
ROBOTS = {
    1: {"name": "터틀봇1", "ip": "192.168.30.10"},
    2: {"name": "터틀봇2", "ip": "192.168.30.2"},
    3: {"name": "터틀봇3", "ip": "192.168.30.9"}
}

SERVER_IP = "127.0.0.1"
SERVER_PORT = 8080

# ==========================================
# ROS2 노드
# ==========================================
class SimpleRobotController(Node):
    def __init__(self, loop):
        super().__init__('simple_controller')

        # [핵심] 메인 루프 저장 (스레드 안전성)
        self.app_loop = loop

        print(f"[ROS] Domain ID: {os.environ.get('ROS_DOMAIN_ID')}")

        # 로봇 상태
        self.robot_states = {}
        for rid in ROBOTS.keys():
            self.robot_states[rid] = {
                'x': 0.0, 'y': 0.0, 'yaw': 0.0,
                'battery': 0.0,
                'front_distance': float('inf'),
                'emergency': False,
                'mode': 'idle',
                'qr_enabled': False,
                'lidar_enabled': False
            }

        self.current_robot = 1
        self.websocket_clients: List[WebSocket] = []

        # 비전 관련
        self.qr_detector = cv2.QRCodeDetector()
        self.mark_aligning = False
        self.last_qr_center = None
        self.last_qr_width = 0
        self.target_qr_width = 150

        # ROS 통신 객체들
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # QoS 설정 (Best Effort - 하드웨어 호환성)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.camera_sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_callback, sensor_qos)
        self.battery_sub = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)

        # 타이머
        self.create_timer(0.1, self.control_loop)
        self.create_timer(3.0, self.publish_initial_pose)

        self.get_logger().info("Simple Controller Ready!")

    # [핵심] 스레드 안전한 Broadcast
    def broadcast(self, message):
        """ROS 스레드 -> FastAPI 메인 루프로 안전하게 전송"""
        if not self.websocket_clients:
            return

        json_msg = json.dumps(message)
        dead_clients = []

        for client in self.websocket_clients:
            try:
                # asyncio.run_coroutine_threadsafe 사용 (필수)
                asyncio.run_coroutine_threadsafe(client.send_text(json_msg), self.app_loop)
            except Exception as e:
                # print(f"[Broadcast Error] {e}")
                dead_clients.append(client)

        for dc in dead_clients:
            if dc in self.websocket_clients:
                self.websocket_clients.remove(dc)

    def control_loop(self):
        """Mark 정밀 제어"""
        if not self.mark_aligning or self.last_qr_center is None:
            return

        image_center_x = 160
        cx, cy = self.last_qr_center
        error_x = cx - image_center_x
        error_size = self.target_qr_width - self.last_qr_width

        if abs(error_x) < 20 and abs(error_size) < 20:
            self.stop()
            self.mark_aligning = False
            print("[MARK] 정밀 정렬 완료!")
            self.broadcast({'type': 'qr_detected', 'robot_id': self.current_robot, 'qr_text': 'MARK - 정렬 완료'})
            return

        angular = -error_x * 0.003
        linear = error_size * 0.001
        self.move(linear, angular)

    def odom_callback(self, msg):
        state = self.robot_states[self.current_robot]
        state['x'] = msg.pose.pose.position.x
        state['y'] = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        state['yaw'] = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)

        self.broadcast({
            'type': 'position',
            'robot_id': self.current_robot,
            'position': {'x': state['x'], 'y': state['y'], 'qz': qz, 'qw': qw}
        })

    def scan_callback(self, msg):
        """
        [Team Last] 직사각형(Box) 필터링 적용 (최종 개선판)
        전방 80cm, 좌우 폭 12cm(중심 기준) 이내 장애물만 정밀 감지
        """
        if not msg.ranges: return

        # [설정값]
        ROBOT_WIDTH_MARGIN = 0.12  # 로봇 중심에서 좌우 12cm (전체 폭 24cm)
        LOOKAHEAD_DIST = 0.8       # 전방 감지 거리 80cm

        min_dist_in_path = 99.9    # 기본값 (장애물 없음)

        ranges = msg.ranges
        num_ranges = len(ranges)
        angle_increment = msg.angle_increment

        # 계산 효율을 위해 전방 90도(좌우 45도) 범위만 루프
        indices_to_check = list(range(0, 45)) + list(range(max(0, num_ranges - 45), num_ranges))

        for i in indices_to_check:
            if i >= num_ranges: continue
            r = ranges[i]

            # 유효하지 않은 거리값 제외
            if r < 0.05 or r > LOOKAHEAD_DIST:
                continue

            # 각도 계산 (Radian)
            if i < num_ranges / 2:
                angle = i * angle_increment
            else:
                angle = (i - num_ranges) * angle_increment

            # 좌표 변환 (Polar -> Cartesian)
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # [핵심] 박스 필터링
            if 0 < x < LOOKAHEAD_DIST and abs(y) < ROBOT_WIDTH_MARGIN:
                if x < min_dist_in_path:
                    min_dist_in_path = x

        # 상태 업데이트
        state = self.robot_states[self.current_robot]
        state['front_distance'] = min_dist_in_path

        # 충돌 방지 로직
        if state['lidar_enabled'] and min_dist_in_path < 0.35 and state['mode'] != 'emergency':
            self.stop()
            print(f"[충돌 방지] 정지! 경로상 장애물: {min_dist_in_path:.2f}m")
            self.broadcast({'type': 'collision_warning', 'robot_id': self.current_robot, 'message': f'경로상 장애물: {min_dist_in_path:.2f}m'})

        # GUI 전송 (유효한 값만)
        # self.broadcast({
        #     'type': 'scan',
        #     'robot_id': self.current_robot,
        #     'ranges': [min_dist_in_path] if min_dist_in_path < 99.0 else []
        # })

    def camera_callback(self, msg):
        """[Team E] 카메라 & QR 처리 (안전 전송 적용)"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: return

            # QR 처리
            state = self.robot_states[self.current_robot]
            if state['qr_enabled']:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                data, points, _ = self.qr_detector.detectAndDecode(gray)

                if data:
                    print(f"[QR] 감지: '{data}'")
                    if points is not None:
                        points = points.reshape(-1, 2)
                        self.last_qr_center = (np.mean(points[:, 0]), np.mean(points[:, 1]))
                        self.last_qr_width = np.linalg.norm(points[0] - points[1])

                    self.handle_qr_command(data)
                    self.broadcast({'type': 'qr_detected', 'robot_id': self.current_robot, 'qr_text': data})
                else:
                    self.last_qr_center = None
                    if self.mark_aligning:
                        print("[QR] MARK 놓침 - 정지")
                        self.stop()
                        self.mark_aligning = False

            # 이미지 전송
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 50])
            image_hex = buffer.tobytes().hex()

            # 3초마다 전송 상태 로그
            current_time = time.time()
            if not hasattr(self, 'last_log_time'): self.last_log_time = 0
            if current_time - self.last_log_time > 3.0:
                # print(f"[Vision] 전송중.. 접속자: {len(self.websocket_clients)}명")
                self.last_log_time = current_time

            self.broadcast({
                'type': 'camera_image',
                'robot_id': self.current_robot,
                'image_data': image_hex
            })

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def handle_qr_command(self, data):
        cmd = data.strip().upper()
        if cmd == "STOP":
            self.stop()
            print("[QR] 정지")
        elif cmd == "MARK":
            self.mark_aligning = True
            self.robot_states[self.current_robot]['mode'] = 'mark_aligning'
            print("[QR] MARK 정밀 정렬 시작")
        elif cmd.startswith("NAV_GOAL:"):
            try:
                parts = cmd.split(':')[1].split(',')
                if len(parts) >= 2:
                    self.navigate_to(float(parts[0]), float(parts[1]))
                    print(f"[QR] 목표 이동: {parts}")
            except: pass

    def battery_callback(self, msg):
        self.robot_states[self.current_robot]['battery'] = msg.voltage
        self.broadcast({'type': 'battery', 'robot_id': self.current_robot, 'voltage': msg.voltage})

    def navigate_to(self, x, y, yaw=0.0):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            print("[Nav] 서버 응답 없음")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)
        self.nav_client.send_goal_async(goal)
        print(f"[Nav] 목표 전송: ({x}, {y})")

    def move(self, linear, angular):
        """[Team D] 로봇 이동 (디버깅 로그 포함)"""
        state = self.robot_states[self.current_robot]
        if state['emergency']:
            print("[디버깅] 비상정지 상태라 이동 불가")
            return

        MAX_LINEAR = 0.22
        MAX_ANGULAR = 2.0

        linear = max(-MAX_LINEAR, min(MAX_LINEAR, float(linear)))
        angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, float(angular)))

        # 장애물 감지 시 전진 차단
        if linear > 0 and state['front_distance'] < 0.3:
            print(f"[디버깅] 장애물 감지됨 ({state['front_distance']:.2f}m) - 정지")
            linear = 0.0

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)
        # print("[디버깅] ROS2 토픽 발행 완료")

    def stop(self):
        self.move(0.0, 0.0)
        self.mark_aligning = False

    def set_emergency(self, robot_id, emergency):
        self.robot_states[robot_id]['emergency'] = emergency
        if emergency:
            self.stop()
            self.robot_states[robot_id]['mode'] = 'emergency'
        else:
            self.robot_states[robot_id]['mode'] = 'idle'

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.init_pose_pub.publish(msg)
        self.get_logger().info("Initial pose published")

# ==========================================
# FastAPI 서버
# ==========================================
app = FastAPI(title="Simple Turtlebot Server")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_node: Optional[SimpleRobotController] = None

@app.on_event("startup")
async def startup():
    global ros_node
    print("=" * 60)
    print(f"SERVER START - Domain ID: {os.environ.get('ROS_DOMAIN_ID')}")
    print("=" * 60)

    # [핵심] 현재 실행 중인 이벤트 루프 가져오기
    loop = asyncio.get_running_loop()

    def ros_spin():
        rclpy.init()
        global ros_node
        # Loop를 ROS 노드에 전달 (스레드 안전성)
        ros_node = SimpleRobotController(loop)
        rclpy.spin(ros_node)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    while ros_node is None:
        await asyncio.sleep(0.1)
    print("[Server] Ready!")

@app.on_event("shutdown")
async def shutdown():
    if ros_node: ros_node.destroy_node()
    rclpy.shutdown()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print(f"[WS] Connected: {websocket.client}")

    if ros_node:
        ros_node.websocket_clients.append(websocket)
        try:
            await websocket.send_text(json.dumps({'type': 'init', 'robots': list(ROBOTS.keys())}))
        except: pass
    else:
        print("[WS] Warning: ROS Node not ready")

    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
                await handle_command(msg)
            except json.JSONDecodeError:
                continue
    except WebSocketDisconnect:
        print(f"[WS] Disconnected: {websocket.client}")
    except Exception as e:
        print(f"[WS] Error: {e}")
    finally:
        if ros_node and websocket in ros_node.websocket_clients:
            ros_node.websocket_clients.remove(websocket)
            print("[WS] Client removed")

async def handle_command(msg: dict):
    if not ros_node: return

    cmd = msg.get('command')
    rid = msg.get('robot_id', 1)
    ros_node.current_robot = rid

    if cmd == 'move':
        ros_node.move(msg.get('linear', 0), msg.get('angular', 0))
        ros_node.robot_states[rid]['mode'] = 'manual'
    elif cmd == 'stop':
        ros_node.stop()
        ros_node.robot_states[rid]['mode'] = 'idle'
    elif cmd == 'emergency_stop':
        ros_node.set_emergency(rid, True)
        print(f"[Command] Emergency Stop: {rid}")
    elif cmd == 'release_emergency':
        ros_node.set_emergency(rid, False)
        print(f"[Command] Release Emergency: {rid}")
    elif cmd == 'navigate_to':
        ros_node.navigate_to(msg.get('x', 0), msg.get('y', 0), msg.get('yaw', 0))
    elif cmd == 'qr_detect':
        ros_node.robot_states[rid]['qr_enabled'] = True
        print(f"[QR] Enabled for {rid}")
    elif cmd == 'qr_stop':
        ros_node.robot_states[rid]['qr_enabled'] = False
        print(f"[QR] Disabled for {rid}")
    elif cmd == 'lidar_check':
        ros_node.robot_states[rid]['lidar_enabled'] = True
        print(f"[LiDAR] Enabled for {rid}")
    elif cmd == 'lidar_stop':
        ros_node.robot_states[rid]['lidar_enabled'] = False
        print(f"[LiDAR] Disabled for {rid}")
    elif cmd == 'camera_start':
        pass
    else:
        print(f"[WS] Unknown command: {cmd}")

if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_IP, port=SERVER_PORT, log_level="warning")
