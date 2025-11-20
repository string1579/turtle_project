#!/usr/bin/env python3
"""
Simple Turtlebot Server - Final Version (Team A Updated)
- LiDAR 박스 필터링(직사각형 감지) 적용
- 감지 폭 축소 (0.25 -> 0.12)
- QoS 설정 완료
- 스레드 안전성 확보
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
os.environ['ROS_DOMAIN_ID'] = '5'

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

        # 메인 이벤트 루프 저장
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

        # QoS 설정 (카메라용)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.camera_sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_callback, sensor_qos)
        self.battery_sub = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)

        # 타이머
        self.create_timer(0.1, self.control_loop)
        self.create_timer(3.0, self.publish_initial_pose)

        self.get_logger().info("Simple Controller Ready!")

    def broadcast(self, message):
        """ROS 스레드에서 FastAPI 루프로 안전하게 메시지 전송"""
        if not self.websocket_clients:
            return

        json_msg = json.dumps(message)
        dead_clients = []

        for client in self.websocket_clients:
            try:
                asyncio.run_coroutine_threadsafe(client.send_text(json_msg), self.app_loop)
            except Exception as e:
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
            'position': {'x': state['x'], 'y': state['y']}
        })

    def scan_callback(self, msg):
        """
        [수정됨] 직사각형(Box) 필터링 적용
        전방 80cm, 좌우 폭 24cm(중심 12cm) 이내 장애물만 감지
        """
        if not msg.ranges: return

        # [수정] 설정값 변경 (0.25 -> 0.12)
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
            self.broadcast({'type': 'collision_warning', 'robot_id': self.current_robot, 'message': f'경로상 장애물: {min_dist_in_path:.2f}m'})

        # GUI 전송
        self.broadcast({
            'type': 'scan',
            'robot_id': self.current_robot,
            'ranges': [min_dist_in_path] if min_dist_in_path < 99.0 else []
        })

    def camera_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: return

            state = self.robot_states[self.current_robot]
            if state['qr_enabled']:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                data, points, _ = self.qr_detector.detectAndDecode(gray)

                if data:
                    if points is not None:
                        points = points.reshape(-1, 2)
                        self.last_qr_center = (np.mean(points[:, 0]), np.mean(points[:, 1]))
                        self.last_qr_width = np.linalg.norm(points[0] - points[1])

                    self.handle_qr_command(data)
                    self.broadcast({'type': 'qr_detected', 'robot_id': self.current_robot, 'qr_text': data})
                else:
                    self.last_qr_center = None
                    if self.mark_aligning:
                        self.stop()
                        self.mark_aligning = False

            # 이미지 전송
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 50])
            image_hex = buffer.tobytes().hex()

            self.broadcast({
                'type': 'camera_image',
                'robot_id': self.current_robot,
                'image_data': image_hex
            })

        except Exception as e:
            pass

    def handle_qr_command(self, data):
        cmd = data.strip().upper()
        if cmd == "STOP": self.stop()
        elif cmd == "MARK":
            self.mark_aligning = True
            self.robot_states[self.current_robot]['mode'] = 'mark_aligning'
        elif cmd.startswith("NAV_GOAL:"):
            try:
                parts = cmd.split(':')[1].split(',')
                if len(parts) >= 2:
                    self.navigate_to(float(parts[0]), float(parts[1]))
            except: pass

    def battery_callback(self, msg):
        self.robot_states[self.current_robot]['battery'] = msg.voltage
        self.broadcast({'type': 'battery', 'robot_id': self.current_robot, 'voltage': msg.voltage})

    def navigate_to(self, x, y, yaw=0.0):
        if not self.nav_client.wait_for_server(timeout_sec=1.0): return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)
        self.nav_client.send_goal_async(goal)

    def move(self, linear, angular):
        state = self.robot_states[self.current_robot]
        if state['emergency']: return

        # 전진 시에만 장애물 체크
        if linear > 0 and state['front_distance'] < 0.3:
            linear = 0.0

        msg = Twist()
        msg.linear.x = max(-0.22, min(0.22, float(linear)))
        msg.angular.z = max(-2.0, min(2.0, float(angular)))
        self.cmd_pub.publish(msg)

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
        msg.pose.pose.orientation.w = 1.0
        self.init_pose_pub.publish(msg)

# ==========================================
# FastAPI 서버
# ==========================================
app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

ros_node: Optional[SimpleRobotController] = None

@app.on_event("startup")
async def startup():
    global ros_node
    print("=" * 60)
    print(f"SERVER START - Domain ID: {os.environ.get('ROS_DOMAIN_ID')}")
    print("=" * 60)

    loop = asyncio.get_running_loop()

    def ros_spin():
        rclpy.init()
        global ros_node
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

    if ros_node:
        ros_node.websocket_clients.append(websocket)
        print(f"[WS] Connected. Total clients: {len(ros_node.websocket_clients)}")
        await websocket.send_text(json.dumps({'type': 'init', 'robots': list(ROBOTS.keys())}))

    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            await handle_command(msg)
    except:
        if ros_node and websocket in ros_node.websocket_clients:
            ros_node.websocket_clients.remove(websocket)
            print("[WS] Disconnected")

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
    elif cmd == 'release_emergency':
        ros_node.set_emergency(rid, False)
    elif cmd == 'navigate_to':
        ros_node.navigate_to(msg.get('x', 0), msg.get('y', 0), msg.get('yaw', 0))
    elif cmd == 'qr_detect':
        ros_node.robot_states[rid]['qr_enabled'] = True
    elif cmd == 'qr_stop':
        ros_node.robot_states[rid]['qr_enabled'] = False
    elif cmd == 'lidar_check':
        ros_node.robot_states[rid]['lidar_enabled'] = True
    elif cmd == 'lidar_stop':
        ros_node.robot_states[rid]['lidar_enabled'] = False
    elif cmd == 'camera_start':
        pass

if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_IP, port=SERVER_PORT)
