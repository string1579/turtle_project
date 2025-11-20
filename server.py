#!/usr/bin/env python3
"""
Simple Turtlebot Server - Final Stable Version
수정: 팀원 A, B, D, E 통합 (스레드 안전성 확보)
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
os.environ['ROS_DOMAIN_ID'] = '17'  # [주의] 로봇과 반드시 일치해야 함

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
# ROS2 노드 (통합 수정됨)
# ==========================================
class SimpleRobotController(Node):
    def __init__(self, loop):
        super().__init__('simple_controller')

        # [핵심 수정] 메인 이벤트 루프 저장 (스레드 간 통신용)
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

        # QoS 설정
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

    # [팀원 A, B 핵심 수정] 스레드 안전한 브로드캐스트
    def broadcast(self, message):
        """ROS 스레드에서 FastAPI 루프로 안전하게 메시지 전송"""
        if not self.websocket_clients:
            return

        json_msg = json.dumps(message)

        # 죽은 클라이언트 정리를 위한 리스트
        dead_clients = []

        for client in self.websocket_clients:
            try:
                # [핵심] run_coroutine_threadsafe 사용
                # ROS 스레드에서 메인 루프(app_loop)에 작업을 예약함
                asyncio.run_coroutine_threadsafe(client.send_text(json_msg), self.app_loop)
            except Exception as e:
                print(f"[Broadcast Error] {e}")
                dead_clients.append(client)

        # 연결 끊긴 클라이언트 정리
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
            'position': {'x': state['x'], 'y': state['y']}
        })

    def scan_callback(self, msg):
        if not msg.ranges: return

        state = self.robot_states[self.current_robot]
        # 전방 데이터 필터링
        num_readings = len(msg.ranges)
        right = msg.ranges[0:min(30, num_readings)]
        left = msg.ranges[max(0, num_readings - 30):num_readings]
        front = list(right) + list(left)
        valid = [r for r in front if not math.isnan(r) and not math.isinf(r) and 0.1 < r < 10.0]

        if valid:
            min_dist = min(valid)
            state['front_distance'] = min_dist

            if state['lidar_enabled'] and min_dist < 0.35 and state['mode'] != 'emergency':
                self.stop()
                self.broadcast({'type': 'collision_warning', 'robot_id': self.current_robot, 'message': f'장애물: {min_dist:.2f}m'})
        else:
            state['front_distance'] = 999.0

        self.broadcast({
            'type': 'scan',
            'robot_id': self.current_robot,
            'ranges': list(msg.ranges[::10])
        })

    def camera_callback(self, msg):
        """카메라 처리 (정석 방법 적용)"""
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

            # 이미지 전송 (broadcast 사용 - 이제 안전함!)
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 50])
            image_hex = buffer.tobytes().hex()

            # 디버깅 로그 (접속자 확인용)
            current_time = time.time()
            if not hasattr(self, 'last_log_time'): self.last_log_time = 0
            if current_time - self.last_log_time > 3.0:
                #print(f"[Vision] 전송중.. 접속자: {len(self.websocket_clients)}명")
                self.last_log_time = current_time

            self.broadcast({
                'type': 'camera_image',
                'robot_id': self.current_robot,
                'image_data': image_hex
            })

        except Exception as e:
            print(f"[Camera Error] {e}")

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

        # 장애물 감지 시 전진 차단
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

    # [핵심 수정] 현재 실행 중인 이벤트 루프 가져오기
    loop = asyncio.get_running_loop()

    def ros_spin():
        rclpy.init()
        global ros_node
        # Loop를 ROS 노드에 전달
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
