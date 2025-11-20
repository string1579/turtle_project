#!/usr/bin/env python3
"""
Simple Turtlebot Server - 발표용 완성본
Domain 5 통일 + GUI 완벽 호환 + 필수 기능 포함
"""

import asyncio
import json
import math
import time
import cv2
import numpy as np
from typing import Dict, Optional
import os

# ==========================================
# ROS Domain ID 설정
# ==========================================
# 개별 테스트 시에는 '3'으로, 전체 통합 시에는 '5'로 설정하세요.
os.environ['ROS_DOMAIN_ID'] = '5'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage, BatteryState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from threading import Thread

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
# ROS2 노드 (GUI 호환 버전)
# ==========================================
class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        print(f"[ROS] Domain ID: {os.environ.get('ROS_DOMAIN_ID')}")

        # 로봇 상태 (GUI 형식에 맞춤)
        self.robot_states = {}
        for rid in ROBOTS.keys():
            self.robot_states[rid] = {
                'x': 0.0, 'y': 0.0, 'yaw': 0.0,
                'battery': 0.0,
                'front_distance': float('inf'),
                'emergency': False,
                'mode': 'idle',  # idle, manual, mark_aligning
                'qr_enabled': False,
                'lidar_enabled': False
            }

        # 현재 제어 중인 로봇
        self.current_robot = 1

        # WebSocket 클라이언트들
        self.websocket_clients = []

        # QR 관련
        self.qr_detector = cv2.QRCodeDetector()

        # Mark 정밀 제어 관련 (Team E 담당 변수)
        self.mark_aligning = False
        self.last_qr_center = None
        self.last_qr_width = 0
        self.target_qr_width = 150  # 목표 크기

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Navigation Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.camera_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.camera_callback, 10)

        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)

        # 초기 위치 퍼블리셔
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # 타이머
        self.create_timer(0.1, self.control_loop)  # 10Hz 제어 루프
        self.create_timer(3.0, self.publish_initial_pose)  # 초기 위치

        self.get_logger().info("Simple Controller Ready!")

    def control_loop(self):
        """Mark 정밀 제어 루프"""
        if not self.mark_aligning or self.last_qr_center is None:
            return

        # PID 제어
        image_center_x = 160  # 320x240 이미지 기준
        cx, cy = self.last_qr_center

        # 에러 계산
        error_x = cx - image_center_x  # 좌우 오차
        error_size = self.target_qr_width - self.last_qr_width  # 거리 오차

        # 허용 오차
        if abs(error_x) < 20 and abs(error_size) < 20:
            # 정렬 완료
            self.stop()
            self.mark_aligning = False
            print("[MARK] 정밀 정렬 완료!")

            self.broadcast({
                'type': 'qr_detected',
                'robot_id': self.current_robot,
                'qr_text': 'MARK - 정렬 완료'
            })
            return

        # 제어 명령
        angular = -error_x * 0.003  # 좌우 회전
        linear = error_size * 0.001  # 전후 이동

        # 속도 제한
        angular = max(-0.3, min(0.3, angular))
        linear = max(-0.05, min(0.05, linear))

        self.move(linear, angular)

    def odom_callback(self, msg):
        """위치 정보 업데이트"""
        state = self.robot_states[self.current_robot]
        state['x'] = msg.pose.pose.position.x
        state['y'] = msg.pose.pose.position.y

        # Yaw 계산
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        state['yaw'] = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)

        # GUI 형식에 맞춘 broadcast
        self.broadcast({
            'type': 'position',
            'robot_id': self.current_robot,
            'position': {
                'x': state['x'],
                'y': state['y'],
                'qz': qz,
                'qw': qw
            }
        })

    def scan_callback(self, msg):
        """LiDAR 충돌 방지"""
        if not msg.ranges:
            return

        state = self.robot_states[self.current_robot]

        # 전방 30도 체크
        front_ranges = list(msg.ranges[0:30]) + list(msg.ranges[-30:])
        valid = [r for r in front_ranges if 0.1 < r < 10.0]

        if valid:
            min_dist = min(valid)
            state['front_distance'] = min_dist

            # 충돌 방지 (lidar_enabled일 때만)
            if state['lidar_enabled'] and min_dist < 0.35:
                if state['mode'] != 'emergency':
                    self.stop()
                    print(f"[충돌 방지] 정지! 거리: {min_dist:.2f}m")

                    self.broadcast({
                        'type': 'collision_warning',
                        'robot_id': self.current_robot,
                        'message': f'장애물 감지: {min_dist:.2f}m'
                    })

        # GUI용 scan 데이터 (간소화)
        self.broadcast({
            'type': 'scan',
            'robot_id': self.current_robot,
            'ranges': list(msg.ranges[::10])  # 10개마다 1개씩만
        })

    def battery_callback(self, msg):
        """배터리 상태 (누락 수정됨)"""
        state = self.robot_states[self.current_robot]
        state['battery'] = msg.voltage

        self.broadcast({
            'type': 'battery',
            'robot_id': self.current_robot,
            'voltage': msg.voltage
        })

    # ========== [팀원E 담당 시작] ==========
    def camera_callback(self, msg):
        """카메라 & QR 처리"""
        try:
            # 이미지 디코드
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                return

            # QR 처리 (qr_enabled일 때만)
            state = self.robot_states[self.current_robot]
            if state['qr_enabled']:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                data, points, _ = self.qr_detector.detectAndDecode(gray)

                if data:
                    print(f"[QR] 감지: '{data}'")

                    # QR 중심점 계산 (MARK용)
                    if points is not None:
                        points = points.reshape(-1, 2)
                        cx = np.mean(points[:, 0])
                        cy = np.mean(points[:, 1])
                        width = np.linalg.norm(points[0] - points[1])

                        self.last_qr_center = (cx, cy)
                        self.last_qr_width = width

                    # QR 명령 처리
                    self.handle_qr_command(data)

                    self.broadcast({
                        'type': 'qr_detected',
                        'robot_id': self.current_robot,
                        'qr_text': data
                    })

                else:
                    # [안전 기능] QR이 시야에서 사라지면 정보 초기화
                    self.last_qr_center = None
                    self.last_qr_width = 0

                    # 정렬 중 QR을 놓치면 즉시 정지
                    if self.mark_aligning:
                        print("[QR] MARK 놓침 - 정지 및 정렬 해제")
                        self.stop()
                        self.mark_aligning = False

            # 이미지 전송 (GUI 표시용)
            _, buffer = cv2.imencode('.jpg', cv_image,
                                    [cv2.IMWRITE_JPEG_QUALITY, 50])

            self.broadcast({
                'type': 'camera_image',
                'robot_id': self.current_robot,
                'image_data': buffer.tobytes().hex()
            })

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def handle_qr_command(self, data):
        """QR 명령 처리"""
        cmd = data.strip().upper()

        if cmd == "STOP":
            self.stop()
            print("[QR] 정지")

        elif cmd == "MARK":
            # 정밀 정렬 모드
            self.mark_aligning = True
            self.robot_states[self.current_robot]['mode'] = 'mark_aligning'
            print("[QR] MARK 정밀 정렬 시작")

        elif cmd.startswith("NAV_GOAL:"):
            # 좌표 이동 (예: NAV_GOAL:2.0,1.0,0.0)
            try:
                parts = cmd.split(':')[1].split(',')
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    yaw = float(parts[2]) if len(parts) > 2 else 0.0

                    self.navigate_to(x, y, yaw)
                    print(f"[QR] 목표 이동: ({x}, {y})")
            except:
                pass
    # ========== [팀원E 담당 끝] ==========

    def navigate_to(self, x, y, yaw=0.0):
        """Navigation2 목표 이동"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            print("[Nav] Nav2 서버 응답 없음")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_client.send_goal_async(goal)
        print(f"[Nav] 목표 전송: ({x:.1f}, {y:.1f})")

    def move(self, linear, angular):
        """로봇 이동"""
        state = self.robot_states[self.current_robot]

        # 비상정지 체크
        if state['emergency']:
            return

        # 충돌 방지 체크
        if linear > 0 and state['front_distance'] < 0.3:
            linear = 0.0

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop(self):
        """정지"""
        self.move(0.0, 0.0)
        self.mark_aligning = False

    def set_emergency(self, robot_id, emergency):
        """비상정지 설정"""
        self.robot_states[robot_id]['emergency'] = emergency
        if emergency:
            self.stop()
            self.robot_states[robot_id]['mode'] = 'emergency'
        else:
            self.robot_states[robot_id]['mode'] = 'idle'

    def publish_initial_pose(self):
        """초기 위치 설정"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.init_pose_pub.publish(msg)
        self.get_logger().info("Initial pose published")

    def broadcast(self, message):
        """WebSocket 브로드캐스트"""
        if not self.websocket_clients:
            return

        json_msg = json.dumps(message)

        for client in self.websocket_clients[:]:
            try:
                asyncio.create_task(client.send_text(json_msg))
            except:
                self.websocket_clients.remove(client)

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
    print("Simple Turtlebot Server")
    print(f"Domain ID: {os.environ.get('ROS_DOMAIN_ID')}")
    print(f"Server: http://{SERVER_IP}:{SERVER_PORT}")
    print("=" * 60)

    # ROS2 스레드
    def ros_spin():
        rclpy.init()
        global ros_node
        ros_node = SimpleRobotController()
        rclpy.spin(ros_node)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # ROS 노드 대기
    while ros_node is None:
        await asyncio.sleep(0.1)

    print("[Server] Ready!")

@app.on_event("shutdown")
async def shutdown():
    if ros_node:
        ros_node.destroy_node()
    rclpy.shutdown()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print(f"[WS] Client connected: {websocket.client}")

    # ros_node가 초기화될 때까지 잠시 대기하거나 안전장치 확인
    if ros_node:
        ros_node.websocket_clients.append(websocket)
    else:
        print("[WS] Warning: ROS Node not ready yet")

    # 초기 정보 전송
    try:
        await websocket.send_text(json.dumps({
            'type': 'init',
            'robots': list(ROBOTS.keys())
        }))
    except Exception as e:
        print(f"[WS] Send error during init: {e}")

    try:
        while True:
            data = await websocket.receive_text()

            try:
                msg = json.loads(data)
                await handle_command(msg)

                # 처리 완료 응답 (선택 사항, 부하가 크다면 제거 가능)
                # await websocket.send_text(json.dumps({'status': 'ok'}))

            except json.JSONDecodeError:
                print(f"[WS] Invalid JSON format received: {data[:50]}...")
                continue
            except Exception as e:
                print(f"[WS] Error processing message: {e}")
                continue

    except WebSocketDisconnect:
        print(f"[WS] Client disconnected: {websocket.client}")
    except Exception as e:
        print(f"[WS] Unexpected connection error: {e}")
    finally:
        # 연결 종료 시 리스트에서 안전하게 제거
        if ros_node and websocket in ros_node.websocket_clients:
            ros_node.websocket_clients.remove(websocket)
            print(f"[WS] Client removed from broadcast list")

async def handle_command(msg: dict):
    """GUI 명령 처리"""
    if not ros_node:
        print("[Server] Error: ROS Node is not running")
        return

    cmd = msg.get('command')
    robot_id = msg.get('robot_id', 1)

    # 로봇 전환 (명령이 들어온 로봇을 현재 제어 대상으로 설정)
    ros_node.current_robot = robot_id

    if cmd == 'move':
        linear = msg.get('linear', 0)
        angular = msg.get('angular', 0)
        ros_node.move(linear, angular)
        ros_node.robot_states[robot_id]['mode'] = 'manual'

    elif cmd == 'stop':
        ros_node.stop()
        ros_node.robot_states[robot_id]['mode'] = 'idle'

    elif cmd == 'emergency_stop':
        ros_node.set_emergency(robot_id, True)
        print(f"[Command] Emergency Stop: Robot {robot_id}")

    elif cmd == 'release_emergency':
        ros_node.set_emergency(robot_id, False)
        print(f"[Command] Release Emergency: Robot {robot_id}")

    elif cmd == 'navigate_to':
        x = msg.get('x', 0)
        y = msg.get('y', 0)
        yaw = msg.get('yaw', 0)
        ros_node.navigate_to(x, y, yaw)

    elif cmd == 'qr_detect':
        ros_node.robot_states[robot_id]['qr_enabled'] = True
        print(f"[QR] Enabled for Robot {robot_id}")

    elif cmd == 'qr_stop':
        ros_node.robot_states[robot_id]['qr_enabled'] = False
        ros_node.mark_aligning = False
        print(f"[QR] Disabled for Robot {robot_id}")

    elif cmd == 'lidar_check':
        ros_node.robot_states[robot_id]['lidar_enabled'] = True
        print(f"[LiDAR] Enabled for Robot {robot_id}")

    elif cmd == 'lidar_stop':
        ros_node.robot_states[robot_id]['lidar_enabled'] = False
        print(f"[LiDAR] Disabled for Robot {robot_id}")

    elif cmd == 'camera_start':
        # 이미 자동 스트리밍 중이므로 로그만 남김
        pass

    else:
        print(f"[WS] Unknown command received: {cmd}")

if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_IP, port=SERVER_PORT, log_level="warning")
