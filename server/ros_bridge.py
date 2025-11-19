import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
from nav2_msgs.action import NavigateToPose

import os
import time
import math
import numpy as np
import cv2
import subprocess
import signal
from cv_bridge import CvBridge
from multiprocessing import Process, Queue

# ==========================================
# [설정] 디버그 모드 & 상수
DEBUG = True
QR_LOST_TIMEOUT = 5.0  # QR 놓쳤을 때 포기하는 시간
# ==========================================

class TurtlebotBridge(Node):
    """
    ROS2 Bridge Node (Merged Version)
    - Yomi: Nav2 Env Fix, Auto-Init, Min Speed Kick
    - Team: Debug Logs, Safety Timeout, Frame Skipping
    """

    def __init__(self, robot_id: int, domain_id: int,
                 command_queue: Queue, status_queue: Queue):
        super().__init__(f'bridge_robot_{robot_id}')
        self.robot_id = robot_id
        self.domain_id = domain_id
        self.command_queue = command_queue
        self.status_queue = status_queue

        # 1. ROS2 Pub/Sub
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_callback, 10)

        # [Yomi] Auto Init Pose Publisher
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(5.0, self.publish_initial_pose_once) # 5초 후 자동 원점 발사
        self.initial_pose_sent = False

        # 2. Nav2 Client
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_process = None
        self.nav_busy = False
        self._goal_handle = None

        # Start Nav2 Stack
        self.start_navigation_server()

        # 3. Visual Servoing (MARK)
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        self.mark_state = "IDLE"
        self.qr_visible = False
        self.last_qr_center = None
        self.last_qr_angle = 0.0
        self.last_qr_width = 0.0
        self.latest_frame_width = 320

        # [Team] Variables for Safety
        self.last_qr_seen_time = 0.0
        self.process_frame_count = 0
        self.success_count = 0  # [Yomi] Stability check

        # PID Gains (Yomi's Tuned Values)
        self.TARGET_QR_WIDTH = 120.0
        self.K_CX = 0.0025
        self.K_ANG = 0.015
        self.K_DIST = 0.0025

        # Thresholds (Yomi's Values - slightly looser for Min Speed Kick)
        self.THRESH_CX = 20.0
        self.THRESH_ANG = 3.0
        self.THRESH_DIST = 20.0

        # Min Velocities (Yomi's Kick Logic)
        self.MIN_LIN_VEL = 0.02
        self.MIN_ANG_VEL = 0.15

        # Throttling
        self._last_cmd_time = 0.0
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Bridge Started: Robot {robot_id} (Domain {domain_id})")

    def start_navigation_server(self):
        """Launch Nav2 Stack with Correct Env (Yomi's Fix included)"""
        map_path = os.path.expanduser("~/maps/map.yaml")
        if not os.path.exists(map_path):
            self.get_logger().error(f"MAP FILE MISSING: {map_path}")
            return

        # [Yomi] CRITICAL FIX: TURTLEBOT3_MODEL added
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self.domain_id)
        env['TURTLEBOT3_MODEL'] = 'waffle_pi'
        env['RCUTILS_COLORIZED_OUTPUT'] = '1'

        cmd = [
            'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
            'use_sim_time:=False',
            f'map:={map_path}',
            'params_file:=/opt/ros/humble/share/turtlebot3_navigation2/param/waffle_pi.yaml'
        ]

        try:
            # [Team] Debug Output Logic + [Yomi] Setsid
            if DEBUG:
                stdout_opt = None # Output to terminal
                stderr_opt = None
            else:
                stdout_opt = subprocess.DEVNULL
                stderr_opt = subprocess.PIPE

            self.nav_process = subprocess.Popen(
                cmd, env=env,
                stdout=stdout_opt,
                stderr=stderr_opt,
                preexec_fn=os.setsid
            )
            self.get_logger().info(f"Nav2 Launched (PID: {self.nav_process.pid})")
        except Exception as e:
            self.get_logger().error(f"Nav2 Launch Failed: {e}")

    def control_loop(self):
        while not self.command_queue.empty():
            cmd = self.command_queue.get()
            self.execute_command(cmd)

        if self.mark_state == "ALIGNING":
            try:
                self.process_alignment()
            except Exception as e:
                self.get_logger().error(f"[Control Loop Error] {e}")
                self.mark_state = "IDLE"

    def process_alignment(self):
        if self.nav_busy: return

        # [Team] Timeout Logic
        if not hasattr(self, 'last_qr_seen_time'):
             self.last_qr_seen_time = time.time()

        time_since_lost = time.time() - self.last_qr_seen_time

        if not self.qr_visible or self.last_qr_center is None:
            if time_since_lost > QR_LOST_TIMEOUT:
                self.get_logger().warn(f"[MARK] Failed: QR Lost for {time_since_lost:.1f}s. Aborting.")
                self.stop()
                self.mark_state = "IDLE"
                self.put_status({'type': 'nav_result', 'success': False, 'reason': 'Mark Alignment Timeout'})
                return

            # Searching...
            self.success_count = 0
            self.move(0.0, 0.2)
            return

        # Error Calculation
        cx, cy = self.last_qr_center
        angle = self.last_qr_angle
        width = self.last_qr_width

        img_cx = self.latest_frame_width // 2
        err_cx = cx - img_cx
        err_ang = angle
        err_dist = self.TARGET_QR_WIDTH - width

        # [Team] Debug Log
        if DEBUG and (self.get_clock().now().nanoseconds % 500000000 < 100000000):
             self.get_logger().info(f"[MARKING] ErrCx:{err_cx:.1f}, ErrAng:{err_ang:.1f}, ErrDist:{err_dist:.1f}")

        # [Yomi] Stability Check (Must hold for 10 ticks)
        if abs(err_cx) < self.THRESH_CX and \
           abs(err_ang) < self.THRESH_ANG and \
           abs(err_dist) < self.THRESH_DIST:

            self.success_count += 1
            if self.success_count > 10:
                self.stop()
                self.mark_state = "DONE"
                self.put_status({'type': 'mark_complete'})
                self.get_logger().info(f"[MARK SUCCESS] Final Error -> Cx:{err_cx:.2f}, Ang:{err_ang:.2f}, Dist:{err_dist:.2f}")
                self.mark_state = "IDLE"
                return
        else:
            self.success_count = 0

        # PID Control
        ang_spd = -(self.K_CX * err_cx) - (self.K_ANG * err_ang)
        lin_spd = self.K_DIST * err_dist

        # [Yomi] Min Speed Kick (Deadzone Compensation)
        if abs(lin_spd) < self.MIN_LIN_VEL:
            if abs(err_dist) > self.THRESH_DIST:
                lin_spd = self.MIN_LIN_VEL * np.sign(lin_spd)
            else:
                lin_spd = 0.0

        if abs(ang_spd) < self.MIN_ANG_VEL:
            if abs(err_cx) > self.THRESH_CX or abs(err_ang) > self.THRESH_ANG:
                ang_spd = self.MIN_ANG_VEL * np.sign(ang_spd)
            else:
                ang_spd = 0.0

        # Max Limit
        lin_spd = np.clip(lin_spd, -0.08, 0.08)
        ang_spd = np.clip(ang_spd, -0.5, 0.5)

        self.move(lin_spd, ang_spd)

    def execute_command(self, cmd: dict):
        c_type = cmd.get('command')
        if DEBUG: self.get_logger().info(f"[Bridge] Executing: {c_type}")

        if c_type == 'move':
            if not self.nav_busy and self.mark_state == "IDLE":
                self.move(cmd.get('linear'), cmd.get('angular'))

        elif c_type == 'stop':
            self.stop()
            if self.nav_busy: self.cancel_navigation()
            self.mark_state = "IDLE"
            self.success_count = 0

        elif c_type == 'navigate_to_pose':
            self.send_nav_goal(cmd.get('x'), cmd.get('y'), cmd.get('yaw'))

        elif c_type == 'cancel_navigation':
            self.cancel_navigation()

        elif c_type == 'start_mark_alignment':
            self.get_logger().info("Starting MARK Alignment")
            self.qr_visible = False
            self.last_qr_seen_time = time.time()
            self.mark_state = "ALIGNING"
            self.success_count = 0

    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        now = time.time()
        if abs(msg.linear.x - self.prev_linear) > 0.001 or \
           abs(msg.angular.z - self.prev_angular) > 0.001 or \
           (now - self._last_cmd_time) > 0.1:
            self.cmd_vel_pub.publish(msg)
            self.prev_linear = msg.linear.x
            self.prev_angular = msg.angular.z
            self._last_cmd_time = now

    def stop(self):
        self.move(0.0, 0.0)

    def camera_callback(self, msg):
        self.put_status({'type': 'camera', 'data': {'data': bytes(msg.data)}})

        if self.mark_state == "ALIGNING":
            self.process_frame_count += 1
            # [Team] Frame Skipping (Process every 3rd frame)
            if self.process_frame_count % 3 != 0:
                return

            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.latest_frame_width = img.shape[1]

                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                _, points, _ = self.qr_detector.detectAndDecode(gray)

                if points is not None:
                    self.qr_visible = True
                    pts = points.reshape(-1, 2)

                    cx = np.mean(pts[:, 0])
                    cy = np.mean(pts[:, 1])

                    dx = pts[1][0] - pts[0][0]
                    dy = pts[1][1] - pts[0][1]
                    angle = math.degrees(math.atan2(dy, dx))

                    w1 = np.linalg.norm(pts[0] - pts[1])
                    w2 = np.linalg.norm(pts[2] - pts[3])

                    self.last_qr_center = (cx, cy)
                    self.last_qr_angle = angle
                    self.last_qr_width = (w1 + w2) / 2.0
                    self.last_qr_seen_time = time.time() # Update seen time
                else:
                    self.qr_visible = False
            except Exception as e:
                if DEBUG: self.get_logger().error(f"[Camera Error] {e}")

    def send_nav_goal(self, x, y, yaw):
        if self.nav_busy: return
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.put_status({'type': 'nav_result', 'success': False, 'reason': 'Nav2 Not Ready'})
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_busy = True
        if DEBUG: self.get_logger().info(f"[Bridge] Sending Nav Goal: {x}, {y}")
        future = self.nav_action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.nav_busy = False
            self.put_status({'type': 'nav_result', 'success': False, 'reason': 'Rejected'})
            return
        self._goal_handle = handle
        res_future = handle.get_result_async()
        res_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.nav_busy = False
        self._goal_handle = None
        self.put_status({'type': 'nav_result', 'success': (status == 4), 'reason': str(status)})

    def cancel_navigation(self):
        if self._goal_handle: self._goal_handle.cancel_goal_async()

    def odom_callback(self, msg):
        self.put_status({'type': 'odom', 'data': {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y}})

    def scan_callback(self, msg):
        self.put_status({'type': 'scan', 'data': {'ranges': list(msg.ranges)}})

    def put_status(self, data):
        try:
            data['robot_id'] = self.robot_id
            self.status_queue.put_nowait(data)
        except: pass

    # [Yomi] Auto-Init Pose Logic
    def publish_initial_pose_once(self):
        if self.initial_pose_sent: return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.init_pose_pub.publish(msg)
        self.get_logger().info(f"📍 [Auto-Init] Robot {self.robot_id} Pose Set to (0,0)")
        self.initial_pose_sent = True

    def __del__(self):
        if self.nav_process:
            os.killpg(os.getpgid(self.nav_process.pid), signal.SIGTERM)

def run_bridge_process(rid, did, cmd_q, stat_q):
    os.environ['ROS_DOMAIN_ID'] = str(did)
    rclpy.init()
    node = TurtlebotBridge(rid, did, cmd_q, stat_q)
    try: rclpy.spin(node)
    except: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

class MultiRobotManager:
    # (변경 사항 없음, 기존과 동일)
    def __init__(self):
        self.processes = {}
        self.cmd_qs = {}
        self.stat_qs = {}

    def add_robot(self, rid, did):
        if rid in self.processes: return
        cq, sq = Queue(), Queue()
        p = Process(target=run_bridge_process, args=(rid, did, cq, sq))
        p.start()
        self.processes[rid] = p
        self.cmd_qs[rid] = cq
        self.stat_qs[rid] = sq

    def send_command(self, rid, cmd):
        if rid in self.cmd_qs: self.cmd_qs[rid].put(cmd)

    def move_robot(self, rid, l, a): self.send_command(rid, {'command': 'move', 'linear': l, 'angular': a}); return True
    def stop_robot(self, rid): self.send_command(rid, {'command': 'stop'}); return True
    def navigate_to_pose(self, rid, x, y, yaw): self.send_command(rid, {'command': 'navigate_to_pose', 'x': x, 'y': y, 'yaw': yaw}); return True
    def cancel_navigation(self, rid): self.send_command(rid, {'command': 'cancel_navigation'}); return True

    def get_all_status(self):
        res = {}
        for rid, sq in self.stat_qs.items():
            msgs = []
            while not sq.empty(): msgs.append(sq.get())
            if msgs: res[rid] = msgs
        return res

    def shutdown(self):
        for p in self.processes.values(): p.terminate()
