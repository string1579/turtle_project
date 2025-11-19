import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Quaternion
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
from geometry_msgs.msg import PoseWithCovarianceStamped

class TurtlebotBridge(Node):
    """
    ROS2 Bridge Node
    - Fix: Nav2 Env Variables
    - Fix: QR Alignment Stability (Success Counter)
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
        self.success_count = 0  # For stability check

        # PID Gains
        self.TARGET_QR_WIDTH = 120.0
        self.K_CX = 0.002
        self.K_ANG = 0.01
        self.K_DIST = 0.002

        # Thresholds
        self.THRESH_CX = 15
        self.THRESH_ANG = 5.0
        self.THRESH_DIST = 20.0

        # Throttling
        self._last_cmd_time = 0.0
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Bridge Started: Robot {robot_id} (Domain {domain_id})")

    def start_navigation_server(self):
        """Launch Nav2 Stack with correct Environment"""
        map_path = os.path.expanduser("~/maps/map.yaml")
        if not os.path.exists(map_path):
            self.get_logger().error(f"MAP FILE MISSING: {map_path}")
            return

        # FIXED: Add TURTLEBOT3_MODEL env variable
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self.domain_id)
        env['TURTLEBOT3_MODEL'] = 'waffle_pi'  # CRITICAL FIX

        cmd = [
            'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
            'use_sim_time:=False',
            f'map:={map_path}',
            'params_file:=/opt/ros/humble/share/turtlebot3_navigation2/param/waffle_pi.yaml'
        ]

        try:
            self.nav_process = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
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
            self.process_alignment()

    def process_alignment(self):
        if self.nav_busy: return

        if not self.qr_visible or self.last_qr_center is None:
            # Lost QR -> Rotate slowly
            self.success_count = 0
            self.move(0.0, 0.2)
            return

        cx, cy = self.last_qr_center
        angle = self.last_qr_angle
        width = self.last_qr_width

        img_cx = self.latest_frame_width // 2
        err_cx = cx - img_cx
        err_ang = angle
        err_dist = self.TARGET_QR_WIDTH - width

        # Debug Log (Check this in terminal!)
        # self.get_logger().info(f"Aligning... CX:{err_cx:.1f}, ANG:{err_ang:.1f}, DIST:{err_dist:.1f}")

        # Check Success (Must be stable for 5 ticks)
        if abs(err_cx) < self.THRESH_CX and \
           abs(err_ang) < self.THRESH_ANG and \
           abs(err_dist) < self.THRESH_DIST:

            self.success_count += 1
            if self.success_count > 5:
                self.stop()
                self.mark_state = "DONE"
                self.put_status({'type': 'mark_complete'})
                self.get_logger().info("MARK ALIGNMENT SUCCESS!")
                self.mark_state = "IDLE"
                return
        else:
            self.success_count = 0

        # PID Control
        ang_spd = -(self.K_CX * err_cx) - (self.K_ANG * err_ang)
        lin_spd = self.K_DIST * err_dist

        # Clamp
        lin_spd = np.clip(lin_spd, -0.08, 0.08)
        ang_spd = np.clip(ang_spd, -0.4, 0.4)

        self.move(lin_spd, ang_spd)

    def execute_command(self, cmd: dict):
        c_type = cmd.get('command')

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
                    width = (w1 + w2) / 2.0

                    self.last_qr_center = (cx, cy)
                    self.last_qr_angle = angle
                    self.last_qr_width = width
                else:
                    self.qr_visible = False
            except: pass

    def send_nav_goal(self, x, y, yaw):
        if self.nav_busy: return

        # Wait longer for server (up to 10s)
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 Action Server Not Ready (Check Logs)")
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

    def __del__(self):
        if self.nav_process:
            os.killpg(os.getpgid(self.nav_process.pid), signal.SIGTERM)

        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # (Nav2가 켜지는 시간을 벌어주는 거예요)
        self.create_timer(5.0, self.publish_initial_pose_once)
        self.initial_pose_sent = False

    def publish_initial_pose_once(self):
        if self.initial_pose_sent:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # 원점 (0, 0) 설정
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0  # 방향은 정면

        self.init_pose_pub.publish(msg)
        self.get_logger().info(f"📍 [Auto-Init] 로봇 {self.robot_id} 초기 위치(0,0) 전송 완료!")
        self.initial_pose_sent = True

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
