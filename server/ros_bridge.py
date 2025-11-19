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

class TurtlebotBridge(Node):
    """
    ROS2 Bridge Node
    - Runs on Server
    - Connects to Turtlebot via ROS2 (Discovery)
    - Handles Navigation2 & Visual Servoing (MARK)
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

        # 2. Nav2 Client & Process
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_process = None
        self.nav_busy = False
        self._goal_handle = None

        # Start Nav2 Stack on Server
        self.start_navigation_server()

        # 3. Visual Servoing (MARK)
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        self.mark_state = "IDLE" # IDLE, ALIGNING, DONE
        self.qr_visible = False
        self.last_qr_center = None
        self.last_qr_angle = 0.0
        self.last_qr_width = 0.0
        self.latest_frame_width = 320

        # PID Gains for Alignment
        self.TARGET_QR_WIDTH = 120.0 # Target size in pixels (Adjust distance)
        self.K_CX = 0.002
        self.K_ANG = 0.01
        self.K_DIST = 0.0015

        # Thresholds
        self.THRESH_CX = 15
        self.THRESH_ANG = 5.0
        self.THRESH_DIST = 15.0

        # Command Throttling
        self._last_cmd_time = 0.0
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        # Main Loop (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"Bridge Started: Robot {robot_id} (Domain {domain_id})")

    def start_navigation_server(self):
        """Launch Nav2 Stack for this robot namespace/domain"""
        map_path = os.path.expanduser("~/maps/map.yaml")
        if not os.path.exists(map_path):
            self.get_logger().warn(f"Map not found at {map_path}, Nav2 might fail.")

        # Environment for this subprocess
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self.domain_id)

        # Command: Launch Nav2 (Headless)
        # Assuming turtlebot3_navigation2 package is installed
        cmd = [
            'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
            'use_sim_time:=False',
            f'map:={map_path}',
            'params_file:=/opt/ros/humble/share/turtlebot3_navigation2/param/waffle_pi.yaml'
        ]

        try:
            # [수정] stdout=None, stderr=None으로 바꾸면 터미널에 Nav2 로그가 콸콸 쏟아집니다!
            self.nav_process = subprocess.Popen(
                cmd, env=env,
                stdout=None,  # 기존: subprocess.DEVNULL
                stderr=None,  # 기존: subprocess.PIPE
                preexec_fn=os.setsid
            )
            self.get_logger().info(f"Nav2 Process Launched (PID: {self.nav_process.pid})")
        except Exception as e:
            self.get_logger().error(f"Failed to launch Nav2: {e}")

    def control_loop(self):
        # 1. Process External Commands
        while not self.command_queue.empty():
            cmd = self.command_queue.get()
            self.execute_command(cmd)

        # 2. Handle Visual Servoing (MARK)
        if self.mark_state == "ALIGNING":
            self.process_alignment()

    def process_alignment(self):
        if self.nav_busy: return # Priority to Nav

        if not self.qr_visible or self.last_qr_center is None:
            # Lost QR -> Rotate slowly to find it
            self.move(0.0, 0.2)
            return

        cx, cy = self.last_qr_center
        angle = self.last_qr_angle
        width = self.last_qr_width

        img_cx = self.latest_frame_width // 2
        err_cx = cx - img_cx
        err_ang = angle
        err_dist = self.TARGET_QR_WIDTH - width

        # Check Success
        if abs(err_cx) < self.THRESH_CX and \
           abs(err_ang) < self.THRESH_ANG and \
           abs(err_dist) < self.THRESH_DIST:

            self.stop()
            self.mark_state = "DONE"
            self.put_status({'type': 'mark_complete'})
            self.get_logger().info("Mark Alignment Complete")

            # Reset to IDLE after a moment
            self.mark_state = "IDLE"
            return

        # PID Control
        # X Error -> Turn
        # Angle Error -> Turn
        # Width Error (Distance) -> Move Forward/Back

        ang_spd = -(self.K_CX * err_cx) - (self.K_ANG * err_ang)
        lin_spd = self.K_DIST * err_dist

        # Clamp
        lin_spd = np.clip(lin_spd, -0.1, 0.1)
        ang_spd = np.clip(ang_spd, -0.5, 0.5)

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

        elif c_type == 'navigate_to_pose':
            self.send_nav_goal(cmd.get('x'), cmd.get('y'), cmd.get('yaw'))

        elif c_type == 'cancel_navigation':
            self.cancel_navigation()

        elif c_type == 'start_mark_alignment':
            self.get_logger().info("Starting MARK Alignment")
            self.mark_state = "ALIGNING"

    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        now = time.time()
        # Throttle duplicate commands to save bandwidth
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
        # 1. Send to Server
        self.put_status({'type': 'camera', 'data': {'data': bytes(msg.data)}})

        # 2. Local Processing for Alignment
        if self.mark_state == "ALIGNING":
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.latest_frame_width = img.shape[1]

                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                _, points, _ = self.qr_detector.detectAndDecode(gray)

                if points is not None:
                    self.qr_visible = True
                    pts = points.reshape(-1, 2) # FIXED TYPO

                    # Calculate Metrics
                    cx = np.mean(pts[:, 0])
                    cy = np.mean(pts[:, 1])

                    # Angle based on top edge
                    dx = pts[1][0] - pts[0][0]
                    dy = pts[1][1] - pts[0][1]
                    angle = math.degrees(math.atan2(dy, dx))

                    # Width (average of top and bottom)
                    w1 = np.linalg.norm(pts[0] - pts[1])
                    w2 = np.linalg.norm(pts[2] - pts[3])

                    self.last_qr_center = (cx, cy)
                    self.last_qr_angle = angle
                    self.last_qr_width = (w1 + w2) / 2.0
                else:
                    self.qr_visible = False
            except:
                pass

    def send_nav_goal(self, x, y, yaw):
        if self.nav_busy:
            self.get_logger().warn("Nav Busy")
            return

        if not self.nav_action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Nav2 Action Server Not Ready")
            self.put_status({'type': 'nav_result', 'success': False, 'reason': 'Nav2 Not Ready'})
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        # Yaw to Quaternion
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
        # Status 4 is Succeeded
        self.put_status({'type': 'nav_result', 'success': (status == 4), 'reason': str(status)})

    def cancel_navigation(self):
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()

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

# --- Process Manager ---
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
