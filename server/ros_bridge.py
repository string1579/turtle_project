"""
ROS2 브릿지 - 멀티프로세스 버전 (AMCL + 탈출 로직 강화)
- AMCL 위치 보정 사용
- 장애물 회피 시 강제 회전 (관성 부여)
- Navigation2 연동
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, CompressedImage
from nav2_msgs.action import NavigateToPose
import os
import time
import math
from multiprocessing import Process, Queue

def euler_to_quaternion(yaw: float) -> Quaternion:
    """Yaw를 Quaternion으로 변환"""
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat

class TurtlebotBridge(Node):
    """터틀봇 ROS2 제어 노드 (AMCL + Navigation 포함)"""

    def __init__(self, robot_id: int, domain_id: int,
                 command_queue: Queue, status_queue: Queue):
        super().__init__(f'bridge_robot_{robot_id}')

        self.robot_id = robot_id
        self.domain_id = domain_id
        self.command_queue = command_queue
        self.status_queue = status_queue

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Subscribers
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.amcl_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.camera_callback, 10)

        # Navigation2 Action Client
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # 상태 변수
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_qz = 0.0
        self.current_pose_qw = 0.0
        self.last_scan = None
        self.is_navigating = False
        self.nav_goal_handle = None

        # 탈출 회전 상태
        self.is_escaping_turn = False
        self.escape_turn_start_time = 0.0
        self.escape_turn_direction = 0.0
        self.ESCAPE_TURN_DURATION = 1.0

        # 타이머
        self.timer = self.create_timer(0.01, self.check_commands)

        self.get_logger().info(
            f'터틀봇 {robot_id} 브릿지 시작 (Domain: {domain_id})')

        # Nav2 서버 대기
        self.get_logger().info('Nav2 서버 대기 중...')
        nav_ready = self.nav_action_client.wait_for_server(timeout_sec=5.0)
        if nav_ready:
            self.get_logger().info('Nav2 연결 완료!')
        else:
            self.get_logger().warn('Nav2 연결 실패 (자율주행 불가)')

    def amcl_callback(self, msg):
        """AMCL 위치 보정 데이터 수신"""
        pose = msg.pose.pose
        self.current_pose_x = pose.position.x
        self.current_pose_y = pose.position.y
        self.current_pose_qz = pose.orientation.z
        self.current_pose_qw = pose.orientation.w

        self.status_queue.put_nowait({
            'type': 'amcl_pose',
            'robot_id': self.robot_id,
            'data': {
                'x': self.current_pose_x,
                'y': self.current_pose_y,
                'qz': self.current_pose_qz,
                'qw': self.current_pose_qw
            }
        })

    def scan_callback(self, msg):
        """LiDAR 데이터"""
        self.last_scan = msg.ranges
        self.status_queue.put_nowait({
            'type': 'scan',
            'robot_id': self.robot_id,
            'data': {'ranges': list(msg.ranges)}
        })

    def camera_callback(self, msg):
        """카메라 이미지"""
        self.status_queue.put_nowait({
            'type': 'camera',
            'robot_id': self.robot_id,
            'data': {'data': bytes(msg.data), 'format': msg.format}
        })

    def check_commands(self):
        """명령 큐 확인"""
        while not self.command_queue.empty():
            command = self.command_queue.get_nowait()
            self.execute_command(command)

    def execute_command(self, command: dict):
        """명령 실행"""
        cmd_type = command.get('command')

        if cmd_type == 'move':
            linear = command.get('linear', 0.0)
            angular = command.get('angular', 0.0)
            self.move(linear, angular)

        elif cmd_type == 'stop':
            self.stop()

        elif cmd_type == 'navigate_to':
            x = command.get('x', 0.0)
            y = command.get('y', 0.0)
            yaw = command.get('yaw', 0.0)
            self.navigate_to(x, y, yaw)

        elif cmd_type == 'cancel_navigation':
            self.cancel_navigation()

        elif cmd_type == 'escape_turn':
            direction = command.get('direction', 0.5)
            self.start_escape_turn(direction)

    def move(self, linear: float, angular: float):
        """수동 이동"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        """정지"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.is_escaping_turn = False
        self.get_logger().info('정지')

    def start_escape_turn(self, angular_direction: float):
        """장애물 탈출 회전 시작 (1초 강제 유지)"""
        self.is_escaping_turn = True
        self.escape_turn_start_time = time.time()
        self.escape_turn_direction = angular_direction

        self.get_logger().info(
            f'탈출 회전 시작: {angular_direction:.2f} rad/s (1초 유지)')

        # 회전 명령 발행
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_direction
        self.cmd_vel_pub.publish(twist)

    def navigate_to(self, x: float, y: float, yaw: float = 0.0):
        """Navigation2 목표 전송"""
        if self.is_navigating:
            self.get_logger().warn('이미 이동 중 - 취소 후 재시작')
            self.cancel_navigation()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose(x, y, yaw)

        self.get_logger().info(
            f'Nav2 Goal: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°')

        send_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_future.add_done_callback(self._nav_response_callback)

        self.is_navigating = True

    def _create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """PoseStamped 생성"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation = euler_to_quaternion(yaw)

        return pose

    def _nav_response_callback(self, future):
        """Nav2 Goal 응답"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal 거부됨')
            self.is_navigating = False
            self._send_nav_status('failed', 'goal_rejected')
            return

        self.get_logger().info('Goal 수락됨')
        self.nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        """Nav2 피드백"""
        feedback = feedback_msg.feedback
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y

        self.status_queue.put_nowait({
            'type': 'nav_feedback',
            'robot_id': self.robot_id,
            'x': x,
            'y': y
        })

    def _nav_result_callback(self, future):
        """Nav2 결과"""
        status = future.result().status

        self.is_navigating = False
        self.nav_goal_handle = None

        if status == 4:
            self.get_logger().info('목표 도착!')
            self._send_nav_status('complete', 'arrived')
        else:
            self.get_logger().warn(f'이동 실패 (status={status})')
            self._send_nav_status('failed', f'status_{status}')

    def _send_nav_status(self, result: str, reason: str = ''):
        """Navigation 상태를 서버로 전송"""
        self.status_queue.put_nowait({
            'type': 'nav_status',
            'robot_id': self.robot_id,
            'result': result,
            'reason': reason
        })

    def cancel_navigation(self):
        """Navigation 취소"""
        if not self.is_navigating or not self.nav_goal_handle:
            self.get_logger().warn('취소할 이동 없음')
            return

        self.get_logger().info('이동 취소 요청')
        cancel_future = self.nav_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_callback)

    def _cancel_callback(self, future):
        """취소 응답"""
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('취소 완료')
        else:
            self.get_logger().warn('취소 실패')

        self.is_navigating = False
        self.nav_goal_handle = None

def run_bridge_process(robot_id: int, domain_id: int,
                      command_queue: Queue, status_queue: Queue):
    """브릿지 프로세스 메인"""
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    rclpy.init()
    bridge = TurtlebotBridge(robot_id, domain_id, command_queue, status_queue)

    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()

class MultiRobotManager:
    """멀티 로봇 관리자"""

    def __init__(self):
        self.processes = {}
        self.command_queues = {}
        self.status_queues = {}
        print("[매니저] 생성")

    def add_robot(self, robot_id: int, domain_id: int):
        """로봇 추가"""
        if robot_id in self.processes:
            print(f"[매니저] 로봇 {robot_id} 이미 존재")
            return False

        cmd_q = Queue()
        stat_q = Queue()

        p = Process(target=run_bridge_process,
                   args=(robot_id, domain_id, cmd_q, stat_q))
        p.start()

        self.processes[robot_id] = p
        self.command_queues[robot_id] = cmd_q
        self.status_queues[robot_id] = stat_q

        print(f"[매니저] 터틀봇 {robot_id} 추가 (Domain: {domain_id}, PID: {p.pid})")
        time.sleep(0.5)

        return True

    def send_command(self, robot_id: int, command: dict):
        """명령 전송"""
        if robot_id not in self.command_queues:
            return False

        self.command_queues[robot_id].put(command)
        return True

    def move_robot(self, robot_id: int, linear: float, angular: float):
        """이동"""
        return self.send_command(robot_id, {
            'command': 'move',
            'linear': linear,
            'angular': angular
        })

    def stop_robot(self, robot_id: int):
        """정지"""
        return self.send_command(robot_id, {'command': 'stop'})

    def navigate_to(self, robot_id: int, x: float, y: float, yaw: float = 0.0):
        """Navigation 이동"""
        return self.send_command(robot_id, {
            'command': 'navigate_to',
            'x': x,
            'y': y,
            'yaw': yaw
        })

    def cancel_navigation(self, robot_id: int):
        """Navigation 취소"""
        return self.send_command(robot_id, {'command': 'cancel_navigation'})

    def escape_turn(self, robot_id: int, angular_direction: float):
        """탈출 회전 명령"""
        return self.send_command(robot_id, {
            'command': 'escape_turn',
            'direction': angular_direction
        })

    def get_status(self, robot_id: int):
        """상태 가져오기"""
        if robot_id not in self.status_queues:
            return None

        try:
            return self.status_queues[robot_id].get_nowait()
        except:
            return None

    def get_all_status(self):
        """모든 상태"""
        all_status = {}

        for robot_id in self.status_queues.keys():
            statuses = []
            while True:
                status = self.get_status(robot_id)
                if status is None:
                    break
                statuses.append(status)

            if statuses:
                all_status[robot_id] = statuses

        return all_status

    def shutdown(self):
        """종료"""
        for robot_id, p in self.processes.items():
            print(f"[매니저] 로봇 {robot_id} 종료")
            p.terminate()
            p.join(timeout=2)
            if p.is_alive():
                p.kill()

        self.processes.clear()
        self.command_queues.clear()
        self.status_queues.clear()
