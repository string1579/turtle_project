"""
Navigation2 제어 모듈
- SLAM 맵 기반 목표 좌표 이동
- 서버에서 Nav2 Goal 전송
- 터틀봇은 명령만 수신 (연산 최소화)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
import math
from typing import Optional, Tuple, Callable

def euler_to_quaternion(yaw: float) -> Quaternion:
    """
    Yaw 각도를 Quaternion으로 변환

    Args:
        yaw: 회전 각도 (라디안)

    Returns:
        Quaternion 객체
    """
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat

class NavigationModule(Node):
    """Navigation2 제어 클래스"""

    def __init__(self, robot_id: int):
        """
        초기화

        Args:
            robot_id: 로봇 번호 (1, 2, 3)
        """
        super().__init__(f'nav_module_robot_{robot_id}')

        self.robot_id = robot_id

        # Action Client 생성
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # 상태 관리
        self.is_navigating = False
        self.current_goal = None
        self.goal_handle = None

        # 콜백 함수
        self.on_goal_reached = None
        self.on_goal_failed = None
        self.on_navigation_feedback = None

        self.get_logger().info(f'Navigation 모듈 초기화 (로봇 {robot_id})')


    def wait_for_nav2(self, timeout_sec: float = 10.0) -> bool:
        """
        Nav2 서버 대기

        Args:
            timeout_sec: 대기 시간 (초)

        Returns:
            True=연결 성공, False=실패
        """
        self.get_logger().info('Nav2 서버 대기 중...')

        result = self._action_client.wait_for_server(timeout_sec=timeout_sec)

        if result:
            self.get_logger().info('Nav2 서버 연결 완료!')
        else:
            self.get_logger().error(f'Nav2 서버 연결 실패 ({timeout_sec}초 초과)')

        return result


    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """
        목표 좌표로 이동

        Args:
            x: 목표 x 좌표 (m)
            y: 목표 y 좌표 (m)
            yaw: 목표 방향 (라디안, 기본값 0.0)

        Returns:
            True=명령 전송 성공, False=실패
        """
        if self.is_navigating:
            self.get_logger().warn('이미 이동 중입니다. 기존 목표를 취소합니다.')
            self.cancel_navigation()

        # Goal 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose(x, y, yaw)

        self.get_logger().info(f'목표 설정: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°')

        # Goal 전송
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        send_goal_future.add_done_callback(self._goal_response_callback)

        self.is_navigating = True
        self.current_goal = (x, y, yaw)

        return True


    def _create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """PoseStamped 메시지 생성"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation = euler_to_quaternion(yaw)

        return pose


    def _goal_response_callback(self, future):
        """Goal 응답 처리"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal이 거부되었습니다!')
            self.is_navigating = False

            if self.on_goal_failed:
                self.on_goal_failed(self.robot_id, "goal_rejected")
            return

        self.get_logger().info('Goal이 수락되었습니다.')
        self.goal_handle = goal_handle

        # 결과 대기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)


    def _feedback_callback(self, feedback_msg):
        """이동 중 피드백 처리"""
        feedback = feedback_msg.feedback

        current_pose = feedback.current_pose.pose
        x = current_pose.position.x
        y = current_pose.position.y

        # 콜백 호출
        if self.on_navigation_feedback:
            self.on_navigation_feedback(self.robot_id, x, y)


    def _result_callback(self, future):
        """이동 완료 결과 처리"""
        result = future.result().result
        status = future.result().status

        self.is_navigating = False
        self.goal_handle = None

        # STATUS: https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html
        # 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info('목표 도착 완료!')

            if self.on_goal_reached:
                self.on_goal_reached(self.robot_id)
        else:
            self.get_logger().warn(f'이동 실패 (status={status})')

            if self.on_goal_failed:
                self.on_goal_failed(self.robot_id, f"status_{status}")


    def cancel_navigation(self) -> bool:
        """현재 이동 취소"""
        if not self.is_navigating or not self.goal_handle:
            self.get_logger().warn('취소할 이동이 없습니다.')
            return False

        self.get_logger().info('이동 취소 요청...')

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_callback)

        return True


    def _cancel_callback(self, future):
        """취소 응답 처리"""
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('이동 취소 완료')
        else:
            self.get_logger().warn('이동 취소 실패')

        self.is_navigating = False
        self.goal_handle = None

class NavigationController:
    """
    멀티 로봇 Navigation 관리자
    서버에서 사용하는 Wrapper 클래스
    """

    def __init__(self):
        """초기화"""
        self.nav_modules = {}
        self.executor = None

        print("[Navigation] 컨트롤러 생성")


    def add_robot(self, robot_id: int) -> bool:
        """
        로봇 추가

        Args:
            robot_id: 로봇 번호

        Returns:
            True=성공, False=실패
        """
        if robot_id in self.nav_modules:
            print(f"[Navigation] 로봇 {robot_id}은 이미 등록됨")
            return False

        # Navigation 모듈 생성
        nav_module = NavigationModule(robot_id)

        # Nav2 서버 대기
        if not nav_module.wait_for_nav2(timeout_sec=10.0):
            print(f"[Navigation] 로봇 {robot_id} Nav2 연결 실패")
            nav_module.destroy_node()
            return False

        self.nav_modules[robot_id] = nav_module
        print(f"[Navigation] 로봇 {robot_id} 추가 완료")

        return True


    def navigate_to(self, robot_id: int, x: float, y: float,
                    yaw: float = 0.0) -> bool:
        """
        목표 좌표로 이동

        Args:
            robot_id: 로봇 번호
            x: 목표 x 좌표
            y: 목표 y 좌표
            yaw: 목표 방향 (라디안)

        Returns:
            True=성공, False=실패
        """
        if robot_id not in self.nav_modules:
            print(f"[Navigation] 로봇 {robot_id}이 등록되지 않음")
            return False

        return self.nav_modules[robot_id].navigate_to(x, y, yaw)


    def cancel_navigation(self, robot_id: int) -> bool:
        """이동 취소"""
        if robot_id not in self.nav_modules:
            return False

        return self.nav_modules[robot_id].cancel_navigation()


    def is_navigating(self, robot_id: int) -> bool:
        """이동 중인지 확인"""
        if robot_id not in self.nav_modules:
            return False

        return self.nav_modules[robot_id].is_navigating


    def register_callbacks(self, robot_id: int,
                          on_goal_reached: Optional[Callable] = None,
                          on_goal_failed: Optional[Callable] = None,
                          on_feedback: Optional[Callable] = None):
        """
        콜백 함수 등록

        Args:
            robot_id: 로봇 번호
            on_goal_reached: 도착 성공 시 호출
            on_goal_failed: 도착 실패 시 호출
            on_feedback: 이동 중 피드백 시 호출
        """
        if robot_id not in self.nav_modules:
            return

        nav = self.nav_modules[robot_id]

        if on_goal_reached:
            nav.on_goal_reached = on_goal_reached
        if on_goal_failed:
            nav.on_goal_failed = on_goal_failed
        if on_feedback:
            nav.on_navigation_feedback = on_feedback


    def spin_once(self):
        """ROS2 스핀 (서버 메인 루프에서 호출)"""
        for nav in self.nav_modules.values():
            rclpy.spin_once(nav, timeout_sec=0.01)


    def shutdown(self):
        """종료"""
        print("[Navigation] 종료 중...")

        for robot_id, nav in self.nav_modules.items():
            print(f"[Navigation] 로봇 {robot_id} 종료")
            nav.destroy_node()

        self.nav_modules.clear()
        print("[Navigation] 종료 완료")

# 테스트 코드
if __name__ == "__main__":
    import time

    print("=" * 60)
    print("Navigation 모듈 테스트")
    print("=" * 60)

    # ROS2 초기화
    rclpy.init()

    # 컨트롤러 생성
    controller = NavigationController()

    # 로봇 추가 (Domain ID는 환경변수로 설정되어야 함)
    print("\n[1] 로봇 1 추가 중...")
    if not controller.add_robot(1):
        print("실패! Nav2가 실행 중인지 확인하세요.")
        print("명령어: ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/path/to/map.yaml")
        rclpy.shutdown()
        exit(1)

    # 콜백 등록
    def on_reached(robot_id):
        print(f"\n[콜백] 로봇 {robot_id} 도착!")

    def on_failed(robot_id, reason):
        print(f"\n[콜백] 로봇 {robot_id} 실패: {reason}")

    def on_feedback(robot_id, x, y):
        print(f"\r[피드백] 로봇 {robot_id} 위치: x={x:.2f}, y={y:.2f}", end='')

    controller.register_callbacks(1, on_reached, on_failed, on_feedback)

    # 목표 설정
    print("\n[2] 목표 설정: x=2.0, y=1.0")
    controller.navigate_to(1, x=2.0, y=1.0, yaw=0.0)

    # 스핀 루프
    print("\n[3] 이동 중... (Ctrl+C로 취소)")
    try:
        while controller.is_navigating(1):
            controller.spin_once()
            time.sleep(0.1)

        print("\n\n테스트 완료!")

    except KeyboardInterrupt:
        print("\n\n취소됨")
        controller.cancel_navigation(1)

    finally:
        controller.shutdown()
        rclpy.shutdown()
