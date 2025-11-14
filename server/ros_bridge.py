"""
=================================================================
ROS2 브릿지 - 멀티프로세스 버전
=================================================================
[목적]
- 각 터틀봇과 ROS2 통신
- 터틀봇마다 별도 프로세스로 실행 (독립 제어)
- 명령 큐로 서버와 통신

[작동 원리]
1. 각 터틀봇마다 별도 프로세스 생성
2. ROS_DOMAIN_ID 환경변수로 통신 분리
   - 터틀봇1: DOMAIN_ID=17
   - 터틀봇2: DOMAIN_ID=3
   - 터틀봇3: DOMAIN_ID=5
3. 명령 큐(Queue)로 서버 -> 브릿지 명령 전달
4. 상태 큐(Queue)로 브릿지 -> 서버 데이터 전달

[ROS2 토픽]
발행(Publish):
- /cmd_vel: 로봇 이동 명령

구독(Subscribe):
- /odom: 위치 정보
- /scan: LiDAR 데이터
- /camera/image_raw/compressed: 카메라 이미지

[수정 금지]
이 파일은 ROS2 통신 핵심이므로 함부로 수정하지 마세요.
토픽 이름만 변경 가능 (실제 터틀봇 설정에 맞춰서)
=================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
import os
import time
from multiprocessing import Process, Queue
import numpy as np

# ======================================================================
# 터틀봇 제어 노드 (각 터틀봇마다 1개씩 실행)
# ======================================================================
class TurtlebotBridge(Node):
    """
    터틀봇 ROS2 제어 노드

    [역할]
    1. 명령 큐에서 명령 받기 (서버 -> 브릿지)
    2. cmd_vel 토픽으로 로봇 제어 (브릿지 -> 터틀봇)
    3. 센서 토픽 구독 (터틀봇 -> 브릿지)
    4. 상태 큐로 데이터 전달 (브릿지 -> 서버)

    [프로세스 구조]
    서버 프로세스
      └─ 브릿지 프로세스 (이 클래스)
           ├─ ROS2 노드 실행
           ├─ 명령 큐 읽기
           └─ 상태 큐 쓰기
    """

    def __init__(self, robot_id: int, domain_id: int,
                 command_queue: Queue, status_queue: Queue):
        """
        브릿지 노드 초기화

        [입력]
        - robot_id: 로봇 번호 (1, 2, 3)
          로그 출력 및 상태 전달 시 사용

        - domain_id: ROS 도메인 ID (17, 3, 5)
          각 터틀봇과 통신을 분리하기 위한 ID
          환경변수 ROS_DOMAIN_ID로 설정됨

        - command_queue: 명령 받는 큐
          서버가 명령을 넣으면 이 노드가 꺼내서 실행
          형식: {"command": "move", "linear": 0.2, ...}

        - status_queue: 상태 보내는 큐
          이 노드가 센서 데이터를 넣으면 서버가 꺼내서 처리
          형식: {"type": "odom", "robot_id": 1, "data": {...}}

        [초기화 순서]
        1. ROS2 노드 생성 (부모 클래스)
        2. 변수 저장
        3. 퍼블리셔 생성 (cmd_vel)
        4. 서브스크라이버 생성 (odom, scan, camera)
        5. 타이머 생성 (명령 체크용)
        """

        # ========================================
        # 1. ROS2 노드 생성
        # ========================================
        # Node 클래스 상속받아서 초기화
        # 노드 이름: bridge_robot_1, bridge_robot_2 등
        super().__init__(f'bridge_robot_{robot_id}')

        # ========================================
        # 2. 변수 저장
        # ========================================
        self.robot_id = robot_id
        self.domain_id = domain_id
        self.command_queue = command_queue
        self.status_queue = status_queue

        # ========================================
        # 3. 퍼블리셔 생성 (로봇에게 명령 전송)
        # ========================================
        # [토픽 설명]
        # /cmd_vel: 로봇 속도 명령
        #   - 타입: Twist (linear, angular)
        #   - 주기: 명령이 있을 때마다
        #   - 용도: 로봇 이동 제어
        self.cmd_vel_pub = self.create_publisher(
            Twist,           # 메시지 타입
            '/cmd_vel',      # 토픽 이름 (수정 가능)
            10               # 큐 크기
        )

        # ========================================
        # 4. 서브스크라이버 생성 (로봇 상태 수신)
        # ========================================

        # [4-1] 위치 정보 구독
        # /odom: 로봇 현재 위치 (Odometry)
        #   - 좌표: x, y, z
        #   - 방향: quaternion
        #   - 속도: linear, angular
        #   - 주기: 약 10Hz
        #   - 용도: Navigation, 위치 추적
        self.odom_sub = self.create_subscription(
            Odometry,        # 메시지 타입
            '/odom',         # 토픽 이름 (수정 가능)
            self.odom_callback,  # 콜백 함수
            10               # 큐 크기
        )

        # [4-2] LiDAR 데이터 구독
        # /scan: 360도 거리 센서 (LaserScan)
        #   - ranges: 거리 배열 (360개)
        #   - angle_min/max: 각도 범위
        #   - 주기: 약 5-10Hz
        #   - 용도: 충돌 회피, 장애물 감지
        self.scan_sub = self.create_subscription(
            LaserScan,       # 메시지 타입
            '/scan',         # 토픽 이름 (수정 가능)
            self.scan_callback,  # 콜백 함수
            10               # 큐 크기
        )

        # [4-3] 카메라 이미지 구독
        # /camera/image_raw/compressed: 압축된 카메라 이미지
        #   - format: "jpeg"
        #   - data: 압축된 이미지 바이트
        #   - 주기: 약 30Hz
        #   - 용도: QR 인식, 비전 처리
        # [참고] 압축 안 된 이미지는 /camera/image_raw
        self.camera_sub = self.create_subscription(
            CompressedImage,     # 메시지 타입
            '/camera/image_raw/compressed',  # 토픽 이름 (수정 가능)
            self.camera_callback,    # 콜백 함수
            10                   # 큐 크기
        )

        # ========================================
        # 5. 내부 상태 변수
        # ========================================
        # 마지막으로 받은 센서 데이터 저장
        # (현재는 사용 안 하지만 추후 확장용)
        self.last_odom = None
        self.last_scan = None
        self.last_image = None

        # ========================================
        # 6. 명령 체크 타이머
        # ========================================
        # [작동]
        # 0.01초(10ms)마다 check_commands() 호출
        # 즉, 100Hz로 명령 큐 확인
        #
        # [이유]
        # 명령이 들어오면 빠르게 반응하기 위해
        # 너무 느리면: 반응 지연
        # 너무 빠르면: CPU 낭비 (10ms면 충분)
        self.timer = self.create_timer(0.01, self.check_commands)

        # 초기화 완료 로그
        self.get_logger().info(
            f'터틀봇 {robot_id} 브릿지 시작 (Domain: {domain_id})'
        )

    # ======================================================================
    # 명령 처리 (서버 -> 브릿지)
    # ======================================================================

    def check_commands(self):
        """
        명령 큐 확인 및 실행

        [호출]
        타이머에 의해 0.01초마다 자동 호출 (100Hz)

        [작동]
        1. 큐에 명령이 있는지 확인
        2. 있으면 꺼내서 execute_command() 호출
        3. 없으면 그냥 리턴

        [명령 형식]
        {
            "command": "move" 또는 "stop",
            "linear": 0.2,      # move일 때만
            "angular": 0.0      # move일 때만
        }

        [while 루프 이유]
        큐에 여러 명령이 쌓여있을 수 있으므로
        한 번에 모두 처리
        """
        while not self.command_queue.empty():
            try:
                # 큐에서 명령 꺼내기 (논블로킹)
                # get_nowait(): 즉시 반환 (대기 안 함)
                command = self.command_queue.get_nowait()

                # 명령 실행
                self.execute_command(command)
            except:
                # 큐가 비었거나 에러 발생 시
                # 무시하고 계속 진행
                pass

    def execute_command(self, command: dict):
        """
        명령 실행

        [입력]
        command: 명령 딕셔너리
          {"command": "move", "linear": 0.2, "angular": 0.0}
          또는
          {"command": "stop"}

        [지원 명령]
        1. "move": 로봇 이동
           - linear: 직진 속도
           - angular: 회전 속도

        2. "stop": 로봇 정지
           - 추가 인자 없음

        [확장 방법]
        새 명령 추가 시:
        elif cmd_type == 'new_command':
            # 처리 로직
            pass
        """
        # 명령 타입 추출
        cmd_type = command.get('command')

        if cmd_type == 'move':
            # 이동 명령
            # get(키, 기본값): 키가 없으면 기본값 사용
            linear = command.get('linear', 0.0)
            angular = command.get('angular', 0.0)
            self.move(linear, angular)

        elif cmd_type == 'stop':
            # 정지 명령
            self.stop()

    def move(self, linear: float, angular: float):
        """
        로봇 이동

        [입력]
        - linear: 직진 속도 (m/s)
          양수: 전진
          음수: 후진
          범위: -0.22 ~ 0.22 (터틀봇3 사양)

        - angular: 회전 속도 (rad/s)
          양수: 좌회전 (반시계)
          음수: 우회전 (시계)
          범위: -2.84 ~ 2.84

        [작동]
        1. Twist 메시지 생성
        2. linear.x, angular.z 설정
        3. /cmd_vel 토픽으로 발행

        [참고]
        Twist 메시지 구조:
          linear:
            x: 전후 속도 (사용)
            y: 좌우 속도 (미사용, 터틀봇은 전후만)
            z: 상하 속도 (미사용)
          angular:
            x: roll (미사용)
            y: pitch (미사용)
            z: yaw 회전 (사용)
        """
        # Twist 메시지 생성
        msg = Twist()

        # 직진 속도 설정
        # float(): 명시적 타입 변환 (안전)
        msg.linear.x = float(linear)

        # 회전 속도 설정
        msg.angular.z = float(angular)

        # 토픽 발행 (로봇으로 전송)
        self.cmd_vel_pub.publish(msg)

        # 로그 출력 (선택)
        # 너무 많이 출력되면 주석 처리
        # self.get_logger().info(f'이동: L={linear:.2f}, A={angular:.2f}')

    def stop(self):
        """
        로봇 정지

        [작동]
        모든 속도를 0으로 설정해서 발행
        """
        # 정지 메시지 생성 (모든 속도 0)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # 토픽 발행
        self.cmd_vel_pub.publish(msg)

        # 정지 로그
        self.get_logger().info('정지')

    # ======================================================================
    # 센서 데이터 수신 (터틀봇 -> 브릿지)
    # ======================================================================

    def odom_callback(self, msg):
        """
        위치 정보 수신 콜백

        [입력]
        msg: Odometry 메시지
          - pose.pose.position: 위치 (x, y, z)
          - pose.pose.orientation: 방향 (quaternion)
          - twist.twist: 속도

        [처리]
        1. 위치 정보 추출 (x, y, z)
        2. 딕셔너리로 변환
        3. 상태 큐에 전달 (서버로)

        [용도]
        - Navigation에서 현재 위치 확인
        - 목표 지점 도달 여부 판단
        - 경로 추적

        [호출 주기]
        터틀봇이 /odom 토픽을 발행할 때마다 (약 10Hz)
        """
        # 위치 정보 추출
        self.last_odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
        }

        # 서버로 전송
        try:
            # 상태 큐에 데이터 넣기 (논블로킹)
            # put_nowait(): 즉시 반환 (대기 안 함)
            self.status_queue.put_nowait({
                'type': 'odom',              # 데이터 타입
                'robot_id': self.robot_id,   # 로봇 번호
                'data': self.last_odom       # 실제 데이터
            })
        except:
            # 큐가 꽉 찼으면 무시
            # (다음 데이터로 대체될 것이므로)
            pass

    def scan_callback(self, msg):
        """
        LiDAR 데이터 수신 콜백

        [입력]
        msg: LaserScan 메시지
          - ranges: 거리 배열 [r0, r1, ..., r359]
          - angle_min: 시작 각도 (라디안)
          - angle_max: 끝 각도 (라디안)
          - angle_increment: 각도 간격
          - range_min: 최소 측정 거리
          - range_max: 최대 측정 거리

        [처리]
        1. ranges 배열을 리스트로 변환
        2. 각도 정보 포함
        3. 상태 큐에 전달

        [용도]
        - LiDAR 모듈에서 충돌 체크
        - 장애물 감지
        - 지도 생성 (SLAM)

        [호출 주기]
        터틀봇이 /scan 토픽을 발행할 때마다 (약 5-10Hz)

        [데이터 크기]
        ranges: 360개 float (약 1.5KB)
        네트워크 부하 적음
        """
        # LiDAR 데이터 추출
        self.last_scan = {
            'ranges': list(msg.ranges),      # 거리 배열 (360개)
            'angle_min': msg.angle_min,      # 시작 각도
            'angle_max': msg.angle_max,      # 끝 각도
        }

        # 서버로 전송
        try:
            self.status_queue.put_nowait({
                'type': 'scan',
                'robot_id': self.robot_id,
                'data': self.last_scan
            })
        except:
            pass

    def camera_callback(self, msg):
        """
        카메라 이미지 수신 콜백

        [입력]
        msg: CompressedImage 메시지
          - format: 이미지 형식 ("jpeg", "png" 등)
          - data: 압축된 이미지 바이트 배열

        [처리]
        1. 압축된 이미지를 바이트로 변환
        2. 형식 정보 포함
        3. 상태 큐에 전달

        [용도]
        - QR 모듈에서 QR 인식
        - 비전 처리
        - 객체 인식

        [호출 주기]
        터틀봇이 /camera 토픽을 발행할 때마다 (약 30Hz)

        [데이터 크기]
        압축 이미지: 약 20-40KB
        압축 안 된 이미지: 약 300KB (사용 안 함)

        [네트워크 부하]
        30Hz x 30KB = 약 900KB/s = 0.9MB/s
        공유기 부하의 주범!
        카메라 FPS를 낮추면 부하 감소
        """
        # 압축 이미지 데이터 저장
        self.last_image = {
            'data': bytes(msg.data),     # 바이트 배열로 변환
            'format': msg.format         # 형식 ("jpeg" 등)
        }

        # 서버로 전송
        try:
            self.status_queue.put_nowait({
                'type': 'camera',
                'robot_id': self.robot_id,
                'data': self.last_image
            })
        except:
            # 큐가 꽉 차면 무시
            # 카메라는 고속이므로 다음 프레임 사용
            pass

# ======================================================================
# 브릿지 프로세스 실행 함수
# ======================================================================
def run_bridge_process(robot_id: int, domain_id: int,
                      command_queue: Queue, status_queue: Queue):
    """
    브릿지 프로세스 메인 함수

    [목적]
    별도 프로세스에서 ROS2 브릿지 노드 실행

    [입력]
    - robot_id: 로봇 번호
    - domain_id: ROS 도메인 ID
    - command_queue: 명령 큐
    - status_queue: 상태 큐

    [작동 순서]
    1. 환경변수 설정 (ROS_DOMAIN_ID)
    2. ROS2 초기화
    3. 브릿지 노드 생성
    4. 노드 실행 (무한 루프)
    5. 종료 시 정리

    [프로세스 격리]
    이 함수는 별도 프로세스에서 실행되므로
    서버 프로세스와 완전히 독립적
    - 메모리 공간 분리
    - 환경변수 분리
    - ROS2 컨텍스트 분리

    [중요]
    ROS_DOMAIN_ID는 이 프로세스에서만 적용됨
    서버 프로세스나 다른 브릿지 프로세스에 영향 없음
    """

    # ========================================
    # 1. 환경변수 설정
    # ========================================
    # ROS_DOMAIN_ID 설정
    # [효과]
    # 같은 도메인 ID의 ROS2 노드끼리만 통신
    # 다른 도메인 ID는 서로 보이지 않음
    #
    # [예시]
    # 터틀봇1 (domain=17): 브릿지1과만 통신
    # 터틀봇2 (domain=3): 브릿지2와만 통신
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    # ========================================
    # 2. ROS2 초기화
    # ========================================
    # rclpy 초기화 (각 프로세스에서 1번만)
    rclpy.init()

    # ========================================
    # 3. 브릿지 노드 생성
    # ========================================
    bridge = TurtlebotBridge(robot_id, domain_id, command_queue, status_queue)

    try:
        # ========================================
        # 4. 노드 실행 (무한 루프)
        # ========================================
        # spin(): 콜백 처리 무한 루프
        # - 토픽 메시지 수신 시 콜백 호출
        # - 타이머 만료 시 콜백 호출
        # - Ctrl+C 누르면 종료
        rclpy.spin(bridge)

    except KeyboardInterrupt:
        # Ctrl+C 시
        pass

    finally:
        # ========================================
        # 5. 정리
        # ========================================
        # 노드 삭제
        bridge.destroy_node()

        # rclpy 종료
        rclpy.shutdown()

# ======================================================================
# 멀티 로봇 매니저 (서버에서 사용)
# ======================================================================
class MultiRobotManager:
    """
    여러 터틀봇을 동시에 관리하는 클래스

    [역할]
    1. 로봇 추가/제거
    2. 명령 전송 (서버 -> 브릿지)
    3. 상태 수신 (브릿지 -> 서버)

    [구조]
    서버 (1개)
      └─ MultiRobotManager (1개)
           ├─ 브릿지 프로세스 1 (터틀봇1)
           ├─ 브릿지 프로세스 2 (터틀봇2)
           └─ 브릿지 프로세스 3 (터틀봇3)

    [사용 예시]
    manager = MultiRobotManager()
    manager.add_robot(1, 17)  # 터틀봇1 추가
    manager.move_robot(1, 0.2, 0.0)  # 전진
    manager.stop_robot(1)  # 정지
    manager.shutdown()  # 종료
    """

    def __init__(self):
        """
        매니저 초기화

        [내부 변수]
        - self.processes: {robot_id: Process}
          각 로봇의 브릿지 프로세스 객체

        - self.command_queues: {robot_id: Queue}
          명령 전달용 큐 (서버 -> 브릿지)

        - self.status_queues: {robot_id: Queue}
          상태 전달용 큐 (브릿지 -> 서버)
        """
        self.processes = {}
        self.command_queues = {}
        self.status_queues = {}

        print("[매니저] 멀티 로봇 매니저 생성")

    def add_robot(self, robot_id: int, domain_id: int):
        """
        터틀봇 추가

        [입력]
        - robot_id: 로봇 번호 (1, 2, 3)
        - domain_id: ROS 도메인 ID (17, 3, 5)

        [출력]
        - True: 성공
        - False: 실패 (이미 존재)

        [작동]
        1. 이미 존재하는지 확인
        2. 명령/상태 큐 생성
        3. 브릿지 프로세스 생성 및 시작
        4. 정보 저장

        [프로세스 시작]
        Process 객체 생성 후 start() 호출하면
        별도 프로세스에서 run_bridge_process() 실행
        """

        # ========================================
        # 1. 중복 확인
        # ========================================
        if robot_id in self.processes:
            print(f"[매니저] 로봇 {robot_id}은 이미 연결됨")
            return False

        # ========================================
        # 2. 큐 생성
        # ========================================
        # Queue(): 프로세스 간 통신용 큐
        # multiprocessing.Queue는 프로세스 안전
        command_queue = Queue()
        status_queue = Queue()

        # ========================================
        # 3. 프로세스 생성 및 시작
        # ========================================
        process = Process(
            target=run_bridge_process,  # 실행할 함수
            args=(robot_id, domain_id, command_queue, status_queue)  # 인자
        )
        process.start()  # 프로세스 시작

        # ========================================
        # 4. 정보 저장
        # ========================================
        self.processes[robot_id] = process
        self.command_queues[robot_id] = command_queue
        self.status_queues[robot_id] = status_queue

        print(f"[매니저] 터틀봇 {robot_id} 추가 (Domain: {domain_id}, PID: {process.pid})")

        # 초기화 대기 (ROS2 노드 준비 시간)
        time.sleep(0.5)

        return True

    def send_command(self, robot_id: int, command: dict):
        """
        명령 전송

        [입력]
        - robot_id: 로봇 번호
        - command: 명령 딕셔너리

        [출력]
        - True: 성공
        - False: 실패

        [작동]
        명령 큐에 명령을 넣으면
        브릿지 프로세스가 꺼내서 실행
        """
        if robot_id not in self.command_queues:
            return False

        try:
            self.command_queues[robot_id].put(command)
            return True
        except:
            return False

    def move_robot(self, robot_id: int, linear: float, angular: float):
        """
        로봇 이동 (편의 함수)

        [입력]
        - robot_id: 로봇 번호
        - linear: 직진 속도
        - angular: 회전 속도

        [출력]
        - True: 성공
        - False: 실패
        """
        command = {
            'command': 'move',
            'linear': linear,
            'angular': angular
        }
        return self.send_command(robot_id, command)

    def stop_robot(self, robot_id: int):
        """
        로봇 정지 (편의 함수)

        [입력]
        - robot_id: 로봇 번호

        [출력]
        - True: 성공
        - False: 실패
        """
        command = {
            'command': 'stop'
        }
        return self.send_command(robot_id, command)

    def get_status(self, robot_id: int):
        """
        상태 가져오기

        [입력]
        - robot_id: 로봇 번호

        [출력]
        - 상태 딕셔너리 또는 None

        [작동]
        상태 큐에서 데이터 꺼내기
        없으면 None 반환
        """
        if robot_id not in self.status_queues:
            return None

        try:
            return self.status_queues[robot_id].get_nowait()
        except:
            return None

    def get_all_status(self):
        """
        모든 로봇 상태 한 번에 가져오기

        [출력]
        {
            robot_id: [status1, status2, ...],
            ...
        }

        [작동]
        각 로봇의 상태 큐에서
        쌓여있는 모든 데이터 꺼내기
        """
        all_status = {}

        for robot_id in self.status_queues.keys():
            statuses = []

            # 큐에서 모든 상태 꺼내기
            while True:
                status = self.get_status(robot_id)
                if status is None:
                    break
                statuses.append(status)

            if statuses:
                all_status[robot_id] = statuses

        return all_status

    def shutdown(self):
        """
        모든 프로세스 종료

        [작동]
        1. 각 브릿지 프로세스에 종료 신호 전송
        2. 정상 종료 대기 (2초)
        3. 강제 종료 (필요 시)
        4. 정보 삭제
        """
        for robot_id, process in self.processes.items():
            print(f"[매니저] 터틀봇 {robot_id} 종료 중")

            # 종료 신호 (SIGTERM)
            process.terminate()

            # 정상 종료 대기 (최대 2초)
            process.join(timeout=2)

            # 아직 살아있으면 강제 종료 (SIGKILL)
            if process.is_alive():
                process.kill()

        # 정보 삭제
        self.processes.clear()
        self.command_queues.clear()
        self.status_queues.clear()

        print("[매니저] 모든 로봇 종료 완료")

# ======================================================================
# 테스트 코드
# ======================================================================
if __name__ == "__main__":
    """
    브릿지 단독 테스트

    [실행 방법]
    $ python3 server/ros_bridge.py

    [필요사항]
    - 터틀봇 1번 켜져있어야 함
    - ROS2 환경 설정됨

    [테스트 내용]
    1. 터틀봇 1번 연결
    2. 3초 대기
    3. 전진 명령
    4. 2초 대기
    5. 정지 명령
    """

    print("=" * 50)
    print("ROS 브릿지 단독 테스트")
    print("=" * 50)

    # 매니저 생성
    manager = MultiRobotManager()

    # 터틀봇 1번 추가
    print("\n터틀봇 1번 연결")
    manager.add_robot(robot_id=1, domain_id=17)

    try:
        print("\n3초 대기")
        time.sleep(3)

        print("\n전진 명령 전송")
        manager.move_robot(1, linear=0.1, angular=0.0)

        time.sleep(2)

        print("\n정지 명령 전송")
        manager.stop_robot(1)

        print("\n상태 확인")
        status = manager.get_status(1)
        if status:
            print(f"상태: {status}")

        print("\n테스트 완료 (Ctrl+C로 종료)")

        # 대기
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n종료 중")

    finally:
        manager.shutdown()
