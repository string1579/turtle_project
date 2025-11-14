"""
=================================================================
QR 인식 + 동작 제어 모듈
=================================================================
[목적]
- 카메라 이미지에서 QR 코드 인식
- QR 내용에 따라 로봇 동작 제어

[작업 방법]
1. process_image(): QR 인식 로직 (이미 구현됨)
2. on_qr_detected(): QR 인식 후 동작 정의 (여기에 작업!)

[테스트 방법]
단독 테스트 (웹캠 사용):
$ cd ~/작업공간/turtle_project
$ python3 modules/qr_module.py

통합 테스트:
1. 서버 실행: python3 -m server.server
2. GUI 실행: python3 -m gui.gui
3. GUI에서 "QR 인식 시작" 버튼 클릭

[QR 코드 준비]
온라인 QR 생성기에서 텍스트 QR 생성:
- "forward" (전진)
- "backward" (후진)
- "left" (좌회전)
- "right" (우회전)
- "stop" (정지)
- "align" (정밀 정렬)
- "goal_1", "goal_home" 등 (목표 이동)
=================================================================
"""

import cv2
import numpy as np
from typing import Optional, Tuple

class QRModule:
    """
    QR 코드 인식 및 동작 제어 클래스

    [구조]
    1. __init__: 초기 설정
    2. process_image: QR 인식
    3. on_qr_detected: 인식 후 동작 (작업 공간!)
    4. align_to_qr: 정밀 정렬
    5. get_qr_position: QR 위치 계산
    """

    def __init__(self, robot_controller=None):
        """
        QR 모듈 초기화

        [입력]
        - robot_controller: 로봇 제어 인터페이스
          서버에서 자동으로 전달되므로 신경쓰지 않아도 됨

        [내부 변수 설명]
        - self.detector: OpenCV QR 인식기
        - self.last_detect_time: 마지막 인식 시간 (쿨타임 계산용)
        - self.cooldown: 같은 QR을 다시 인식하기까지 대기 시간 (초)
        - self.robot_controller: 로봇 제어용 객체

        [수정 가능]
        self.cooldown 값을 바꾸면 QR 재인식 주기 조절 가능
        - 3.0: 3초마다 같은 QR 인식 가능 (기본값)
        - 1.0: 1초마다 인식 (빠름)
        - 5.0: 5초마다 인식 (느림)
        """
        # OpenCV QR 인식기 생성
        # [참고] pyzbar 등 다른 라이브러리로 교체 가능
        self.detector = cv2.QRCodeDetector()

        # 쿨타임 관련 변수
        self.last_detect_time = 0.0     # 마지막 인식 시간 (초기값 0)
        self.cooldown = 3.0              # 3초마다 인식 (수정 가능!)

        # 로봇 제어 인터페이스 (서버에서 전달받음)
        self.robot_controller = robot_controller

        print("[QR 모듈] 초기화 완료")

    def process_image(self, image: np.ndarray, current_time: float,
                     robot_id: int) -> Optional[str]:
        """
        이미지에서 QR 코드 인식

        [입력]
        - image: OpenCV 이미지 (numpy 배열, BGR 형식)
          서버가 카메라에서 받아서 자동으로 전달

        - current_time: 현재 시간 (초 단위)
          time.time() 값, 서버가 자동으로 전달

        - robot_id: 로봇 번호 (1, 2, 3)
          어떤 로봇이 QR을 인식했는지 구분

        [출력]
        - QR 코드 내용 (문자열)
          예: "forward", "goal_1" 등
        - 인식 실패 시: None

        [작동 흐름]
        1. QR 인식 시도
        2. 인식 성공하면 쿨타임 체크
        3. 쿨타임 OK면 on_qr_detected() 호출
        4. QR 텍스트 반환

        [수정 금지]
        이 함수는 건드리지 마세요.
        동작 로직은 on_qr_detected()에 작성하세요.
        """
        try:
            # ========================================
            # 1단계: QR 인식 시도
            # ========================================
            # detectAndDecode()는 OpenCV 함수
            # [반환값]
            # - data: QR 내용 (문자열, 없으면 빈 문자열)
            # - points: QR 코드 4개 꼭짓점 좌표
            # - _: 직선 QR 코드 (사용 안 함)
            data, points, _ = self.detector.detectAndDecode(image)

            # ========================================
            # 2단계: 인식 성공 확인
            # ========================================
            # data가 있고 (QR 인식됨)
            # 쿨타임이 지났으면 (같은 QR 반복 인식 방지)
            if data and (current_time - self.last_detect_time > self.cooldown):

                # 쿨타임 갱신 (다음 인식까지 대기)
                self.last_detect_time = current_time

                # QR 텍스트 정리
                # strip(): 앞뒤 공백 제거
                # lower(): 소문자로 변환 ("Forward" -> "forward")
                qr_text = data.strip().lower()

                print(f"[QR 인식] {qr_text}")

                # ========================================
                # 3단계: 동작 실행
                # ========================================
                # on_qr_detected()를 호출해서 실제 동작 수행
                # 이 함수는 아래에 정의되어 있음
                self.on_qr_detected(qr_text, robot_id, image)

                # QR 텍스트 반환 (서버로 전달됨)
                return qr_text

            # 인식 실패하거나 쿨타임 중이면 None 반환
            return None

        except Exception as e:
            # 에러 발생 시 (카메라 문제 등)
            print(f"[QR 에러] {e}")
            return None

    # ======================================================================
    # 여기서부터 작업 공간!
    # ======================================================================

    def on_qr_detected(self, qr_text: str, robot_id: int, image: np.ndarray):
        """
        QR 인식 후 실행할 동작 정의

        [입력]
        - qr_text: QR 코드 내용 (문자열)
          예: "forward", "goal_1", "stop" 등

        - robot_id: 로봇 번호 (1, 2, 3)
          여러 로봇이 각자 다른 동작을 해야 할 때 사용

        - image: 현재 카메라 이미지
          정밀 정렬 시 QR 위치 계산에 사용

        [사용 가능한 제어 함수]
        1. self.robot_controller.move(robot_id, linear, angular)
           - robot_id: 제어할 로봇 번호
           - linear: 직진 속도 (m/s, 양수=전진, 음수=후진)
           - angular: 회전 속도 (rad/s, 양수=좌회전, 음수=우회전)

        2. self.robot_controller.stop(robot_id)
           - 로봇 정지

        3. self.robot_controller.sleep(seconds)
           - 지정한 시간만큼 대기 (초 단위)

        [작업 방법]
        1. if-elif 구조로 QR 내용별 동작 정의
        2. 각 동작은 move() -> sleep() -> stop() 패턴
        3. 새 QR 코드 추가는 elif 블록 추가

        [예시 1: 2초 전진]
        if qr_text == "forward_long":
            self.robot_controller.move(robot_id, 0.2, 0.0)
            self.robot_controller.sleep(2.0)
            self.robot_controller.stop(robot_id)

        [예시 2: 90도 회전]
        if qr_text == "turn_90":
            self.robot_controller.move(robot_id, 0.0, 0.5)
            self.robot_controller.sleep(3.14)  # 90도 = 약 3.14초
            self.robot_controller.stop(robot_id)

        [예시 3: 로봇별 다른 동작]
        if qr_text == "split":
            if robot_id == 1:
                # 로봇1은 전진
                self.robot_controller.move(robot_id, 0.2, 0.0)
            else:
                # 나머지는 정지
                self.robot_controller.stop(robot_id)
        """

        # ========================================
        # 로봇 컨트롤러 확인
        # ========================================
        if not self.robot_controller:
            print("[경고] 로봇 컨트롤러 없음 (서버 연동 필요)")
            return

        print(f"[동작 실행] QR={qr_text}, 로봇={robot_id}")

        # ========================================
        # QR 내용별 동작 정의
        # ========================================

        # ----------------------------------------
        # 기본 이동 명령
        # ----------------------------------------

        if qr_text == "forward":
            """
            전진 동작

            [동작]
            1. 0.2 m/s로 전진
            2. 1초 대기 (약 20cm 이동)
            3. 정지

            [수정 방법]
            - 속도 변경: 0.2를 다른 값으로 (최대 0.22)
            - 시간 변경: 1.0을 다른 값으로
            - 거리 = 속도 x 시간
            """
            print("-> 전진 실행")
            self.robot_controller.move(robot_id, linear=0.2, angular=0.0)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "backward":
            """
            후진 동작

            [동작]
            1. -0.15 m/s로 후진 (음수 = 뒤로)
            2. 1초 대기
            3. 정지
            """
            print("-> 후진 실행")
            self.robot_controller.move(robot_id, linear=-0.15, angular=0.0)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "left":
            """
            좌회전 동작

            [동작]
            1. 0.5 rad/s로 좌회전 (양수 = 왼쪽)
            2. 1초 대기 (약 30도 회전)
            3. 정지

            [참고]
            - 90도 회전하려면: 약 3.14초 필요
            - 180도 회전하려면: 약 6.28초 필요
            """
            print("-> 좌회전 실행")
            self.robot_controller.move(robot_id, linear=0.0, angular=0.5)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "right":
            """
            우회전 동작

            [동작]
            1. -0.5 rad/s로 우회전 (음수 = 오른쪽)
            2. 1초 대기
            3. 정지
            """
            print("-> 우회전 실행")
            self.robot_controller.move(robot_id, linear=0.0, angular=-0.5)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "stop":
            """
            즉시 정지

            [동작]
            바로 정지 (대기 시간 없음)
            """
            print("-> 정지 실행")
            self.robot_controller.stop(robot_id)

        # ----------------------------------------
        # 정밀 정렬
        # ----------------------------------------

        elif qr_text == "align":
            """
            QR 중심으로 정밀 정렬

            [동작]
            1. QR 코드 위치 찾기
            2. 화면 중앙과 비교
            3. 오차만큼 회전
            4. 오차 10픽셀 이하까지 반복

            [용도]
            - 정밀한 위치 보정
            - 적재/하역 지점 정렬

            [참고]
            align_to_qr() 함수 사용 (아래에 구현됨)
            """
            print("-> QR 정렬 시작")
            self.align_to_qr(robot_id, image)

        # ----------------------------------------
        # 목표 좌표로 이동
        # ----------------------------------------

        elif qr_text.startswith("goal_"):
            """
            목표 지점으로 이동

            [QR 형식]
            - "goal_1": 목표1로 이동
            - "goal_2": 목표2로 이동
            - "goal_home": 원점으로 복귀

            [동작]
            1. QR에서 목표 ID 추출 ("goal_1" -> "1")
            2. goals 딕셔너리에서 좌표 찾기
            3. Navigation으로 이동 (추후 구현)

            [목표 좌표 추가 방법]
            goals 딕셔너리에 새 항목 추가:
            "3": {"x": 2.0, "y": 3.0}
            """
            # QR 텍스트에서 목표 ID 추출
            # "goal_1" -> ["goal", "1"] -> "1"
            goal_id = qr_text.split("_")[1]
            print(f"-> 목표 {goal_id}로 이동")

            # 목표 좌표 정의
            # [수정 방법]
            # 새 목표 추가: "4": {"x": 4.0, "y": 2.0}
            goals = {
                "1": {"x": 1.0, "y": 2.0},      # 목표1 좌표
                "2": {"x": 3.0, "y": 1.5},      # 목표2 좌표
                "home": {"x": 0.0, "y": 0.0}    # 원점 좌표
            }

            # 목표 ID가 딕셔너리에 있는지 확인
            if goal_id in goals:
                goal = goals[goal_id]
                print(f"  목표 좌표: x={goal['x']}, y={goal['y']}")

                # Navigation 연동 (추후 구현 예정)
                # 현재는 좌표만 출력
                # 나중에 아래 줄 주석 해제:
                # self.robot_controller.navigate_to(robot_id, goal['x'], goal['y'])
            else:
                print(f"  [경고] 알 수 없는 목표: {goal_id}")

        # ----------------------------------------
        # 알 수 없는 QR 코드
        # ----------------------------------------

        else:
            """
            정의되지 않은 QR 코드

            [동작]
            경고 메시지만 출력하고 아무 동작 안 함

            [추가 방법]
            위에 새 elif 블록을 추가하세요:

            elif qr_text == "my_custom":
                # 원하는 동작 작성
                pass
            """
            print(f"-> 알 수 없는 QR 코드: {qr_text}")

    # ======================================================================
    # 보조 함수들 (수정 가능)
    # ======================================================================

    def align_to_qr(self, robot_id: int, image: np.ndarray,
                    max_iterations: int = 30):
        """
        QR 코드 중심으로 정밀 정렬

        [입력]
        - robot_id: 로봇 번호
        - image: 현재 카메라 이미지
        - max_iterations: 최대 반복 횟수 (30번, 수정 가능)

        [작동 원리]
        1. QR 위치 찾기 (get_qr_position 사용)
        2. 화면 중앙(image_center_x)과 QR 중심 비교
        3. 오차 계산 (error_x = QR_x - 중앙_x)
        4. 비례 제어로 회전 (오차가 크면 빠르게, 작으면 천천히)
        5. 오차가 tolerance(10픽셀) 이하가 될 때까지 반복

        [수정 가능한 값]
        - tolerance: 허용 오차 (픽셀)
          10 -> 5: 더 정밀하게 (시간 오래 걸림)
          10 -> 20: 덜 정밀하게 (빠름)

        - angular 계산식의 0.002: P 제어 게인
          0.002 -> 0.003: 더 빠르게 회전 (진동 가능)
          0.002 -> 0.001: 더 천천히 회전 (안정적)

        - max(-0.3, min(0.3, angular)): 속도 제한
          -0.3, 0.3을 바꾸면 최대 회전 속도 조절

        [주의]
        실제로는 카메라 이미지가 계속 업데이트되어야 하지만
        현재는 초기 이미지만 사용 (간단한 버전)
        실전에서는 서버에서 최신 이미지를 받아와야 함
        """

        if not self.robot_controller:
            return

        # 화면 중앙 x 좌표 계산
        # image.shape = [높이, 너비, 채널]
        # 너비 // 2 = 중앙 x 좌표
        image_center_x = image.shape[1] // 2

        # 허용 오차 (픽셀)
        # 이 값보다 작으면 정렬 완료로 판단
        tolerance = 10

        # 반복 정렬
        for i in range(max_iterations):

            # ========================================
            # 1. QR 위치 찾기
            # ========================================
            qr_pos = self.get_qr_position(image)

            if not qr_pos:
                # QR을 찾을 수 없으면 종료
                print("[정렬] QR 찾을 수 없음")
                break

            # QR 중심 좌표
            qr_x, qr_y = qr_pos

            # ========================================
            # 2. 오차 계산
            # ========================================
            # error_x > 0: QR이 오른쪽에 있음 -> 오른쪽으로 회전 필요
            # error_x < 0: QR이 왼쪽에 있음 -> 왼쪽으로 회전 필요
            error_x = qr_x - image_center_x

            print(f"[정렬 {i+1}/{max_iterations}] 오차: {error_x}px")

            # ========================================
            # 3. 종료 조건 확인
            # ========================================
            if abs(error_x) < tolerance:
                # 오차가 허용 범위 내면 정렬 완료
                print("[정렬 완료]")
                self.robot_controller.stop(robot_id)
                break

            # ========================================
            # 4. 비례 제어 (P 제어)
            # ========================================
            # angular = -error_x * 게인
            # 음수 부호: QR이 오른쪽(+)이면 오른쪽 회전(-)
            # 게인 0.002: 오차에 비례해서 속도 조절
            angular = -error_x * 0.002

            # 속도 제한 (-0.3 ~ 0.3 rad/s)
            # 너무 빠르면 오버슈트 발생
            angular = max(-0.3, min(0.3, angular))

            # ========================================
            # 5. 로봇 제어
            # ========================================
            # 제자리 회전 (linear=0)
            self.robot_controller.move(robot_id, linear=0.0, angular=angular)

            # 짧은 대기 (0.1초)
            self.robot_controller.sleep(0.1)

        # 최종 정지
        self.robot_controller.stop(robot_id)

    def get_qr_position(self, image: np.ndarray) -> Optional[Tuple[int, int]]:
        """
        QR 코드의 화면상 위치 구하기

        [입력]
        - image: OpenCV 이미지

        [출력]
        - (x, y): QR 중심 좌표 (픽셀 단위)
        - None: QR을 찾지 못함

        [작동 원리]
        1. QR 인식 (detectAndDecode)
        2. QR 코드 4개 꼭짓점 좌표 받기
        3. 4개 점의 평균으로 중심 계산

        [예시]
        QR 꼭짓점: [(100,100), (200,100), (200,200), (100,200)]
        중심 x = (100+200+200+100)/4 = 150
        중심 y = (100+100+200+200)/4 = 150
        반환: (150, 150)

        [용도]
        - align_to_qr()에서 사용
        - 화면 중앙과 비교해서 정렬
        """
        try:
            # QR 인식 시도
            data, points, _ = self.detector.detectAndDecode(image)

            # QR이 인식되고 좌표가 있으면
            if data and points is not None:

                # points 형태: [[[x1,y1], [x2,y2], [x3,y3], [x4,y4]]]
                # points[0]: 첫 번째 QR의 좌표들
                # points[0][:, 0]: 모든 x 좌표
                # points[0][:, 1]: 모든 y 좌표

                # 중심 x 좌표 = 모든 x의 평균
                center_x = int(np.mean(points[0][:, 0]))

                # 중심 y 좌표 = 모든 y의 평균
                center_y = int(np.mean(points[0][:, 1]))

                return (center_x, center_y)

            # QR 없으면 None 반환
            return None

        except Exception as e:
            print(f"[QR 위치 에러] {e}")
            return None

# ======================================================================
# 테스트 코드 (이 파일을 직접 실행했을 때만 동작)
# ======================================================================
if __name__ == "__main__":
    """
    단독 테스트 모드

    [실행 방법]
    $ python3 modules/qr_module.py

    [필요사항]
    - 웹캠 연결
    - QR 코드 준비

    [종료]
    q 키 누르기
    """

    print("=" * 50)
    print("QR 모듈 단독 테스트")
    print("=" * 50)

    import time

    # ========================================
    # 가짜 로봇 컨트롤러 (테스트용)
    # ========================================
    class FakeController:
        """
        실제 로봇 없이 테스트하기 위한 가짜 컨트롤러

        [기능]
        - 명령을 콘솔에 출력만 함
        - 실제로는 로봇을 움직이지 않음
        """
        def move(self, robot_id, linear, angular):
            print(f"  [제어] 로봇{robot_id} 이동: L={linear:.2f}, A={angular:.2f}")

        def stop(self, robot_id):
            print(f"  [제어] 로봇{robot_id} 정지")

        def sleep(self, seconds):
            print(f"  [대기] {seconds}초")
            time.sleep(seconds)

    # ========================================
    # QR 모듈 생성
    # ========================================
    qr = QRModule(robot_controller=FakeController())

    # ========================================
    # 웹캠 열기
    # ========================================
    # VideoCapture(0): 첫 번째 카메라
    # 여러 카메라 있으면 0, 1, 2 등으로 선택
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[에러] 카메라를 열 수 없습니다")
        print("  - 웹캠이 연결되어 있나요?")
        print("  - 다른 프로그램이 카메라를 사용 중인가요?")
        exit()

    print("\n카메라 시작! QR 코드를 보여주세요 (q: 종료)")
    print("\n테스트용 QR 코드 텍스트:")
    print("  - forward (전진)")
    print("  - backward (후진)")
    print("  - left (좌회전)")
    print("  - right (우회전)")
    print("  - stop (정지)")
    print("  - align (정렬)")
    print("  - goal_1, goal_home (목표 이동)")

    # ========================================
    # 메인 루프
    # ========================================
    while True:
        # 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            print("[에러] 프레임을 읽을 수 없습니다")
            break

        # QR 인식 (robot_id=1로 테스트)
        qr_data = qr.process_image(frame, time.time(), robot_id=1)

        # QR 인식되면 화면에 표시
        if qr_data:
            cv2.putText(frame, f"QR: {qr_data}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # QR 위치 표시
        pos = qr.get_qr_position(frame)
        if pos:
            # QR 중심에 빨간 점
            cv2.circle(frame, pos, 10, (0, 0, 255), -1)

            # 화면 중앙에 파란 점
            center = (frame.shape[1]//2, frame.shape[0]//2)
            cv2.circle(frame, center, 5, (255, 0, 0), -1)

        # 화면 표시
        cv2.imshow("QR Test", frame)

        # q 키 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # ========================================
    # 종료 처리
    # ========================================
    cap.release()
    cv2.destroyAllWindows()
    print("\n테스트 종료")
