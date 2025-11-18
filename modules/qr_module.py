"""
QR 인식 + 동작 제어 모듈
- 카메라 이미지에서 QR 인식
- QR 내용에 따라 로봇 동작 제어
- QR 중심으로 정밀 정렬
"""

import cv2
import numpy as np
from typing import Optional, Tuple

class QRModule:
    def __init__(self, robot_controller=None):
        """초기화: QR 인식기 생성, 쿨타임 3초"""
        self.detector = cv2.QRCodeDetector()
        self.last_detect_time = 0.0
        self.cooldown = 3.0
        self.robot_controller = robot_controller
        print("[QR] 초기화 완료")

    def process_image(self, image: np.ndarray, current_time: float,
                     robot_id: int) -> Optional[str]:
        """
        이미지에서 QR 인식

        Args:
            image: OpenCV 이미지 (BGR)
            current_time: 현재 시간 (초)
            robot_id: 로봇 번호

        Returns:
            QR 텍스트 또는 None
        """
        try:
            data, points, _ = self.detector.detectAndDecode(image)

            if data and (current_time - self.last_detect_time > self.cooldown):
                self.last_detect_time = current_time
                qr_text = data.strip().lower()
                print(f"[QR 인식] {qr_text}")

                self.on_qr_detected(qr_text, robot_id, image)
                return qr_text

            return None

        except Exception as e:
            print(f"[QR 에러] {e}")
            return None

    def on_qr_detected(self, qr_text: str, robot_id: int, image: np.ndarray):
        """
        QR 인식 후 동작 실행

        사용 가능 함수:
        - self.robot_controller.move(robot_id, linear, angular)
        - self.robot_controller.stop(robot_id)
        - self.robot_controller.sleep(seconds)
        """
        if not self.robot_controller:
            print("[경고] 로봇 컨트롤러 없음")
            return

        print(f"[동작] QR={qr_text}, 로봇={robot_id}")

        # 기본 이동 명령
        if qr_text == "forward":
            print("-> 전진")
            self.robot_controller.move(robot_id, linear=0.2, angular=0.0)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "backward":
            print("-> 후진")
            self.robot_controller.move(robot_id, linear=-0.15, angular=0.0)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "left":
            print("-> 좌회전")
            self.robot_controller.move(robot_id, linear=0.0, angular=0.5)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "right":
            print("-> 우회전")
            self.robot_controller.move(robot_id, linear=0.0, angular=-0.5)
            self.robot_controller.sleep(1.0)
            self.robot_controller.stop(robot_id)

        elif qr_text == "stop":
            print("-> 정지")
            self.robot_controller.stop(robot_id)

        # QR 정밀 정렬
        elif qr_text == "align":
            print("-> QR 정렬 시작")
            self.align_to_qr(robot_id, image)

        # 목표 좌표로 이동
        elif qr_text.startswith("goal_"):
            goal_id = qr_text.split("_")[1]
            print(f"-> 목표 {goal_id}로 이동")

            goals = {
                "1": {"x": 1.0, "y": 2.0},
                "2": {"x": 3.0, "y": 1.5},
                "home": {"x": 0.0, "y": 0.0}
            }

            if goal_id in goals:
                goal = goals[goal_id]
                print(f"  목표 좌표: x={goal['x']}, y={goal['y']}")
                # TODO: Navigation 연동
                # self.robot_controller.navigate_to(robot_id, goal['x'], goal['y'])
            else:
                print(f"  [경고] 알 수 없는 목표: {goal_id}")

        else:
            print(f"-> 알 수 없는 QR: {qr_text}")

    def align_to_qr(self, robot_id: int, image: np.ndarray, max_iterations: int = 30):
        """
        QR 중심으로 정밀 정렬

        Args:
            max_iterations: 최대 반복 횟수
        """
        if not self.robot_controller:
            return

        image_center_x = image.shape[1] // 2
        tolerance = 10  # 허용 오차 (픽셀)

        for i in range(max_iterations):
            qr_pos = self.get_qr_position(image)
            if not qr_pos:
                print("[정렬] QR 찾을 수 없음")
                break

            qr_x, qr_y = qr_pos
            error_x = qr_x - image_center_x

            print(f"[정렬 {i+1}/{max_iterations}] 오차: {error_x}px")

            if abs(error_x) < tolerance:
                print("[정렬 완료]")
                self.robot_controller.stop(robot_id)
                break

            # 비례 제어
            angular = -error_x * 0.002
            angular = max(-0.3, min(0.3, angular))

            self.robot_controller.move(robot_id, linear=0.0, angular=angular)
            self.robot_controller.sleep(0.1)

        self.robot_controller.stop(robot_id)

    def get_qr_position(self, image: np.ndarray) -> Optional[Tuple[int, int]]:
        """QR 중심 좌표 반환"""
        try:
            data, points, _ = self.detector.detectAndDecode(image)
            if data and points is not None:
                center_x = int(np.mean(points[0][:, 0]))
                center_y = int(np.mean(points[0][:, 1]))
                return (center_x, center_y)
            return None
        except Exception as e:
            print(f"[QR 위치 에러] {e}")
            return None

# 테스트 코드
if __name__ == "__main__":
    import time

    print("=" * 50)
    print("QR 모듈 테스트 (웹캠 필요)")
    print("=" * 50)

    class FakeController:
        """테스트용 가짜 컨트롤러"""
        def move(self, robot_id, linear, angular):
            print(f"  [제어] 로봇{robot_id} 이동: L={linear:.2f}, A={angular:.2f}")
        def stop(self, robot_id):
            print(f"  [제어] 로봇{robot_id} 정지")
        def sleep(self, seconds):
            print(f"  [대기] {seconds}초")
            time.sleep(seconds)

    qr = QRModule(robot_controller=FakeController())

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[에러] 카메라 열 수 없음")
        exit()

    print("\nQR 코드를 보여주세요 (q: 종료)")
    print("테스트 QR: forward, backward, left, right, stop, align")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        qr_data = qr.process_image(frame, time.time(), robot_id=1)

        if qr_data:
            cv2.putText(frame, f"QR: {qr_data}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        pos = qr.get_qr_position(frame)
        if pos:
            cv2.circle(frame, pos, 10, (0, 0, 255), -1)
            center = (frame.shape[1]//2, frame.shape[0]//2)
            cv2.circle(frame, center, 5, (255, 0, 0), -1)

        cv2.imshow("QR Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
