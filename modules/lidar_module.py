"""
LiDAR 충돌 회피 모듈 (탈출 방향 탐색 강화)
- 진행 방향 체크로 충돌 판단
- 360도 스캔에서 최대 거리 방향 찾기
- 안전한 회피 방향 제안
"""

import numpy as np
import math
from typing import List, Tuple

class LidarModule:
    def __init__(self):
        """초기화: 안전거리 50cm, 체크각도 ±30도"""
        self.safe_distance = 0.5
        self.check_angle = 30
        print(f"[LiDAR] 초기화 (안전거리: {self.safe_distance}m, 각도: ±{self.check_angle}°)")

    def check_collision(self, ranges: List[float], moving_forward: bool = True) -> bool:
        """
        충돌 위험 체크

        Args:
            ranges: LiDAR 거리 데이터 (360개)
            moving_forward: True=전진(전방체크), False=후진(후방체크)

        Returns:
            True=충돌위험, False=안전
        """
        if not ranges or len(ranges) == 0:
            return False

        # 체크할 인덱스 범위 계산
        if moving_forward:
            start_idx = 360 - self.check_angle
            check_indices = list(range(start_idx, 360)) + list(range(0, self.check_angle))
        else:
            back_center = 180
            check_indices = list(range(back_center - self.check_angle,
                                     back_center + self.check_angle))

        # 유효한 거리값만 추출
        distances = []
        for i in check_indices:
            if i < len(ranges):
                dist = ranges[i]
                if not np.isinf(dist) and not np.isnan(dist) and dist > 0.01:
                    distances.append(dist)

        # 최소 거리 확인
        if distances:
            min_distance = min(distances)
            if min_distance < self.safe_distance:
                print(f"[LiDAR 경고] 충돌 위험! 최소거리: {min_distance:.2f}m")
                return True

        return False

    def find_best_escape_direction(self, ranges: List[float]) -> float:
        """
        360도 스캔에서 가장 멀리 갈 수 있는 방향 찾기

        Args:
            ranges: LiDAR 거리 데이터

        Returns:
            angular_velocity: 회전 속도 (양수=좌회전, 음수=우회전)
        """
        if not ranges:
            return 0.5

        max_distance = -1.0
        best_index = 0
        num_readings = len(ranges)

        for i in range(num_readings):
            r = ranges[i]
            if not math.isnan(r) and not math.isinf(r) and r > 0.01:
                if r > max_distance:
                    max_distance = r
                    best_index = i

        if max_distance < self.safe_distance + 0.1:
            print("[LiDAR] 주변 완전 막힘 - 임의 회전")
            return 0.5

        # 중앙(0도)에서 얼마나 떨어졌는지 계산
        center_index = 0
        angle_diff = best_index - center_index

        # 각도 차이를 180도 이내로 정규화
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # 회전 방향 결정
        if angle_diff > 0:
            angular = 0.5
            print(f"[LiDAR] 좌회전 ({angle_diff}도)")
        else:
            angular = -0.5
            print(f"[LiDAR] 우회전 ({angle_diff}도)")

        return angular

    def get_safe_direction(self, ranges: List[float]) -> Tuple[float, float]:
        """
        안전한 방향 찾기 (이전 버전 호환용)

        Returns:
            (linear, angular): 안전한 속도 값
        """
        angular = self.find_best_escape_direction(ranges)
        return (0.0, angular)

if __name__ == "__main__":
    print("=" * 50)
    print("LiDAR 모듈 테스트")
    print("=" * 50)

    lidar = LidarModule()

    # 테스트 1: 전방 장애물
    print("\n[테스트 1] 전방 장애물 (40cm)")
    test_ranges = [1.0] * 360
    test_ranges[0] = 0.4
    result = lidar.check_collision(test_ranges, moving_forward=True)
    print(f"결과: {'위험' if result else '안전'}")

    # 테스트 2: 탈출 방향 (왼쪽 개방)
    print("\n[테스트 2] 탈출 방향 (왼쪽 개방)")
    test_ranges = [0.3] * 360
    test_ranges[60:120] = [2.0] * 60
    angular = lidar.find_best_escape_direction(test_ranges)
    print(f"회전 속도: {angular:.2f} rad/s")
    print(f"방향: {'좌회전' if angular > 0 else '우회전'}")
