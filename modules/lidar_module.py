"""
=================================================================
LiDAR 충돌 회피 모듈
=================================================================
[목적]
- LiDAR 센서 데이터를 분석해서 충돌 위험 판단
- 진행 방향만 체크 (요미 아이디어)
- 안전한 회피 방향 제안

[작업 방법]
1. check_collision(): 충돌 판단 로직 (이미 구현됨, 수정 가능)
2. get_safe_direction(): 안전한 방향 찾기 (수정 가능)
3. safe_distance, check_angle 조절로 민감도 변경

[테스트 방법]
단독 테스트:
$ cd ~/작업공간/turtle_project
$ python3 modules/lidar_module.py

통합 테스트:
1. 서버 실행: python3 -m server.server
2. GUI 실행: python3 -m gui.gui
3. GUI에서 "충돌 체크 시작" 버튼 클릭
4. 로봇 앞에 장애물 놓기

[LiDAR 데이터 구조]
- 360개 거리 값 (0~359도)
- 인덱스 0 = 정면
- 인덱스 90 = 왼쪽
- 인덱스 180 = 뒤
- 인덱스 270 = 오른쪽
=================================================================
"""

import numpy as np
from typing import List, Tuple

class LidarModule:
    """
    LiDAR 충돌 회피 전담 클래스

    [구조]
    1. __init__: 초기 설정
    2. check_collision: 충돌 위험 판단 (작업 공간!)
    3. get_safe_direction: 안전한 방향 찾기 (작업 공간!)
    """

    def __init__(self):
        """
        LiDAR 모듈 초기화

        [내부 변수 설명]
        - self.safe_distance: 안전 거리 (미터)
          이 거리보다 가까우면 충돌 위험으로 판단

        - self.check_angle: 체크할 각도 범위 (도)
          진행 방향 기준 좌우로 이 각도만큼만 체크

        [수정 가능]
        이 두 값을 조절해서 충돌 감지 민감도 변경

        [예시 1: 더 안전하게]
        self.safe_distance = 0.5  # 50cm로 증가
        self.check_angle = 45     # +-45도로 확대

        [예시 2: 덜 민감하게]
        self.safe_distance = 0.2  # 20cm로 감소
        self.check_angle = 20     # +-20도로 축소

        [권장값]
        - safe_distance: 0.3m (30cm)
          너무 크면: 항상 정지 (진행 불가)
          너무 작면: 충돌 위험 증가

        - check_angle: 30도
          너무 크면: 옆 장애물에도 반응 (과민)
          너무 작면: 충돌 회피 실패 가능
        """

        # 안전 거리 (미터)
        # 이 거리보다 가까운 장애물이 있으면 충돌 위험
        self.safe_distance = 0.3  # 30cm (수정 가능!)

        # 체크할 각도 범위 (도)
        # 진행 방향 기준 좌우로 이 각도만큼 체크
        # 예: 30이면 전방 -30도 ~ +30도 (총 60도)
        self.check_angle = 30  # +-30도 (수정 가능!)

        print(f"[LiDAR 모듈] 초기화 완료")
        print(f"  안전 거리: {self.safe_distance}m")
        print(f"  체크 각도: +-{self.check_angle}도")

    def check_collision(self, ranges: List[float], moving_forward: bool = True) -> bool:
        """
        충돌 위험 체크

        [입력]
        - ranges: LiDAR 거리 데이터 (리스트, 길이 360)
          ranges[0]: 정면 거리 (미터)
          ranges[90]: 왼쪽 거리
          ranges[180]: 뒤쪽 거리
          ranges[270]: 오른쪽 거리
          특수값: inf (무한대, 장애물 없음), nan (잘못된 값)

        - moving_forward: 이동 방향
          True: 전진 중 (전방 체크)
          False: 후진 중 (후방 체크)

        [출력]
        - True: 충돌 위험 있음 (로봇 정지 필요!)
        - False: 안전함 (계속 진행 가능)

        [작동 원리]
        1. 체크할 각도 범위 계산
           전진: 0도 기준 +-check_angle
           후진: 180도 기준 +-check_angle

        2. 해당 범위의 거리 값들 추출
           inf, nan 값은 제외 (잘못된 데이터)

        3. 최소 거리 찾기

        4. safe_distance와 비교
           최소 거리 < safe_distance -> 위험 (True)
           최소 거리 >= safe_distance -> 안전 (False)

        [수정 예시 1: 옆면도 체크]
        # 전방뿐만 아니라 좌우측도 체크
        if moving_forward:
            front_indices = list(range(360-30, 360)) + list(range(0, 30))
            left_indices = list(range(60, 120))
            right_indices = list(range(240, 300))
            check_indices = front_indices + left_indices + right_indices

        [수정 예시 2: 거리별 다른 판단]
        # 30cm 이내: 즉시 정지
        # 30~50cm: 경고만
        if min_distance < 0.3:
            return True  # 위험
        elif min_distance < 0.5:
            print("[LiDAR 경고] 장애물 접근 중")
            return False  # 아직 안전
        """

        try:
            # ========================================
            # 1단계: 데이터 유효성 체크
            # ========================================
            # ranges가 비어있거나 None이면 체크 불가
            if not ranges or len(ranges) == 0:
                return False

            # ========================================
            # 2단계: 체크할 인덱스 범위 계산
            # ========================================

            if moving_forward:
                # 전진 중: 전방 체크
                # 전방은 인덱스 0 기준
                # 예: check_angle=30이면
                # 330~359도 (오른쪽 앞)
                # 0~29도 (왼쪽 앞)

                start_idx = 360 - self.check_angle  # 330
                # range(330, 360): [330, 331, ..., 359]
                # range(0, 30): [0, 1, ..., 29]
                check_indices = list(range(start_idx, 360)) + list(range(0, self.check_angle))

            else:
                # 후진 중: 후방 체크
                # 후방은 인덱스 180 기준
                # 예: check_angle=30이면
                # 150~209도

                back_center = 180
                # range(150, 210): [150, 151, ..., 209]
                check_indices = list(range(back_center - self.check_angle,
                                         back_center + self.check_angle))

            # ========================================
            # 3단계: 해당 범위의 거리 값 추출
            # ========================================
            distances = []

            for i in check_indices:
                # 인덱스가 범위 내인지 확인
                if i < len(ranges):
                    dist = ranges[i]

                    # 유효한 거리 값인지 확인
                    # np.isinf(dist): 무한대인지 (장애물 없음)
                    # np.isnan(dist): 잘못된 값인지
                    # dist > 0: 음수 값 제외
                    if not np.isinf(dist) and not np.isnan(dist) and dist > 0:
                        distances.append(dist)

            # ========================================
            # 4단계: 최소 거리 확인
            # ========================================
            if distances:
                # 가장 가까운 장애물까지 거리
                min_distance = min(distances)

                # 안전 거리와 비교
                if min_distance < self.safe_distance:
                    # 충돌 위험!
                    print(f"[LiDAR 경고] 충돌 위험! 최소거리: {min_distance:.2f}m")
                    return True

            # 안전함
            return False

        except Exception as e:
            # 에러 발생 시 (데이터 파싱 오류 등)
            print(f"[LiDAR 에러] {e}")
            # 에러 시에는 안전하다고 가정 (계속 진행)
            return False

    def get_safe_direction(self, ranges: List[float]) -> Tuple[float, float]:
        """
        안전한 방향 찾기

        [입력]
        - ranges: LiDAR 거리 데이터 (리스트, 길이 360)

        [출력]
        - (linear, angular): 안전한 속도 값
          linear: 직진 속도 (m/s)
          angular: 회전 속도 (rad/s)

        [용도]
        - 충돌 위험 시 자동으로 회피 방향 제안
        - 서버에서 이 값을 사용해 로봇 제어 가능

        [작동 원리]
        1. 좌측(0~90도) 평균 거리 계산
        2. 우측(270~360도) 평균 거리 계산
        3. 더 넓은 쪽으로 회전 제안

        [현재 구현]
        - 좌측이 넓으면: 천천히 좌회전
        - 우측이 넓으면: 천천히 우회전
        - 속도: 직진 0.05 m/s, 회전 0.3 rad/s

        [수정 예시 1: 제자리 회전]
        # 직진하지 않고 제자리에서 회전
        if left_dist > right_dist:
            return (0.0, 0.5)  # linear=0
        else:
            return (0.0, -0.5)

        [수정 예시 2: 거리 비율로 속도 조절]
        # 공간이 넓을수록 빠르게 회전
        ratio = left_dist / (left_dist + right_dist)
        angular = (ratio - 0.5) * 2.0  # -1.0 ~ 1.0
        return (0.05, angular)

        [수정 예시 3: 후진 제안]
        # 양쪽 다 막혔으면 후진
        if left_dist < 0.5 and right_dist < 0.5:
            return (-0.15, 0.0)  # 후진
        """

        try:
            # ========================================
            # 1단계: 데이터 확인
            # ========================================
            if not ranges:
                # 데이터 없으면 정지
                return (0.0, 0.0)

            # ========================================
            # 2단계: 좌우측 평균 거리 계산
            # ========================================

            # 좌측 (0~90도) 거리 추출
            # r for r in ranges[0:90]: 0~89 인덱스의 거리들
            # if not np.isinf(r) and r > 0: 유효한 값만
            left_ranges = [r for r in ranges[0:90]
                          if not np.isinf(r) and r > 0]

            # 좌측 평균 거리
            # np.mean(): 평균 계산
            if left_ranges:
                left_dist = np.mean(left_ranges)
            else:
                left_dist = 0.0

            # 우측 (270~360도) 거리 추출
            right_ranges = [r for r in ranges[270:360]
                           if not np.isinf(r) and r > 0]

            # 우측 평균 거리
            if right_ranges:
                right_dist = np.mean(right_ranges)
            else:
                right_dist = 0.0

            # ========================================
            # 3단계: 더 넓은 쪽으로 회전
            # ========================================

            if left_dist > right_dist:
                # 왼쪽이 더 넓음 -> 좌회전
                # linear: 0.05 m/s (천천히 전진)
                # angular: 0.3 rad/s (좌회전, 양수)
                return (0.05, 0.3)
            else:
                # 오른쪽이 더 넓음 -> 우회전
                # linear: 0.05 m/s (천천히 전진)
                # angular: -0.3 rad/s (우회전, 음수)
                return (0.05, -0.3)

        except Exception as e:
            print(f"[LiDAR 회피 에러] {e}")
            # 에러 시 정지
            return (0.0, 0.0)

# ======================================================================
# 테스트 코드 (이 파일을 직접 실행했을 때만 동작)
# ======================================================================
if __name__ == "__main__":
    """
    단독 테스트 모드

    [실행 방법]
    $ python3 modules/lidar_module.py

    [테스트 내용]
    1. 전방 장애물 있을 때
    2. 전방 장애물 없을 때
    3. 안전한 방향 찾기

    [실제 LiDAR 연동]
    실제 터틀봇에서는 서버가 자동으로 LiDAR 데이터를 전달
    여기서는 가짜 데이터로 테스트
    """

    print("=" * 50)
    print("LiDAR 모듈 단독 테스트")
    print("=" * 50)

    # ========================================
    # LiDAR 모듈 생성
    # ========================================
    lidar = LidarModule()

    # ========================================
    # 테스트 1: 전방 장애물 있음
    # ========================================
    print("\n[테스트 1] 전방 장애물 있음")
    print("  시나리오: 정면(0도)에 20cm 장애물")

    # 가짜 LiDAR 데이터 생성
    # 모든 방향에 1.0m 거리 (안전)
    test_ranges = [1.0] * 360

    # 정면(인덱스 0)에만 0.2m (20cm)
    test_ranges[0] = 0.2

    # 충돌 체크 (전진 중)
    result = lidar.check_collision(test_ranges, moving_forward=True)

    # 결과 출력
    if result:
        print("  결과: 위험 감지 (정상 작동)")
    else:
        print("  결과: 안전 (오류!)")

    # ========================================
    # 테스트 2: 전방 장애물 없음
    # ========================================
    print("\n[테스트 2] 전방 장애물 없음")
    print("  시나리오: 모든 방향 1.0m (안전)")

    # 모든 방향에 1.0m
    test_ranges = [1.0] * 360

    # 충돌 체크
    result = lidar.check_collision(test_ranges, moving_forward=True)

    # 결과 출력
    if result:
        print("  결과: 위험 감지 (오류!)")
    else:
        print("  결과: 안전 (정상 작동)")

    # ========================================
    # 테스트 3: 안전한 방향 찾기
    # ========================================
    print("\n[테스트 3] 안전한 방향 찾기")
    print("  시나리오: 왼쪽은 넓고(2.0m) 오른쪽은 좁음(0.5m)")

    # 기본값 0.5m
    test_ranges = [0.5] * 360

    # 왼쪽(0~90도)은 2.0m로 설정
    test_ranges[0:90] = [2.0] * 90

    # 안전한 방향 찾기
    linear, angular = lidar.get_safe_direction(test_ranges)

    # 결과 출력
    print(f"  제안 속도: linear={linear}, angular={angular}")
    if angular > 0:
        print("  방향: 좌회전 (정상 작동, 왼쪽이 넓음)")
    elif angular < 0:
        print("  방향: 우회전 (오류!)")
    else:
        print("  방향: 직진")

    # ========================================
    # 테스트 4: 후방 장애물 체크
    # ========================================
    print("\n[테스트 4] 후방 장애물 체크")
    print("  시나리오: 후진 중 뒤(180도)에 25cm 장애물")

    # 모든 방향 1.0m
    test_ranges = [1.0] * 360

    # 뒤쪽(180도)에 0.25m
    test_ranges[180] = 0.25

    # 충돌 체크 (후진 중)
    result = lidar.check_collision(test_ranges, moving_forward=False)

    # 결과 출력
    if result:
        print("  결과: 위험 감지 (정상 작동)")
    else:
        print("  결과: 안전 (오류!)")

    # ========================================
    # 테스트 5: 잘못된 데이터 처리
    # ========================================
    print("\n[테스트 5] 잘못된 데이터 처리")
    print("  시나리오: inf, nan 값 포함")

    # 잘못된 값들 포함
    test_ranges = [1.0] * 360
    test_ranges[0] = float('inf')    # 무한대
    test_ranges[1] = float('nan')    # 잘못된 값
    test_ranges[2] = -0.5            # 음수 (비정상)
    test_ranges[10] = 0.15           # 15cm (위험)

    # 충돌 체크
    result = lidar.check_collision(test_ranges, moving_forward=True)

    # 결과 출력
    if result:
        print("  결과: 위험 감지 (정상 작동, 15cm 감지)")
    else:
        print("  결과: 안전 (오류!)")

    # ========================================
    # 테스트 완료
    # ========================================
    print("\n" + "=" * 50)
    print("모든 테스트 완료")
    print("=" * 50)
    print("\n[다음 단계]")
    print("1. 서버 통합 테스트:")
    print("   python3 -m server.server")
    print("2. 실제 터틀봇으로 테스트")
    print("3. safe_distance, check_angle 조절해서 최적값 찾기")
