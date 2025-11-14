"""
=================================================================
서버-클라이언트 통신 메시지 형식 정의
=================================================================
[목적]
- 서버와 GUI 간에 주고받는 메시지 형식을 정의
- 오타나 형식 오류를 방지

[중요]
이 파일도 모든 작업자가 공유합니다.
함부로 수정하지 마세요.

[새 명령어 추가 방법]
1. CommandType 클래스에 명령어 추가
   예: NAV_GOAL = "nav_goal"

2. server.py의 handle_command() 함수에서 처리 로직 추가
   예: elif command == 'nav_goal':

3. gui.py에서 버튼 클릭 시 해당 명령어 전송
   예: {"command": "nav_goal", "robot_id": 1, ...}
=================================================================
"""

from enum import Enum
from typing import Dict, Any, Optional
from pydantic import BaseModel

# ==============================
# 명령어 타입 정의
# ==============================
class CommandType(str, Enum):
    """
    로봇 제어 명령어 종류

    [설명]
    GUI에서 서버로 보낼 수 있는 명령어 목록
    여기에 정의된 명령어만 사용 가능

    [각 명령어 의미]
    - MOVE: 로봇 이동 (직진/회전)
    - STOP: 로봇 정지
    - EMERGENCY_STOP: 모든 로봇 긴급 정지
    - QR_DETECT: QR 인식 시작
    - QR_STOP: QR 인식 중지
    - LIDAR_CHECK: LiDAR 충돌 체크 시작
    - LIDAR_STOP: LiDAR 충돌 체크 중지
    """
    MOVE = "move"                       # 수동 이동
    STOP = "stop"                       # 정지
    EMERGENCY_STOP = "emergency_stop"   # 비상 정지

    # QR 관련
    QR_DETECT = "qr_detect"            # QR 인식 시작
    QR_STOP = "qr_stop"                # QR 인식 중지

    # LiDAR 관련
    LIDAR_CHECK = "lidar_check"        # 충돌 체크 시작
    LIDAR_STOP = "lidar_stop"          # 충돌 체크 중지

# ==============================
# 명령 메시지 구조
# ==============================
class RobotCommand(BaseModel):
    """
    로봇에게 보내는 명령 구조

    [형식]
    {
        "robot_id": 1,              # 제어할 로봇 번호
        "command": "move",          # 명령어 (CommandType에서 선택)
        "data": {                   # 추가 데이터 (명령어마다 다름)
            "linear": 0.2,          # move일 때: 직진 속도
            "angular": 0.0          # move일 때: 회전 속도
        }
    }

    [예시 1: 전진]
    {
        "robot_id": 1,
        "command": "move",
        "data": {"linear": 0.2, "angular": 0.0}
    }

    [예시 2: 좌회전]
    {
        "robot_id": 1,
        "command": "move",
        "data": {"linear": 0.0, "angular": 0.5}
    }

    [예시 3: 정지]
    {
        "robot_id": 1,
        "command": "stop",
        "data": {}
    }
    """
    robot_id: int                       # 로봇 번호 (필수)
    command: CommandType                # 명령어 (필수)
    data: Dict[str, Any] = {}          # 추가 데이터 (선택)

# ==============================
# 상태 메시지 구조
# ==============================
class RobotStatus(BaseModel):
    """
    로봇 상태 정보 구조

    [형식]
    {
        "robot_id": 1,              # 로봇 번호
        "position": {               # 현재 위치 (odom 데이터)
            "x": 0.5,
            "y": 0.3
        },
        "status": "moving",         # 상태 텍스트
        "extra": {}                 # 추가 정보
    }

    [용도]
    - 서버가 GUI로 로봇 상태를 알릴 때 사용
    - Navigation 연동 시 위치 정보 표시
    """
    robot_id: int                                   # 로봇 번호
    position: Optional[Dict[str, float]] = None    # 위치 정보 (x, y)
    status: str = "idle"                           # 상태 ("idle", "moving" 등)
    extra: Dict[str, Any] = {}                     # 추가 정보
