"""
서버-클라이언트 통신 메시지 형식
"""

from enum import Enum
from typing import Dict, Any, Optional
from pydantic import BaseModel

class CommandType(str, Enum):
    """로봇 제어 명령어"""
    MOVE = "move"
    STOP = "stop"
    EMERGENCY_STOP = "emergency_stop"
    QR_DETECT = "qr_detect"
    QR_STOP = "qr_stop"
    LIDAR_CHECK = "lidar_check"
    LIDAR_STOP = "lidar_stop"

class RobotCommand(BaseModel):
    """로봇 명령 구조"""
    robot_id: int
    command: CommandType
    data: Dict[str, Any] = {}

class RobotStatus(BaseModel):
    """로봇 상태 구조"""
    robot_id: int
    position: Optional[Dict[str, float]] = None
    status: str = "idle"
    extra: Dict[str, Any] = {}
