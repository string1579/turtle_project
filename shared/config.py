"""
전체 프로젝트 공통 설정
"""

import os

# 서버 설정
SERVER_IP = "192.168.30.5"
SERVER_PORT = 8080

# 터틀봇 정보
ROBOTS = {
    1: {
        "ip": "192.168.30.10",
        "domain_id": 17,
        "name": "터틀봇1"
    },
    2: {
        "ip": "192.168.30.2",
        "domain_id": 5,
        "name": "터틀봇2"
    },
    3: {
        "ip": "192.168.30.9",
        "domain_id": 3,
        "name": "터틀봇3"
    }
}

# 환경변수로 사용할 로봇 제한 (예: "1,3" 또는 "1")
ACTIVE_ROBOTS_ENV = os.getenv("ACTIVE_ROBOTS", "1, 2, 3")

# 활성화할 로봇 ID 리스트
ACTIVE_ROBOTS = [int(rid.strip()) for rid in ACTIVE_ROBOTS_ENV.split(",") if rid.strip().isdigit()]

print(f"[Config] 활성 로봇: {ACTIVE_ROBOTS}")

# 속도 제한 (안전장치)
MAX_LINEAR_SPEED = 0.05
MAX_ANGULAR_SPEED = 2.85
