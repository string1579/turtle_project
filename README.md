# 터틀봇 멀티 제어 프로젝트

ROS2 + FastAPI + PyQt5 기반 멀티 터틀봇 제어 시스템

---

## 프로젝트 구조

```
~/작업공간/turtle_project/
├── shared/              # 공통 설정 (모두 공유)
│   ├── config.py       # IP, 로봇 정보
│   └── message_types.py
│
├── modules/             # 각 담당자 작업 공간
│   ├── qr_module.py    # QR 인식 + 동작 제어
│   ├── lidar_module.py # LiDAR 충돌 회피
│   └── nav_module.py   # (추후) Navigation
│
├── server/              # 서버
│   ├── server.py       # FastAPI 서버
│   └── ros_bridge.py   # ROS2 브릿지
│
├── gui/                 # GUI
│   └── gui.py          # PyQt5 GUI
│
├── requirements.txt     # 필수 패키지 목록
└── README.md           # 이 파일
```

---

## 설치 방법 (전체 필수!)

### 1단계: 프로젝트 다운로드

```bash
# 본인의 작업 폴더로 이동
cd ~
mkdir -p 작업공간
cd 작업공간

# 프로젝트 복사 (zip파일 압축 해제 후 turtle_project 이동, 혹은 git 사용)
```

### 2단계: Python 패키지 설치

```bash
cd ~/작업공간/turtle_project
pip3 install -r requirements.txt
```

**중요:**
- 이 명령어를 꼭 실행하세요!
- 안 하면 ModuleNotFoundError 발생
- WiFi 연결 필요 (패키지 다운로드)

**설치 확인:**
```bash
pip3 list | grep fastapi
pip3 list | grep PyQt5
pip3 list | grep opencv
```

---

## 실행 방법

### 실행 전 체크리스트

- [ ] requirements.txt 설치 완료
- [ ] 같은 WiFi 연결됨 (192.168.30.x)
- [ ] 터틀봇 전원 켜짐

---

### A. 서버 실행 (서버 노트북에서만)

```bash
cd ~/작업공간/turtle_project
python3 -m server.server
```

또는

```bash
cd ~/작업공간/turtle_project
python3 server/server.py
```

**실행 확인:**
```
출력 예시:
====================================
터틀봇 제어 서버 시작
====================================
[1단계] 터틀봇 연결 중
  연결 중: 터틀봇1 (ID:1, Domain:17)
  ...
====================================
서버 준비 완료
HTTP: http://192.168.30.5:8080
WebSocket: ws://192.168.30.5:8080/ws
====================================
```

**브라우저 테스트:**
- 브라우저에서 http://192.168.30.5:8080 접속
- JSON 응답 확인

---

### B. GUI 실행 (모든 PC에서 가능)

```bash
cd ~/작업공간/turtle_project
python3 -m gui.gui
```

또는

```bash
cd ~/작업공간/turtle_project
python3 gui/gui.py
```

**연결 확인:**
- 창 상단에 "서버 연결됨" 표시
- 로그에 "서버 연결 성공!" 메시지

**연결 안될 때:**
1. 서버가 실행 중인지 확인
2. shared/config.py에서 SERVER_IP 확인
3. 같은 WiFi인지 확인 (ip addr)

---

### C. 터틀봇 실행 (각 터틀봇에서)

```bash
# 터틀봇 1번 (IP: 192.168.30.10)
ssh ubuntu@192.168.30.10
# 비밀번호 입력 후
export ROS_DOMAIN_ID=17
ros2 launch turtlebot3_bringup robot.launch.py
```

```bash
# 터틀봇 2번 (IP: 192.168.30.2)
ssh ubuntu@192.168.30.2
export ROS_DOMAIN_ID=3
ros2 launch turtlebot3_bringup robot.launch.py
```

```bash
# 터틀봇 3번 (IP: 192.168.30.9)
ssh ubuntu@192.168.30.9
export ROS_DOMAIN_ID=5
ros2 launch turtlebot3_bringup robot.launch.py
```

**실행 순서:**
1. 서버 실행 (서버 노트북)
2. 터틀봇 실행 (각 로봇)
3. GUI 실행 (제어용 PC)

---

## 작업 가이드

### 공통 규칙

1. **절대 수정 금지:**
   - shared/config.py (IP 주소만 수정 가능)
   - shared/message_types.py
   - 다른 담당자 파일

2. **본인 파일만 수정:**
   - QR 담당: modules/qr_module.py
   - LiDAR 담당: modules/lidar_module.py
   - GUI 담당: gui/gui.py
   - 서버 담당: server/ 폴더

3. **테스트 후 공유:**
   - 단독 테스트
   - 통합 테스트
   - 공유

---

### QR 담당 작업 가이드

**작업 파일:** modules/qr_module.py

**단독 테스트:**
```bash
cd ~/작업공간/turtle_project
python3 -m modules.qr_module
```
- 웹캠으로 QR 인식 테스트
- q 키로 종료

**수정 포인트:**
- on_qr_detected(): QR 인식 후 동작 정의

**예시: 새 QR 추가**
```python
def on_qr_detected(self, qr_text, robot_id, image):
    # 기존 코드...

    elif qr_text == "my_custom":
        self.robot_controller.move(robot_id, 0.2, 0.0)
        self.robot_controller.sleep(2.0)
        self.robot_controller.stop(robot_id)
```

---

### LiDAR 담당 작업 가이드

**작업 파일:** modules/lidar_module.py

**단독 테스트:**
```bash
cd ~/작업공간/turtle_project
python3 -m modules.lidar_module
```
- 시뮬레이션 데이터로 테스트

**수정 포인트:**
- check_collision(): 충돌 판단 로직
- safe_distance: 안전 거리 (미터)
- check_angle: 체크할 각도 범위 (도)

**예시: 안전 거리 변경**
```python
def __init__(self):
    self.safe_distance = 0.5  # 50cm로 변경
    self.check_angle = 45     # ±45도로 변경
```

---

### GUI 담당 작업 가이드

**작업 파일:** gui/gui.py

**실행:**
```bash
cd ~/작업공간/turtle_project
python3 -m gui.gui
```

**수정 포인트:**
- create_control_group(): 제어 버튼 추가/변경
- create_mode_group(): 새 기능 버튼 추가
- on_message(): 서버 메시지 처리 로직

**예시: 새 버튼 추가**
```python
def create_mode_group(self):
    # 기존 코드...

    # 새 버튼 추가
    new_layout = QHBoxLayout()
    btn_new = QPushButton("새 기능")
    btn_new.clicked.connect(self.new_function)
    new_layout.addWidget(btn_new)
    layout.addLayout(new_layout)

def new_function(self):
    message = {
        "command": "my_command",
        "robot_id": self.selected_robot
    }
    self.ws_thread.send(message)
```

---

## 설정 변경

### IP 주소 변경

shared/config.py 파일 수정:

```python
# 서버 IP (서버 노트북)
SERVER_IP = "192.168.30.5"  # 여기만 수정

# 로봇 IP는 변경 금지! (라즈베리파이 고정 IP)
```

**변경 후:**
1. 서버 재시작
2. GUI 재시작
3. 모든 담당자에게 공유

---

## 문제 해결

### 1. ModuleNotFoundError

```
ModuleNotFoundError: No module named 'fastapi'
```

**해결:**
```bash
cd ~/작업공간/turtle_project
pip3 install -r requirements.txt
```

---

### 2. 서버 연결 안됨

GUI에서 "서버 연결 안됨"

**체크리스트:**
1. 서버 실행 중?
2. IP 주소 맞음? (shared/config.py 확인)
3. 같은 WiFi? (ip addr로 확인)
4. 방화벽? (sudo ufw disable, 테스트용)

**IP 확인 방법:**
```bash
# 서버 노트북에서
ip addr show

# 192.168.30.x 형태 찾기
```

---

### 3. 로봇이 움직이지 않음

**체크리스트:**
1. 터틀봇에서 robot.launch.py 실행됨?
2. ROS_DOMAIN_ID 맞음?
   ```bash
   echo $ROS_DOMAIN_ID
   ```
3. 서버 로그에 에러?

**도메인 ID:**
- 터틀봇1: 17
- 터틀봇2: 3
- 터틀봇3: 5

---

### 4. import 에러

```
ImportError: cannot import name 'ROBOTS'
```

**해결:**
```bash
# 방법 1: 프로젝트 루트에서 실행
cd ~/작업공간/turtle_project
python3 -m server.server

# 방법 2: PYTHONPATH 설정
export PYTHONPATH=$PYTHONPATH:~/작업공간/turtle_project
```

---

## 작업 통합 방법

### 방법 1: 파일 교체 (현재 방식)

```bash
# 수정한 파일 받기
cp ~/Downloads/qr_module.py ~/작업공간/turtle_project/modules/

# 서버 재시작
python3 -m server.server
```

### 방법 2: Git 사용

```bash
# 변경사항 확인
git status

# 커밋
git add modules/qr_module.py
git commit -m "QR 인식 개선"

# 푸시
git push

# 다른 팀원은 pull
git pull
```

---

## 참고 자료

- ROS2 Humble: https://docs.ros.org/en/humble/
- FastAPI: https://fastapi.tiangolo.com/
- PyQt5: https://www.riverbankcomputing.com/static/Docs/PyQt5/
