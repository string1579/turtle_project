import cv2
import re
import time
import numpy as np

# ==========================================
# [설정] 디버그 모드 (True: 상세 로그 출력, False: 숨김)
DEBUG = True
# ==========================================

class QRModule:
    """
    QR Processing Module (Server Side)
    Analyzes images and generates event strings.
    """

    def __init__(self, robot_controller=None):
        print("[Module] QRModule Initialized.")
        self.qr_detector = cv2.QRCodeDetector()

        self.last_qr_data = {}
        self.last_qr_time = {}
        self.DEBOUNCE_TIME = 3.0
        self.robot_controller = robot_controller

    def _parse_qr_goal(self, text: str):
        text = text.strip()

        if DEBUG:
            print(f"[DEBUG-QR] Parsing text for Goal: '{text}'")

        # Format: goal_name:x,y,yaw
        m = re.match(r"([a-zA-Z0-9_]+)\s*[:]\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)", text)
        if m:
            if DEBUG:
                print(f"[DEBUG-QR] Regex Match Success: Name={m.group(1)}, X={m.group(2)}, Y={m.group(3)}, Yaw={m.group(4)}")
            return m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4))

        if DEBUG:
            print(f"[DEBUG-QR] Regex Match FAILED for text: '{text}'")
        return None

    def _create_event_string(self, data: str) -> str | None:
        event_str = None

        if DEBUG:
            print(f"[DEBUG-QR] Creating event string from raw data: '{data}'")

        if data == "stop":
            event_str = "STOP"
        elif data == "cancel":
            event_str = "CANCEL"
        elif data == "mark":
            event_str = "MARK"
        elif data == "home":
            # Example home coordinate
            event_str = "NAV_GOAL:home,0.0,0.0,0.0"
        else:
            parsed = self._parse_qr_goal(data)
            if parsed:
                name, x, y, yaw = parsed
                event_str = f"NAV_GOAL:{name},{x},{y},{yaw}"
            else:
                if DEBUG:
                    print(f"[DEBUG-QR] Could not parse custom goal data: '{data}'")

        return event_str

    def process_image(self, image: np.ndarray, current_time: float, robot_id: int) -> str | None:
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            data, _, _ = self.qr_detector.detectAndDecode(gray)

            if data:
                processed_data = data.strip().lower()

                last_data = self.last_qr_data.get(robot_id, "")
                last_time = self.last_qr_time.get(robot_id, 0)

                if processed_data == last_data and (current_time - last_time) < self.DEBOUNCE_TIME:
                    if DEBUG:
                        print(f"[DEBUG-QR] Ignored (Debounce): '{processed_data}' (Time diff: {current_time - last_time:.2f}s)")
                    return None

                if DEBUG:
                    print(f"[DEBUG-QR] New Valid QR Accepted: '{processed_data}'")

                self.last_qr_data[robot_id] = processed_data
                self.last_qr_time[robot_id] = current_time

                event_str = self._create_event_string(processed_data)

                if event_str:
                    print(f"[QR] Robot {robot_id} Detected: '{processed_data}' -> Event: '{event_str}'")
                    return event_str
                elif DEBUG:
                    print(f"[DEBUG-QR] Event creation failed for: '{processed_data}'")

        except Exception as e:
            print(f"[QR] Processing Error (Robot {robot_id}): {e}")

        return None
