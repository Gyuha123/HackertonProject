import cv2
import urllib.request
import mediapipe as mp
from ultralytics import YOLO

# YOLO 모델 로드 (yolov8m.pt 모델 사용)
model = YOLO("yolov8m.pt")  # 사람 감지용 YOLO 모델
esp_ip = "192.168.231.132"  # ESP32 IP 주소

# MediaPipe 얼굴 감지 초기화
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

# 기본 설정
IMAGE_HEIGHT = 720  # 카메라 해상도 세로 길이
FACE_WARNING_RATIO = 1 / 3  # 얼굴이 화면의 1/3 이상 차지하면 경고

# 신호 중복 방지를 위한 변수
last_signal_sent = None

# 경고 메시지 출력 함수
def trigger_warning(message):
    print(f"⚠️ 경고: {message}")

# 신호를 보내는 함수
def send_signal(url):
    try:
        with urllib.request.urlopen(url) as response:
            response_text = response.read().decode('utf-8')
            print("신호 전송 성공:", response_text)
    except urllib.error.URLError as e:
        print(f"Error sending signal: {e}")

# 얼굴 감지 함수
def detect_face1():
    global last_signal_sent  # 신호 상태 추적

    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)  # 가로 해상도
    cap.set(4, IMAGE_HEIGHT)  # 세로 해상도

    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다!")
        return  # 카메라가 열리지 않으면 종료

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("[INFO] Frame capture failed or stream ended.")
                break

            face_count = 0  # 감지된 얼굴 개수
            face_detected = False  # 얼굴 감지 여부
            is_any_face_detected = False  # 아무 얼굴도 감지되지 않으면 L 신호 보내기

            # YOLO로 사람 감지 (객체 감지 한 번만)
            results = model.predict(source=frame, conf=0.5)  # confidence threshold 0.5로 설정

            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    if model.names[class_id] == "person":
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        person_roi = frame[y1:y2, x1:x2]  # 사람 영역만 자르기

                        # MediaPipe 얼굴 감지
                        person_rgb = cv2.cvtColor(person_roi, cv2.COLOR_BGR2RGB)
                        detections = face_detection.process(person_rgb)

                        if detections.detections:
                            is_any_face_detected = True  # 얼굴이 감지되었음
                            for detection in detections.detections:
                                face_count += 1  # 얼굴 개수 증가
                                bboxC = detection.location_data.relative_bounding_box
                                ih, iw, _ = person_roi.shape
                                fx, fy, fw, fh = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                                                 int(bboxC.width * iw), int(bboxC.height * ih)

                                face_x1 = x1 + fx
                                face_y1 = y1 + fy
                                face_x2 = face_x1 + fw
                                face_y2 = face_y1 + fh

                                face_ratio = fh / IMAGE_HEIGHT
                                status = "FAR"

                                # 얼굴이 너무 가까운 경우
                                if face_ratio > FACE_WARNING_RATIO:
                                    status = "CLOSE"
                                    trigger_warning("얼굴이 너무 가까움!")
                                    print(f"[WARNING] Face too close! Position: (x={face_x1}, y={face_y1}), Ratio: {face_ratio:.2f}")

                                # 얼굴 영역을 사각형으로 표시
                                cv2.rectangle(frame, (face_x1, face_y1), (face_x2, face_y2), (0, 255, 0), 2)
                                cv2.putText(frame, f"{status}", (face_x1, face_y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # 3명 이상 얼굴 감지 시 즉시 HB 신호 전송
            if face_count >= 3:
                if last_signal_sent != "HB":
                    url = f"http://{esp_ip}/HB"  # 3명 이상이면 HB 신호
                    send_signal(url)  # 신호 전송
                    last_signal_sent = "HB"  # 마지막 신호 기록
                    print("[INFO] 3 or more faces detected, sending 'HB' signal.")
            else:
                # 3명 미만일 경우 H, L 신호 처리
                if face_count > 0:  # 얼굴이 감지되었을 때
                    if face_ratio > FACE_WARNING_RATIO:
                        # 가까운 경우 H 신호
                        if last_signal_sent != "H":
                            url = f"http://{esp_ip}/H"  # 가까운 경우 H 신호
                            send_signal(url)
                            last_signal_sent = "H"  # 마지막 신호 기록
                            print("[INFO] Close face detected, sending 'H' signal.")
                    else:
                        # 멀리 있는 경우 L 신호
                        if last_signal_sent != "L":
                            url = f"http://{esp_ip}/L"  # 멀리 있는 경우 L 신호
                            send_signal(url)
                            last_signal_sent = "L"  # 마지막 신호 기록
                            print("[INFO] Far face detected, sending 'L' signal.")
                else:
                    # 얼굴이 감지되지 않으면 L 신호 보내기
                    if last_signal_sent != "L":
                        url = f"http://{esp_ip}/L"  # 얼굴이 감지되지 않으면 L 신호
                        send_signal(url)
                        last_signal_sent = "L"  # 마지막 신호 기록
                        print("[INFO] No faces detected, sending 'L' signal.")

            # 화면 출력
            cv2.imshow("YOLO + MediaPipe Face Detection", frame)

            # 'q' 키를 누르면 프로그램 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] Exiting face detection.")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Camera and windows released.")

# 바로 실행되는 프로그램
detect_face1()
