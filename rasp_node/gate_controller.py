# node_b_gate_controller.py
import json
import re
import time
import queue
import threading
from pathlib import Path
from collections import Counter

import cv2
import easyocr
import paho.mqtt.client as mqtt
from ultralytics import YOLO

# ================= CẤU HÌNH =================
MQTT_BROKER = "192.168.5.103"
MQTT_PORT = 1883

# Bãi xe mặc định mà Node B này phụ trách (nếu để None sẽ tự động xử lý tất cả bãi nhận được)
TARGET_LOT_ID = "lot_mydinh"  

CAMERA_INDEX = 2
MODEL_PATH = Path(__file__).resolve().parent / "best.pt"
YOLO_CONF = 0.5
SPLIT_RATIO = 0.5  # Nửa trái: Làn IN, Nửa phải: Làn OUT

# ================= QUẢN LÝ LUỒNG =================
stop_event = threading.Event()
event_queue = queue.Queue()
frame_lock = threading.Lock()

latest_frame = None
latest_frame_at = 0.0

def clean_plate(text):
    return re.sub(r"[^A-Z0-9]", "", text.upper())

def detect_plate(model, frame):
    results = model(frame, conf=YOLO_CONF, verbose=False)
    best_crop = None
    best_area = 0
    h, w = frame.shape[:2]

    for box in results[0].boxes.xyxy:
        x1, y1, x2, y2 = map(int, box.tolist())
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        bw, bh = x2 - x1, y2 - y1
        if bw < 80 or bh < 20:
            continue

        if bw * bh > best_area:
            best_area = bw * bh
            best_crop = frame[y1:y2, x1:x2].copy()

    return best_crop

def ocr_plate(reader, plate_crop):
    enlarged = cv2.resize(plate_crop, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(enlarged, cv2.COLOR_BGR2GRAY)
    texts = reader.readtext(
        gray, detail=0, paragraph=False, allowlist="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    )
    raw = "".join(texts)
    clean = clean_plate(raw)
    return clean if 5 <= len(clean) <= 10 else ""

def camera_worker(cap):
    global latest_frame, latest_frame_at
    failures = 0
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            failures += 1
            if failures >= 10:
                print("[LỖI] Mất webcam!")
                stop_event.set()
                return
            time.sleep(0.1)
            continue
        failures = 0
        with frame_lock:
            latest_frame = frame.copy()
            latest_frame_at = time.monotonic()

def get_lane_crop(lane):
    with frame_lock:
        if latest_frame is None or (time.monotonic() - latest_frame_at > 2.0):
            return None
        frame = latest_frame.copy()

    split_x = int(frame.shape[1] * SPLIT_RATIO)
    return frame[:, :split_x].copy() if lane == "in" else frame[:, split_x:].copy()

# ================= MQTT HANDLERS =================
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"[MQTT] Đã kết nối Broker {MQTT_BROKER}")
        # Subscribe wildcard cho tất cả các bãi xe hoặc theo TARGET_LOT_ID
        if TARGET_LOT_ID:
            topic_filter = f"parking/v1/lots/{TARGET_LOT_ID}/ir/+/event"
        else:
            topic_filter = "parking/v1/lots/+/ir/+/event"
            
        client.subscribe(topic_filter, qos=1)
        print(f"[MQTT] Đã sub topic: {topic_filter}")
    else:
        print(f"[MQTT] Lỗi kết nối: {rc}")

def on_message(client, userdata, msg):
    try:
        # Bóc tách thông tin từ cấu trúc Topic: parking/v1/lots/{lot_id}/ir/{lane}/event
        topic_parts = msg.topic.split("/")
        if len(topic_parts) >= 6:
            lot_id = topic_parts[3]
            lane = topic_parts[5]
            
            payload = json.loads(msg.payload.decode("utf-8"))
            event = payload.get("event")
            detected = payload.get("detected", False)

            if detected and event == "VEHICLE_ARRIVED":
                print(f"\n[SỰ KIỆN] Xe tới cổng - Bãi: '{lot_id}' | Làn: '{lane.upper()}'")
                event_queue.put((lot_id, lane))
    except Exception as e:
        print(f"[LỖI PARSE MQTT]: {e}")

# ================= CHƯƠNG TRÌNH CHÍNH =================
def main():
    print("Đang tải YOLOv8 và EasyOCR...")
    model = YOLO(str(MODEL_PATH))
    reader = easyocr.Reader(["en"], gpu=False)

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("Không mở được Webcam")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    threading.Thread(target=camera_worker, args=(cap,), daemon=True).start()

    client = mqtt.Client(client_id="Node_B_Central_Controller")
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        client.loop_start()
    except Exception as e:
        print(f"[LỖI MQTT] Không kết nối được Broker: {e}")
        return

    print("\n=== NODE B ĐÃ SẴN SÀNG QUẢN LÝ DỮ LIỆU MULTI-LOT ===")

    try:
        while not stop_event.is_set():
            try:
                lot_id, lane = event_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            print(f"\n---> XỬ LÝ: Bãi '{lot_id}' | Làn '{lane.upper()}'")
            ocr_results = []
            start_scan = time.monotonic()

            while time.monotonic() - start_scan < 3.0 and len(ocr_results) < 5:
                lane_frame = get_lane_crop(lane)
                if lane_frame is None:
                    time.sleep(0.1)
                    continue

                plate_crop = detect_plate(model, lane_frame)
                if plate_crop is not None:
                    plate_text = ocr_plate(reader, plate_crop)
                    if plate_text:
                        ocr_results.append(plate_text)
                        print(f"  [OCR Thử {len(ocr_results)}]: {plate_text}")
                time.sleep(0.15)

            if ocr_results:
                final_plate = Counter(ocr_results).most_common(1)[0][0]
                print(f"[KẾT QUẢ] Biển số chốt: {final_plate}")

                # 1. Báo cáo dữ liệu ra/vào về Server theo bãi tương ứng
                event_type = "entry" if lane == "in" else "exit"
                server_topic = f"parking/v1/lots/{lot_id}/events/{event_type}"
                confirm_payload = json.dumps({
                    "lotId": lot_id,
                    "plate": final_plate,
                    "lane": lane.upper(),
                    "timestamp": int(time.time()),
                    "status": "CONFIRMED"
                })
                client.publish(server_topic, confirm_payload, qos=1)
                print(f"  -> Đã gửi Server [{server_topic}]: {confirm_payload}")

                # 2. Gửi lệnh mở cổng chính xác tới Node A của bãi đó
                cmd_topic = f"parking/v1/lots/{lot_id}/gates/{lane}/command"
                client.publish(cmd_topic, "open", qos=1)
                print(f"  -> Đã gửi Lệnh Mở Cổng [{cmd_topic}]: open")
            else:
                print(f"[THẤT BẠI] Không đọc được biển số tại Bãi '{lot_id}' - Làn '{lane.upper()}'")

    finally:
        stop_event.set()
        client.loop_stop()
        client.disconnect()
        cap.release()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nĐã dừng chương trình.")