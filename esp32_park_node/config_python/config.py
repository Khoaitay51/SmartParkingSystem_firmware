import serial
import time
import os

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE   = 115200
FILE_NAME   = 'config.json'

try:
    print(f"[*] Đang mở cổng Serial {SERIAL_PORT}...")
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1
    )
    ser.dtr = False
    ser.rts = False
    
    file_path = FILE_NAME
    if not os.path.exists(file_path):
        file_path = os.path.join('..', FILE_NAME)

    with open(file_path, 'r', encoding='utf-8') as f:
        config_data = f.read()

    print("[*] Đang chờ tín hiệu READY từ ESP32 (Bấm nút RESET trên ESP32)...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    start_time = time.time()
    ready = False
    while time.time() - start_time < 10:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "READY_FOR_CONFIG" in line:
                ready = True
                break
        time.sleep(0.02)

    if ready:
        time.sleep(0.1) # Chờ 100ms để ESP32 chuyển hẳn sang trạng thái chờ read UART
        print(f"[*] Đang truyền file {os.path.basename(file_path)} ({len(config_data)} bytes)...")
        
        # Gửi chuỗi JSON + ký tự xuống dòng
        ser.write((config_data + "\n").encode('utf-8'))
        ser.flush()

        # Đọc lọc kết quả trả về trong 3 giây
        success = False
        start_wait = time.time()
        while time.time() - start_wait < 3:
            if ser.in_waiting > 0:
                resp = ser.readline().decode('utf-8', errors='ignore').strip()
                if "CONFIG_OK" in resp:
                    success = True
                    break
            time.sleep(0.05)

        if success:
            print("\n>>> CẬP NHẬT CONFIG THÀNH CÔNG! <<<")
        else:
            print("\n[!] Không nhận được phản hồi CONFIG_OK từ ESP32.")
    else:
        print("[!] Không bắt được tín hiệu READY_FOR_CONFIG từ ESP32.")

    ser.close()

except Exception as e:
    print(f"\n[LỖI] {e}")