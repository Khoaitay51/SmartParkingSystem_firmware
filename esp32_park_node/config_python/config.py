import argparse
import json
import time
from pathlib import Path

import serial


def wait_for(port, signal, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if signal in port.readline():
            return
    raise TimeoutError(f"Không nhận được {signal.decode()} từ ESP32")


def main():
    parser = argparse.ArgumentParser(description="Nạp cấu hình Park Node qua UART")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Ví dụ COM5 hoặc /dev/ttyUSB0")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise ValueError("Cấu hình phải là một đối tượng JSON")
        payload = json.dumps(config, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        if len(payload) > 1022:
            raise ValueError("Cấu hình vượt quá bộ đệm UART của firmware")

        with serial.Serial(args.port, 115200, timeout=0.2) as port:
            port.dtr = False
            port.rts = False
            port.reset_input_buffer()
            print("Bấm RESET trên ESP32 để bắt đầu nạp cấu hình.")
            wait_for(port, b"READY_FOR_CONFIG", 15)
            port.write(payload + b"\n")
            port.flush()
            wait_for(port, b"CONFIG_OK", 3)
        print("Đã lưu cấu hình Park Node.")
    except (OSError, ValueError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
