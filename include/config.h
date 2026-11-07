#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ============================================================
   CẤU HÌNH WIFI & TIMESCALEDB BACKEND API
   ============================================================ */
// Thông tin WiFi (Thay đổi theo mạng thực tế của bạn)
#define WIFI_SSID             "YOUR_WIFI_SSID"
#define WIFI_PASSWORD         "YOUR_WIFI_PASS"

// URL Docker TimescaleDB Backend Server (Node.js API kết nối TimescaleDB)
// Điền IP máy tính chạy Docker backend (VD: "http://192.168.1.100:3000")
#define BACKEND_URL           "http://192.168.1.16:3000"

// API Key xác thực cho thiết bị IoT của bãi xe
#define API_KEY               "sec_iot_mydinh_key"

/* ============================================================
   ĐỊNH DANH BÃI XE (MULTI-LOT ARCHITECTURE)
   Để cấu hình kit ESP32 cho bãi xe khác, bạn chỉ cần thay đổi
   LOT_ID sang ID tương ứng (VD: lot_caugiay, lot_hoankiem...)
   ============================================================ */
#define LOT_ID                "lot_mydinh"          // ID bãi xe trong TimescaleDB
#define LOT_NAME              "B7 My Dinh"          // Tên bãi đỗ xe hiển thị trên OLED
#define TOTAL_SLOTS           4

/* ============================================================
   CẤU HÌNH CHÂN PHẦN CỨNG (PIN CONFIGURATION)
   ============================================================ */
// Cảm biến hồng ngoại IR 4 ô đỗ (Active LOW - LOW khi có xe)
const uint8_t PIN_SLOTS[TOTAL_SLOTS] = {32, 33, 25, 26};

// Cảm biến IR cổng vào / ra (Active LOW)
#define PIN_GATE_IN           27
#define PIN_GATE_OUT          14

// Servo Barrier
// LƯU Ý: SERVO_IN đổi sang GPIO 4 (tránh GPIO 12 là Strapping Pin gây bootloop)
#define PIN_SERVO_IN          4
#define PIN_SERVO_OUT         13

#define SERVO_ANGLE_CLOSED    0     // Barrier đóng (độ)
#define SERVO_ANGLE_OPEN      90    // Barrier mở (độ)
#define BARRIER_TIMEOUT_MS    4000  // Thời gian tối thiểu barrier mở (ms)

// Cảm biến siêu âm HC-SR04
#define PIN_TRIG              5
#define PIN_ECHO              18
#define DIST_ALERT_FAR        30.0f // cm (ngưỡng cảnh báo từ xa)
#define DIST_ALERT_NEAR       10.0f // cm (ngưỡng cảnh báo quá gần)

// Buzzer & OLED
#define PIN_BUZZER            19
#define PIN_SDA               21
#define PIN_SCL               22
#define OLED_ADDR             0x3C
#define SCREEN_WIDTH          128
#define SCREEN_HEIGHT         64

/* ============================================================
   CHU KỲ ĐỒNG BỘ (TIMING INTERVALS)
   ============================================================ */
#define DISPLAY_INTERVAL_MS       200   // Cập nhật OLED và còi mỗi 200ms
#define DISTANCE_READ_INTERVAL_MS 250   // Đọc khoảng cách HC-SR04
#define BACKEND_SYNC_INTERVAL_MS  1200  // Đẩy trạng thái slot lên TimescaleDB Backend (ms)
#define COMMAND_POLL_INTERVAL_MS  500   // Kiểm tra lệnh mở cổng từ Backend (ms)

#endif // CONFIG_H
