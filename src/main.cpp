/* ============================================================
   HỆ THỐNG BÃI ĐỖ XE THÔNG MINH - ESP32 + ARDUINO FRAMEWORK
   SIC IoT - Tích hợp Firebase Realtime Database
   Ánh xạ bãi xe: B7 Mỹ Đình (lot_mydinh)
   ============================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include "config.h"

/* ================= ĐỐI TƯỢNG PHẦN CỨNG ================= */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledAvailable = false;

Servo servoIn;
Servo servoOut;

/* ================= BIẾN TRẠNG THÁI HỆ THỐNG ================= */
uint8_t slotStatus[TOTAL_SLOTS] = {0}; // 0: trống, 1: có xe
uint8_t currentFreeSlots = TOTAL_SLOTS;
float currentDistance = -1.0f;

// Lệnh điều khiển từ xa qua Firebase
bool remoteOpenGateIn = false;
bool remoteOpenGateOut = false;

/* ================= STATE MACHINE BARRIER AN TOÀN ================= */
enum BarrierState
{
    BARRIER_CLOSED = 0,
    BARRIER_OPENED,
    BARRIER_WAIT_CLEAR
};

BarrierState barrierInState  = BARRIER_CLOSED;
BarrierState barrierOutState = BARRIER_CLOSED;

unsigned long barrierInOpenTick  = 0;
unsigned long barrierOutOpenTick = 0;

/* ================= BIẾN NON-BLOCKING TIMERS ================= */
unsigned long lastDisplayTick     = 0;
unsigned long lastDistanceTick    = 0;
unsigned long lastBackendSync     = 0;
unsigned long lastCommandPoll     = 0;
unsigned long lastWifiCheck       = 0;

// Buzzer Non-blocking state
bool buzzerActive = false;
bool buzzerIsOn   = false;
unsigned long buzzerStateChangeTick = 0;

/* =========================================================
   CẢM BIẾN Ô ĐỖ (CÓ DEBOUNCE PHẦN MỀM)
   ========================================================= */
uint8_t IsSlotOccupied(uint8_t index)
{
    // Đọc cảm biến IR: LOW là có vật cản (có xe)
    if (digitalRead(PIN_SLOTS[index]) == LOW)
    {
        delayMicroseconds(200); // Lọc nhiễu xung ngắn
        return (digitalRead(PIN_SLOTS[index]) == LOW) ? 1 : 0;
    }
    return 0;
}

uint8_t CountFreeSlots(uint8_t *statusArr)
{
    uint8_t freeCount = 0;
    for (uint8_t i = 0; i < TOTAL_SLOTS; i++)
    {
        statusArr[i] = IsSlotOccupied(i);
        if (statusArr[i] == 0) freeCount++;
    }
    return freeCount;
}

/* =========================================================
   CẢM BIẾN SIÊU ÂM HC-SR04
   ========================================================= */
float HCSR04_ReadDistance(void)
{
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // pulseIn timeout 25ms (tương đương max ~4m)
    long duration = pulseIn(PIN_ECHO, HIGH, 25000UL);

    if (duration == 0)
    {
        return -1.0f; // Quá xa hoặc không bắt được sóng phản xạ
    }

    return duration / 58.0f; // Quy đổi ra cm
}

/* =========================================================
   BUZZER CẢNH BÁO - HOÀN TOÀN NON-BLOCKING (KHÔNG DELAY)
   ========================================================= */
void Buzzer_Update(float distance)
{
    unsigned long now = millis();

    // Ngoài phạm vi cảnh báo -> Tắt còi
    if (distance < 0 || distance > DIST_ALERT_FAR)
    {
        if (buzzerIsOn)
        {
            digitalWrite(PIN_BUZZER, LOW);
            buzzerIsOn = false;
        }
        buzzerActive = false;
        return;
    }

    // Đỗ quá sát (<= 10cm) -> Còi kêu liên tục
    if (distance <= DIST_ALERT_NEAR)
    {
        digitalWrite(PIN_BUZZER, HIGH);
        buzzerIsOn = true;
        buzzerActive = true;
        return;
    }

    // Khoảng cách 10cm - 30cm: Kêu ngắt quãng (khoảng cách càng gần chu kỳ càng ngắn)
    buzzerActive = true;
    unsigned long beepOffTime = (unsigned long)(distance * 10); // 100ms - 300ms
    const unsigned long beepOnTime = 50; // Kêu ngắn 50ms

    if (buzzerIsOn)
    {
        if (now - buzzerStateChangeTick >= beepOnTime)
        {
            digitalWrite(PIN_BUZZER, LOW);
            buzzerIsOn = false;
            buzzerStateChangeTick = now;
        }
    }
    else
    {
        if (now - buzzerStateChangeTick >= beepOffTime)
        {
            digitalWrite(PIN_BUZZER, HIGH);
            buzzerIsOn = true;
            buzzerStateChangeTick = now;
        }
    }
}

/* =========================================================
   BARRIER - STATE MACHINE AN TOÀN CHỐNG ĐÈ NÓC XE
   ========================================================= */
uint8_t IsCarAtGate(uint8_t pin)
{
    return (digitalRead(pin) == LOW) ? 1 : 0;
}

void Barrier_Update(
    uint8_t sensorPin,
    Servo &servo,
    BarrierState *state,
    unsigned long *openTick,
    uint8_t allowOpen,
    bool &remoteRequest
)
{
    unsigned long now = millis();
    bool carDetected = IsCarAtGate(sensorPin);

    switch (*state)
    {
    case BARRIER_CLOSED:
        // Mở nếu cảm biến cổng phát hiện xe (và bãi còn chỗ) HOẶC có lệnh mở từ xa (Remote)
        if ((carDetected && allowOpen) || remoteRequest)
        {
            servo.write(SERVO_ANGLE_OPEN);
            *state = BARRIER_OPENED;
            *openTick = now;
            remoteRequest = false; // Đã tiêu thụ lệnh remote
            Serial.printf("[BARRIER] Mo cong (Pin %d)\n", sensorPin);
        }
        break;

    case BARRIER_OPENED:
        // Khi hết thời gian mở tối thiểu (4000ms):
        if (now - *openTick >= BARRIER_TIMEOUT_MS)
        {
            // TÍNH NĂNG AN TOÀN: Nếu xe vẫn còn đang đứng chắn cảm biến, KHÔNG ĐÓNG!
            if (carDetected)
            {
                // Gia hạn mở để xe đi qua hết, tránh va đập vào nóc xe
                *openTick = now;
            }
            else
            {
                // Xe đã đi qua an toàn -> Đóng cánh chắn
                servo.write(SERVO_ANGLE_CLOSED);
                *state = BARRIER_WAIT_CLEAR;
                Serial.printf("[BARRIER] Dong cong an toan (Pin %d)\n", sensorPin);
            }
        }
        break;

    case BARRIER_WAIT_CLEAR:
        // Đảm bảo khu vực cổng thông thoáng trước khi sẵn sàng cho xe tiếp theo
        if (!carDetected)
        {
            *state = BARRIER_CLOSED;
        }
        break;
    }
}

/* =========================================================
   OLED DISPLAY
   ========================================================= */
void DisplayParkingStatus(uint8_t freeCount, uint8_t *statusArr, float distance)
{
    if (!oledAvailable) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Tiêu đề & trạng thái kết nối
    display.setCursor(0, 0);
    display.printf("%s %s\n", LOT_NAME, WiFi.status() == WL_CONNECTED ? "[IoT]" : "[OFF]");

    display.setCursor(0, 12);
    display.printf("Trong: %d/%d cho\n", freeCount, TOTAL_SLOTS);

    display.setCursor(0, 24);
    display.printf("S1:%s S2:%s\n", statusArr[0] ? "X" : "O", statusArr[1] ? "X" : "O");

    display.setCursor(0, 34);
    display.printf("S3:%s S4:%s\n", statusArr[2] ? "X" : "O", statusArr[3] ? "X" : "O");

    display.setCursor(0, 46);
    if (distance >= 0)
        display.printf("Khoang cach: %.0f cm\n", distance);
    else
        display.println("Khoang cach: --");

    display.setCursor(0, 56);
    if (freeCount == 0)
        display.println("TRANG THAI: HET CHO!");
    else if (distance >= 0 && distance <= DIST_ALERT_NEAR)
        display.println("CANH BAO: DO SAT!");
    else
        display.printf("C.Vao:%s C.Ra:%s\n",
            barrierInState == BARRIER_OPENED ? "MO" : "DONG",
            barrierOutState == BARRIER_OPENED ? "MO" : "DONG");

    display.display();
}

/* =========================================================
   WIFI & FIREBASE REALTIME DATABASE REST SYNC
   ========================================================= */
void WiFi_Init()
{
    WiFi.mode(WIFI_STA);
    Serial.printf("[WIFI] Dang ket noi toi SSID: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000)
    {
        delay(300);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\n[WIFI] Da ket noi thanh cong!");
        Serial.printf("[WIFI] IP Address: %s\n", WiFi.localIP().toString().c_str());
    }
    else
    {
        Serial.println("\n[WIFI] Ket noi timeout! He thong van hoat dong offline va se thu lai sau.");
    }
}

void WiFi_CheckConnection()
{
    unsigned long now = millis();
    if (now - lastWifiCheck >= 10000) // Kiểm tra kết nối mỗi 10 giây
    {
        lastWifiCheck = now;
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[WIFI] Mat ket noi, dang thu ket noi lai...");
            WiFi.reconnect();
        }
    }
}

// Gửi trạng thái bãi xe lên Docker TimescaleDB Backend: /api/lots/{LOT_ID}/status
void Backend_PushStatus()
{
    if (WiFi.status() != WL_CONNECTED) return;

    WiFiClient client;
    HTTPClient http;
    String url = String(BACKEND_URL) + "/api/lots/" + String(LOT_ID) + "/status";

    if (http.begin(client, url))
    {
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-Api-Key", API_KEY);

        // Tạo JSON payload bằng ArduinoJson
        JsonDocument doc;
        doc["lotId"] = LOT_ID;
        doc["lotName"] = LOT_NAME;
        doc["totalSlots"] = TOTAL_SLOTS;
        doc["freeSlots"] = currentFreeSlots;
        doc["distance"] = currentDistance >= 0 ? currentDistance : -1;
        doc["lastHeartbeat"] = millis();

        JsonObject slots = doc["slots"].to<JsonObject>();
        slots["1"] = (slotStatus[0] == 1);
        slots["2"] = (slotStatus[1] == 1);
        slots["3"] = (slotStatus[2] == 1);
        slots["4"] = (slotStatus[3] == 1);

        JsonObject gates = doc["gates"].to<JsonObject>();
        gates["in"] = (barrierInState == BARRIER_OPENED) ? "open" : "closed";
        gates["out"] = (barrierOutState == BARRIER_OPENED) ? "open" : "closed";

        String jsonPayload;
        serializeJson(doc, jsonPayload);

        int httpCode = http.POST(jsonPayload);
        if (httpCode >= 200 && httpCode < 300)
        {
            // Thành công (200 OK)
        }
        else
        {
            Serial.printf("[BACKEND] Push status that bai (HTTP %d)\n", httpCode);
        }
        http.end();
    }
}

bool Backend_AcknowledgeGate(const String &url, const char *gate)
{
    WiFiClient client;
    HTTPClient http;
    if (!http.begin(client, url + "/" + gate)) return false;

    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Api-Key", API_KEY);
    int code = http.PUT("{\"value\":\"idle\"}");
    http.end();
    if (code >= 200 && code < 300) return true;

    Serial.printf("[BACKEND CMD] Khong xac nhan duoc %s (HTTP %d)\n", gate, code);
    return false;
}

// Kiểm tra lệnh mở cổng từ xa từ Docker TimescaleDB Backend: /api/lots/{LOT_ID}/commands
void Backend_PollCommands()
{
    if (WiFi.status() != WL_CONNECTED) return;

    WiFiClient client;
    HTTPClient http;
    String url = String(BACKEND_URL) + "/api/lots/" + String(LOT_ID) + "/commands";

    if (http.begin(client, url))
    {
        http.addHeader("X-Api-Key", API_KEY);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK)
        {
            String payload = http.getString();
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, payload);

            if (!err)
            {
                const char* cmdIn = doc["gateIn"];
                const char* cmdOut = doc["gateOut"];
                http.end();

                if (cmdIn && strcmp(cmdIn, "open") == 0 && Backend_AcknowledgeGate(url, "gateIn"))
                {
                    Serial.println("[BACKEND CMD] Nhan lenh MO CONG VAO!");
                    remoteOpenGateIn = barrierInState == BARRIER_CLOSED;
                }

                if (cmdOut && strcmp(cmdOut, "open") == 0 && Backend_AcknowledgeGate(url, "gateOut"))
                {
                    Serial.println("[BACKEND CMD] Nhan lenh MO CONG RA!");
                    remoteOpenGateOut = barrierOutState == BARRIER_CLOSED;
                }
                return;
            }
        }
        http.end();
    }
}

/* =========================================================
   SETUP
   ========================================================= */
void setup()
{
    Serial.begin(115200);
    Serial.println("\n=== HE THONG BAI DO XE THONG MINH (SIC IoT) ===");

    // 1. Cấu hình chân cảm biến IR ô đỗ
    for (uint8_t i = 0; i < TOTAL_SLOTS; i++)
    {
        pinMode(PIN_SLOTS[i], INPUT_PULLUP);
    }

    // 2. Cấu hình chân cảm biến IR cổng
    pinMode(PIN_GATE_IN, INPUT_PULLUP);
    pinMode(PIN_GATE_OUT, INPUT_PULLUP);

    // 3. Cấu hình HC-SR04
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);

    // 4. Cấu hình Buzzer
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    // 5. Cấu hình Servo Barrier (GPIO 4 & GPIO 13 an toàn)
    servoIn.attach(PIN_SERVO_IN);
    servoOut.attach(PIN_SERVO_OUT);
    servoIn.write(SERVO_ANGLE_CLOSED);
    servoOut.write(SERVO_ANGLE_CLOSED);

    // 6. Cấu hình OLED (Không để while(1) làm tê liệt hệ thống nếu lỏng dây)
    Wire.begin(PIN_SDA, PIN_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        Serial.println("[WARN] Khong tim thay OLED SSD1306. He thong tiep tuc chay!");
        oledAvailable = false;
    }
    else
    {
        oledAvailable = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 20);
        display.println("KHOI DONG HE THONG");
        display.setCursor(10, 35);
        display.println("Smart Parking...");
        display.display();
    }

    // 7. Kết nối WiFi
    WiFi_Init();

    Serial.println("[SYSTEM] San sang hoat dong!");
}

/* =========================================================
   LOOP CHÍNH (HOÀN TOÀN NON-BLOCKING)
   ========================================================= */
void loop()
{
    unsigned long now = millis();

    // 1. Đọc cảm biến ô đỗ (cung cấp số chỗ trống tức thì cho barrier)
    currentFreeSlots = CountFreeSlots(slotStatus);

    // 2. Cập nhật Barrier cổng Vào (ưu tiên kiểm tra liên tục mỗi chu kỳ)
    Barrier_Update(
        PIN_GATE_IN, servoIn,
        &barrierInState, &barrierInOpenTick,
        (currentFreeSlots > 0),
        remoteOpenGateIn
    );

    // 3. Cập nhật Barrier cổng Ra
    Barrier_Update(
        PIN_GATE_OUT, servoOut,
        &barrierOutState, &barrierOutOpenTick,
        1,
        remoteOpenGateOut
    );

    // 4. Đo khoảng cách HC-SR04 định kỳ
    if (now - lastDistanceTick >= DISTANCE_READ_INTERVAL_MS)
    {
        lastDistanceTick = now;
        currentDistance = HCSR04_ReadDistance();
    }

    // 5. Cập nhật Còi cảnh báo (Non-blocking buzzer state)
    Buzzer_Update(currentDistance);

    // 6. Cập nhật Màn hình OLED
    if (now - lastDisplayTick >= DISPLAY_INTERVAL_MS)
    {
        lastDisplayTick = now;
        DisplayParkingStatus(currentFreeSlots, slotStatus, currentDistance);
    }

    // 7. Đồng bộ dữ liệu lên Docker TimescaleDB Backend
    if (now - lastBackendSync >= BACKEND_SYNC_INTERVAL_MS)
    {
        lastBackendSync = now;
        Backend_PushStatus();
    }

    // 8. Lắng nghe lệnh từ xa từ Backend
    if (now - lastCommandPoll >= COMMAND_POLL_INTERVAL_MS)
    {
        lastCommandPoll = now;
        Backend_PollCommands();
    }

    // 9. Giám sát tự động kết nối lại WiFi
    WiFi_CheckConnection();
}
