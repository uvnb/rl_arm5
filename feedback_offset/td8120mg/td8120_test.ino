/*
  HỆ THỐNG ĐO PHẢN HỒI BƯỚC ĐỂ XÁC ĐỊNH THÔNG SỐ SERVO (THỜI HẰNG, VÙNG CHẾT, ĐỘ RƠ)
  - Đạt tần số ghi mẫu >= 100Hz (10ms một mẫu) trong vòng 2 giây.
  - Hỗ trợ đổi cấu hình linh hoạt cho 3 dòng Servo: TD8120MG, RDS3120MG, MG995.
*/

#include <ESP32Servo.h>
#include <Preferences.h>

// ================== CẤU HÌNH LOẠI SERVO ĐANG TEST ==================
// Bạn hãy comment/uncomment dòng này để chọn loại Servo tương ứng
#define SERVO_TYPE_TD8120
// #define SERVO_TYPE_RDS3120
// #define SERVO_TYPE_MG995

// ================== CẤU HÌNH PHẦN CỨNG & KIỂM THỬ ==================
const int SERVO_PIN = 18;
const int FB_PIN    = 34;

// Tỷ số chia áp (Nếu đấu thẳng = 1.0, mạch cầu phân áp 47k+47k = 2.0)
#if defined(SERVO_TYPE_RDS3120) || defined(SERVO_TYPE_MG995)
  const float FB_DIVIDER_RATIO = 2.0f;
  const char* PREFS_NAMESPACE  = (defined(SERVO_TYPE_RDS3120)) ? "rds3120" : "mg995";
#else
  const float FB_DIVIDER_RATIO = 1.0f; // TD8120 mặc định đọc thẳng qua ADC chân ESP
  const char* PREFS_NAMESPACE  = "td8120";
#endif

// Thông số lấy mẫu tốc độ cao
const unsigned long SAMPLE_INTERVAL_MS = 10; // 10ms = 100Hz (Thoả mãn cấu hình >= 100Hz)
const int SAMPLES_FAST = 4; // Giảm số mẫu lấy trung bình xuống để tránh block tiến trình 100Hz

// Thông số hiệu chỉnh mặc định (Sẽ nạp lại từ Flash nếu có)
int pulseMin = 500;
int pulseMax = 2500;
int adc0     = 1000;
int adcMid   = 2000;
int adcMax   = 3000;
bool calibrated = false;

// Biến toàn cục điều khiển kiểm thử
Servo servo;
Preferences prefs;
float currentCmdAngle = 90.0f;

// Trạng thái bài test tự động
bool isTesting = false;
unsigned long testStartTime = 0;
float testStepTarget = 90.0f;

// ================== CÁC HÀM NỘI SUY GÓC PHẢN HỒI ==================
float readWiperVoltage() {
  float sum = 0;
  for (int i = 0; i < SAMPLES_FAST; i++) {
    sum += (float)analogReadMilliVolts(FB_PIN);
  }
  return (sum / SAMPLES_FAST) * FB_DIVIDER_RATIO;
}

float mvToAngle(float mv) {
  if (adcMax == adc0) return 0.0f;
  const bool isReverse = (adcMax < adc0);
  const bool inFirstHalf = isReverse ? (mv >= adcMid) : (mv <= adcMid);

  float a;
  if (inFirstHalf) {
    if (adcMid == adc0) return 0.0f;
    a = (mv - adc0) * 90.0f / (adcMid - adc0);
  } else {
    if (adcMax == adcMid) return 90.0f;
    a = 90.0f + (mv - adcMid) * 90.0f / (adcMax - adcMid);
  }
  return constrain(a, 0.0f, 180.0f);
}

int angleToUs(float a) {
  a = constrain(a, 0.0f, 180.0f);
  return pulseMin + (long)((pulseMax - pulseMin) * a / 180.0f);
}

void moveTo(float a) {
  currentCmdAngle = constrain(a, 0.0f, 180.0f);
  servo.writeMicroseconds(angleToUs(currentCmdAngle));
}

// Tải thông số hiệu chuẩn tương ứng từ bộ nhớ Flash đã lưu trước đó
void loadCalibration() {
  if (!prefs.begin(PREFS_NAMESPACE, true)) {
    calibrated = false;
    return;
  }
  pulseMin = prefs.getInt("pMin", pulseMin);
  pulseMax = prefs.getInt("pMax", pulseMax);
  adc0     = prefs.getInt("a0", adc0);
  adcMid   = prefs.getInt("aMid", adcMid);
  
  if (prefs.isKey("aMax")) {
    adcMax = prefs.getInt("aMax", adcMax);
  } else if (prefs.isKey("a180")) {
    adcMax = prefs.getInt("a180", adcMax);
  }
  
  calibrated = true;
  prefs.end();
}

// ================== KHỞI TẠO HỆ THỐNG ==================
void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  analogSetPinAttenuation(FB_PIN, ADC_11db);

  loadCalibration();

  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 400, 2600);
  
  // Đưa servo về điểm chính giữa ổn định trước khi test
  moveTo(90.0f);
  delay(1000);

  Serial.println("\n=============================================");
  Serial.printf("HỆ THỐNG THU THẬP PHẢN HỒI BƯỚC: %s\n", PREFS_NAMESPACE);
  Serial.printf("Trạng thái Calib: %s (adc0:%d, adcMid:%d, adcMax:%d)\n", 
                calibrated ? "OK" : "CHƯA CALIB (Dùng mặc định)", adc0, adcMid, adcMax);
  Serial.println("👉 Gõ 's' để bắt đầu kích hoạt bước nhảy thử nghiệm (Step Response)");
  Serial.println("=============================================");
}

// ================== VÒNG LẶP CHÍNH ==================
void loop() {
  // 1. Kiểm tra lệnh từ máy tính qua Serial Monitor
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("s")) {
      if (!isTesting) {
        // Thiết lập bước nhảy: Nhảy góc 0.2 rad (~11.5 độ) từ 90 độ lên 101.5 độ
        testStepTarget = 90.0f + 11.46f; 
        
        // Đưa về vị trí gốc ban đầu trước
        moveTo(90.0f);
        delay(800); 
        
        // Bắt đầu ghi dữ liệu tần số cao
        Serial.println("START_DATA"); // Ký hiệu bắt đầu khối dữ liệu để script Python nhận diện
        isTesting = true;
        testStartTime = millis();
        
        // Ra lệnh nhảy bước lập tức
        moveTo(testStepTarget);
      }
    }
  }

  // 2. Thu thập dữ liệu thời gian thực nếu đang trong phiên Test
  static unsigned long lastSampleTime = 0;
  if (isTesting) {
    unsigned long currentMillis = millis();
    unsigned long elapsed = currentMillis - testStartTime;

    if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
      lastSampleTime = currentMillis;

      // Đọc điện áp cảm biến hiện tại và quy đổi sang độ thực tế
      float wiperMv = readWiperVoltage();
      float currentFbAngle = mvToAngle(wiperMv);

      // Quy đổi từ độ sang Radian để khớp hoàn toàn với cấu hình config.py trong simulator
      float cmdRad = currentCmdAngle * 0.0174532925f;
      float fbRad  = currentFbAngle * 0.0174532925f;

      // Xuất dữ liệu định dạng CSV gọn gàng: Thời gian(ms), Góc Lệnh(rad), Góc Thực Tế(rad)
      Serial.printf("%lu,%.4f,%.4f\n", elapsed, cmdRad, fbRad);
    }

    // Kết thúc bài test sau đúng 2000ms (2 giây) theo quy chuẩn checklist
    if (elapsed >= 2000) {
      isTesting = false;
      Serial.println("END_DATA");
      moveTo(90.0f); // Đưa servo về lại vị trí an toàn
      Serial.println(">> Đã hoàn thành 1 phiên ghi dữ liệu 2 giây. Hãy lưu lại kết quả.");
    }
  }
}
