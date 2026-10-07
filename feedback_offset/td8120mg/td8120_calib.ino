/*
  TD8120 (servo 180 do) - Hieu chinh offset + doc feedback ADC tren ESP32
  Phien ban: Noi suy 2 doan (0-90-180) de sua loi phi tuyen tinh cua bien tro.

  Dau noi:
    Servo signal  -> GPIO18
    Servo V+      -> nguon rieng 5-6V
    Servo GND     -> GND chung voi ESP32
    Feedback wiper-> GPIO34 (ADC1), dien ap toi da 3.3V

  Serial Monitor: 115200 baud, chon "Newline"
*/

#include <ESP32Servo.h>
#include <Preferences.h>

// ================== CAU HINH ==================
const int SERVO_PIN = 18;
const int FB_PIN    = 34;

const int PULSE_LIMIT_LOW  = 400;    // gioi han an toan khi chinh offset (us)
const int PULSE_LIMIT_HIGH = 2600;
const int PULSE_STEP       = 5;      // moi lan bam q/e/z/c doi bao nhieu us

const int DEFAULT_PULSE_MIN = 500;
const int DEFAULT_PULSE_MAX = 2500;

// ================== BIEN TOAN CUC ==================
Servo servo;
Preferences prefs;

int pulseMin = DEFAULT_PULSE_MIN;    // us ung voi 0 do
int pulseMax = DEFAULT_PULSE_MAX;    // us ung voi 180 do
int adc0     = 2533;                 // mV feedback tai 0 do   (se duoc ghi de khi chay 'k')
int adcMid   = 1618;                 // mV feedback tai 90 do  (se duoc ghi de khi chay 'k')
int adc180   = 703;                  // mV feedback tai 180 do (se duoc ghi de khi chay 'k')

float cmdAngle = 90.0f;

// So chu so thap phan khi lam tron fb va err
const int FB_DECIMALS = 0;

// ================== TIEN ICH ==================
int readFeedbackMv() {
  long sum = 0;
  const int N = 32;
  for (int i = 0; i < N; i++) {
    sum += analogReadMilliVolts(FB_PIN);
    delay(2);
  }
  return sum / N;
}

int angleToUs(float a) {
  a = constrain(a, 0.0f, 180.0f);
  return pulseMin + (long)((pulseMax - pulseMin) * a / 180.0);
}

// Noi suy 2 doan de xu ly bien tro phi tuyen tinh
float mvToAngle(int mv) {
  if (adc180 == adc0) return 0;
  
  float a = 0;
  bool isReverse = (adc180 < adc0); // Kiem tra feedback thuan hay nguoc
  
  if (!isReverse) {
    if (mv <= adcMid) {
      if (adcMid == adc0) return 0;
      a = (mv - adc0) * 90.0f / (adcMid - adc0);
    } else {
      if (adc180 == adcMid) return 90.0f;
      a = 90.0f + (mv - adcMid) * 90.0f / (adc180 - adcMid);
    }
  } else {
    if (mv >= adcMid) {
      if (adcMid == adc0) return 0;
      a = (mv - adc0) * 90.0f / (adcMid - adc0);
    } else {
      if (adc180 == adcMid) return 90.0f;
      a = 90.0f + (mv - adcMid) * 90.0f / (adc180 - adcMid);
    }
  }
  return constrain(a, 0.0f, 180.0f);
}

// Lam tron x den d chu so thap phan (tranh in ra "-0")
float roundTo(float x, int d) {
  float p = powf(10.0f, d);
  float r = roundf(x * p) / p;
  if (r == 0) r = 0.0f;
  return r;
}

void moveTo(float a, int settleMs = 700) {
  cmdAngle = constrain(a, 0.0f, 180.0f);
  servo.writeMicroseconds(angleToUs(cmdAngle));
  delay(settleMs);
}

void saveCal() {
  prefs.begin("td8120", false);
  prefs.putInt("pMin", pulseMin);
  prefs.putInt("pMax", pulseMax);
  prefs.putInt("a0", adc0);
  prefs.putInt("aMid", adcMid);
  prefs.putInt("a180", adc180);
  prefs.end();
  Serial.println(">> Da luu hieu chinh vao flash.");
}

void loadCal() {
  prefs.begin("td8120", true);
  pulseMin = prefs.getInt("pMin", pulseMin);
  pulseMax = prefs.getInt("pMax", pulseMax);
  adc0     = prefs.getInt("a0", adc0);
  adcMid   = prefs.getInt("aMid", adcMid);
  adc180   = prefs.getInt("a180", adc180);
  prefs.end();
}

void printCal() {
  Serial.printf("pulseMin=%d us | pulseMax=%d us | adc0=%d mV | adcMid=%d mV | adc180=%d mV\n",
                pulseMin, pulseMax, adc0, adcMid, adc180);
}

// Tu dong do lay mau ADC tai 3 diem (0, 90, 180)
void calibrateADC() {
  Serial.println(">> Calib ADC: ve 0 do...");
  moveTo(0, 2000);
  int v0 = readFeedbackMv();

  Serial.println(">> Calib ADC: ve 90 do...");
  moveTo(90, 1500);
  int vMid = readFeedbackMv();

  Serial.println(">> Ve 180 do...");
  moveTo(180, 2000);
  int v180 = readFeedbackMv();

  Serial.printf(">> Do duoc: adc0=%d mV, adcMid=%d mV, adc180=%d mV\n", v0, vMid, v180);

  if (abs(v180 - v0) < 200) {
    Serial.println("!! Chenh lech < 200 mV, feedback co the chua noi hoac sai chan. KHONG cap nhat.");
  } else {
    adc0 = v0;
    adcMid = vMid;
    adc180 = v180;
    if (adc180 < adc0) Serial.println(">> Feedback dao chieu (goc tang -> dien ap giam), da tu xu ly.");
    Serial.println(">> Calib hoan tat! Bam 'p' de luu vao Flash.");
  }
  moveTo(90, 1000);
}

// Quet 5 diem de kiem tra do tuyen tinh
void sweepTest() {
  Serial.println(">> Quet kiem tra (5 diem):");
  const float pts[5] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f};
  for (int i = 0; i < 5; i++) {
    moveTo(pts[i], 1500);
    int mv = readFeedbackMv();
    float fbRaw = mvToAngle(mv);
    float fb    = roundTo(fbRaw, FB_DECIMALS);
    float err   = roundTo(fbRaw - pts[i], FB_DECIMALS);
    Serial.printf("   cmd=%5.1f | ADC=%4d mV | fb=%6.*f | err=%6.*f\n",
                  pts[i], mv, FB_DECIMALS, fb, FB_DECIMALS, err);
  }
  moveTo(90, 800);
}

void printHelp() {
  Serial.println(F(
    "\n=============== LENH ===============\n"
    "g<goc> : di toi goc, vd g45\n"
    "0 9 1  : di toi 0 / 90 / 180 do\n"
    "q / e  : pulseMin  -5 / +5 us (offset 0 do)\n"
    "z / c  : pulseMax  -5 / +5 us (offset 180 do)\n"
    "k      : tu do ADC tai 3 diem (0, 90, 180 do)\n"
    "s      : quet 5 diem kiem tra sai so\n"
    "p      : luu vao flash\n"
    "r      : reset ve mac dinh (chua luu)\n"
    "i      : in thong so hien tai\n"
    "h      : tro giup\n"
    "====================================\n"));
}

void handleCmd(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = s[0];

  switch (c) {
    case 'g': moveTo(s.substring(1).toFloat(), 500); break;
    case '0': moveTo(0);   break;
    case '9': moveTo(90);  break;
    case '1': moveTo(180); break;

    case 'q': pulseMin = max(pulseMin - PULSE_STEP, PULSE_LIMIT_LOW);  moveTo(0, 300);   break;
    case 'e': pulseMin = min(pulseMin + PULSE_STEP, pulseMax - 100);   moveTo(0, 300);   break;
    case 'z': pulseMax = max(pulseMax - PULSE_STEP, pulseMin + 100);   moveTo(180, 300); break;
    case 'c': pulseMax = min(pulseMax + PULSE_STEP, PULSE_LIMIT_HIGH); moveTo(180, 300); break;

    case 'k': calibrateADC(); break;
    case 's': sweepTest(); break;
    case 'p': saveCal(); break;
    case 'r':
      pulseMin = DEFAULT_PULSE_MIN;
      pulseMax = DEFAULT_PULSE_MAX;
      Serial.println(">> Da reset pulse ve mac dinh.");
      break;
    case 'i': break;
    case 'h': printHelp(); return;
    default:  Serial.println("Lenh khong hop le. Go h de xem tro giup."); return;
  }
  printCal();
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(FB_PIN, ADC_11db);   // 0 - ~3.3V

  loadCal();

  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, PULSE_LIMIT_LOW, PULSE_LIMIT_HIGH);
  moveTo(90, 1000);

  printHelp();
  printCal();
}

void loop() {
  if (Serial.available()) {
    handleCmd(Serial.readStringUntil('\n'));
  }

  static uint32_t t = 0;
  if (millis() - t > 200) {
    t = millis();
    int mv = readFeedbackMv();
    float fbRaw = mvToAngle(mv);
    float fb    = roundTo(fbRaw, FB_DECIMALS);
    float err   = roundTo(fbRaw - cmdAngle, FB_DECIMALS);
    Serial.printf("cmd=%6.1f deg | pulse=%4d us | ADC=%4d mV | fb=%4.*f deg | err=%4.*f\n",
                  cmdAngle, angleToUs(cmdAngle), mv, FB_DECIMALS, fb, FB_DECIMALS, err);
  }
}

