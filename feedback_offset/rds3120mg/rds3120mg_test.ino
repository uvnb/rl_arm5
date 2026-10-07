/*
  RDS3120MG (servo 20kg, metal gear) - BAN 180 DO
  KIEM TRA + LAY FEEDBACK ADC tren ESP32

  Sketch nay CHI DOC thong so hieu chinh da luu trong flash (namespace "rds3120")
  boi sketch rds3120mg_180_final.ino (lenh 'k' roi 'p').
  Sketch nay KHONG ghi de du lieu hieu chinh.

  Dau noi (giong sketch hieu chinh):
    Servo signal  -> GPIO18
    Servo V+      -> nguon RIENG 6-7.2V, >= 5A (KHONG lay tu ESP32)
    Servo GND     -> GND chung voi ESP32
    Feedback wiper-> cau chia ap -> GPIO34 (ADC1)

  FB_DIVIDER_RATIO trong sketch nay PHAI GIONG sketch hieu chinh va KHOP mach that
  (47k+47k = 2.0 | 47k+22k = 3.14 | noi thang = 1.0).
  Neu khac ty so luc calib, sketch se bao va coi nhu CHUA CALIB.

  Cac bai kiem tra:
    t : kiem tra day du - quet tien/lui moi 15 do, do sai so, hysteresis, nhieu, PASS/FAIL
    r : kiem tra do lap lai - toi cung 1 goc tu hai phia (vd r90)
    n : kiem tra nhieu ADC tai vi tri hien tai
    v : quet cham xuat CSV de ve do thi (do tre cua feedback)
    d / a : nha servo (de xoay tay doc feedback) / bat servo lai
    m : doi che do in feedback lien tuc (van ban / Serial Plotter / tat)

  Serial Monitor: 115200 baud, chon "Newline"
*/

#include <ESP32Servo.h>
#include <Preferences.h>

// ================== CAU HINH ==================
const int SERVO_PIN = 18;
const int FB_PIN    = 34;

// Ty so chia ap = (R1 + R2) / R2. PHAI GIONG sketch hieu chinh
const float FB_DIVIDER_RATIO = 2.0f;

const int PULSE_LIMIT_LOW  = 400;
const int PULSE_LIMIT_HIGH = 2600;

// Nguong bao hoa ADC (mV tai chan ESP32)
const int ADC_SAT_MV = 3100;

const int   TEST_STEP_DEG = 15;                       // buoc goc khi kiem tra day du
const int   N_POINTS      = 180 / TEST_STEP_DEG + 1;  // 13 diem: 0, 15, ..., 180
const int   SETTLE_MS     = 1200;                     // servo 20kg quay cham hon, cho lau hon
const int   SAMPLES       = 64;                       // so mau ADC trung binh moi diem

const float PASS_TOL_DEG  = 3.0f;                     // sai so toi da chap nhan duoc (do)
const float PASS_HYST_DEG = 3.0f;                     // hysteresis toi da chap nhan duoc (do)

const int   FB_DECIMALS   = 1;                        // so chu so thap phan khi in fb/err

// ================== THONG SO HIEU CHINH (nap tu flash) ==================
int pulseMin = 500;
int pulseMax = 2500;
int adc0     = 1100;     // mV wiper tai 0 do
int adcMid   = 3278;     // mV wiper tai 90 do
int adcMax   = 5400;     // mV wiper tai 180 do
bool calibrated = false;

// ================== BIEN TOAN CUC ==================
Servo servo;
Preferences prefs;

float cmdAngle   = 90.0f;
bool  attached   = false;
int   streamMode = 1;        // 0 = tat, 1 = van ban, 2 = Serial Plotter

// Thong ke ADC, tat ca tinh theo dien ap WIPER (da nhan lai ty so chia ap)
struct Stat { float mean, sd, mn, mx; bool sat; };

// ================== TIEN ICH ==================
// Doc n mau ADC, tra ve thong ke theo dien ap wiper (mV)
Stat readStats(int n, int delayUs) {
  double sum = 0, sum2 = 0;
  float mn = 1e9f, mx = -1e9f;
  for (int i = 0; i < n; i++) {
    float v = (float)analogReadMilliVolts(FB_PIN);   // mV tai chan ESP32
    sum += v;
    sum2 += (double)v * v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    if (delayUs > 0) delayMicroseconds(delayUs);
  }
  float meanPin = (float)(sum / n);
  double var = sum2 / n - (double)meanPin * meanPin;
  float sdPin = var > 0 ? (float)sqrt(var) : 0.0f;

  Stat s;
  s.mean = meanPin * FB_DIVIDER_RATIO;
  s.sd   = sdPin * FB_DIVIDER_RATIO;
  s.mn   = mn * FB_DIVIDER_RATIO;
  s.mx   = mx * FB_DIVIDER_RATIO;
  s.sat  = (mx > ADC_SAT_MV);
  return s;
}

int angleToUs(float a) {
  a = constrain(a, 0.0f, 180.0f);
  return pulseMin + (long)((pulseMax - pulseMin) * a / 180.0f);
}

// Noi suy 2 doan (0 - 90 - 180), tu xu ly truong hop feedback dao chieu
float mvToAngle(float mv) {
  if (adcMax == adc0) return 0;

  const bool isReverse = (adcMax < adc0);
  const bool inFirstHalf = isReverse ? (mv >= adcMid) : (mv <= adcMid);

  float a;
  if (inFirstHalf) {
    if (adcMid == adc0) return 0;
    a = (mv - adc0) * 90.0f / (adcMid - adc0);
  } else {
    if (adcMax == adcMid) return 90.0f;
    a = 90.0f + (mv - adcMid) * 90.0f / (adcMax - adcMid);
  }
  return constrain(a, 0.0f, 180.0f);
}

// He so quy doi do nhieu mV -> do (trung binh tren toan hanh trinh)
float degPerMv() {
  int span = abs(adcMax - adc0);
  return span > 0 ? 180.0f / span : 0.0f;
}

void moveTo(float a, int settleMs = 800) {
  cmdAngle = constrain(a, 0.0f, 180.0f);
  if (attached) servo.writeMicroseconds(angleToUs(cmdAngle));
  delay(settleMs);
}

void attachServo() {
  if (attached) return;
  servo.attach(SERVO_PIN, PULSE_LIMIT_LOW, PULSE_LIMIT_HIGH);
  attached = true;
}

void detachServo() {
  if (!attached) return;
  servo.detach();
  attached = false;
}

void loadCal() {
  if (!prefs.begin("rds3120", true)) {
    calibrated = false;
    return;
  }
  pulseMin = prefs.getInt("pMin", pulseMin);
  pulseMax = prefs.getInt("pMax", pulseMax);
  adc0     = prefs.getInt("a0", adc0);
  adcMid   = prefs.getInt("aMid", adcMid);
  adcMax   = prefs.getInt("aMax", adcMax);
  calibrated = prefs.getBool("cal", false);
  float savedRatio = prefs.getFloat("ratio", FB_DIVIDER_RATIO);
  prefs.end();

  // Ty so chia ap luc calib khac ty so hien tai -> du lieu da luu khong con dung
  if (calibrated && fabsf(savedRatio - FB_DIVIDER_RATIO) > 0.01f) {
    calibrated = false;
    Serial.printf("!! Ty so chia ap luc calib (%.2f) khac sketch nay (%.2f). Chay lai 'k' o sketch hieu chinh.\n",
                  savedRatio, FB_DIVIDER_RATIO);
  }
}

void printCal() {
  Serial.printf("ty so chia ap=%.2f | pulseMin=%d us | pulseMax=%d us | adc0=%d | adcMid=%d | adcMax=%d mV (wiper)%s\n",
                FB_DIVIDER_RATIO, pulseMin, pulseMax, adc0, adcMid, adcMax,
                calibrated ? "" : "  [CHUA CALIB]");
}

void printRow(float cmd, const Stat &s, float fb, float err) {
  Serial.printf("   cmd=%5.1f | wiper=%7.1f mV (sd %5.1f) | fb=%7.*f | err=%+6.*f%s\n",
                cmd, s.mean, s.sd, FB_DECIMALS, fb, FB_DECIMALS, err,
                s.sat ? "  !! ADC BAO HOA" : "");
}

// ================== BAI KIEM TRA ==================
// 1) Kiem tra day du: quet tien + lui, do sai so, hysteresis, nhieu
void fullTest() {
  if (!attached) { Serial.println("!! Servo dang nha. Go 'a' de bat servo truoc."); return; }
  if (!calibrated) Serial.println("!! Chua co du lieu calib hop le trong flash, ket qua chi mang tinh tham khao.");

  float fwdFb[N_POINTS], bwdFb[N_POINTS];
  float maxSd = 0;
  bool anySat = false;

  Serial.println("\n>> KIEM TRA DAY DU (khoang 40 giay), khong cham vao servo...");
  moveTo(0, 2000);

  Serial.println("--- CHIEU TIEN (0 -> 180) ---");
  for (int i = 0; i < N_POINTS; i++) {
    float a = i * TEST_STEP_DEG;
    moveTo(a, SETTLE_MS);
    Stat s = readStats(SAMPLES, 2000);
    float fb = mvToAngle(s.mean);
    fwdFb[i] = fb;
    if (s.sd > maxSd) maxSd = s.sd;
    if (s.sat) anySat = true;
    printRow(a, s, fb, fb - a);
  }

  Serial.println("--- CHIEU LUI (180 -> 0) ---");
  for (int i = N_POINTS - 1; i >= 0; i--) {
    float a = i * TEST_STEP_DEG;
    moveTo(a, SETTLE_MS);
    Stat s = readStats(SAMPLES, 2000);
    float fb = mvToAngle(s.mean);
    bwdFb[i] = fb;
    if (s.sd > maxSd) maxSd = s.sd;
    if (s.sat) anySat = true;
    printRow(a, s, fb, fb - a);
  }

  // Tong ket
  float maxAbs = 0, maxAbsAt = 0, sumAbs = 0, sumSq = 0;
  float maxHyst = 0, maxHystAt = 0;
  int cnt = 0;
  for (int i = 0; i < N_POINTS; i++) {
    float a = i * TEST_STEP_DEG;
    float e1 = fwdFb[i] - a;
    float e2 = bwdFb[i] - a;
    float ae1 = fabsf(e1), ae2 = fabsf(e2);
    if (ae1 > maxAbs) { maxAbs = ae1; maxAbsAt = a; }
    if (ae2 > maxAbs) { maxAbs = ae2; maxAbsAt = a; }
    sumAbs += ae1 + ae2;
    sumSq  += e1 * e1 + e2 * e2;
    cnt += 2;
    if (i > 0 && i < N_POINTS - 1) {            // hysteresis chi co y nghia o cac diem giua
      float h = fabsf(bwdFb[i] - fwdFb[i]);
      if (h > maxHyst) { maxHyst = h; maxHystAt = a; }
    }
  }
  float meanAbs = sumAbs / cnt;
  float rms = sqrtf(sumSq / cnt);
  float noiseDeg = maxSd * degPerMv();

  Serial.println("\n=============== KET QUA ===============");
  Serial.printf("Sai so lon nhat    : %.2f do (tai %.0f do)\n", maxAbs, maxAbsAt);
  Serial.printf("Sai so trung binh  : %.2f do | RMS: %.2f do\n", meanAbs, rms);
  Serial.printf("Hysteresis lon nhat: %.2f do (tai %.0f do)\n", maxHyst, maxHystAt);
  Serial.printf("Nhieu ADC lon nhat : %.1f mV tai wiper (khoang %.2f do)\n", maxSd, noiseDeg);
  bool passErr  = maxAbs  <= PASS_TOL_DEG;
  bool passHyst = maxHyst <= PASS_HYST_DEG;
  Serial.printf("Sai so     <= %.1f do: %s\n", PASS_TOL_DEG,  passErr  ? "DAT" : "KHONG DAT");
  Serial.printf("Hysteresis <= %.1f do: %s\n", PASS_HYST_DEG, passHyst ? "DAT" : "KHONG DAT");
  if (anySat) {
    Serial.println("!! ADC tung bao hoa (> 3100 mV tai chan ESP32): ket qua KHONG DANG TIN.");
    Serial.println("   Tang ty so chia ap (vd 47k + 22k = 3.14), calib lai roi kiem tra lai.");
    Serial.println(">> KET LUAN: KHONG HOP LE");
  } else {
    Serial.println(passErr && passHyst ? ">> KET LUAN: PASS" : ">> KET LUAN: FAIL");
  }
  Serial.println("Luu y: err tai 0, 90, 180 do gan 0 vi do la cac diem da dung de hieu chinh.");
  Serial.println("========================================");
  moveTo(90, 1200);
}

// 2) Do lap lai: toi cung mot goc tu phia thap va phia cao
void repeatTest(float target) {
  if (!attached) { Serial.println("!! Servo dang nha. Go 'a' de bat servo truoc."); return; }
  target = constrain(target, 0.0f, 180.0f);
  const int R = 8;
  float low[R], high[R];

  Serial.printf("\n>> KIEM TRA DO LAP LAI tai %.1f do (%d lan moi phia)...\n", target, R);
  for (int i = 0; i < R; i++) {
    moveTo(0, 1200);
    moveTo(target, SETTLE_MS);
    low[i] = mvToAngle(readStats(SAMPLES, 2000).mean);

    moveTo(180, 1200);
    moveTo(target, SETTLE_MS);
    high[i] = mvToAngle(readStats(SAMPLES, 2000).mean);

    Serial.printf("   lan %d | tu phia thap: %6.*f | tu phia cao: %6.*f\n",
                  i + 1, FB_DECIMALS, low[i], FB_DECIMALS, high[i]);
  }

  float mnL = low[0], mxL = low[0], sL = 0;
  float mnH = high[0], mxH = high[0], sH = 0;
  for (int i = 0; i < R; i++) {
    mnL = min(mnL, low[i]);   mxL = max(mxL, low[i]);   sL += low[i];
    mnH = min(mnH, high[i]);  mxH = max(mxH, high[i]);  sH += high[i];
  }
  float meanL = sL / R, meanH = sH / R;

  Serial.println("\n=============== KET QUA ===============");
  Serial.printf("Phia thap: TB=%.2f do | dao dong (max-min)=%.2f do\n", meanL, mxL - mnL);
  Serial.printf("Phia cao : TB=%.2f do | dao dong (max-min)=%.2f do\n", meanH, mxH - mnH);
  Serial.printf("Lech giua 2 phia (deadband/backlash): %.2f do\n", fabsf(meanH - meanL));
  Serial.println("========================================");
  moveTo(90, 1200);
}

// 3) Nhieu ADC tai vi tri hien tai
void noiseTest() {
  Serial.printf("\n>> KIEM TRA NHIEU ADC tai vi tri hien tai (%.1f do)...\n", cmdAngle);
  Stat s = readStats(200, 5000);
  float k = degPerMv();
  Serial.printf("TB=%.1f mV | sd=%.2f mV (%.3f do) | min=%.0f | max=%.0f | dao dong=%.0f mV (%.2f do)%s\n",
                s.mean, s.sd, s.sd * k, s.mn, s.mx, s.mx - s.mn, (s.mx - s.mn) * k,
                s.sat ? "  !! ADC BAO HOA" : "");
}

// 4) Quet cham xuat CSV de ve do thi (copy vao Excel/Sheets)
void sweepCsv() {
  if (!attached) { Serial.println("!! Servo dang nha. Go 'a' de bat servo truoc."); return; }
  Serial.println("\n>> QUET CHAM (copy cac dong CSV ben duoi vao Excel/Sheets de ve do thi)");
  moveTo(0, 2000);
  Serial.println("t_ms,cmd_deg,fb_deg,err_deg");
  uint32_t t0 = millis();

  for (int a = 0; a <= 180; a += 2) {
    moveTo(a, 50);
    float fb = mvToAngle(readStats(8, 200).mean);
    Serial.printf("%lu,%d,%.2f,%.2f\n", (unsigned long)(millis() - t0), a, fb, fb - a);
  }
  for (int a = 180; a >= 0; a -= 2) {
    moveTo(a, 50);
    float fb = mvToAngle(readStats(8, 200).mean);
    Serial.printf("%lu,%d,%.2f,%.2f\n", (unsigned long)(millis() - t0), a, fb, fb - a);
  }
  Serial.println(">> Xong.");
  moveTo(90, 1200);
}

// ================== LENH ==================
void printHelp() {
  Serial.println(F(
    "\n=============== LENH ===============\n"
    "t      : kiem tra day du (tien/lui, sai so, hysteresis, nhieu, PASS/FAIL)\n"
    "r[goc] : kiem tra do lap lai, vd r45 (mac dinh 90)\n"
    "n      : kiem tra nhieu ADC tai vi tri hien tai\n"
    "v      : quet cham xuat CSV de ve do thi\n"
    "g<goc> : di toi goc, vd g45\n"
    "0 9 1  : di toi 0 / 90 / 180 do\n"
    "d      : nha servo (xoay tay CHAM de doc feedback)\n"
    "a      : bat servo lai\n"
    "m      : doi che do in feedback: van ban / Serial Plotter / tat\n"
    "i      : in thong so hieu chinh dang dung\n"
    "h      : tro giup\n"
    "====================================\n"));
}

void handleCmd(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = s[0];

  switch (c) {
    case 't': fullTest(); break;
    case 'r': repeatTest(s.length() > 1 ? s.substring(1).toFloat() : 90.0f); break;
    case 'n': noiseTest(); break;
    case 'v': sweepCsv(); break;

    case 'g': attachServo(); moveTo(s.substring(1).toFloat(), 600); break;
    case '0': attachServo(); moveTo(0);   break;
    case '9': attachServo(); moveTo(90);  break;
    case '1': attachServo(); moveTo(180); break;

    case 'd':
      detachServo();
      Serial.println(">> Da nha servo. Xoay tay CHAM, khong cuong buc, va xem cot fb. Go 'a' de bat lai.");
      break;
    case 'a':
      attachServo();
      moveTo(cmdAngle, 800);
      Serial.println(">> Da bat servo.");
      break;

    case 'm':
      streamMode = (streamMode + 1) % 3;
      Serial.printf(">> Che do in feedback: %s\n",
                    streamMode == 0 ? "TAT" : (streamMode == 1 ? "VAN BAN" : "SERIAL PLOTTER"));
      break;

    case 'i': printCal(); break;
    case 'h': printHelp(); break;
    default:  Serial.println("Lenh khong hop le. Go h de xem tro giup."); break;
  }
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
  attachServo();
  moveTo(90, 1500);

  printHelp();
  printCal();
  if (!calibrated) {
    Serial.println("!! Chua co du lieu calib hop le trong flash.");
    Serial.println("   Hay nap sketch hieu chinh, chay 'k' roi 'p', sau do nap lai sketch nay.");
  }
}

void loop() {
  if (Serial.available()) {
    handleCmd(Serial.readStringUntil('\n'));
  }

  static uint32_t t = 0;
  uint32_t period = (streamMode == 2) ? 50 : 200;
  if (streamMode != 0 && millis() - t >= period) {
    t = millis();
    Stat s = readStats(16, 300);
    float fb = mvToAngle(s.mean);

    if (streamMode == 1) {
      if (attached) {
        Serial.printf("cmd=%6.1f deg | pulse=%4d us | wiper=%7.1f mV | fb=%7.*f deg | err=%+6.*f%s%s\n",
                      cmdAngle, angleToUs(cmdAngle), s.mean,
                      FB_DECIMALS, fb, FB_DECIMALS, fb - cmdAngle,
                      calibrated ? "" : "  [CHUA CALIB]",
                      s.sat ? "  !! ADC BAO HOA" : "");
      } else {
        Serial.printf("(nha servo) wiper=%7.1f mV | fb=%7.*f deg%s\n",
                      s.mean, FB_DECIMALS, fb, s.sat ? "  !! ADC BAO HOA" : "");
      }
    } else {
      if (attached) Serial.printf("cmd:%.1f,fb:%.1f\n", cmdAngle, fb);
      else          Serial.printf("fb:%.1f\n", fb);
    }
  }
}
