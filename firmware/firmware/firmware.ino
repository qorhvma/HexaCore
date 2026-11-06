/* ============================================================================
   안전
     - 조종기 신호가 0.5초 끊기면 그 자리에서 서기 자세로 멈춤 (블루투스 끊김 포함)
     - 켜질 때 서보마다 자세별 ticks 표 출력. ★ = 서보 한계에 걸려 안 움직이는 값

   [USB 시리얼 115200 / 블루투스 명령]
     o : 전부 힘 빼기
     i : IMU 스트림 켜기 (#imu,<ms>,<roll>,<pitch>,<정확도 0~3>,<샘플 나이 ms> 를 100Hz로)
     k : IMU 스트림 끄기
     p : 핀 스캔 켜기/끄기 — 쓸 수 있는 GPIO 를 전부 감시하다가 값이 바뀌면 "[핀] GPIO 17: 1→0" 처럼 알림
         (발 버튼이 실제로 어느 핀에 연결됐는지 찾을 때. 누르면 1→0, 떼면 0→1 이 정상)
     z : 지금 자세를 수평으로 기억 (플래시에 저장, 껐다 켜도 유지) → #zero,<ux>,<uy>,<uz>
     roll / pitch = '기억한 수평 자세'에서 중력 방향이 기운 각도 (IMU 를 어떤 방향으로 붙여도 0 근처에서 연속)
     IMU 가 없거나 초기화 실패면 스트림 중 1초마다 #imu,err
     e : (ENV_SENSORS_ENABLED=1 일 때만) 환경 센서 상태 한 번 출력

   [환경 센서 — 아직 미장착, ENV_SENSORS_ENABLED=0 이라 컴파일만 되고 동작 안 함]
     DHT22  (온도·습도)       : GPIO 27
     MQ-2   (연기·가스, 아날로그): GPIO 34 (ADC1, 입력 전용)
     불꽃 센서 (IR, 디지털)     : GPIO 35 (입력 전용, 외부 풀업)
     부저   (경보)             : GPIO 25
     출력: #env,<ms>,<온도 C>,<습도 %>,<연기 ppm>,<MQ-2 raw>,<불꽃 0/1>,<경보 비트>  (1Hz)
       경보 비트: bit0 = 고온, bit1 = 연기, bit2 = 불꽃, bit3 = 센서 오류

   ★ 처음엔 반드시 몸통을 들고 다리를 공중에 띄운 상태로 할 것.
============================================================================ */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>   // ESP32 보드 패키지에 들어있음 (클래식 블루투스)
#include <esp_mac.h>           // 블루투스 주소 출력용
#include <Adafruit_BNO08x.h>   // IMU (BNO086)
#include <Preferences.h>       // 수평 기준(z) 플래시 저장

Adafruit_PWMServoDriver pwmU1(0x41);
Adafruit_PWMServoDriver pwmU2(0x40);

#define SERVO_FREQ 50
const uint16_t TICK_HARD_MIN = 123;   // ≈600us
const uint16_t TICK_HARD_MAX = 492;   // ≈2400us
enum { HIP = 0, FEMUR = 1, TIBIA = 2 };

const char* BT_NAME = "Hyperism-HEXAPODv1";      // 컴퓨터에서 이 이름으로 페어링

const float REF_ANG[3] = { 180, 180, 120 };   // 서보 90°일 때 관절 각도 (hip, femur, tibia)
const float CAL_ANG[3] = { 40, 65, 30 };      // 캘리브레이션 기준 서보 각도
const float ANGLE_GAIN = 0.8f;                // 관절 1도 → 서보 0.8도 (네가 맞춘 값)

struct Sv {
  Adafruit_PWMServoDriver* drv;
  uint8_t ch;
  const char* name;
  int tCal;
  int dir;
};

const float TICKS_PER_DEG   = 11.111f / 4.889f;   // ≈ 2.27 (공칭)
const int   TICK_90_NOMINAL = 307;

Sv SV[6][3] = {
  //            HIP                                 FEMUR                               TIBIA
  { {&pwmU1,  2, "tfh1-3", 177, +1}, {&pwmU1,  1, "tfh1-2", 234, +1}, {&pwmU1,  0, "tfh1-1", 184, +1} }, // 다리1
  { {&pwmU1,  5, "tfh2-3", 184, +1}, {&pwmU1,  4, "tfh2-2", 233, +1}, {&pwmU1,  3, "tfh2-1", 170, +1} }, // 다리2
  { {&pwmU1,  8, "tfh3-3", 182, +1}, {&pwmU1,  7, "tfh3-2", 251, +1}, {&pwmU1,  6, "tfh3-1", 171, +1} }, // 다리3
  { {&pwmU1, 11, "hft4-3", 191, -1}, {&pwmU1, 10, "hft4-2", 237, +1}, {&pwmU1,  9, "hft4-1", 183, +1} }, // 다리4
  { {&pwmU1, 14, "hft5-3", 193, -1}, {&pwmU1, 13, "hft5-2", 227, +1}, {&pwmU1, 12, "hft5-1", 166, +1} }, // 다리5
  { {&pwmU2,  2, "hft6-3", 195, -1}, {&pwmU2,  1, "hft6-2", 229, +1}, {&pwmU2,  0, "hft6-1", 179, +1} }, // 다리6
};

// 관절별 각도 보정 [관절 도]. 받은 각도에 더해서 서보로 보냄 (tCal 은 그대로 두고 여기서만 미세 조정)
//   - 값을 줄이면(−) 그 관절이 '각도가 작은 쪽'으로 감: tibia 는 무릎이 더 접힘, femur 는 위로 듦
//   - 반대로 갔으면 부호만 바꿀 것
const float JOINT_OFS[6][3] = {
  //  HIP  FEMUR  TIBIA
  {   0,   0,     0 },   // 다리1
  {   0,   0,     0 },   // 다리2
  {   0,   0,    -10 },   // 다리3  (2026-10-17: tibia 가 수직에서 어긋나 보여서 기준을 5° 내림)
  {   0,   0,     0 },   // 다리4
  {   0,   0,     0 },   // 다리5
  {   0,   0,     0 },   // 다리6
};

BluetoothSerial CTRL;
const int FAILSAFE_MS = 2000;

// USB 시리얼과 블루투스(연결돼 있으면) 양쪽에 출력 → 컴퓨터 브리지 화면에 [로봇]으로 보임
#define LOG(...) do { Serial.printf(__VA_ARGS__); if (CTRL.hasClient()) CTRL.printf(__VA_ARGS__); } while (0)

float cur[6][3];
bool  powered = false;
bool  abortMove = false;
uint32_t lastPacket = 0;

// ------------------------------- 서보 변환 -------------------------------
float rawTicks(int l, int j, float jointAng) {
  const Sv& s = SV[l][j];
  jointAng += JOINT_OFS[l][j];
  float servoAng = 90.0f + s.dir * (jointAng - REF_ANG[j]) * ANGLE_GAIN;
  float sign = (s.tCal > TICK_90_NOMINAL) ? -1.0f : 1.0f;   // 최신 코드 방식 그대로
  return s.tCal + (servoAng - CAL_ANG[j]) * TICKS_PER_DEG * sign;
}
uint16_t jointToTicks(int l, int j, float a) {
  return (uint16_t)constrain(lroundf(rawTicks(l, j, a)), TICK_HARD_MIN, TICK_HARD_MAX);
}
void writeOne(int l, int j) { SV[l][j].drv->setPWM(SV[l][j].ch, 0, jointToTicks(l, j, cur[l][j])); }
void writeAll()             { for (int l = 0; l < 6; l++) for (int j = 0; j < 3; j++) writeOne(l, j); }
void allOff() {
  for (int l = 0; l < 6; l++) for (int j = 0; j < 3; j++) SV[l][j].drv->setPWM(SV[l][j].ch, 0, 4096);
  powered = false;
}

// ------------------------------- 발 접지 버튼 -------------------------------
// 다리1~6 순서. 버튼 한쪽은 GPIO, 다른 쪽은 GND (내부 풀업 → 눌림 = LOW = 닿음)
// ※ GPIO5는 부팅 스트래핑 핀: 켤 때 버튼이 눌려 있으면 부팅 모드가 바뀔 수 있음 → 다리 띄우고 켤 것
const uint8_t FOOT_PIN[6] = { 4, 16, 17, 5, 18, 19 };
const int FOOT_DEBOUNCE_MS = 5;      // 이 시간 동안 값이 그대로여야 바뀐 걸로 인정
const int FOOT_PERIOD_MS   = 100;    // 바뀌지 않아도 이 주기로 한 번씩 보냄 (PC가 최신값 확인용)

uint8_t  footState = 0;              // bit l = 다리 l+1 접지 (디바운스 끝난 값)
uint8_t  footRaw = 0;
uint32_t footRawSince = 0;

uint8_t readFootRaw() {
  uint8_t b = 0;
  for (int l = 0; l < 6; l++) if (digitalRead(FOOT_PIN[l]) == LOW) b |= (1 << l);
  return b;
}

// #ft,<ms>,<다리1><다리2>...<다리6>   예) #ft,12345,110111  (1 = 닿음)
void sendFoot() {
  char line[32];
  int n = snprintf(line, sizeof(line), "#ft,%lu,", (unsigned long)millis());
  for (int l = 0; l < 6; l++) line[n++] = (footState & (1 << l)) ? '1' : '0';
  line[n++] = '\n'; line[n] = 0;
  Serial.print(line);
  if (CTRL.hasClient()) CTRL.print(line);
}

void pollFoot() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();
  uint8_t raw = readFootRaw();
  if (raw != footRaw) { footRaw = raw; footRawSince = now; }
  bool changed = false;
  if (footRaw != footState && now - footRawSince >= FOOT_DEBOUNCE_MS) { footState = footRaw; changed = true; }
  if (changed || now - lastSend >= FOOT_PERIOD_MS) { sendFoot(); lastSend = now; }
}

// ------------------------------- IMU (BNO086) -------------------------------
// 서보 드라이버(Wire, 기본 SDA 21 / SCL 22)와 버스를 나눠서 Wire1 사용: SDA 32, SCL 33, 주소 0x4B
// 게임 회전 벡터(지자기 안 씀)에서 '센서 기준 위쪽(중력 반대) 방향'만 뽑아서, 기억한 수평 자세의 위쪽과 비교한 기울기를 보냄
//   ★ 예전엔 오일러 각(roll, pitch)을 바로 빼서 보냈는데, IMU 를 뒤집어 붙이면 roll 이 ±180° 근처라
//     0.2° 만 기울어도 +180 → -180 으로 넘어가 -359.8° 같은 값이 나옴 → PC 가 반대로 맞추는 것처럼 보였음
//   중력만 쓰니까 몸이 돌거나(yaw) 자이로가 흘러도 상관없음. 방향(축·부호)은 PC 쪽(hexapod_joy_walk2.py)이 시작할 때 재서 맞춤
//   펌웨어 기준을 바꾸고 싶으면 아래 IMU_SWAP / IMU_ROLL_SIGN / IMU_PITCH_SIGN
const int      IMU_SDA = 32, IMU_SCL = 33;
const uint8_t  IMU_ADDRS[2] = { 0x4B, 0x4A };   // BNO08x 주소는 보드마다 다름 (SparkFun 0x4B, Adafruit 0x4A) → 둘 다 시도
const uint32_t IMU_I2C_HZ = 100000;    // BNO08x 는 400kHz 에서 클럭 스트레칭 때문에 안 잡히는 경우가 있어 100kHz
const uint32_t IMU_REPORT_US = 5000;   // 센서 갱신 200Hz
const int      IMU_SEND_MS = 10;       // PC 로 100Hz (200Hz 는 블루투스/시리얼이 버거움)
const bool     IMU_SWAP = false;       // true 면 roll 과 pitch 를 바꿈 (IMU 를 90° 돌려 붙였을 때)
const float    IMU_ROLL_SIGN = 1.0f, IMU_PITCH_SIGN = 1.0f;

Adafruit_BNO08x bno(-1);
Preferences prefs;
bool     imuOk = false, imuStream = false;
float    up[3] = { 0, 0, 1 };          // 지금 위쪽 방향 (센서 좌표, 단위 벡터)
float    up0[3] = { 0, 0, 0 };         // 'z' 로 기억한 수평 자세의 위쪽 (0,0,0 = 아직 없음 → 켜진 뒤 첫 값을 씀)
float    imuRoll = 0, imuPitch = 0;    // 기억한 수평에서 기운 각도 [도]
uint8_t  imuAcc = 0;
uint32_t imuT = 0;                     // 마지막 샘플 받은 millis (0 = 아직 없음)

// #imu 는 초당 100줄이라 블루투스가 연결돼 있으면 블루투스로만, 아니면 USB 시리얼로만 보냄
void sendData(const char* line) {
  if (CTRL.hasClient()) CTRL.print(line); else Serial.print(line);
}

uint8_t imuAddr = 0;                   // 찾은 주소 (0 = 못 찾음)

// I2C 버스에서 대답하는 주소를 다 찾아서 출력 (인식 안 될 때 원인 찾기용)
void i2cScan(TwoWire& w, const char* name) {
  Serial.printf("[I2C] %s 스캔:", name);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    w.beginTransmission(a);
    if (w.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " (아무것도 없음 → 배선·전원·풀업 확인)");
}

bool imuBegin() {
  Wire1.begin(IMU_SDA, IMU_SCL, IMU_I2C_HZ);
  for (uint8_t a : IMU_ADDRS) {
    Wire1.beginTransmission(a);
    if (Wire1.endTransmission() != 0) continue;          // 그 주소엔 장치 없음
    if (bno.begin_I2C(a, &Wire1) && bno.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_US)) {
      imuAddr = a;
      return true;
    }
    Serial.printf("[IMU] 0x%02X 에 장치는 있는데 BNO08x 시작 실패\n", a);
  }
  return false;
}

// up0 → up 으로 기운 각도를, up0 에 수직인 두 축(e1, e2)으로 나눠서 [도]
//   e1 = 센서 x 축을 up0 에 수직으로 깎은 것 (x 가 up0 과 거의 나란하면 센서 y 축), e2 = up0 × e1
void tiltFromUp(float& a1, float& a2) {
  const float* u = up0;
  float ax[3] = { 1, 0, 0 };
  if (fabsf(u[0]) > 0.9f) { ax[0] = 0; ax[1] = 1; }
  float d = ax[0] * u[0] + ax[1] * u[1] + ax[2] * u[2];
  float e1[3] = { ax[0] - d * u[0], ax[1] - d * u[1], ax[2] - d * u[2] };
  float n = sqrtf(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
  for (int i = 0; i < 3; i++) e1[i] /= n;
  float e2[3] = { u[1] * e1[2] - u[2] * e1[1], u[2] * e1[0] - u[0] * e1[2], u[0] * e1[1] - u[1] * e1[0] };
  float c[3] = { u[1] * up[2] - u[2] * up[1], u[2] * up[0] - u[0] * up[2], u[0] * up[1] - u[1] * up[0] };   // 회전축 × sin
  float s = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
  float cosv = u[0] * up[0] + u[1] * up[1] + u[2] * up[2];
  float ang = atan2f(s, cosv) * RAD_TO_DEG;            // 기운 전체 각도
  float k = s > 1e-6f ? ang / s : RAD_TO_DEG;          // 축 성분 → 각도
  a1 = (c[0] * e1[0] + c[1] * e1[1] + c[2] * e1[2]) * k;
  a2 = (c[0] * e2[0] + c[1] * e2[1] + c[2] * e2[2]) * k;
}

void pollImu() {
  static uint32_t lastSend = 0, lastErr = 0, lastRetry = 0;
  uint32_t now = millis();
  if (!imuOk) {
    if (!imuStream) return;
    if (now - lastErr >= 1000) { sendData("#imu,err\n"); lastErr = now; }
    if (now - lastRetry >= 3000) { lastRetry = now; imuOk = imuBegin(); }   // 스트림을 켠 동안만 다시 시도
    return;
  }
  if (bno.wasReset()) bno.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_US);
  sh2_SensorValue_t v;
  while (bno.getSensorEvent(&v)) {
    if (v.sensorId != SH2_GAME_ROTATION_VECTOR) continue;
    float w = v.un.gameRotationVector.real, x = v.un.gameRotationVector.i;
    float y = v.un.gameRotationVector.j,    z = v.un.gameRotationVector.k;
    // 세상 위쪽(z)을 센서 좌표로 = 회전행렬 3번째 줄
    up[0] = 2 * (x * z - w * y);
    up[1] = 2 * (y * z + w * x);
    up[2] = 1 - 2 * (x * x + y * y);
    if (up0[0] == 0 && up0[1] == 0 && up0[2] == 0) memcpy(up0, up, sizeof(up));   // 기준이 없으면 켜진 자세를 수평으로
    float r, p;
    tiltFromUp(r, p);
    if (IMU_SWAP) { float t = r; r = p; p = t; }
    imuRoll = IMU_ROLL_SIGN * r;
    imuPitch = IMU_PITCH_SIGN * p;
    imuAcc = v.status & 3;
    imuT = now;
  }
  if (imuStream && imuT && now - lastSend >= (uint32_t)IMU_SEND_MS) {
    lastSend = now;
    char line[64];
    snprintf(line, sizeof(line), "#imu,%lu,%.2f,%.2f,%u,%lu\n", (unsigned long)now,
             imuRoll, imuPitch, imuAcc, (unsigned long)(now - imuT));
    sendData(line);
  }
}

void imuZero() {
  char line[64];
  if (!imuOk || !imuT) { sendData("#zero,err\n"); return; }
  memcpy(up0, up, sizeof(up));
  prefs.putFloat("up0x", up0[0]);
  prefs.putFloat("up0y", up0[1]);
  prefs.putFloat("up0z", up0[2]);
  snprintf(line, sizeof(line), "#zero,%.3f,%.3f,%.3f\n", up0[0], up0[1], up0[2]);
  sendData(line);
}

// ------------------------------- 환경 센서 (온도·습도 / 연기 / 불꽃) -------------------------------
// ★ 하드웨어 미장착 상태. 0 이면 아래 #else 쪽 빈 함수만 컴파일되어 기존 동작에 아무 영향 없음
//   센서를 달고 1 로 바꾸려면 라이브러리 매니저에서 "DHT sensor library"(Adafruit) 설치 필요
#define ENV_SENSORS_ENABLED 0

#if ENV_SENSORS_ENABLED
#include <DHT.h>

const uint8_t  ENV_DHT_PIN    = 27;
const uint8_t  ENV_MQ2_PIN    = 34;       // ADC1 (블루투스/와이파이 켜져 있어도 ADC2 와 달리 읽힘)
const uint8_t  ENV_FLAME_PIN  = 35;       // 불꽃 감지 시 LOW (모듈 DO 출력)
const uint8_t  ENV_BUZZER_PIN = 25;
const int      ENV_SEND_MS    = 1000;     // #env 1Hz
const int      ENV_DHT_MS     = 2000;     // DHT22 는 2초보다 빨리 읽으면 NaN
const int      MQ2_WARMUP_MS  = 20000;    // MQ-2 히터 예열 (실제론 24시간 번인 권장)

// 경보 기준
const float    TEMP_ALARM_C     = 60.0f;  // 서보·배터리 주변 과열
const float    SMOKE_ALARM_PPM  = 300.0f;
const float    ALARM_HYST       = 0.9f;   // 해제는 기준의 90% 아래로 내려가야

// MQ-2 변환 (데이터시트 LPG/연기 곡선 근사: ppm = A * (Rs/R0)^B)
const float    MQ2_RL_KOHM   = 5.0f;      // 모듈 부하 저항
const float    MQ2_R0_KOHM   = 9.83f;     // 깨끗한 공기에서 Rs/R0 = 9.83 → 부팅 때 보정해서 덮어씀
const float    MQ2_CURVE_A   = 3616.1f;
const float    MQ2_CURVE_B   = -2.675f;
const float    ADC_VREF      = 3.3f;
const int      ADC_MAX       = 4095;

enum { ENV_ALM_TEMP = 1, ENV_ALM_SMOKE = 2, ENV_ALM_FLAME = 4, ENV_ALM_FAULT = 8 };

DHT dht(ENV_DHT_PIN, DHT22);
float    envTemp = NAN, envHum = NAN, envSmokePpm = 0, mq2R0 = MQ2_R0_KOHM;
int      envSmokeRaw = 0;
bool     envFlame = false;
uint8_t  envAlarm = 0;
uint32_t envBootMs = 0;

float mq2Rs(int raw) {
  float v = raw * ADC_VREF / ADC_MAX;
  if (v < 0.01f) v = 0.01f;
  return MQ2_RL_KOHM * (ADC_VREF - v) / v;
}

float mq2Ppm(int raw) {
  float ratio = mq2Rs(raw) / mq2R0;
  return MQ2_CURVE_A * powf(ratio, MQ2_CURVE_B);
}

// 깨끗한 공기에서 R0 보정 (50번 평균)
void mq2Calibrate() {
  float sum = 0;
  for (int i = 0; i < 50; i++) { sum += mq2Rs(analogRead(ENV_MQ2_PIN)); delay(20); }
  mq2R0 = (sum / 50) / 9.83f;
  Serial.printf("[ENV] MQ-2 R0 = %.2f kΩ\n", mq2R0);
}

void envBegin() {
  pinMode(ENV_FLAME_PIN, INPUT);
  pinMode(ENV_BUZZER_PIN, OUTPUT);
  digitalWrite(ENV_BUZZER_PIN, LOW);
  analogSetPinAttenuation(ENV_MQ2_PIN, ADC_11db);   // 0~3.3V 전 범위
  dht.begin();
  envBootMs = millis();
  mq2Calibrate();
  Serial.println("[ENV] 환경 센서 시작 (MQ-2 예열 중 — 20초 동안 연기 경보 무시)");
}

void updateAlarm() {
  uint8_t a = envAlarm;
  if (!isnan(envTemp)) {
    if (envTemp >= TEMP_ALARM_C) a |= ENV_ALM_TEMP;
    else if (envTemp < TEMP_ALARM_C * ALARM_HYST) a &= ~ENV_ALM_TEMP;
  }
  bool warm = millis() - envBootMs >= MQ2_WARMUP_MS;
  if (warm && envSmokePpm >= SMOKE_ALARM_PPM) a |= ENV_ALM_SMOKE;
  else if (envSmokePpm < SMOKE_ALARM_PPM * ALARM_HYST) a &= ~ENV_ALM_SMOKE;
  if (envFlame) a |= ENV_ALM_FLAME; else a &= ~ENV_ALM_FLAME;
  if (isnan(envTemp) || isnan(envHum)) a |= ENV_ALM_FAULT; else a &= ~ENV_ALM_FAULT;

  if ((a & ~ENV_ALM_FAULT) && !(envAlarm & ~ENV_ALM_FAULT)) LOG("[ENV] ★ 경보 발생 0x%02X\n", a);
  envAlarm = a;
  digitalWrite(ENV_BUZZER_PIN, (envAlarm & (ENV_ALM_SMOKE | ENV_ALM_FLAME)) ? HIGH : LOW);
}

void sendEnv() {
  char line[96];
  snprintf(line, sizeof(line), "#env,%lu,%.1f,%.1f,%.0f,%d,%d,%u\n", (unsigned long)millis(),
           envTemp, envHum, envSmokePpm, envSmokeRaw, envFlame ? 1 : 0, envAlarm);
  sendData(line);
}

void pollEnv() {
  static uint32_t lastDht = 0, lastSend = 0;
  uint32_t now = millis();
  if (now - lastDht >= (uint32_t)ENV_DHT_MS) {
    lastDht = now;
    envTemp = dht.readTemperature();
    envHum  = dht.readHumidity();
  }
  // MQ-2 는 노이즈가 커서 지수 이동 평균
  envSmokeRaw = (envSmokeRaw * 7 + analogRead(ENV_MQ2_PIN)) / 8;
  envSmokePpm = mq2Ppm(envSmokeRaw);
  envFlame    = digitalRead(ENV_FLAME_PIN) == LOW;
  updateAlarm();
  if (now - lastSend >= (uint32_t)ENV_SEND_MS) { lastSend = now; sendEnv(); }
}

bool envCommand(char c) {
  if (c == 'e') { sendEnv(); return true; }
  return false;
}

#else   // ENV_SENSORS_ENABLED == 0 : 아무것도 안 함
void envBegin() {}
void pollEnv() {}
bool envCommand(char) { return false; }
#endif

// 한 글자 명령 (o 는 각 poll 함수에서 따로 처리). 처리했으면 true
// ------------------------------- 핀 스캔 (배선 확인용) -------------------------------
// 서보 I2C(21·22), IMU I2C(32·33), USB 시리얼(1·3), 플래시(6~11), 부팅 버튼(0)은 뺌
const uint8_t SCAN_PINS[] = { 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27, 34, 35, 36, 39 };
const int SCAN_N = sizeof(SCAN_PINS);
bool pinScan = false;
uint8_t scanPrev[SCAN_N];

void pinScanStart() {
  pinScan = true;
  LOG("[핀] 스캔 시작 (34·35·36·39 는 내부 풀업이 없어서 버튼만 달면 값이 떠 있음)\n[핀] 지금 값:");
  for (int i = 0; i < SCAN_N; i++) {
    uint8_t g = SCAN_PINS[i];
    if (g < 34) pinMode(g, INPUT_PULLUP); else pinMode(g, INPUT);
  }
  delay(5);
  for (int i = 0; i < SCAN_N; i++) {
    scanPrev[i] = digitalRead(SCAN_PINS[i]);
    LOG(" %d=%d", SCAN_PINS[i], scanPrev[i]);
  }
  LOG("\n[핀] 발 버튼 배정: 다리1=%d 다리2=%d 다리3=%d 다리4=%d 다리5=%d 다리6=%d — 버튼을 하나씩 눌러 볼 것\n",
      FOOT_PIN[0], FOOT_PIN[1], FOOT_PIN[2], FOOT_PIN[3], FOOT_PIN[4], FOOT_PIN[5]);
}

void pollPinScan() {
  if (!pinScan) return;
  static uint32_t last = 0;
  if (millis() - last < 20) return;
  last = millis();
  for (int i = 0; i < SCAN_N; i++) {
    uint8_t v = digitalRead(SCAN_PINS[i]);
    if (v != scanPrev[i]) {
      int leg = -1;
      for (int l = 0; l < 6; l++) if (FOOT_PIN[l] == SCAN_PINS[i]) leg = l + 1;
      if (leg > 0) LOG("[핀] GPIO %d: %d→%d  (다리%d 버튼 자리)\n", SCAN_PINS[i], scanPrev[i], v, leg);
      else LOG("[핀] GPIO %d: %d→%d  (발 버튼 자리가 아닌 핀! 이 버튼은 여기 꽂혀 있음)\n", SCAN_PINS[i], scanPrev[i], v);
      scanPrev[i] = v;
    }
  }
}

bool imuCommand(char c) {
  if (c == 'i') { imuStream = true; return true; }
  if (c == 'k') { imuStream = false; return true; }
  if (c == 'z') { imuZero(); return true; }
  if (c == 'p') {
    if (pinScan) { pinScan = false; LOG("[핀] 스캔 끔\n"); } else pinScanStart();
    return true;
  }
  return envCommand(c);
}

// ------------------------------- 입력 -------------------------------
char  pkt[40];
int   pktLen = -1;

// true면 받은 명령마다 로그 출력. 걸을 땐 초당 수백 줄이라 시리얼이 막히고 #ft 가 늦어짐 → 디버깅할 때만 켤 것
const bool LOG_MOVES = false;

void moveTo(int l, int tfh, float d) {
  cur[l][tfh] = d;
  if (LOG_MOVES) {
    static const char* NAMES[3] = { "hip", "femur", "tibia" };
    LOG("LEG:%d, %s, DEG:%.1f\n", l, NAMES[tfh], d);
  }
}

// bluetooth communication
void pollCtrl() {
  // 1초마다 브리지에 "살아있음" 신호 (브리지가 로봇 멈춤/재부팅을 알아챔)
  static uint32_t lastHb = 0;
  if (millis() - lastHb > 1000 && CTRL.hasClient()) { CTRL.print("#hb\n"); lastHb = millis(); }
  while (CTRL.available()) {
    char c = CTRL.read();
    if (c == 'o') { allOff(); LOG("> 전부 힘 뺌 (다시 일어나려면 조종기 버튼)\n"); continue; }
    if (pktLen < 0 && imuCommand(c)) continue;
    if (c == '<') { pktLen = 0; continue; }
    if (pktLen < 0) continue;
    if (c == '>') {
      pkt[pktLen] = 0;
      int l, tfh; float d;   // 각도는 소수 (PC가 0.1° 단위로 보냄. 예전엔 %d 라 소수점 아래를 버렸음)
      if (sscanf(pkt, "%d,%d,%f", &l, &tfh, &d) == 3) {
        lastPacket = millis();
        // 입력된 값으로 이동
        if (l>=0 && l<6 && tfh>=0 && tfh<3) moveTo(l, tfh, d);
      }
      pktLen = -1;
      powered = true;
      continue;
    }
    if (pktLen < (int)sizeof(pkt) - 1) pkt[pktLen++] = c; else pktLen = -1;
  }
}

void pollSerial() {
  // 1초마다 브리지에 "살아있음" 신호 (브리지가 로봇 멈춤/재부팅을 알아챔)
  static uint32_t lastHb = 0;
  if (millis() - lastHb > 1000) { Serial.print("#hb\n"); lastHb = millis(); }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'o') { allOff(); LOG("> 전부 힘 뺌 (다시 일어나려면 조종기 버튼)\n"); continue; }
    if (pktLen < 0 && imuCommand(c)) continue;
    if (c == '<') { pktLen = 0; continue; }
    if (pktLen < 0) continue;
    if (c == '>') {
      pkt[pktLen] = 0;
      int l, tfh; float d;   // 각도는 소수 (PC가 0.1° 단위로 보냄. 예전엔 %d 라 소수점 아래를 버렸음)
      if (sscanf(pkt, "%d,%d,%f", &l, &tfh, &d) == 3) {
        lastPacket = millis();
        // 입력된 값으로 이동
        if (l>=0 && l<6 && tfh>=0 && tfh<3) moveTo(l, tfh, d);
      }
      pktLen = -1;
      powered = true;
      continue;
    }
    if (pktLen < (int)sizeof(pkt) - 1) pkt[pktLen++] = c; else pktLen = -1;
  }
}

bool ctrlAlive() { return lastPacket != 0 && millis() - lastPacket < FAILSAFE_MS; }

// ------------------------------- 메인 -------------------------------
void setup() {
  Serial.begin(115200);
  if (!CTRL.begin(BT_NAME)) Serial.println("★ 블루투스 시작 실패");
  else {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    Serial.printf("블루투스 '%s' 대기 중\n", BT_NAME);
    Serial.printf("블루투스 주소: %02X:%02X:%02X:%02X:%02X:%02X   ← 리눅스 브리지 --robot 에 넣기\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  Wire.begin();
  Wire.setClock(400000);   // 서보 18개 갱신을 빠르게 (100kHz면 한 번에 약 11ms)
  delay(100);

  pwmU1.begin(); pwmU1.setOscillatorFrequency(27000000); pwmU1.setPWMFreq(SERVO_FREQ);
  pwmU2.begin(); pwmU2.setOscillatorFrequency(27000000); pwmU2.setPWMFreq(SERVO_FREQ);
  delay(10);
  allOff();

  for (int i=0; i<6; i++) for (int j=0; j<3; j++) cur[i][j] = REF_ANG[j];

  for (int l = 0; l < 6; l++) pinMode(FOOT_PIN[l], INPUT_PULLUP);
  footRaw = footState = readFootRaw();

  prefs.begin("hexapod", false);
  up0[0] = prefs.getFloat("up0x", 0);              // 예전 roll0/pitch0 기준은 안 씀 (--zero 로 다시 맞출 것)
  up0[1] = prefs.getFloat("up0y", 0);
  up0[2] = prefs.getFloat("up0z", 0);
  Wire1.begin(IMU_SDA, IMU_SCL, IMU_I2C_HZ);
  delay(300);                          // BNO08x 는 켜지고 부팅하는 데 시간이 걸림
  i2cScan(Wire, "서보 버스 (SDA 21 / SCL 22, 0x40·0x41 이 보여야 함)");
  i2cScan(Wire1, "IMU 버스 (SDA 32 / SCL 33, 0x4A 또는 0x4B 가 보여야 함)");
  imuOk = imuBegin();
  if (imuOk) Serial.printf("IMU(BNO08x) OK 주소 0x%02X, 수평 기준(위쪽) %.3f %.3f %.3f%s\n", imuAddr, up0[0], up0[1], up0[2],
                           (up0[0] == 0 && up0[1] == 0 && up0[2] == 0) ? " → 저장된 기준 없음, 켜진 자세를 수평으로 씀 (--zero 권장)" : "");
  else Serial.println("★ IMU(BNO08x) 못 찾음 — 위 [I2C] 스캔 결과 확인");

  envBegin();   // ENV_SENSORS_ENABLED=0 이면 빈 함수

  Serial.println("\n조종기 아무 버튼(또는 USB 시리얼 아무 글자) → 서기. 다리 띄웠는지 확인!");
}

void loop() {
  pollSerial();
  pollCtrl();
  pollFoot();   // 힘 빠진 상태에서도 접지 상태는 계속 보냄
  pollImu();    // IMU 도 힘과 상관없이 계속
  pollEnv();    // 환경 센서 (비활성 시 빈 함수)
  pollPinScan();

  static bool wasConnected = false;
  bool conn = CTRL.hasClient();
  if (conn && !wasConnected) LOG("> 블루투스 연결됨. %s\n", powered ? "조종 가능" : "조종기 아무 버튼 → 서기");
  wasConnected = conn;

  // power control
  if (!powered) {
    allOff();
    delay(5);
    return;
  }

  writeAll();
}
