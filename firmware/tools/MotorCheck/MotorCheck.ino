/* ============================================================================
   PCA9685 2개(0x41, 0x40) 에 달린 서보 18개 점검용

   채널 배치는 firmware/firmware/firmware.ino 의 SV 표와 동일
     0x41 : ch0~14  → 다리1~5 (tibia, femur, hip 순서로 3채널씩)
     0x40 : ch0~2   → 다리6
   기준 ticks(서기 자세) = SV 표의 tCal

   [USB 시리얼 115200, 줄 끝 개행(Newline) 선택]
     l        : I2C 스캔 (두 보드가 다 보이는지)
     m        : 다리/관절 ↔ 보드/채널 표 출력
     x1t      : 다리1 tibia 선택 (관절 h=hip f=femur t=tibia) → 기준 ticks 로 이동
     a40 c3   : 보드 주소·채널 직접 선택 (표에 없는 채널 확인용, 기준 307 = 서보 90°)
     t184     : 선택한 서보를 ticks 184 로
     + / -    : 선택한 서보 ±STEP_TICKS
     w        : 선택한 서보 천천히 왕복 시작/정지 (기준 ± SWEEP_TICKS)
     A        : 자동 점검 — 다리1 hip 부터 18개를 하나씩 왕복 후 힘 빼기 (아무 키나 누르면 중단)
     S        : 18개 전부 기준 ticks(서기 자세)로
     o        : 선택한 서보 힘 빼기
     O        : 두 보드 32채널 전부 힘 빼기

   ★ 몸통을 들고 다리를 공중에 띄운 채로 할 것.
   ★ 같은 주소를 가진 보드가 둘이면 스캔에는 하나로만 보이고 둘 다 같이 움직임 → 점퍼(A0~A5) 확인.
============================================================================ */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwmU1(0x41);
Adafruit_PWMServoDriver pwmU2(0x40);
Adafruit_PWMServoDriver* const BOARDS[2] = { &pwmU1, &pwmU2 };
const uint8_t BOARD_ADDR[2] = { 0x41, 0x40 };

#define SERVO_FREQ 50
const uint16_t TICK_HARD_MIN = 123;   // ≈600us
const uint16_t TICK_HARD_MAX = 492;   // ≈2400us
const uint16_t TICK_90       = 307;   // 표에 없는 채널의 기준
const uint16_t SWEEP_TICKS   = 30;    // 왕복 폭 (≈ 서보 ±13°)
const uint16_t STEP_TICKS    = 5;
const uint16_t SWEEP_STEP_MS = 15;    // 1 tick 이동 주기

enum { HIP = 0, FEMUR = 1, TIBIA = 2 };
const char JOINT_CH[3] = { 'h', 'f', 't' };

struct Sv {
  uint8_t board;   // BOARDS 인덱스
  uint8_t ch;
  const char* name;
  uint16_t tCal;   // 서기 자세 ticks
};

Sv SV[6][3] = {
  //  HIP                FEMUR              TIBIA
  { {0,  2, "tfh1-3", 177}, {0,  1, "tfh1-2", 234}, {0,  0, "tfh1-1", 184} }, // 다리1
  { {0,  5, "tfh2-3", 184}, {0,  4, "tfh2-2", 233}, {0,  3, "tfh2-1", 170} }, // 다리2
  { {0,  8, "tfh3-3", 182}, {0,  7, "tfh3-2", 251}, {0,  6, "tfh3-1", 171} }, // 다리3
  { {0, 11, "hft4-3", 191}, {0, 10, "hft4-2", 237}, {0,  9, "hft4-1", 183} }, // 다리4
  { {0, 14, "hft5-3", 193}, {0, 13, "hft5-2", 227}, {0, 12, "hft5-1", 166} }, // 다리5
  { {1,  2, "hft6-3", 195}, {1,  1, "hft6-2", 229}, {1,  0, "hft6-1", 179} }, // 다리6
};

bool     boardOk[2] = { false, false };
uint8_t  curBoard  = 0;
uint8_t  curCh     = 0;
uint16_t curCenter = 184;
uint16_t curTick   = 184;
bool     sweeping  = false;
int      sweepDir  = 1;

// ------------------------------- I2C -------------------------------
bool probe(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

void scanBoards() {
  Serial.print("[I2C] 전체 응답:");
  for (uint8_t a = 1; a < 127; a++) if (probe(a)) Serial.printf(" 0x%02X", a);
  Serial.println();
  for (int b = 0; b < 2; b++) {
    boardOk[b] = probe(BOARD_ADDR[b]);
    Serial.printf("[보드] 0x%02X %s\n", BOARD_ADDR[b], boardOk[b] ? "OK" : "없음 → 전원·SDA/SCL·주소 점퍼 확인");
  }
}

int boardIndex(uint8_t addr) {
  for (int b = 0; b < 2; b++) if (BOARD_ADDR[b] == addr) return b;
  return -1;
}

// ------------------------------- 서보 -------------------------------
void writeTick(uint8_t b, uint8_t ch, uint16_t t) {
  BOARDS[b]->setPWM(ch, 0, constrain(t, TICK_HARD_MIN, TICK_HARD_MAX));
}
void chOff(uint8_t b, uint8_t ch) { BOARDS[b]->setPWM(ch, 0, 4096); }   // full off = 서보 힘 빠짐
void allOff() {
  for (int b = 0; b < 2; b++) for (uint8_t ch = 0; ch < 16; ch++) chOff(b, ch);
}

void setCur(uint16_t t) {
  curTick = constrain(t, TICK_HARD_MIN, TICK_HARD_MAX);
  writeTick(curBoard, curCh, curTick);
}

const Sv* findSv(uint8_t b, uint8_t ch, int* leg = nullptr, int* joint = nullptr) {
  for (int l = 0; l < 6; l++) for (int j = 0; j < 3; j++)
    if (SV[l][j].board == b && SV[l][j].ch == ch) {
      if (leg) *leg = l;
      if (joint) *joint = j;
      return &SV[l][j];
    }
  return nullptr;
}

void printSel() {
  int l, j;
  const Sv* s = findSv(curBoard, curCh, &l, &j);
  if (s) Serial.printf("[선택] 다리%d %c (%s)  0x%02X ch%u  기준 %u  현재 %u\n",
                       l + 1, JOINT_CH[j], s->name, BOARD_ADDR[curBoard], curCh, curCenter, curTick);
  else   Serial.printf("[선택] 0x%02X ch%u (표에 없음)  기준 %u  현재 %u\n",
                       BOARD_ADDR[curBoard], curCh, curCenter, curTick);
}

void select(uint8_t b, uint8_t ch) {
  sweeping = false;
  curBoard = b;
  curCh = ch;
  const Sv* s = findSv(b, ch);
  curCenter = s ? s->tCal : TICK_90;
  setCur(curCenter);
  printSel();
}

void printMap() {
  Serial.println("다리  관절   이름     보드  ch  기준ticks");
  for (int l = 0; l < 6; l++) for (int j = 0; j < 3; j++) {
    const Sv& s = SV[l][j];
    Serial.printf(" %d    %c     %s   0x%02X  %2u  %u\n",
                  l + 1, JOINT_CH[j], s.name, BOARD_ADDR[s.board], s.ch, s.tCal);
  }
}

// 블로킹 이동. 시리얼에 아무 입력이 들어오면 중단하고 false
bool moveTo(uint8_t b, uint8_t ch, int from, int to) {
  int dir = (to > from) ? 1 : -1;
  for (int t = from; t != to; t += dir) {
    writeTick(b, ch, t);
    delay(SWEEP_STEP_MS);
    if (Serial.available()) return false;
  }
  writeTick(b, ch, to);
  return true;
}

void autoTest() {
  sweeping = false;
  Serial.println("[자동 점검] 시작 — 아무 키나 누르면 중단");
  for (int l = 0; l < 6; l++) for (int j = 0; j < 3; j++) {
    const Sv& s = SV[l][j];
    if (!boardOk[s.board]) {
      Serial.printf("  다리%d %c (%s) 건너뜀: 보드 0x%02X 없음\n", l + 1, JOINT_CH[j], s.name, BOARD_ADDR[s.board]);
      continue;
    }
    Serial.printf("  다리%d %c (%s)  0x%02X ch%u\n", l + 1, JOINT_CH[j], s.name, BOARD_ADDR[s.board], s.ch);
    int c = s.tCal;
    writeTick(s.board, s.ch, c);
    delay(300);
    bool ok = moveTo(s.board, s.ch, c, c + SWEEP_TICKS)
           && moveTo(s.board, s.ch, c + SWEEP_TICKS, c - SWEEP_TICKS)
           && moveTo(s.board, s.ch, c - SWEEP_TICKS, c);
    delay(200);
    chOff(s.board, s.ch);
    if (!ok) {
      while (Serial.available()) Serial.read();
      Serial.println("[자동 점검] 중단");
      return;
    }
    delay(300);
  }
  Serial.println("[자동 점검] 끝");
}

void standAll() {
  sweeping = false;
  for (int l = 0; l < 6; l++) for (int j = 0; j < 3; j++) {
    const Sv& s = SV[l][j];
    writeTick(s.board, s.ch, s.tCal);
    delay(50);                           // 한꺼번에 켜면 전류가 몰리므로 하나씩
  }
  Serial.println("[서기 자세] 18개 기준 ticks 출력");
}

// ------------------------------- 명령 -------------------------------
void printHelp() {
  Serial.println("명령: l 스캔 / m 표 / x1t 서보선택 / a40 보드 / c0 채널 / t184 ticks / +,- 미세조정");
  Serial.println("      w 왕복 / A 자동점검 / S 서기자세 / o 선택 힘빼기 / O 전부 힘빼기");
}

void handleLine(String s) {
  s.trim();
  if (!s.length()) return;
  char c = s[0];
  String arg = s.substring(1);
  if (c == 'l') scanBoards();
  else if (c == 'm') printMap();
  else if (c == 'x') {
    int l = arg.length() >= 2 ? arg[0] - '1' : -1;
    int j = -1;
    if (arg.length() >= 2) for (int k = 0; k < 3; k++) if (arg[1] == JOINT_CH[k]) j = k;
    if (l < 0 || l > 5 || j < 0) { Serial.println("예) x1h / x3f / x6t  (다리 1~6, 관절 h f t)"); return; }
    select(SV[l][j].board, SV[l][j].ch);
  } else if (c == 'a') {
    int b = boardIndex((uint8_t)strtol(arg.c_str(), nullptr, 16));
    if (b < 0) { Serial.println("보드 주소는 40 또는 41"); return; }
    select(b, curCh);
  } else if (c == 'c') {
    int ch = arg.toInt();
    if (ch < 0 || ch > 15 || !arg.length()) { Serial.println("채널은 0~15"); return; }
    chOff(curBoard, curCh);              // 이전 채널은 놓아줌
    select(curBoard, ch);
  } else if (c == 't') {
    sweeping = false;
    setCur(arg.toInt());
    printSel();
  } else if (c == '+' || c == '-') {
    sweeping = false;
    setCur(curTick + (c == '+' ? STEP_TICKS : -(int)STEP_TICKS));
    printSel();
  } else if (c == 'w') {
    sweeping = !sweeping;
    if (sweeping) setCur(curCenter);     // 기준에서 시작
    Serial.printf("[왕복] %s  0x%02X ch%u\n", sweeping ? "시작" : "정지", BOARD_ADDR[curBoard], curCh);
  } else if (c == 'A') autoTest();
  else if (c == 'S') standAll();
  else if (c == 'o') {
    sweeping = false;
    chOff(curBoard, curCh);
    Serial.println("[선택 서보 힘 뺌]");
  } else if (c == 'O') {
    sweeping = false;
    allOff();
    Serial.println("[전부 힘 뺌]");
  } else printHelp();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();                          // SDA 21 / SCL 22 (펌웨어와 동일)
  Wire.setClock(400000);
  delay(300);
  scanBoards();
  for (int b = 0; b < 2; b++) {
    BOARDS[b]->begin();
    BOARDS[b]->setOscillatorFrequency(27000000);
    BOARDS[b]->setPWMFreq(SERVO_FREQ);
  }
  delay(10);
  allOff();                              // 켜질 때는 전부 힘 뺀 상태
  printMap();
  printHelp();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleLine(line); line = ""; }
    else line += ch;
  }

  static uint32_t last = 0;
  if (sweeping && millis() - last >= SWEEP_STEP_MS) {
    last = millis();
    int t = (int)curTick + sweepDir;
    if (t >= (int)curCenter + SWEEP_TICKS) { t = curCenter + SWEEP_TICKS; sweepDir = -1; }
    if (t <= (int)curCenter - SWEEP_TICKS) { t = curCenter - SWEEP_TICKS; sweepDir = 1; }
    setCur(t);
  }
}
