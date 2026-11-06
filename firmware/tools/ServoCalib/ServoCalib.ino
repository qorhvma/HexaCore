/*
모터별 최소/최대 틱 측정 (Serial 115200)
  m <0~17>  모터 선택 (leg*3 + joint)
  t <tick>  틱 직접 지정
  + / -     ±10 틱
  ] / [     ±1 틱
  n / x     현재 틱을 최소 / 최대로 저장
  p         측정 결과 출력
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

using namespace std;

#define SERVO_FREQ 50
#define TICK_LO 80   // 약 0.4ms
#define TICK_HI 620  // 약 3.0ms

Adafruit_PWMServoDriver pwmU1(0x41);
Adafruit_PWMServoDriver pwmU2(0x40);

int motor = 0, tick = 307;  // 307 = 약 1.5ms (중앙)
int minTick[18], maxTick[18];

void writeTick(int m, int t) {
    int leg = m / 3, joint = m % 3;
    if (leg <= 4) pwmU1.setPWM(joint + leg * 3, 0, t);
    else pwmU2.setPWM(joint, 0, t);
}

void setup() {
    Serial.begin(115200);
    for (auto* p : {&pwmU1, &pwmU2}) {
        p->begin();
        p->setOscillatorFrequency(27000000);
        p->setPWMFreq(SERVO_FREQ);
    }
    for (int i = 0; i < 18; i++) minTick[i] = maxTick[i] = -1;
    Serial.println("ServoCalib ready. m <0~17> 로 모터를 선택하세요.");
}

void loop() {
    if (!Serial.available()) return;
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.isEmpty()) return;

    char c = s[0];
    int v = s.substring(1).toInt();
    if (c == 'm' && v >= 0 && v < 18) { motor = v; tick = 307; }
    else if (c == 't') tick = v;
    else if (c == '+') tick += 10;
    else if (c == '-') tick -= 10;
    else if (c == ']') tick += 1;
    else if (c == '[') tick -= 1;
    else if (c == 'n') minTick[motor] = tick;
    else if (c == 'x') maxTick[motor] = tick;
    else if (c == 'p') {
        for (int i = 0; i < 18; i++)
            Serial.printf("motor %2d (leg %d, joint %d): min %4d, max %4d\n", i, i / 3, i % 3, minTick[i], maxTick[i]);
        return;
    }

    tick = constrain(tick, TICK_LO, TICK_HI);
    writeTick(motor, tick);
    Serial.printf("motor %d -> tick %d (min %d, max %d)\n", motor, tick, minTick[motor], maxTick[motor]);
}
