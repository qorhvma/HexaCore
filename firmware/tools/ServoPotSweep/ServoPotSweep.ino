#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwmU1(0x41);
Adafruit_PWMServoDriver pwmU2(0x40);

#define SERVO_FREQ 50

// 펄스 범위 (50Hz 기준 약 1.0ms ~ 2.0ms)
const uint16_t PULSE_MIN = 205/2;
const uint16_t PULSE_MID = 307;   // 가운데(중립) 위치
const uint16_t PULSE_MAX = 410*2;
const int potPin = 33;
const float COUNT_PER_DEG = (PULSE_MAX - PULSE_MIN) / 90.0;  // 1도당 펄스 값

void sweep() {
  Serial.println("START");
  for (int i=PULSE_MIN; i<=PULSE_MAX; i++) {
    pwmU1.setPWM(0, 0, i);
    Serial.print("Deg: ");
    Serial.println(map(i, PULSE_MIN, PULSE_MAX, 0, 360));
    Serial.print("Pot: ");
    Serial.println(analogRead(potPin));
    delay(100);
  }
  Serial.println("END");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  pwmU1.begin();
  pwmU1.setOscillatorFrequency(27000000);
  pwmU1.setPWMFreq(SERVO_FREQ);

  pwmU2.begin();
  pwmU2.setOscillatorFrequency(27000000);
  pwmU2.setPWMFreq(SERVO_FREQ);

  delay(10);
  Serial.println("READY");
}

void loop() {
  // 시리얼로 아무 바이트나 들어오면 스윕 1회 실행
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    sweep();
  }
}
