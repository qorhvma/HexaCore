# 5. 펌웨어 올리기

펌웨어: [`firmware/firmware/firmware.ino`](repo:firmware/firmware/firmware.ino)

## 준비

1. [Arduino IDE](https://www.arduino.cc/en/software)를 설치합니다.
2. 보드 매니저에서 **esp32** (Espressif Systems)를 설치합니다.
3. 라이브러리 매니저에서 설치합니다.
    - Adafruit PWM Servo Driver Library
    - Adafruit BNO08x
4. 보드는 **ESP32 Dev Module**을 고릅니다. 펌웨어가 클래식 블루투스(`BluetoothSerial`)를 쓰므로 블루투스 클래식이 있는 ESP32여야 합니다.

## 올리기

1. `firmware/firmware/firmware.ino`를 엽니다.
2. ESP32를 USB로 연결하고 포트를 고른 뒤 업로드합니다.
3. 시리얼 모니터를 **115200**, 줄 끝 **Newline**으로 엽니다.

켜지면 두 I2C 버스를 스캔한 결과가 나옵니다.

- 서보 버스(SDA 21 / SCL 22)에 `0x40`, `0x41`이 보여야 합니다.
- IMU 버스(SDA 32 / SCL 33)에 `0x4A` 또는 `0x4B`가 보여야 합니다.

!!! warning "처음 켤 때"
    몸통을 들고 다리를 공중에 띄운 상태로 켜세요.

## 시리얼 명령

USB 시리얼과 블루투스(이름 `Hyperism-HEXAPODv1`) 양쪽에서 받습니다.

| 명령 | 동작 |
|---|---|
| `o` | 서보 전부 힘 빼기 |
| `i` / `k` | IMU 스트림 켜기 / 끄기 (`#imu,...` 100 Hz) |
| `z` | 지금 자세를 수평으로 기억 (플래시에 저장) |
| `p` | 핀 스캔 켜기/끄기 — 발 스위치가 어느 핀에 붙었는지 찾을 때 |

라즈베리파이는 `<다리,관절,각도>` 형식으로 관절 명령을 보냅니다.

!!! warning "통신이 끊겨도 펌웨어는 멈추지 않습니다"
    `firmware.ino`에는 명령이 2초 끊기면 멈추려는 함수(`ctrlAlive`)가 있지만 아직 어디서도 부르지 않습니다.
    지금은 USB가 빠지거나 라즈베리파이가 멈추면 서보가 마지막 자세를 그대로 유지합니다.

## 다음 펌웨어

[`firmware/myFirmware/`](repo:firmware/myFirmware/)는 최종판으로 준비 중인 펌웨어입니다(Wi-Fi 설정을 BLE로 받음). 지금 가이드는 `firmware/firmware/` 기준입니다.
