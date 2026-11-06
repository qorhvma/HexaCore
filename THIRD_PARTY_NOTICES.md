# 외부 라이브러리와 라이선스

이 저장소에는 아래 라이브러리의 **소스가 들어 있지 않습니다**. 설치해서 쓰는 의존성이며, 각 라이브러리의 라이선스를 따릅니다.
(2026-11-07, 각 저장소의 LICENSE 파일 기준으로 확인. 법률 자문이 아닙니다.)

## ESP32 펌웨어 (`firmware/`)

| 라이브러리 | 용도 | 라이선스 |
|---|---|---|
| [arduino-esp32](https://github.com/espressif/arduino-esp32) (`Wire`, `WiFi`, `BluetoothSerial`, `Preferences`, `esp_mac`) | ESP32 보드 코어 | LGPL-2.1 |
| [Adafruit PWM Servo Driver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) | PCA9685 서보 드라이버 | BSD |
| [Adafruit BNO08x](https://github.com/adafruit/Adafruit_BNO08x) | BNO086 IMU | BSD (포함된 SH-2 드라이버는 Apache-2.0, Hillcrest Laboratories) |
| [DHT sensor library](https://github.com/adafruit/DHT-sensor-library) | 온습도 센서 (기본 비활성) | MIT |

## 라즈베리파이·PC (`software/`, `legacy/`, `tools/`, `data/`)

| 라이브러리 | 라이선스 |
|---|---|
| [NumPy](https://numpy.org) | BSD-3-Clause |
| [websockets](https://github.com/python-websockets/websockets) | BSD-3-Clause |
| [pySerial](https://github.com/pyserial/pyserial) | BSD-3-Clause |
| [Matplotlib](https://matplotlib.org) | Matplotlib License (PSF 기반) |
| [PyTorch](https://pytorch.org) | BSD-3-Clause |
| [ONNX](https://github.com/onnx/onnx) | Apache-2.0 |
| [ONNX Runtime](https://github.com/microsoft/onnxruntime) | MIT |
| [Unitree unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2) (L2 라이다, 연동 예정) | BSD-3-Clause |

## 강화학습 (`learning/`)

| 라이브러리 | 라이선스 |
|---|---|
| [Isaac Lab 3.0](https://github.com/isaac-sim/IsaacLab) | BSD-3-Clause (일부 Apache-2.0) |
| [Newton](https://github.com/newton-physics/newton) | Apache-2.0 |
| [RSL-RL](https://github.com/leggedrobotics/rsl_rl) | BSD-3-Clause |
| [Gymnasium](https://github.com/Farama-Foundation/Gymnasium) | MIT |
| [OpenUSD](https://github.com/PixarAnimationStudios/OpenUSD) (`pxr`) | Tomorrow Open Source Technology License 1.0 (수정 Apache-2.0) |
| [Isaac Sim](https://github.com/isaac-sim/IsaacSim) | `scripts/convert_urdf.py`와 선택 설치 항목(`isaacsim`)에서만 사용. 학습·평가에는 필요 없음. NVIDIA 라이선스 조건을 따름 |

## 하드웨어 설계 도구

| 도구 | 라이선스 |
|---|---|
| [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) | MIT |

## SLAM (연동 예정, 이 저장소에 포함하지 않음)

[Point-LIO](https://github.com/hku-mars/Point-LIO)는 최상위 LICENSE가 BSD-3-Clause지만, 함께 들어 있는 `include/IKFoM`은 GPL-2.0 LICENSE 파일을 가지고 있습니다.
그래서 **GPL-2.0으로 취급**합니다. 수정하지 않고 별도 패키지·별도 프로세스(ROS 토픽)로만 연결하며, 이 저장소의 코드와 섞지 않습니다.
