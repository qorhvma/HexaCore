# 라즈베리파이 노드 (헥사포드 상위 제어기)

```
[PC 조종 / 대시보드] ⇄ WebSocket :8765 ⇄ [Raspberry Pi] ⇄ USB 시리얼 115200 ⇄ [ESP32 firmware/firmware/firmware.ino]
                                               ⇅ UDP 6101/6201
                                        [Unitree 4D LiDAR L2]
```

| 파일 | 역할 |
|---|---|
| `robot_main.py` | 메인 노드: 보행(tripod) 50Hz, 지도 갱신 5Hz, 안전 정지, WebSocket 서버 실행 |
| `ik_gait.py` | IK 삼각 보행 (기본). 발끝 궤적 → 역기구학, 캘리브레이션 범위·관절 속도·보폭 한계 안에서만 |
| `leg_model.py` | 캘리브레이션 표 `data/calibration/calibration.json` 을 읽어 관절 범위·FK/IK |
| `gait_check.py` | 보행 점검: 로봇 없이 시뮬레이션 검사, `--identify` 로 실물 다리 번호·방향 확인 |
| `firmware_link.py` | ESP32 펌웨어 제어: `<다리,관절,각도>` 전송, `#hb/#ft/#imu/#env` 파싱 |
| `pc_link.py` | Pi ↔ PC 통신: WebSocket + JSON (텔레메트리 10Hz, 스캔 5Hz, 지도 1Hz) |
| `lidar_l2.py` | Unitree L2 제어: 회전 시작/정지, 작업 모드, 리셋, 스캔 수신 |
| `mapping.py` | 2D 점유 격자 지도 (log-odds), PGM 저장 |
| `../pc/pc_client.py` | PC 터미널 조종 클라이언트 (WASD) |
| `../dashboard/index.html` | 지도·센서 대시보드 (브라우저로 열기) |

## 설치

```bash
pip install -r requirements.txt
```

Pi 의 이더넷은 L2 와 같은 대역 고정 IP `192.168.1.2/24` 로 설정 (Unitree `unilidar_sdk2` 예제 `main.cpp` 와 동일, 라이다 IP 192.168.1.62).

## 실행

```bash
python3 robot_main.py                       # 실제 하드웨어 (없는 장치는 자동으로 시뮬레이션)
python3 robot_main.py --sim                 # 전부 시뮬레이션 — PC 에서도 동작
python3 robot_main.py --port /dev/serial0   # GPIO UART 로 ESP32 연결 시
```

### 보행

기본은 IK 보행(`ik_gait.py`)이다. 예전 관절 사인파 보행은 `python3 robot_main.py --gait sine`.

- 캘리브레이션 값은 전부 표 하나 [`data/calibration/calibration.json`](../../data/calibration/README.md) 에서 읽는다
  (펌웨어 `SV`·`JOINT_OFS`·틱 한계, `hexapod_sdk.py` 의 다리 길이·관절 범위·부호·서기 자세, CAD 다리 위치,
  `hexapod_pose_gui.py` 의 관절 보정). 보행은 실제 관절 각도로 계산하고, 보낼 때 관절 보정을 적용한다
  (서기 명령도 보정된 자세를 보낸다).
  **펌웨어나 hexapod_sdk.py 를 고치면 `python3 data/calibration/check_calibration.py --update` 로 표를 갱신할 것.**
  `gait_check.py` 가 표와 원본이 다르면 실패로 알려 준다.
- **서기 자세 자동 선택**: 캘리브레이션 때문에 기본 서기 자세(hexapod_sdk `STAND_POSE`)에서 다리를 들 여유가 없으면
  (보폭 < 60mm), 기본에 가장 가까우면서 걸을 수 있는 자세를 고른다 (femur 를 덜 들고 tibia 를 덜 접는 쪽). 시작 로그에
  `서기 자세 ... 자동 선택` 으로 나온다. 직접 정하려면 `HEXAPOD_STAND=150,65 python3 robot_main.py ...` (femur,tibia).
- 측정점을 잠시 끄려면 `HEXAPOD_JOINT_POINTS=0` (pose_gui 보정만 씀).

### 일어서기 · 몸 높이

- `stand` 명령(조이스틱 짧게 누름) = **일어서기**: 몸이 바닥에 앉은 높이(몸통 바닥 40mm + 3mm)에서 발을 제자리로
  천천히 옮긴 뒤(관절 60°/s), 발을 바닥에 붙인 채 몸을 25mm/s 로 서기 높이까지 들어 올린다 (약 3~4초).
- 서기 높이 기본 80mm (바닥 → 몸통 원점, 몸통 바닥이 바닥에서 40mm). `HEXAPOD_HEIGHT=90` 으로 바꾼다.
- 걷는 중에도 `{"cmd":"height","delta_mm":10}` / `{"cmd":"height","mm":100}` 로 바꾼다 (조이스틱 노랑/파랑).
  걸을 수 있는 높이 범위로 자른다 (시작 로그 `몸 높이: ... 조절 범위`). 텔레메트리 `gait.height` 로 지금 높이를 본다.
- 서기 높이에서 계산한 보폭 74.9mm, 직진 최고 약 0.15 m/s (노드 한계 MAX_VX 0.12 로 다시 잘림).
- 처음 쓰기 전에 (몸통을 받침에 올리고):

```bash
python3 gait_check.py                                  # 로봇 없이 안전 조건 검사 (통과해야 함)
python3 gait_check.py --identify --port /dev/ttyUSB0   # 다리 번호·hip 방향 가정 확인
```

`--identify` 에서 화면에 적힌 위치의 다리가 '앞으로 돌고 → 위로 들리지' 않으면 `leg_model.LAYOUT` 을 고친다.
가정: 펌웨어 다리 0 왼앞, 1 왼가운데, 2 왼뒤, 3 오른뒤, 4 오른가운데, 5 오른앞, hip 각도 + = 앞쪽.

PC 에서:

```bash
python3 software/pc/pc_client.py --host <pi-ip>
```

대시보드: `software/dashboard/index.html` 을 브라우저로 열고 상단에 `ws://<pi-ip>:8765` 입력 → 연결.
연결이 안 되면 브라우저 안 데모 모드로 동작합니다.

## 환경 센서 (미장착)

펌웨어의 `ENV_SENSORS_ENABLED` 가 `0` 이라 `#env` 줄이 오지 않습니다. 대시보드에는 "센서 미장착" 으로 표시됩니다.
DHT22(GPIO 27), MQ-2(GPIO 34), 불꽃 센서(GPIO 35), 부저(GPIO 25) 를 단 뒤 `1` 로 바꾸고
"DHT sensor library" 를 설치하면 켜집니다.

## 아직 안 된 것

- 라이다 패킷 디코더(`decode_point_packet`)는 SDK 구조체 오프셋을 반영하지 않은 틀만 있음 → 실제 L2 는 C++ SDK 연동 필요
- 위치 추정이 오도메트리뿐이라 오래 걸으면 지도가 밀림 (스캔 매칭 없음)
- 인증은 `HEXAPOD_TOKEN` 환경변수 토큰 비교뿐 (암호화 없음)
