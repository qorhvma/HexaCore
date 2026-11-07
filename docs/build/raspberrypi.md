# 7. 라즈베리파이 설정

코드: [`software/raspberrypi/`](repo:software/raspberrypi/). Python 3가 있는 OS면 됩니다.

## 설치

```bash
git clone {{ repo_url }}.git
cd HexaCore
pip install -r software/raspberrypi/requirements.txt
```

## 라이다 네트워크

Unitree L2는 이더넷으로 연결합니다. 라즈베리파이 이더넷을 고정 IP **`192.168.1.2/24`**로 설정합니다 (라이다 기본 IP `192.168.1.62`).

## 실행

```bash
python3 software/raspberrypi/robot_main.py
```

| 옵션 | 내용 |
|---|---|
| (없음) | 실제 하드웨어. 연결되지 않은 장치는 자동으로 시뮬레이션 |
| `--sim` | 전부 시뮬레이션 — PC에서도 돕니다 |
| `--port /dev/serial0` | ESP32 시리얼 포트 (기본 `/dev/ttyUSB0`, 환경변수 `HEXAPOD_SERIAL`) |
| `--no-lidar` | 라이다 끄기 |
| `--ws-port 8765` | PC가 접속할 WebSocket 포트 |

환경변수 `HEXAPOD_TOKEN`을 정해 두면 같은 토큰을 가진 클라이언트만 조종할 수 있습니다 (암호화는 없습니다).

## 하는 일

- 삼각 보행(tripod) 50 Hz — 다리 1·3·5와 2·4·6이 번갈아 듭니다.
- 2D 점유 격자 지도 5 Hz
- 안전 정지 ([8단계](operate.md#안전-정지))
- PC에 텔레메트리 10 Hz, 스캔 5 Hz, 지도 1 Hz 전송

!!! note "아직 안 된 것"
    - 라이다 패킷 해석기가 틀만 있습니다. 실제 L2 데이터는 Unitree C++ SDK 연동이 필요합니다.
    - 위치 추정이 오도메트리뿐이라 오래 걸으면 지도가 밀립니다.
