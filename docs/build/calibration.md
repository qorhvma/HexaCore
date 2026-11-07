# 6. 서보 보정

저가 서보는 같은 펄스를 줘도 개체마다 각도가 조금씩 다릅니다. 그래서 서보마다 기준값을 재서 펌웨어에 넣습니다.
도구: [`firmware/tools/`](repo:firmware/tools/). 모두 시리얼 115200, 줄 끝 Newline으로 씁니다.

!!! warning
    모든 단계는 몸통을 들고 다리를 공중에 띄운 채로 하세요.

## 1. 배선 점검 — MotorCheck

[`MotorCheck.ino`](repo:firmware/tools/MotorCheck/MotorCheck.ino)를 올립니다.

| 명령 | 동작 |
|---|---|
| `l` | I2C 스캔 (두 보드가 다 보이는지) |
| `m` | 다리·관절 ↔ 보드·채널 표 출력 |
| `x1t` | 다리 1 tibia 선택 (h = hip, f = femur, t = tibia) |
| `w` | 선택한 서보 천천히 왕복 |
| `A` | 다리 1 hip부터 18개를 하나씩 왕복 (아무 키나 누르면 중단) |
| `S` | 18개 전부 기준 자세로 |
| `O` | 32채널 전부 힘 빼기 |

`A`를 돌리면서 움직이는 서보가 [4단계 채널 표](assembly.md#다리-번호와-서보)와 맞는지 봅니다.
두 서보가 같이 움직이면 두 PCA9685의 주소가 같은 것입니다. 점퍼를 확인하세요.

## 2. 서보마다 범위 재기 — ServoCalib

[`ServoCalib.ino`](repo:firmware/tools/ServoCalib/ServoCalib.ino)를 올립니다.

| 명령 | 동작 |
|---|---|
| `m <0~17>` | 서보 선택. 번호 = 다리 인덱스(0~5) × 3 + 채널 순서(0 = tibia, 1 = femur, 2 = hip) |
| `t <tick>` | 틱 직접 지정 (307 ≈ 1.5 ms, 가운데) |
| `+` / `-`, `]` / `[` | ±10 틱, ±1 틱 |
| `n` / `x` | 지금 틱을 최소 / 최대로 저장 |
| `p` | 잰 값 출력 |

## 3. 펌웨어에 넣기

[`firmware.ino`](repo:firmware/firmware/firmware.ino)의 두 표를 고칩니다.

- **`SV` 표**: 서보마다 `{보드, 채널, 이름, tCal, 방향}`. `tCal`이 그 서보의 기준 틱, 방향은 `+1` 또는 `-1`입니다.
- **`JOINT_OFS` 표**: 다리·관절별 미세 보정(관절 각도, 도). `tCal`은 그대로 두고 여기서만 조금씩 맞춥니다.
  값을 줄이면 tibia는 무릎이 더 접히고 femur는 위로 듭니다. 반대로 가면 부호를 바꾸세요.

펌웨어는 켜질 때 서보마다 자세별 틱 표를 출력합니다. `★`가 붙은 값은 서보 한계에 걸려 움직이지 않는 값입니다.

## 4. IMU 수평 기억

로봇을 평평한 바닥에 세우고 시리얼로 `z`를 보냅니다. 이 자세가 roll = pitch = 0이 되고, 껐다 켜도 유지됩니다.

## (선택) 실제 관절 각도 재기 — ServoPotSweep

서보 안 포텐셔미터에 선을 납땜해 ESP32 ADC(GPIO 33)로 읽으면 실제 각도를 잴 수 있습니다.
[`ServoPotSweep.ino`](repo:firmware/tools/ServoPotSweep/ServoPotSweep.ino)가 펄스를 바꿔 가며 값을 기록하고,
팀이 잰 결과는 [`data/servo_pot_sweep/`](repo:data/servo_pot_sweep/)에 있습니다 (femur 부하 처짐 약 1.3°).
