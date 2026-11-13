# 캘리브레이션 표

`calibration.json` 하나에 로봇 소프트웨어가 쓰는 캘리브레이션 값을 모았다.
파이의 보행(`software/raspberrypi/leg_model.py`, `ik_gait.py`)은 이 표만 읽는다.

| 묶음 | 내용 | 원본 |
|---|---|---|
| `firmware` | 틱 한계 `tick_hard_min/max`, 변환 상수 `ref_ang_deg`·`cal_ang_deg`·`angle_gain`·`ticks_per_deg` | `firmware/firmware/firmware.ino` |
| `servos` (18개) | 다리·관절별 PCA9685 주소·채널, `t_cal`, `dir`, 각도 보정 `joint_ofs_deg` | `firmware/firmware/firmware.ino` (`SV`, `JOINT_OFS`) |
| `kinematics` | 다리 길이, 관절 범위 `joint_limits_deg`, 좌우 부호 `hip_sign`, 서기 자세 `stand_pose_deg` | `legacy/bluetooth_pc/hexapod_sdk.py` |
| `kinematics.mounts` 등 | 다리 장착 위치·중립 방향, femur 축 높이 `coxa_dz_mm`, `hip_z_mm` | 최종 CAD URDF (2026-11-07), 원본 파일 없음 |
| `pose_calibration` | 관절 보정: 서보별 가변저항 곡선 `calib_pot`·`calib_tick`, 일자 자세 `extended_raw_deg`, 90° 이동량 `slider_per_90` | `legacy/bluetooth_pc/hexapod_pose_gui.py` |
| `joint_points` | **관절 측정점** (있는 관절은 `pose_calibration` 대신 이것): 실제 각도별로 그 자세를 만든 펌웨어 각도 | `data/calibration/joint_points.json` (`calibrate_joints.py`가 씀) |

각도는 두 가지다.

- **실제 관절 각도**: 180 = 다리를 수평 일자로 편 자세. `joint_limits_deg`, `stand_pose_deg`, 보행·IK 가 이 각도를 쓴다.
- **펌웨어 각도**: `<다리,관절,각도>`로 보내는 값. 실제 각도를 `pose_calibration`으로 바꾼 값이다
  (`hexapod_pose_gui.calibrated_joint` 와 같은 계산, `leg_model.to_firmware`). 예: 서기 자세 실제 [180, 135, 45] →
  다리1 펌웨어 [166.0, 133.2, 64.5].

관절 보정은 "실제 일자 자세(180,180,180)를 만드는 펌웨어 각도 `extended_raw_deg`"에서 시작해, 실제로 1° 움직일 때
`slider_per_90 / 90` 만큼 펌웨어 각도를 움직이고, 서보 각도↔틱은 가변저항 곡선(`calib_pot`)으로 바꾼다.
펌웨어가 펌웨어 각도를 틱으로 바꾸는 식은 아래와 같다:

```
servo = 90 + dir × (각도 + joint_ofs − ref_ang) × angle_gain
tick  = t_cal + (servo − cal_ang) × ticks_per_deg × (t_cal > tick_90_nominal ? −1 : +1),  tick_hard_min~max 로 자름
```

보행은 관절 보정 + 이 식으로 "펌웨어가 잘라 버리지 않는 실제 각도"를 계산해서, `joint_limits_deg`와 겹치는 범위 안에서만 움직인다.

## 관절 측정 (`calibrate_joints.py`)

다리마다 아래 자세를 버튼으로 맞추고 **기록(Enter)** 을 누른다. 기록할 때마다 `joint_points.json`과 `calibration.json`이 저장된다.

| 자세 | 맞출 모양 | 기록되는 값 |
|---|---|---|
| ① 쫙 편 자세 | hip 중립 방향, femur 수평, tibia 가 femur 와 일직선 | hip·femur·tibia 180 |
| ② tibia 90° 아래 | femur 수평 그대로, tibia 만 수직 아래 (femur 와 직각) | tibia 90 |
| ③ femur 90° 아래 | tibia 는 편 채로, femur 를 수직 아래 | femur 270 |
| ④ hip 앞으로 90° | 다리가 ① 방향에서 앞쪽(라이다 쪽)으로 90° | hip 270 |
| ⑤ hip 뒤로 90° | 다리가 ① 방향에서 뒤쪽으로 90° | hip 90 |

컴퓨터에서 블루투스로 (파이 화면이 없을 때):

```bash
python3 data/calibration/calibrate_joints.py --bt     # 이름 'Hyperism-HEXAPODv1' 로 ESP32 를 찾아 연결
```

- 파이의 `robot_main.py`는 끈다 (`pkill -f robot_main.py`). UART 와 블루투스로 같이 명령하면 서로 덮어쓴다.
- anaconda 파이썬처럼 블루투스 소켓이 없는 파이썬이면 `/usr/bin/python3`로 알아서 다시 실행한다.
- 안 보이면: ESP32 전원·펌웨어 확인, 휴대폰 등 다른 기기와 연결돼 있으면 끊기 (연결 중엔 검색에 안 나온다).
  연결이 거부되면 한 번 `bluetoothctl pair <주소>` → `bluetoothctl trust <주소>`.

파이 화면에서 (UART):

```bash
pkill -f robot_main.py                                              # 노드가 포트를 쓰고 있으면 끈다
python3 data/calibration/calibrate_joints.py --robot /dev/ttyAMA0   # 파이 데스크톱 터미널에서. SSH 면 ssh -X
```

- **몸통을 받침에 올려 다리가 아무 데도 닿지 않게** 하고 시작한다. 처음 명령에 서보 18개가 서기 자세로 켜진다.
- 버튼 −5 / −1 / −0.2 / +0.2 / +1 / +5 로 고른 다리의 관절을 움직인다 (보정 없는 펌웨어 각도). 빨간 이름이 이 자세에서 기록할 관절.
- 처음 값은 지금 보정으로 추정한 값이라 대개 조금만 움직이면 된다. 고르지 않은 다리는 서기 자세로 접혀 있다.
- ④⑤는 이웃 다리와 부딪히면 건너뛰어도 된다. 측정점이 1개뿐인 관절은 그 점 + pose_gui 기울기, 0개면 pose_gui 보정을 쓴다.
- 측정점은 실제 각도에 대해 한 방향이어야 한다 (예: tibia 90 을 180 보다 더 편 값으로 맞추면 거부).
- **확인: 쫙 편 자세 / 확인: 서기** 버튼으로 모든 다리를 지금 보정으로 움직여 6개가 같은 모양인지 본다.
- 점 사이는 서보 각도(가변저항 곡선) 공간에서 직선으로 잇는다 (pose_gui 와 같은 방식). 끝 점 밖은 끝 구간을 연장한다.

측정을 마치면 `calibration.json`·`joint_points.json`을 커밋·푸시하고, 파이에서 `git pull` 뒤 노드를 다시 켜면 보행에 반영된다.

## 확인 · 갱신

```bash
python3 data/calibration/check_calibration.py            # 표 = 원본인지 (다르면 종료 코드 1)
python3 data/calibration/check_calibration.py --update   # 원본에서 다시 읽어 표를 씀
```

`software/raspberrypi/gait_check.py`도 시작할 때 같은 대조를 하고, 다르면 실패로 끝난다.

## 재캘리브레이션 순서

1. `firmware/tools/ServoCalib`으로 서보를 재고 `firmware.ino`의 `SV`/`JOINT_OFS`/틱 한계를 고친 뒤 ESP32에 올린다.
   관절 보정(가변저항 곡선·일자 자세·90° 이동량)은 `hexapod_pose_gui.py`에서 맞추고 그 파일의 값을 고친다.
   다리 길이·관절 범위를 바꿨다면 `hexapod_sdk.py`를 고친다.
2. `check_calibration.py --update`로 표를 갱신한다 (`mounts`처럼 원본이 없는 값은 스크립트 안의 `MOUNTS`를 고친다).
3. 파이에서 `python3 gait_check.py`가 통과하는지 본다.

## 지금 값 (2026-11-08 펌웨어 기준)

- 틱 한계 82~488 (서보 5°~175°). 이 값은 아직 커밋 안 된 `firmware.ino` 변경에서 왔다 — ESP32에 올렸는지 확인할 것.
- 다리 길이 coxa 42.4 / femur 100.02 / tibia 137.5 mm, 관절 범위 hip 90~270, femur 62~270, tibia 23~180.
- 다리3(`tfh3`) tibia 보정 −10°.
- 관절 보정은 2026-11-08 22:43 `hexapod_pose_gui.py` (아직 커밋 안 된 `SLIDER_PER_90` 포함). 17번 모터(다리6 femur)는
  가변저항 값을 못 믿어 직선으로 쓴다 (`calib_straight_motors`).
