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
