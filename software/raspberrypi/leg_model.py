"""
다리 모델: 펌웨어 모터 캘리브레이션 + 최종 CAD(URDF) 치수 → 관절 허용 범위, 순기구학(FK), 역기구학(IK)

각도는 펌웨어 단위 [도] 그대로다 (`<다리,관절,각도>` 로 보내는 값).
    hip   : 180 = 다리가 CAD 중립 방향(LAYOUT 의 heading)으로 뻗음. 각도가 커지면 다리가 앞쪽으로 돈다 (양쪽 모두)
    femur : 180 = 수평. 작아질수록 위로 든다 (135 = 45° 위)
    tibia : femur 와 tibia 사이 안쪽 각도. 180 = 일자. 서기 자세 45 → tibia 가 거의 수직 아래
    (펌웨어 ANGLE_GAIN 0.8 = "관절 1도 → 서보 0.8도" 를 사용자가 맞춘 값이라 펌웨어 1도 = 실제 관절 1도로 본다)

좌표: 몸 기준 [mm]. x = 앞(라이다 쪽), y = 왼쪽, z = 위. 원점 = 몸통 원점 (URDF body).

★ 아직 실물로 확인하지 않은 가정 (gait_check.py --identify 로 확인할 것)
    - 펌웨어 다리 번호 ↔ 위치: 0 왼앞, 1 왼가운데, 2 왼뒤, 3 오른뒤, 4 오른가운데, 5 오른앞
      (지금 robot_main.py 의 MOUNT_ANGLE 60/90/120/-120/-90/-60 과 같은 순서. URDF 로는 L3 L2 L1 L6 L5 L4)
    - hip 각도 + = 앞쪽 (옛 hexapod_body.py 의 HIP_SIGN 과 같다)
"""

import math

NUM_LEGS, NUM_JOINTS = 6, 3
HIP, FEMUR, TIBIA = 0, 1, 2
STAND_POSE = [[180.0, 135.0, 45.0] for _ in range(NUM_LEGS)]   # firmware_link.STAND_POSE 와 같아야 함

# ═══════════════ 모터 캘리브레이션 (firmware/firmware/firmware.ino 와 똑같이 맞출 것) ═══════════════
# 2026-11-08 기준 값. 펌웨어를 고치면 여기도 고친다 (gait_check.py 가 서기 자세 틱을 찍어 주니 펌웨어 부팅 출력과 비교)
TICK_HARD_MIN, TICK_HARD_MAX = 82, 488          # 서보 5°~175° (0~180° = 틱 70~500 측정)
REF_ANG = (180.0, 180.0, 120.0)                 # 서보 90°일 때 관절 각도 (hip, femur, tibia)
CAL_ANG = (40.0, 65.0, 30.0)                    # 캘리브레이션 기준 서보 각도
ANGLE_GAIN = 0.8                                # 관절 1도 → 서보 0.8도
TICKS_PER_DEG = 11.111 / 4.889
TICK_90_NOMINAL = 307
# SV[다리][관절] = (tCal, dir)
SV = [
    [(177, +1), (234, +1), (184, +1)],   # 다리1 tfh1
    [(184, +1), (233, +1), (170, +1)],   # 다리2 tfh2
    [(182, +1), (251, +1), (171, +1)],   # 다리3 tfh3
    [(191, -1), (237, +1), (183, +1)],   # 다리4 hft4
    [(193, -1), (227, +1), (166, +1)],   # 다리5 hft5
    [(195, -1), (229, +1), (179, +1)],   # 다리6 hft6
]
JOINT_OFS = [
    [0, 0, 0], [0, 0, 0], [0, 0, -10], [0, 0, 0], [0, 0, 0], [0, 0, 0],
]
# 기구 한계 (옛 SDK JOINT_LIMITS: 다리끼리·몸통에 부딪히지 않는 범위). 캘리브레이션 범위와 겹치는 곳만 쓴다
MECH_LIMITS = [(90.0, 270.0), (62.0, 270.0), (23.0, 180.0)]
LIMIT_MARGIN = 3.0                              # 범위 끝은 이만큼 남긴다 [도] (틱 반올림·서보 오차)


def fw_ticks(l, j, ang):
    """펌웨어 rawTicks() 와 같은 계산 (클램프 전)."""
    t_cal, d = SV[l][j]
    ang += JOINT_OFS[l][j]
    servo = 90.0 + d * (ang - REF_ANG[j]) * ANGLE_GAIN
    sign = -1.0 if t_cal > TICK_90_NOMINAL else 1.0
    return t_cal + (servo - CAL_ANG[j]) * TICKS_PER_DEG * sign


def calib_range(l, j):
    """틱이 TICK_HARD_MIN~MAX 안에 드는 관절 각도 범위 (펌웨어가 잘라 버리지 않는 범위)."""
    t0, t1 = fw_ticks(l, j, 0.0), fw_ticks(l, j, 1.0)
    slope = t1 - t0
    a = (TICK_HARD_MIN - t0) / slope
    b = (TICK_HARD_MAX - t0) / slope
    return (min(a, b), max(a, b))


def _joint_limits():
    out = []
    for l in range(NUM_LEGS):
        row = []
        for j in range(NUM_JOINTS):
            c0, c1 = calib_range(l, j)
            m0, m1 = MECH_LIMITS[j]
            row.append((max(c0, m0) + LIMIT_MARGIN, min(c1, m1) - LIMIT_MARGIN))
        out.append(row)
    return out


JOINT_LIMITS = _joint_limits()


def in_limits(l, joints):
    return all(JOINT_LIMITS[l][j][0] <= joints[j] <= JOINT_LIMITS[l][j][1] for j in range(NUM_JOINTS))


# ═══════════════ 기하 (HexapodRobot/RLForHexapod/src/RLforHexapod/assets/urdf/robot.urdf, 2026-11-07 최종 CAD) ═══════════════
COXA_LEN = 42.4      # hip 축 → femur 축, 수평 바깥쪽 [mm]
COXA_DZ = -11.4      # femur 축이 hip 원점보다 이만큼 아래 [mm]
FEMUR_LEN = 100.0    # femur 축 → 무릎 축
TIBIA_LEN = 147.0    # 무릎 축 → 발끝 (다리 평면 안 성분. 옆으로 21mm 비켜나지만 femur 의 +20.8mm 와 상쇄)
HIP_Z = 4.0          # hip 원점 높이 (몸통 원점 기준)
# 펌웨어 다리 번호별 (hip x, hip y, 중립 방향[도], hip 부호). hip 부호 = 각도 + 일 때 위에서 본 회전 (반시계 +)
#   왼쪽 다리는 앞으로 돌면 시계방향(-), 오른쪽은 반시계(+)
LAYOUT = [
    (90.9, 54.7, 35.0, -1),     # 0 왼앞     (URDF L3)
    (0.0, 99.0, 90.0, -1),      # 1 왼가운데  (URDF L2)
    (-90.9, 54.8, 145.2, -1),   # 2 왼뒤     (URDF L1)
    (-90.9, -54.7, -144.0, +1), # 3 오른뒤   (URDF L6)
    (0.0, -99.0, -90.0, +1),    # 4 오른가운데 (URDF L5)
    (90.9, -54.7, -34.8, +1),   # 5 오른앞   (URDF L4)
]
LEG_NAMES = ["왼앞", "왼가운데", "왼뒤", "오른뒤", "오른가운데", "오른앞"]
TRIPODS = ((0, 2, 4), (1, 3, 5))   # 이웃 다리는 다른 조


def fk(l, joints):
    """관절 [hip, femur, tibia] → 발끝 (x, y, z) [mm]."""
    hip, fem, tib = joints
    mx, my, th, s = LAYOUT[l]
    psi = math.radians(th + (hip - 180.0) * s)
    a = math.radians(180.0 - fem)                 # femur 든 각도 (위 +)
    ta = a - math.pi + math.radians(tib)          # tibia 방향 (수평 기준)
    r = COXA_LEN + FEMUR_LEN * math.cos(a) + TIBIA_LEN * math.cos(ta)
    z = HIP_Z + COXA_DZ + FEMUR_LEN * math.sin(a) + TIBIA_LEN * math.sin(ta)
    return [mx + r * math.cos(psi), my + r * math.sin(psi), z]


def ik(l, foot, check_limits=True):
    """발끝 (x, y, z) → 관절 [hip, femur, tibia]. 못 닿거나 허용 범위 밖이면 None. 무릎이 위로 꺾인 해."""
    mx, my, th, s = LAYOUT[l]
    dx, dy = foot[0] - mx, foot[1] - my
    psi = math.atan2(dy, dx) - math.radians(th)
    psi = math.atan2(math.sin(psi), math.cos(psi))
    r = math.hypot(dx, dy) - COXA_LEN
    z = foot[2] - HIP_Z - COXA_DZ
    D = math.hypot(r, z)
    if not (abs(FEMUR_LEN - TIBIA_LEN) + 1.0 < D < FEMUR_LEN + TIBIA_LEN - 1.0):
        return None
    b = math.acos((FEMUR_LEN ** 2 + TIBIA_LEN ** 2 - D ** 2) / (2 * FEMUR_LEN * TIBIA_LEN))
    g = math.acos((FEMUR_LEN ** 2 + D ** 2 - TIBIA_LEN ** 2) / (2 * FEMUR_LEN * D))
    a = math.atan2(z, r) + g
    joints = [180.0 + math.degrees(psi) * s, 180.0 - math.degrees(a), math.degrees(b)]
    if check_limits and not in_limits(l, joints):
        return None
    return joints


def ik_reason(l, foot):
    """ik 가 None 일 때 이유 (점검용)."""
    j = ik(l, foot, check_limits=False)
    if j is None:
        return "다리 길이로 닿지 않음"
    names = ("hip", "femur", "tibia")
    bad = [f"{names[k]} {j[k]:.1f}° (허용 {JOINT_LIMITS[l][k][0]:.1f}~{JOINT_LIMITS[l][k][1]:.1f})"
           for k in range(NUM_JOINTS) if not JOINT_LIMITS[l][k][0] <= j[k] <= JOINT_LIMITS[l][k][1]]
    return ", ".join(bad) or "알 수 없음"
