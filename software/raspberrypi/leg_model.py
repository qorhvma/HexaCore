"""
다리 모델: data/calibration/calibration.json (캘리브레이션 표) → 관절 허용 범위, 순기구학(FK), 역기구학(IK)

표 하나에 모든 값이 있다. 원본과 맞는지는 `python3 data/calibration/check_calibration.py` 로 확인한다.
    firmware   : 틱 한계·변환 상수            ← firmware/firmware/firmware.ino
    servos     : 서보별 tCal·dir·JOINT_OFS     ← firmware/firmware/firmware.ino
    kinematics : 다리 길이·관절 범위·좌우 부호·서기 자세 ← legacy/bluetooth_pc/hexapod_sdk.py
                 다리 장착 위치·방향 ← 최종 CAD URDF

각도는 두 가지다. 실제 관절 각도(IK·보행·관절 범위) 와 펌웨어 각도(`<다리,관절,각도>` 로 보내는 값).
    (아래는 '실제 관절 각도'. 펌웨어로 보낼 때 to_firmware() 로 바꾼다 = hexapod_pose_gui 의 보정)
    hip   : 180 = 다리가 중립 방향(mounts.heading_deg)으로 뻗음. 각도가 커지면 다리가 앞쪽으로 돈다 (양쪽 모두)
    femur : 180 = 수평. 작아질수록 위로 든다 (135 = 45° 위)
    tibia : femur 와 tibia 사이 안쪽 각도. 180 = 일자. 서기 자세 45 → tibia 가 거의 수직 아래

좌표: 몸 기준 [mm]. x = 앞(라이다 쪽), y = 왼쪽, z = 위. 원점 = 몸통 원점 (URDF body).

★ 아직 실물로 확인하지 않은 가정 (gait_check.py --identify 로 확인할 것)
    - 펌웨어 다리 번호 ↔ 위치: 0 왼앞, 1 왼가운데, 2 왼뒤, 3 오른뒤, 4 오른가운데, 5 오른앞 (표의 mounts 순서)
    - hip 각도 + = 앞쪽 (kinematics.hip_sign = hexapod_sdk.SIDE 를 '위에서 본 회전 부호'로 해석)
"""

import json
import math
import os

NUM_LEGS, NUM_JOINTS = 6, 3
HIP, FEMUR, TIBIA = 0, 1, 2
JOINT_NAMES = ("hip", "femur", "tibia")

TABLE_PATH = os.environ.get("HEXAPOD_CALIBRATION") or os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "calibration", "calibration.json"))


def load_table(path=TABLE_PATH):
    if not os.path.exists(path):
        raise FileNotFoundError(f"캘리브레이션 표가 없음: {path} (HexaCore 저장소 전체가 있어야 함)")
    with open(path, encoding="utf-8") as f:
        t = json.load(f)
    if len(t["servos"]) != NUM_LEGS * NUM_JOINTS or len(t["kinematics"]["mounts"]) != NUM_LEGS:
        raise ValueError(f"{path}: 서보 18개·다리 6개가 있어야 함")
    return t


CAL = load_table()
_fw, _kin = CAL["firmware"], CAL["kinematics"]

# ═══════════════ 모터 캘리브레이션 (표의 firmware / servos) ═══════════════
TICK_HARD_MIN, TICK_HARD_MAX = int(_fw["tick_hard_min"]), int(_fw["tick_hard_max"])
REF_ANG = tuple(_fw["ref_ang_deg"])           # 서보 90°일 때 관절 각도 (hip, femur, tibia)
CAL_ANG = tuple(_fw["cal_ang_deg"])           # 캘리브레이션 기준 서보 각도
ANGLE_GAIN = float(_fw["angle_gain"])         # 관절 1도 → 서보 몇 도
TICKS_PER_DEG = float(_fw["ticks_per_deg"])
TICK_90_NOMINAL = int(_fw["tick_90_nominal"])
SV = [[None] * NUM_JOINTS for _ in range(NUM_LEGS)]          # SV[다리][관절] = (tCal, dir)
JOINT_OFS = [[0.0] * NUM_JOINTS for _ in range(NUM_LEGS)]
for _s in CAL["servos"]:
    _l, _j = int(_s["leg"]), JOINT_NAMES.index(_s["joint"])
    SV[_l][_j] = (int(_s["t_cal"]), int(_s["dir"]))
    JOINT_OFS[_l][_j] = float(_s["joint_ofs_deg"])

# ═══════════════ 기구 (표의 kinematics) ═══════════════
COXA_LEN = float(_kin["coxa_len_mm"])         # hip 축 → femur 축, 수평 바깥쪽
FEMUR_LEN = float(_kin["femur_len_mm"])       # femur 축 → 무릎 축
TIBIA_LEN = float(_kin["tibia_len_mm"])       # 무릎 축 → 발끝
COXA_DZ = float(_kin["coxa_dz_mm"])           # femur 축이 hip 원점보다 이만큼 위(+) / 아래(-)
HIP_Z = float(_kin["hip_z_mm"])               # hip 원점 높이 (몸통 원점 기준)
HIP_SIGN = [int(v) for v in _kin["hip_sign"]]                 # 각도 + 일 때 위에서 본 회전 (반시계 +)
SDK_LIMITS = [[(float(a), float(b)) for a, b in leg] for leg in _kin["joint_limits_deg"]]
STAND_POSE = [[float(v) for v in leg] for leg in _kin["stand_pose_deg"]]
LAYOUT = [(float(m["x"]), float(m["y"]), float(m["heading_deg"])) for m in _kin["mounts"]]   # (hip x, hip y, 중립 방향)
LEG_NAMES = [m["name"] for m in _kin["mounts"]]
TRIPODS = ((0, 2, 4), (1, 3, 5))   # 이웃 다리는 다른 조


def fw_ticks(l, j, ang):
    """펌웨어 rawTicks() 와 같은 계산 (클램프 전)."""
    t_cal, d = SV[l][j]
    ang += JOINT_OFS[l][j]
    servo = 90.0 + d * (ang - REF_ANG[j]) * ANGLE_GAIN
    sign = -1.0 if t_cal > TICK_90_NOMINAL else 1.0
    return t_cal + (servo - CAL_ANG[j]) * TICKS_PER_DEG * sign


# ═══════════════ 관절 보정 (표의 pose_calibration = hexapod_pose_gui.calibrated_joint) ═══════════════
# IK·보행은 '실제 관절 각도'(180 = 다리를 수평 일자로 편 자세)로 계산하고, 보낼 때만 펌웨어 각도로 바꾼다.
_pc = CAL["pose_calibration"]
CALIB_TICK = tuple(float(t) for t in _pc["calib_tick"])          # 서보 0° / 90° / 180° 의 틱
CALIB_POT = [tuple(float(p) for p in row) for row in _pc["calib_pot"]]
CALIB_STRAIGHT = set(int(m) for m in _pc["calib_straight_motors"])   # 모터 번호 (1부터). 가변저항 없이 직선
EXTENDED_ANG = [float(a) for a in _pc["extended_ang_deg"]]
EXTENDED_RAW = [[float(a) for a in row] for row in _pc["extended_raw_deg"]]
SLIDER_PER_90 = [[float(a) for a in row] for row in _pc["slider_per_90"]]


def _motor(l, j):
    """모터 번호 - 1 = 다리 × 3 + 채널 (채널 0 tibia, 1 femur, 2 hip)."""
    return l * 3 + 2 - j


def servo_to_tick(motor, servo_deg):
    """motor(0~17)를 실제 서보 각도 servo_deg 로 보내는 틱 (가변저항 곡선)."""
    t0, t90, t180 = CALIB_TICK
    if motor + 1 in CALIB_STRAIGHT:
        return t0 + servo_deg * (t180 - t0) / 180
    p0, p90, p180 = CALIB_POT[motor]
    pot = p0 + (p180 - p0) * servo_deg / 180
    if pot <= p90:
        return t0 + (pot - p0) * (t90 - t0) / (p90 - p0)
    return t90 + (pot - p90) * (t180 - t90) / (p180 - p90)


def tick_to_servo(motor, tick):
    """servo_to_tick 의 역변환."""
    t0, t90, t180 = CALIB_TICK
    if motor + 1 in CALIB_STRAIGHT:
        return (tick - t0) * 180 / (t180 - t0)
    p0, p90, p180 = CALIB_POT[motor]
    if tick <= t90:
        pot = p0 + (tick - t0) * (p90 - p0) / (t90 - t0)
    else:
        pot = p90 + (tick - t90) * (p180 - p90) / (t180 - t90)
    return (pot - p0) * 180 / (p180 - p0)


def _fw_from_tick(l, j, tick):
    """firmware rawTicks() 의 역변환: 틱 → 펌웨어 각도."""
    t_cal, d = SV[l][j]
    sign = -1.0 if t_cal > TICK_90_NOMINAL else 1.0
    fw_servo = CAL_ANG[j] + (tick - t_cal) / (TICKS_PER_DEG * sign)
    return REF_ANG[j] - JOINT_OFS[l][j] + d * (fw_servo - 90.0) / ANGLE_GAIN


def _extended_servo(l, j):
    return tick_to_servo(_motor(l, j), fw_ticks(l, j, EXTENDED_RAW[l][j]))


# ─── 측정점 (data/calibration/joint_points.json → 표의 joint_points) ───
# 다리·관절마다 "실제 각도 → 그 자세를 만든 펌웨어 각도" 를 직접 잰 점들. 있으면 pose_gui 보정보다 우선한다.
# 점 사이는 서보 각도(가변저항 곡선) 공간에서 직선으로 잇고, 범위 밖은 끝 구간을 연장한다.
#   2점 이상: 측정점만으로.  1점: 그 점 + pose_gui 의 SLIDER_PER_90 기울기.  0점: pose_gui 보정 그대로.
JOINT_POINTS = [[[] for _ in range(NUM_JOINTS)] for _ in range(NUM_LEGS)]   # [다리][관절] = [(실제, 서보), ...]


def raw_to_servo(l, j, raw_deg):
    """펌웨어 각도 → 실제 서보 각도 (펌웨어 틱 변환 + 가변저항 곡선)."""
    return tick_to_servo(_motor(l, j), fw_ticks(l, j, raw_deg))


def set_joint_points(legs):
    """측정점을 바꾼다. legs = [ {"hip": {"180": 펌웨어각, ...}, "femur": {...}, "tibia": {...}} × 6 ] 또는 None.
    관절 범위(JOINT_LIMITS)도 다시 계산한다. 측정점이 실제 각도에 대해 한 방향으로 가지 않으면 ValueError."""
    global JOINT_POINTS, JOINT_LIMITS
    pts = [[[] for _ in range(NUM_JOINTS)] for _ in range(NUM_LEGS)]
    for l, leg in enumerate(legs or []):
        for j, name in enumerate(JOINT_NAMES):
            for k, raw in (leg or {}).get(name, {}).items():
                pts[l][j].append((float(k), raw_to_servo(l, j, float(raw))))
            pts[l][j].sort()
            s = [p[1] for p in pts[l][j]]
            d = [b - a for a, b in zip(s, s[1:])]
            if d and not (all(x > 0 for x in d) or all(x < 0 for x in d)):
                raise ValueError(f"다리 {l} {name} 측정점이 한 방향이 아님 (실제 각도가 커질 때 서보가 갔다 왔다 함): "
                                 f"{[(t, round(v, 1)) for t, v in pts[l][j]]}")
    JOINT_POINTS = pts
    if "JOINT_LIMITS" in globals():
        JOINT_LIMITS = _joint_limits()


def calib_source(l, j):
    n = len(JOINT_POINTS[l][j])
    return f"측정 {n}점" if n else "pose_gui"


def _interp(x, xs, ys):
    if len(xs) == 2 or x <= xs[1]:
        i = 0
    elif x >= xs[-2]:
        i = len(xs) - 2
    else:
        i = max(k for k in range(len(xs) - 1) if xs[k] <= x)
    return ys[i] + (ys[i + 1] - ys[i]) * (x - xs[i]) / (xs[i + 1] - xs[i])


def _true_to_servo(l, j, true_deg):
    pts = JOINT_POINTS[l][j]
    d = SV[l][j][1]
    if len(pts) >= 2:
        return _interp(true_deg, [p[0] for p in pts], [p[1] for p in pts])
    t0, s0 = pts[0] if pts else (EXTENDED_ANG[j], _extended_servo(l, j))
    return s0 + d * (true_deg - t0) * SLIDER_PER_90[l][j] / 90.0 * ANGLE_GAIN


def _servo_to_true(l, j, servo):
    pts = JOINT_POINTS[l][j]
    d = SV[l][j][1]
    if len(pts) >= 2:
        xs, ys = [p[1] for p in pts], [p[0] for p in pts]
        if xs[0] > xs[-1]:
            xs, ys = xs[::-1], ys[::-1]
        return _interp(servo, xs, ys)
    t0, s0 = pts[0] if pts else (EXTENDED_ANG[j], _extended_servo(l, j))
    return t0 + (servo - s0) / (d * ANGLE_GAIN) * 90.0 / SLIDER_PER_90[l][j]


def to_firmware(l, j, true_deg):
    """실제 관절 각도 → 펌웨어로 보낼 각도. 측정점이 없으면 hexapod_pose_gui.calibrated_joint 와 같은 계산."""
    return _fw_from_tick(l, j, servo_to_tick(_motor(l, j), _true_to_servo(l, j, true_deg)))


def from_firmware(l, j, fw_deg):
    """펌웨어 각도 → 실제 관절 각도 (to_firmware 의 역변환)."""
    return _servo_to_true(l, j, raw_to_servo(l, j, fw_deg))


# HEXAPOD_JOINT_POINTS=0 이면 측정점을 무시하고 pose_gui 보정만 쓴다 (측정이 덜 됐을 때 임시로)
USE_JOINT_POINTS = os.environ.get("HEXAPOD_JOINT_POINTS", "1") != "0"
set_joint_points((CAL.get("joint_points") or {}).get("legs") if USE_JOINT_POINTS else None)


def stand_margins(min_deg=15.0):
    """서기 자세에서 관절 범위 끝까지 남은 각도가 min_deg 보다 작은 관절 [(다리, 관절 이름, 남은 각도, 보정 출처)]."""
    out = []
    for l in range(NUM_LEGS):
        for j in range(NUM_JOINTS):
            lo, hi = JOINT_LIMITS[l][j]
            m = min(STAND_POSE[l][j] - lo, hi - STAND_POSE[l][j])
            if m < min_deg:
                out.append((l, JOINT_NAMES[j], m, calib_source(l, j)))
    return out


def pose_to_firmware(pose):
    """실제 관절 각도 6×3 → 펌웨어 각도 6×3."""
    return [[to_firmware(l, j, pose[l][j]) for j in range(NUM_JOINTS)] for l in range(NUM_LEGS)]


def tick_of(l, j, true_deg):
    """실제 관절 각도로 보냈을 때 서보에 나가는 틱 (클램프 전)."""
    return fw_ticks(l, j, to_firmware(l, j, true_deg))


def tick_range(l, j):
    """틱이 TICK_HARD_MIN~MAX 안에 드는 실제 관절 각도 범위 (펌웨어가 잘라 버리지 않는 범위)."""
    def ok(a):
        return TICK_HARD_MIN <= tick_of(l, j, a) <= TICK_HARD_MAX
    center = STAND_POSE[l][j]
    if not ok(center):
        raise ValueError(f"다리 {l} 관절 {j}: 서기 자세 {center}° 의 틱 {tick_of(l, j, center):.0f} 이 한계 밖")

    def edge(step):
        a = center
        while ok(a + step) and abs(a - center) < 400:
            a += step
        lo, hi = a, a + step                     # lo 는 안, hi 는 밖
        for _ in range(30):
            m = (lo + hi) / 2
            lo, hi = (m, hi) if ok(m) else (lo, m)
        return lo
    return (edge(-1.0), edge(1.0))


def _joint_limits():
    """표의 joint_limits_deg (= hexapod_sdk.JOINT_LIMITS, 실제 관절 각도) 를 쓰되, 펌웨어 틱 한계로 잘리는 부분만 뺀다."""
    out = []
    for l in range(NUM_LEGS):
        row = []
        for j in range(NUM_JOINTS):
            c0, c1 = tick_range(l, j)
            m0, m1 = SDK_LIMITS[l][j]
            row.append((max(c0, m0), min(c1, m1)))
        out.append(row)
    return out


JOINT_LIMITS = _joint_limits()


def in_limits(l, joints):
    """실제 관절 각도 [hip, femur, tibia] 가 허용 범위 안인가."""
    return all(JOINT_LIMITS[l][j][0] <= joints[j] <= JOINT_LIMITS[l][j][1] for j in range(NUM_JOINTS))


def fk(l, joints):
    """관절 [hip, femur, tibia] → 발끝 (x, y, z) [mm]."""
    hip, fem, tib = joints
    mx, my, th = LAYOUT[l]
    s = HIP_SIGN[l]
    psi = math.radians(th + (hip - 180.0) * s)
    a = math.radians(180.0 - fem)                 # femur 든 각도 (위 +)
    ta = a - math.pi + math.radians(tib)          # tibia 방향 (수평 기준)
    r = COXA_LEN + FEMUR_LEN * math.cos(a) + TIBIA_LEN * math.cos(ta)
    z = HIP_Z + COXA_DZ + FEMUR_LEN * math.sin(a) + TIBIA_LEN * math.sin(ta)
    return [mx + r * math.cos(psi), my + r * math.sin(psi), z]


def ik(l, foot, check_limits=True):
    """발끝 (x, y, z) → 관절 [hip, femur, tibia]. 못 닿거나 허용 범위 밖이면 None. 무릎이 위로 꺾인 해."""
    mx, my, th = LAYOUT[l]
    s = HIP_SIGN[l]
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
