"""
몸체 이동 (발은 바닥에 고정, 몸만 움직임) + 3개 다리씩 들어서 제자리로 옮기기
네 SDK(Link, Hexapod) 위에서 동작한다.

  from hexapod_sdk import Link, Hexapod        ← 네 SDK 파일 이름에 맞게 바꿀 것
  body = Body(hexapod, link)
  body.capture()                               # 지금 자세의 발끝 위치를 "바닥에 붙은 위치"로 기록
  body.move_body(x=20)                         # 몸만 앞으로 20mm (H발은 그대로)
  body.replant()                               # 3개씩 들어서 몸 아래 원래 자리로 → 결과적으로 20mm 전진
  body.step(dx=20, dyaw=5)                     # 위 두 개를 한 번에 (= 한 걸음)

좌표 (몸 기준):  x = 앞, y = 왼쪽, z = 위 [mm], 회전은 [도]

★ 아래 "기하 설정"의 값들은 추정이 섞여 있다. 처음 쓰기 전에
   python3 hexapod_body.py --check   로 발끝 위치를 찍어보고 자로 재서 맞출 것.
"""

import math
import sys
import time

# ============================== 기하 설정 ==============================
# 다리 길이 [mm]  ★ 프로젝트 기록에 tibia 77mm / 137mm 두 값이 있음 — 실측해서 넣을 것
COXA_LEN = 46.6      # hip 축 ~ femur 축 (수평 거리)
FEMUR_LEN = 100.0    # femur 축 ~ 무릎 축
TIBIA_LEN = 137.0    # 무릎 축 ~ 발끝

# 다리가 몸에 붙은 위치: 몸 중심 → 각 다리 hip 회전축 거리 [mm] (실측)
#   다리는 60° 간격. 방향: 앞=0°, 왼쪽=+90°.  몸통은 앞뒤로 긴 타원
#   위에서 본 배치 (2026-10-17 확인. 예전엔 앞에 1·6, 뒤에 3·4 로 잘못 들어가 있었음):
#              [3]                 3 = 맨 앞,  6 = 맨 뒤   (타원 끝, 앞뒤를 받침)
#         [2]       [4]            2 = 왼앞,   4 = 오른앞
#            ( 몸 )                1 = 왼뒤,   5 = 오른뒤
#         [1]       [5]
#              [6]
#   코드 번호는 0부터 (다리 1 = 0번 … 다리 6 = 5번)
D_TIP = 112.0       # 앞뒤 끝 다리 2개 (3, 6) (실측)
D_SIDE = 99.0       # 옆 다리 4개 (1, 2, 4, 5) (실측)
LEG_DIR = [120, 60, 0, -60, -120, 180]    # 다리 1~6
LEG_D = [D_SIDE, D_SIDE, D_TIP, D_SIDE, D_SIDE, D_TIP]
MOUNT = [(d * math.cos(math.radians(t)), d * math.sin(math.radians(t)), t)
         for d, t in zip(LEG_D, LEG_DIR)]

# 관절 각도 해석 (지금 자세 값들에서 추정)
#   hip   : 180 = 몸에서 수직으로 바깥.  HIP_SIGN = +1 이면 각도가 커질 때 위에서 봐서 반시계로 돎
#   femur : 180 = 수평, 각도가 작아질수록 위로 듦 (90 = 수직 위, 225 = 45° 아래)
#           서기 자세는 SDK STAND_POSE (2026-10-14부터 105 = 75° 위 — 펌웨어 femur 기준을 150으로 고친 뒤 숫자)
#   tibia : femur와 tibia 사이 안쪽 각도 (180 = 일자로 폄)  (서기 45 → tibia가 수직 아래)
HIP_CENTER = 180
# 펌웨어 dir 때문에 hip 관절각 + 가 양쪽 다 "앞쪽"이라, 위에서 본 회전 방향은 좌우가 반대.
#   (같은 부호면 x 명령이 회전처럼, yaw 명령이 앞뒤 이동처럼 보임)
# ★ x 10 을 줬을 때 다리가 앞으로 쓸리면(몸이 뒤로 가는 방향) 6개 부호를 전부 뒤집을 것
HIP_SIGN = [-1, -1, -1, 1, 1, 1]

# 관절 각도 허용 범위 — SDK의 JOINT_LIMITS를 그대로 씀 (한 곳만 고치면 되게).
#   예전엔 여기에 따로 80~280이 있어서, SDK 범위를 넓혀도 여기서 막혔음.
from hexapod_sdk import JOINT_LIMITS


def _in_limits(l, joints):
    return all(JOINT_LIMITS[l][j][0] <= joints[j] <= JOINT_LIMITS[l][j][1] for j in range(3))

# 움직임
STEP_HZ = 25          # 보간 갱신 횟수 [회/초]
BODY_SPEED = 30.0     # 몸 이동 속도 [mm/초]
TURN_SPEED = 15.0     # 몸 회전 속도 [도/초]
LIFT_MM = 60.0        # 다리 옮길 때 드는 높이 [mm]
LEG_MOVE_SEC = 0.6    # 다리 하나 조 옮기는 시간 (들기~내리기) [초]
GROUP_A = [0, 2, 4]   # 3각 그룹 (이웃 다리끼리 다른 조)
GROUP_B = [1, 3, 5]
# ======================================================================


def _rot(roll, pitch, yaw):
    """회전행렬 (도 단위 입력). 몸 → 바닥 좌표."""
    r, p, y = (math.radians(a) for a in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def _mul(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def _mul_t(R, v):  # R^T · v
    return [sum(R[k][i] * v[k] for k in range(3)) for i in range(3)]


# ------------------------------ 한 다리 계산 ------------------------------
def leg_fk(l, joints):
    """관절 각도 [hip, femur, tibia] → 발끝 위치 (몸 기준 x, y, z)."""
    hip, fem, tib = joints
    psi = math.radians((hip - HIP_CENTER) * HIP_SIGN[l])   # hip 회전 (반시계 +)
    a = math.radians(180 - fem)                             # femur 든 각도 (위 +)
    b = math.radians(tib)                                   # 무릎 안쪽 각도
    ta = a - math.pi + b                                    # tibia 방향
    r = FEMUR_LEN * math.cos(a) + TIBIA_LEN * math.cos(ta)
    z = FEMUR_LEN * math.sin(a) + TIBIA_LEN * math.sin(ta)
    rho = COXA_LEN + r                                      # hip 축에서 수평 거리
    mx, my, th = MOUNT[l]
    ang = math.radians(th) + psi
    return [mx + rho * math.cos(ang), my + rho * math.sin(ang), z]


def leg_ik_reason(l, foot):
    """leg_ik 가 None 일 때 이유를 글로."""
    mx, my, th = MOUNT[l]
    dx, dy, z = foot[0] - mx, foot[1] - my, foot[2]
    r = math.hypot(dx, dy) - COXA_LEN
    D = math.hypot(r, z)
    if D >= FEMUR_LEN + TIBIA_LEN - 1:
        return f"다리가 다 펴져도 모자람 (필요 {D:.0f}mm > 최대 {FEMUR_LEN + TIBIA_LEN:.0f}mm)"
    if D <= abs(FEMUR_LEN - TIBIA_LEN) + 1:
        return f"너무 가까움 (필요 {D:.0f}mm)"
    psi = math.degrees(math.atan2(dy, dx)) - th
    psi = (psi + 180) % 360 - 180
    hip = HIP_CENTER + psi * HIP_SIGN[l]
    b = math.degrees(math.acos((FEMUR_LEN ** 2 + TIBIA_LEN ** 2 - D ** 2) / (2 * FEMUR_LEN * TIBIA_LEN)))
    g = math.acos((FEMUR_LEN ** 2 + D ** 2 - TIBIA_LEN ** 2) / (2 * FEMUR_LEN * D))
    fem = 180 - math.degrees(math.atan2(z, r) + g)
    names = ["hip", "femur", "tibia"]
    for j, v in enumerate((hip, fem, b)):
        lo, hi = JOINT_LIMITS[l][j]
        if not (lo <= v <= hi):
            return f"{names[j]} {v:.0f}도 → 허용 범위 {lo}~{hi} 밖"
    return "알 수 없음"


def leg_ik(l, foot):
    """발끝 위치 (몸 기준) → 관절 각도 [hip, femur, tibia]. 못 닿으면 None."""
    mx, my, th = MOUNT[l]
    dx, dy, z = foot[0] - mx, foot[1] - my, foot[2]
    psi = math.atan2(dy, dx) - math.radians(th)
    psi = math.atan2(math.sin(psi), math.cos(psi))          # -180~180 으로 정리
    rho = math.hypot(dx, dy)
    r = rho - COXA_LEN
    D = math.hypot(r, z)
    if not (abs(FEMUR_LEN - TIBIA_LEN) + 1 < D < FEMUR_LEN + TIBIA_LEN - 1):
        return None
    b = math.acos((FEMUR_LEN ** 2 + TIBIA_LEN ** 2 - D ** 2) / (2 * FEMUR_LEN * TIBIA_LEN))
    g = math.acos((FEMUR_LEN ** 2 + D ** 2 - TIBIA_LEN ** 2) / (2 * FEMUR_LEN * D))
    a = math.atan2(z, r) + g                                # 무릎이 위로 꺾인 해
    joints = [HIP_CENTER + math.degrees(psi) * HIP_SIGN[l], 180 - math.degrees(a), math.degrees(b)]
    if not _in_limits(l, joints):
        return None
    return joints


# ------------------------------ 몸 전체 ------------------------------
class Body:
    def __init__(self, hexapod, link):
        self.hx = hexapod
        self.link = link
        self.feet = None                 # 바닥 기준 발끝 위치 6개
        self.home = None                 # 몸 기준 "기본" 발끝 위치 6개 (replant 목표)
        self.pose = [0.0] * 6            # 몸 자세 x, y, z, roll, pitch, yaw
        self.sent = [[None] * 3 for _ in range(6)]

    # ---- 보내기 ----
    def _apply(self, joints_all):
        for l in range(6):
            for j in range(3):
                v = joints_all[l][j]
                self.hx.pos[l][j] = v
                self.hx.cur[l][j] = v          # move_update 가 되돌리지 않게 목표도 같이
                d = int(round(v))
                if d != self.sent[l][j]:
                    self.link.send(f"<{l}, {j}, {d}>\n".encode())
                    self.sent[l][j] = d

    def _solve(self, pose, feet):
        """몸 자세 + 바닥 발끝 → 18개 관절. 하나라도 못 닿으면 (None, 다리번호)."""
        x, y, z, ro, pi, ya = pose
        R = _rot(ro, pi, ya)
        out = []
        for l in range(6):
            fb = _mul_t(R, [feet[l][0] - x, feet[l][1] - y, feet[l][2] - z])
            jt = leg_ik(l, fb)
            if jt is None:
                return None, l
            out.append(jt)
        return out, None

    # ---- 기준 잡기 ----
    def capture(self):
        """지금 관절 각도(hexapod.pos)로 발끝 위치를 계산해서 바닥에 붙은 위치로 기록."""
        print("[기준] 지금 관절 각도:", [[round(v) for v in p] for p in self.hx.pos])
        self.feet = [leg_fk(l, self.hx.pos[l]) for l in range(6)]
        self.home = [list(f) for f in self.feet]
        self.pose = [0.0] * 6
        bad = [l for l in range(6) if leg_ik(l, self.feet[l]) is None]
        if bad:
            raise RuntimeError(f"다리 {bad}: 지금 자세가 계산 범위 밖 — 기하 설정을 확인할 것")
        return self.feet

    # ---- 1) 몸만 움직이기 ----
    def move_body(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, relative=True):
        """몸 자세를 바꾼다. 발은 바닥에 고정. relative=True 면 지금 자세에서 더함."""
        start = list(self.pose)
        target = list(self.pose)
        for i, v in enumerate((x, y, z, roll, pitch, yaw)):
            if v is not None:
                target[i] = (start[i] + v) if relative else v

        # 미리 끝 자세가 닿는지 확인 (중간에 멈추는 일 방지)
        _, bad = self._solve(target, self.feet)
        if bad is not None:
            x, y, z, ro, pi, ya = target
            fb = _mul_t(_rot(ro, pi, ya), [self.feet[bad][0] - x, self.feet[bad][1] - y, self.feet[bad][2] - z])
            print(f"[몸] 다리 {bad} 못 닿음: {leg_ik_reason(bad, fb)} — 이동 취소")
            return False

        dist = math.dist(start[:3], target[:3]) / BODY_SPEED
        turn = max(abs(target[i] - start[i]) for i in range(3, 6)) / TURN_SPEED
        dur = max(dist, turn, 0.05)
        n = max(1, int(dur * STEP_HZ))
        for k in range(1, n + 1):
            u = k / n
            u = 0.5 - 0.5 * math.cos(math.pi * u)          # 부드럽게 출발·정지
            p = [start[i] + (target[i] - start[i]) * u for i in range(6)]
            joints, bad = self._solve(p, self.feet)
            if joints is None:
                print(f"[몸] 중간에 다리 {bad} 범위 밖 — 여기서 멈춤")
                return False
            self._apply(joints)
            self.pose = p
            time.sleep(1.0 / STEP_HZ)
        return True

    # ---- 2) 3개 다리씩 들어서 몸 아래 원래 자리로 ----
    def _move_group(self, group):
        x, y, z, ro, pi, ya = self.pose
        R = _rot(ro, pi, ya)
        # 목표: 몸 기준 home 위치를 바닥 좌표로 (높이는 바닥 = 지금 발 높이)
        targets = {}
        for l in group:
            w = _mul(R, self.home[l])
            targets[l] = [w[0] + x, w[1] + y, self.feet[l][2]]
        starts = {l: list(self.feet[l]) for l in group}

        n = max(2, int(LEG_MOVE_SEC * STEP_HZ))
        for k in range(1, n + 1):
            u = k / n
            s = 0.5 - 0.5 * math.cos(math.pi * u)            # 수평 이동
            h = LIFT_MM * math.sin(math.pi * u)               # 위로 들었다 내리기
            feet = [list(f) for f in self.feet]
            for l in group:
                feet[l] = [starts[l][i] + (targets[l][i] - starts[l][i]) * s for i in range(3)]
                feet[l][2] += h
            joints, bad = self._solve(self.pose, feet)
            if joints is None:
                print(f"[다리] {bad}번이 옮기는 중 범위 밖 — 멈춤 (몸을 덜 움직여서 다시)")
                return False
            self._apply(joints)
            time.sleep(1.0 / STEP_HZ)
        for l in group:
            self.feet[l] = targets[l]
        return True

    def replant(self):
        """A조 → B조 순서로 다리를 옮겨서, 몸 기준 기본 자세로 되돌린다."""
        if not self._move_group(GROUP_A):
            return False
        if not self._move_group(GROUP_B):
            return False
        # 이제 발이 전부 몸 아래 기본 자리 → 지금 몸 위치·방향을 새 기준(0)으로.
        # (바닥 좌표를 x, y 만큼 옮기고 yaw 만큼 돌림. 높이 z, 기울기 roll/pitch 는 유지)
        x, y, z, ro, pi, ya = self.pose
        c, s_ = math.cos(math.radians(-ya)), math.sin(math.radians(-ya))
        self.feet = [[c * (f[0] - x) - s_ * (f[1] - y), s_ * (f[0] - x) + c * (f[1] - y), f[2]]
                     for f in self.feet]
        self.pose = [0.0, 0.0, z, ro, pi, 0.0]
        return True

    # ---- 1 + 2 = 한 걸음 ----
    def step(self, dx=0.0, dy=0.0, dyaw=0.0):
        """몸을 (dx, dy, dyaw)만큼 옮긴 뒤 다리를 따라오게 한다 = 한 걸음."""
        if not self.move_body(x=dx, y=dy, yaw=dyaw):
            return False
        return self.replant()


# ------------------------------ 확인 / 데모 ------------------------------
def check(stand=None):
    if stand is None:
        from hexapod_sdk import STAND_POSE
        stand = tuple(STAND_POSE[1])
    print("서기 자세", stand, "에서 발끝 위치 (몸 중심 기준, mm):")
    for l in range(6):
        f = leg_fk(l, list(stand))
        back = leg_ik(l, f)
        print(f"  다리 {l}: x={f[0]:6.0f}  y={f[1]:6.0f}  z={f[2]:6.0f}   "
              f"(역계산 {['%.1f' % v for v in back] if back else '실패'})")
    r = leg_fk(1, list(stand))
    print(f"\n★ 자로 확인: hip 축에서 발끝까지 수평 {math.hypot(r[0] - MOUNT[1][0], r[1] - MOUNT[1][1]):.0f}mm, "
          f"발끝이 hip 축보다 {-r[2]:.0f}mm 아래 — 실제와 다르면 길이/각도 해석을 고칠 것")
    print("★ hip 방향 시험: 한 다리 hip을 180 → 200 으로 보냈을 때 위에서 봐서 반시계로 돌면 HIP_SIGN=+1")


def demo(target):
    from hexapod_sdk import Link, Hexapod      # ← 네 SDK 파일 이름
    link = Link(target)
    if not link.connect():
        print("\n".join(link.log_queue))
        return
    hx = Hexapod()
    hx.standing(link)
    while hx.move_update_available():          # 서기 자세까지 천천히
        hx.move_update(link)
        time.sleep(0.01)
    body = Body(hx, link)
    body.capture()
    
    print("명령: x 20 / y -15 / z 10 / roll 5 / pitch 5 / yaw 10 / home / replant / step dx dy dyaw / o / q")
    while True:
        c = input("> ").split()
        if not c:
            continue
        k = c[0]
        try:
            if k in ("x", "y", "z", "roll", "pitch", "yaw"):
                body.move_body(**{k: float(c[1])})
            elif k == "home":
                body.move_body(0, 0, 0, 0, 0, 0, relative=False)
            elif k == "replant":
                body.replant()
            elif k == "step":
                v = [float(a) for a in c[1:4]] + [0, 0, 0]
                body.step(v[0], v[1], v[2])
            elif k == "o":
                hx.poweroff(link)
            elif k == "q":
                break
        except (IndexError, ValueError):
            print("숫자를 같이 써줘 (예: x 20)")
        print("  몸 자세:", ["%.1f" % v for v in body.pose])


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--check":
        check()
    elif len(sys.argv) > 1:
        demo(sys.argv[1])
    else:
        print("사용법: python3 hexapod_body.py --check          (기하 확인, 로봇 없이)\n"
              "        python3 hexapod_body.py AA:BB:CC:DD:EE:FF (조종)")