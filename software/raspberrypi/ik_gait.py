"""
역기구학(IK) 삼각 보행 — robot_main.py 의 TripodGait 와 같은 인터페이스 (update(vx, vy, wz, dt) → 18관절)

예전 TripodGait 는 관절 각도에 사인파를 얹어서 다리 길이·서보 범위를 몰랐다. 이 보행은
  - 발끝 궤적을 몸 좌표 [mm]로 만들고 leg_model.ik 로 관절을 푼다 (최종 CAD 치수)
  - 실제 관절 각도로 계산하고, 보낼 때 관절 보정(측정점 / hexapod_pose_gui)을 적용한다
  - 관절은 허용 범위(leg_model.JOINT_LIMITS = SDK 범위 ∩ 펌웨어 틱 한계) 밖으로 절대 안 보낸다
  - 보폭은 지금 몸 높이에서 모든 다리가 닿는 만큼까지만 (시작할 때 높이별로 계산), 넘으면 속도를 줄인다

몸 높이 (height = 바닥 → 몸통 원점 [mm]. 몸통 바닥은 원점보다 BODY_BOTTOM_MM 아래)
  - 일어서기(stand_up): 몸이 바닥에 앉은 높이에서 발을 제자리로 천천히 옮긴 뒤 (관절 STANDUP_JOINT_SPEED),
    발을 바닥에 붙인 채 몸을 HEIGHT_SPEED 로 서기 높이까지 들어 올린다
  - set_height / change_height: 걷는 중에도 몸 높이를 바꾼다 (걸을 수 있는 높이 범위 안에서)

걷는 방식
  - 시계(phase)가 돌면서 두 조(TRIPODS)가 번갈아 공중으로 (공중 시간 < 반 주기 → 바꿀 때 잠깐 6발이 다 바닥)
  - 바닥 다리: 몸이 가는 만큼 바닥에 붙은 채 반대로 밀림 (회전 포함)
  - 공중 다리: 들고 → '원래 자리 + 반 걸음 앞' 으로 옮기고 → 내려놓음. 내려놓을 자리는 공중에 있는 동안 명령을 따라 고침
  - 명령이 0 이면 남은 다리를 제자리로 옮긴 뒤 시계를 멈춘다 (이미 제자리인 다리는 안 듦)
  - 매 순간 18개 관절을 풀어서 하나라도 못 닿으면 몸 이동·높이 변경부터 빼고, 관절이 너무 빨리 돌면 그 틱의 시간을 늦춘다
"""

import math
import os

import leg_model as lm

# ───────────── 보행 파라미터 ─────────────
PERIOD = float(os.environ.get("HEXAPOD_PERIOD") or 0.9)   # 한 주기 [s]. 길게 하면 같은 속도에서 한 걸음이 길어짐
SWING_FRAC = 0.45       # 주기 중 한 다리가 공중에 있는 비율 (< 0.5)
LIFT_MM = float(os.environ.get("HEXAPOD_LIFT") or 45.0)   # 다리를 드는 높이 [mm]. 높일수록 보폭은 줄어든다 (몸을 높이면 회복)
LIFT_RISE = 0.3         # 공중 시간 중 들어 올리는(또 내려놓는) 비율. 그 사이에는 LIFT_MM 을 유지한 채 옮긴다
STRIDE_USE = 0.8        # 계산한 최대 보폭 중 이만큼만 씀 (착지 오차 여유)
STRIDE_MAX = float(os.environ.get("HEXAPOD_STRIDE_MAX") or 160.0)   # 보폭 상한 [mm] (이웃 발 간격 236mm 이상)
ACCEL = 250.0           # 몸 가속 한계 [mm/s²]
W_ACCEL = 1.5           # 회전 가속 한계 [rad/s²]
JOINT_SPEED_MAX = 300.0 # 관절 최고 속도 [도/s] (MG996R 무부하 ≈ 350). 넘는 틱은 시간을 늦춰 계산
SETTLE_TOL = 5.0        # 이 안이면 제자리로 본다 [mm]
LOOKAHEAD = 0.3         # 바닥 다리가 이 시간 뒤에 못 닿을 것 같으면 미리 줄인다 [s]

# ───────────── 몸 높이 ─────────────
BODY_BOTTOM_MM = 40.0   # 몸통 바닥이 몸통 원점보다 이만큼 아래 (URDF 몸통 메시)
SIT_CLEARANCE = 3.0     # 앉은 높이 = 몸통 바닥 + 이만큼 (발이 바닥을 살짝 누르는 정도)
HEIGHT_SPEED = 25.0     # 몸을 들고 내리는 속도 [mm/s]
STANDUP_JOINT_SPEED = 60.0   # 일어서기 전에 발을 제자리로 옮기는 관절 속도 [도/s] (천천히)
HEIGHT_MAX = 170.0      # 이보다 높게는 안 봄 [mm]
MIN_WALK_STRIDE = 40.0  # 이 보폭이 안 나오는 높이에서는 걷지 않는다 [mm] (set_height 범위)
AUTO_STAND_MIN_STRIDE = 60.0   # 기본 서기 자세의 보폭이 이보다 작으면 다른 서기 자세를 찾는다 [mm]
DEFAULT_STAND_HEIGHT = 80.0    # 서기 높이가 이보다 낮으면 여기까지 든다 (몸통 바닥이 바닥에서 40mm). HEXAPOD_HEIGHT 로 바꿈

ST, SW = 0, 1


def _smoothstep(x):
    x = 0.0 if x < 0 else 1.0 if x > 1 else x
    return x * x * (3.0 - 2.0 * x)


def _rotz(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y


def stand_from_env():
    """HEXAPOD_STAND="femur,tibia" (또는 "hip,femur,tibia") → 6다리 서기 자세 (실제 관절 각도). 없으면 None."""
    v = os.environ.get("HEXAPOD_STAND")
    if not v:
        return None
    nums = [float(x) for x in v.replace(" ", "").split(",") if x]
    if len(nums) == 2:
        nums = [180.0] + nums
    if len(nums) != 3:
        raise ValueError(f"HEXAPOD_STAND={v!r}: 'femur,tibia' 또는 'hip,femur,tibia' 로 줄 것")
    return [nums[:] for _ in range(lm.NUM_LEGS)]


def _stand_candidates(base):
    """tibia 를 (거의) 수직으로 둔 채 femur 를 덜 드는 서기 자세들. base 에 가까운 순."""
    hip, fem0, tib0 = base
    out = []
    for fem in range(int(fem0), 171, 5):
        a = 180 - fem
        for extra in (0, 5, 10, 15):
            tib = 90 - a + extra
            out.append([hip, float(fem), float(tib)])
    out.sort(key=lambda p: abs(p[1] - fem0) + abs(p[2] - tib0))
    return out


def lift_profile(u):
    """공중 진행도 u (0~1) → 드는 높이 비율. 빨리 들고 → LIFT_MM 을 유지한 채 옮기고 → 내려놓는다 (사다리꼴).
    예전 sin² 은 한가운데서만 최고 높이라 서보가 따라가다 다 못 들고 내려왔다."""
    if u < LIFT_RISE:
        return _smoothstep(u / LIFT_RISE)
    if u > 1.0 - LIFT_RISE:
        return _smoothstep((1.0 - u) / LIFT_RISE)
    return 1.0


def move_profile(u):
    """공중 진행도 u → 수평으로 옮긴 비율. 거의 다 든 뒤 시작해 내리기 전에 끝낸다."""
    a = LIFT_RISE * 0.8
    return _smoothstep((u - a) / (1.0 - 2.0 * a))


def _capacity(home):
    """모든 다리가 원래 자리 ± 반 걸음까지 (어느 방향이든) 닿고, 그 사이를 LIFT_MM 든 채로 옮길 수 있는 보폭 [mm]."""
    dirs = [(math.cos(k * math.pi / 6), math.sin(k * math.pi / 6)) for k in range(12)]
    checks = ((0.5, 0.0), (0.5, LIFT_MM), (0.25, LIFT_MM), (0.0, LIFT_MM))

    def ok(S):
        for l, h in enumerate(home):
            for dx, dy in dirs:
                for frac, dz in checks:
                    if lm.ik(l, [h[0] + dx * S * frac, h[1] + dy * S * frac, h[2] + dz]) is None:
                        return False
        return True

    if not ok(0.0):
        return 0.0
    lo, hi = 0.0, 300.0
    for _ in range(12):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def _stride_of(home):
    return min(_capacity(home), STRIDE_MAX) * STRIDE_USE


class IkTripodGait:
    def __init__(self, stand_pose=None, auto_stand=True):
        """stand_pose 없으면 HEXAPOD_STAND, 그것도 없으면 leg_model.STAND_POSE (hexapod_sdk).
        auto_stand: 그 자세에서 캘리브레이션 범위 때문에 걷기 어려우면 (보폭 < AUTO_STAND_MIN_STRIDE)
        가장 가까우면서 걸을 수 있는 서기 자세를 고른다.
        서기 자세의 발 위치(xy)는 그대로 두고 몸 높이만 바꿀 수 있다 (높이별 보폭은 여기서 미리 계산)."""
        self.t_swing = PERIOD * SWING_FRAC
        self.t_stance = PERIOD * (1.0 - SWING_FRAC)
        self.stats = {"ticks": 0, "slowed": 0, "reduced": 0, "held": 0}
        env = stand_from_env()
        requested = [row[:] for row in (stand_pose or env or lm.STAND_POSE)]
        self.stand_note = "HEXAPOD_STAND" if (env and not stand_pose) else "기본"
        ok, stride = self._try_stand(requested)
        chosen = requested
        if auto_stand and not env and (not ok or stride < AUTO_STAND_MIN_STRIDE):
            for cand in _stand_candidates(requested[0]):
                c_ok, c_stride = self._try_stand([cand[:] for _ in range(lm.NUM_LEGS)])
                if c_ok and c_stride >= AUTO_STAND_MIN_STRIDE:
                    chosen = [cand[:] for _ in range(lm.NUM_LEGS)]
                    self.stand_note = (f"자동 선택 (기본 {requested[0][1]:.0f}/{requested[0][2]:.0f} 은 보폭 "
                                       f"{stride if ok else 0:.0f}mm → femur {cand[1]:.0f}, tibia {cand[2]:.0f})")
                    break
        homes = [lm.fk(l, chosen[l]) for l in range(lm.NUM_LEGS)]
        bad = [l for l in range(lm.NUM_LEGS) if lm.ik(l, homes[l]) is None]
        if bad:
            raise ValueError(f"서기 자세가 허용 범위 밖인 다리: {bad} — 캘리브레이션/한계를 확인할 것")
        self.home_xy = [h[:2] for h in homes]
        self.stand_height = -sum(h[2] for h in homes) / lm.NUM_LEGS
        self._build_height_table()
        pose_height = self.stand_height
        want = float(os.environ.get("HEXAPOD_HEIGHT") or max(pose_height, DEFAULT_STAND_HEIGHT))
        homes = self._homes_at(want)
        if (abs(want - pose_height) > 0.1 and all(lm.ik(l, homes[l]) is not None for l in range(lm.NUM_LEGS))
                and self.stride_at(want) >= MIN_WALK_STRIDE):
            self.stand_height = want
            self.height_range = (min(self.height_range[0], want), max(self.height_range[1], want))
            self.stand_note += f", 몸 높이 {pose_height:.0f} → {want:.0f}mm"
        self.reset()

    def _try_stand(self, stand):
        homes = [lm.fk(l, stand[l]) for l in range(lm.NUM_LEGS)]
        if any(lm.ik(l, homes[l]) is None for l in range(lm.NUM_LEGS)):
            return False, 0.0
        return True, _stride_of(homes)

    def _homes_at(self, H):
        return [[x, y, -H] for x, y in self.home_xy]

    def _build_height_table(self):
        """높이별 (발이 닿나, 보폭). 앉은 높이·걸을 수 있는 높이 범위를 정한다."""
        self._table = {}
        sit = BODY_BOTTOM_MM + SIT_CLEARANCE
        hs = sorted({round(h, 1) for h in [sit, self.stand_height]
                     + [float(h) for h in range(int(math.ceil(sit / 5) * 5), int(HEIGHT_MAX) + 1, 5)]})
        for H in hs:
            homes = self._homes_at(H)
            ok = all(lm.ik(l, homes[l]) is not None for l in range(lm.NUM_LEGS))
            self._table[H] = _stride_of(homes) if ok else None
        reach = [H for H, s in self._table.items() if s is not None]
        walk = [H for H, s in self._table.items() if s is not None and s >= MIN_WALK_STRIDE]
        self.sit_height = min(reach) if reach else self.stand_height      # 일어서기 시작 높이
        lo, hi = (min(walk), max(walk)) if walk else (self.stand_height, self.stand_height)
        self.height_range = (min(lo, self.stand_height), max(hi, self.stand_height))

    def stride_at(self, H):
        """높이 H 의 보폭 [mm] (표 사이는 직선, 닿지 않는 높이는 0)."""
        keys = sorted(self._table)
        if H <= keys[0]:
            s = self._table[keys[0]]
            return s or 0.0
        if H >= keys[-1]:
            s = self._table[keys[-1]]
            return s or 0.0
        for a, b in zip(keys, keys[1:]):
            if a <= H <= b:
                sa, sb = self._table[a], self._table[b]
                if sa is None or sb is None:
                    return 0.0
                return sa + (sb - sa) * (H - a) / (b - a)
        return 0.0

    @property
    def stride(self):
        """지금 몸 높이의 보폭 [mm]."""
        return self.stride_at(self.height)

    @property
    def stand_pose(self):
        """서기 높이의 관절 (실제 각도)."""
        homes = self._homes_at(self.stand_height)
        return [lm.ik(l, homes[l], check_limits=False) for l in range(lm.NUM_LEGS)]

    # ───────────── 상태 ─────────────
    def _set_height(self, H):
        self.height = H
        self.ground = -H
        self.home = self._homes_at(H)

    def reset(self, height=None):
        """모든 발 제자리, 몸 높이 height (기본 = 서기 높이) 에서 다시 시작. 서기·비상정지·힘 빼기 뒤에 부른다."""
        self._set_height(self.stand_height if height is None else height)
        self.height_target = self.height
        self.feet = [h[:] for h in self.home]
        self.leg = [ST] * lm.NUM_LEGS
        self.u = [0.0] * lm.NUM_LEGS
        self.p0 = [h[:] for h in self.home]
        self.tgt = [h[:2] for h in self.home]
        self.phase = 0.0
        self.active = False
        self.v = [0.0, 0.0]       # 몸 속도 [mm/s]
        self.w = 0.0              # [rad/s]
        self.settling = False     # 일어서기: 발을 제자리로 천천히 옮기는 중
        self.pose_true = self._solve(self.feet) or [row[:] for row in self.stand_pose]   # 마지막으로 낸 자세 (실제 각도)

    def stand_up(self, current_true=None, height=None):
        """일어서기. 몸이 바닥에 앉은 높이에서 발을 제자리로 천천히 옮긴 뒤 (current_true 가 있으면 그 자세에서 출발),
        발을 바닥에 붙인 채 몸을 height (기본 = 서기 높이) 까지 HEIGHT_SPEED 로 들어 올린다."""
        self.reset(height=self.sit_height)
        if current_true is not None:
            self.pose_true = [[min(max(current_true[l][j], lm.JOINT_LIMITS[l][j][0]), lm.JOINT_LIMITS[l][j][1])
                               for j in range(lm.NUM_JOINTS)] for l in range(lm.NUM_LEGS)]
            self.settling = True
        self.set_height(self.stand_height if height is None else height)

    def set_height(self, H):
        """몸 높이 목표 [mm] (걸을 수 있는 범위로 자름). 실제 높이는 HEIGHT_SPEED 로 따라간다. 자른 값을 돌려준다."""
        lo, hi = self.height_range
        self.height_target = min(max(float(H), lo), hi)
        return self.height_target

    def change_height(self, delta):
        return self.set_height(self.height_target + float(delta))

    def stand_command(self):
        """서기 자세를 펌웨어로 보낼 각도 (캘리브레이션 적용)."""
        return lm.pose_to_firmware(self.stand_pose)

    def status(self):
        return {"height": round(self.height, 1), "height_target": round(self.height_target, 1),
                "height_range": [round(v, 1) for v in self.height_range], "stride": round(self.stride, 1),
                "standing_up": self.settling or abs(self.height - self.height_target) > 0.5}

    def walk_warning(self, min_stride=30.0):
        """걷기 어려우면 이유를 글로 (괜찮으면 None)."""
        s = self.stride_at(self.stand_height)
        if s >= min_stride:
            return None
        tight = lm.stand_margins(stand=self.stand_pose)
        why = ", ".join(f"다리 {l} {n} 범위 끝까지 {m:.0f}° ({src})" for l, n, m, src in tight) or "알 수 없음"
        fix = []
        if any(src.startswith("측정 1") for *_, src in tight):
            fix.append("측정점이 1개뿐인 관절이 있음 → calibrate_joints.py 로 ②③ 도 잴 것")
        if lm.USE_JOINT_POINTS and any(lm.calib_source(l, j) != "pose_gui" for l in range(6) for j in range(3)):
            fix.append("임시로 측정점 끄기: HEXAPOD_JOINT_POINTS=0")
        fix.append("측정해도 같으면 그 서보 혼을 몇 칸 돌려 다시 끼울 것 (서기 자세가 서보 끝에 붙어 있음)")
        return (f"보폭 {s:.1f}mm — 걸을 수 없음 (서기 자세에서 다리를 {LIFT_MM:.0f}mm 들 여유가 없는 관절: {why})\n"
                + "\n".join("  → " + f for f in fix))

    def max_speed(self):
        """서기 높이의 직진 최고 속도 [m/s] (보폭 ÷ 바닥 시간)."""
        return self.stride_at(self.stand_height) / self.t_stance / 1000.0

    def _limit_motion(self, v, w):
        """한 번 디딘 동안 발이 밀리는 거리(이동 + 회전)가 보폭을 넘지 않게 줄인다."""
        T = self.t_stance
        S = self.stride
        worst = 0.0
        for h in self.home:
            worst = max(worst, math.hypot(v[0] * T - w * T * h[1], v[1] * T + w * T * h[0]))
        k = 1.0 if worst <= S or worst < 1e-9 else S / worst
        return [v[0] * k, v[1] * k], w * k

    def _headroom(self, v, w):
        """지금 바닥 다리들이 (v, w)로 LOOKAHEAD 더 가도 닿는가 → 속도 비율."""
        if math.hypot(*v) < 1e-6 and abs(w) < 1e-6:
            return 1.0
        for k in (1.0, 0.5, 0.25):
            T = LOOKAHEAD * k
            good = True
            for l in range(lm.NUM_LEGS):
                if self.leg[l] == SW:
                    continue
                p = self.feet[l]
                x, y = _rotz(p[0] - v[0] * T, p[1] - v[1] * T, -w * T)
                if lm.ik(l, [x, y, p[2]]) is None:
                    good = False
                    break
            if good:
                return k
        return 0.0

    # ───────────── 한 순간 진행 ─────────────
    def _land_target(self, l, v, w):
        x0, y0 = self.home_xy[l]
        x, y = _rotz(x0, y0, w * self.t_stance / 2)
        return [x + v[0] * self.t_stance / 2, y + v[1] * self.t_stance / 2]

    def _advance(self, dt, v, w, idle, dH):
        """dt 만큼 진행한 새 상태 (feet, leg, u, p0, tgt, phase, height). self 는 바꾸지 않는다.
        dH: 이번 틱에 몸을 올릴 높이 [mm] (바닥 발은 몸 기준으로 그만큼 내려간다)."""
        feet = [p[:] for p in self.feet]
        leg, u = self.leg[:], self.u[:]
        p0, tgt = [p[:] for p in self.p0], [t[:] for t in self.tgt]
        phase = self.phase
        height = self.height + dH
        ground = -height

        # 0) 몸 높이: 바닥 다리는 바닥에 붙은 채 몸만 오르내림
        if dH:
            for l in range(lm.NUM_LEGS):
                if leg[l] == ST:
                    feet[l][2] = ground

        # 1) 바닥 다리: 몸이 가는 만큼 반대로
        if v[0] or v[1] or w:
            for l in range(lm.NUM_LEGS):
                if leg[l] == ST:
                    x, y = _rotz(feet[l][0] - v[0] * dt, feet[l][1] - v[1] * dt, -w * dt)
                    feet[l][0], feet[l][1] = x, y

        # 2) 공중 다리: 들고 → 옮기고 → 내림
        for l in range(lm.NUM_LEGS):
            if leg[l] != SW:
                continue
            u[l] = min(1.0, u[l] + dt / self.t_swing)
            tgt[l] = self._land_target(l, v, w)
            sh = move_profile(u[l])                             # 수평은 다 든 뒤 시작, 내리기 전에 끝
            x = p0[l][0] + (tgt[l][0] - p0[l][0]) * sh
            y = p0[l][1] + (tgt[l][1] - p0[l][1]) * sh
            z = p0[l][2] + (ground - p0[l][2]) * _smoothstep(u[l]) + LIFT_MM * lift_profile(u[l])
            feet[l] = [x, y, z]
            if u[l] >= 1.0:
                leg[l] = ST
                feet[l] = [tgt[l][0], tgt[l][1], ground]

        # 3) 시계 → 차례가 온 조 들기
        if self.active:
            new = phase + dt / PERIOD
            for k, grp in enumerate(lm.TRIPODS):
                off = 0.5 * k
                if (off - phase) % 1.0 >= new - phase:          # 이번 틱에 off 를 지나지 않음
                    continue
                for l in grp:
                    if leg[l] != ST:
                        continue
                    if idle and math.dist(feet[l][:2], self.home_xy[l]) < SETTLE_TOL:
                        continue                                # 이미 제자리 → 안 듦
                    leg[l], u[l], p0[l] = SW, 0.0, feet[l][:]
                    tgt[l] = self._land_target(l, v, w)
            phase = new % 1.0
        return feet, leg, u, p0, tgt, phase, height

    def _solve(self, feet):
        out = []
        for l in range(lm.NUM_LEGS):
            j = lm.ik(l, feet[l])
            if j is None:
                return None
            out.append(j)
        return out

    def _glide(self, target, speed, dt):
        """pose_true 를 target 쪽으로 관절 speed [도/s] 로 옮긴다. 다 왔으면 True."""
        step = speed * max(dt, 1e-3)
        done = True
        out = []
        for l in range(lm.NUM_LEGS):
            row = []
            for i in range(3):
                d = target[l][i] - self.pose_true[l][i]
                if abs(d) > step:
                    done = False
                    d = math.copysign(step, d)
                row.append(self.pose_true[l][i] + d)
            out.append(row)
        self.pose_true = out
        return done

    # ───────────── 매 제어 틱 ─────────────
    def update(self, vx, vy, wz, dt):
        """명령 (m/s, m/s, rad/s) → 펌웨어로 보낼 18관절 [도] (다리 6 × [hip, femur, tibia]).
        계산은 실제 관절 각도로 하고 (self.pose_true), 돌려줄 때 캘리브레이션(leg_model.to_firmware)을 적용한다."""
        self.stats["ticks"] += 1
        dt = max(0.0, min(dt, 0.1))

        # 일어서기 1단계: 앉은 높이의 제자리 자세로 천천히 (그동안 걷기·높이 명령은 기다림)
        if self.settling:
            target = self._solve(self.feet) or self.pose_true
            if self._glide(target, STANDUP_JOINT_SPEED, dt):
                self.settling = False
            return lm.pose_to_firmware(self.pose_true)

        cmd_v = [vx * 1000.0, vy * 1000.0]
        cmd_w = float(wz)
        idle = math.hypot(*cmd_v) < 1.0 and abs(cmd_w) < 0.01

        # 속도: 명령 → 보폭 한계 → 미리 줄이기 → 가속 한계
        v_des, w_des = self._limit_motion(cmd_v, cmd_w)
        k = self._headroom(v_des, w_des)
        v_des, w_des = [x * k for x in v_des], w_des * k
        dv = [v_des[0] - self.v[0], v_des[1] - self.v[1]]
        n = math.hypot(*dv)
        a = ACCEL * dt
        if n > a:
            dv = [dv[0] / n * a, dv[1] / n * a]
        self.v = [self.v[0] + dv[0], self.v[1] + dv[1]]
        self.w += max(-W_ACCEL * dt, min(W_ACCEL * dt, w_des - self.w))
        self.v, self.w = self._limit_motion(self.v, self.w)

        # 몸 높이: 목표로 HEIGHT_SPEED (끝에서 부드럽게)
        err = self.height_target - self.height
        dH = max(-HEIGHT_SPEED * dt, min(HEIGHT_SPEED * dt, err * 4.0 * dt)) if abs(err) > 0.05 else 0.0
        if abs(err) <= 0.5 and dH:
            dH = err if abs(err) <= HEIGHT_SPEED * dt else dH

        # 시계를 돌릴지
        moving = math.hypot(*self.v) > 1.0 or abs(self.w) > 0.005
        far = max(math.dist(self.feet[l][:2], self.home_xy[l]) for l in range(lm.NUM_LEGS))
        if not idle or moving:
            self.active = True
        elif all(x == ST for x in self.leg) and far < SETTLE_TOL:
            self.active = False
            self.phase = 0.0

        # 계산 → 못 닿으면 몸 이동·높이 변경부터 빼고, 관절이 너무 빨리 돌면 시간을 늦춘다
        step_max = JOINT_SPEED_MAX * max(dt, 1e-3)
        chosen = None
        for scale in (1.0, 0.5, 0.25, 0.125):
            found = None
            for mk, hk in ((1.0, 1.0), (0.5, 1.0), (0.25, 1.0), (0.0, 1.0), (0.0, 0.0)):
                st = self._advance(dt * scale, [x * mk for x in self.v], self.w * mk, idle, dH * scale * hk)
                j = self._solve(st[0])
                if j is not None:
                    found = (st, j, mk, scale)
                    break
            if found is None:
                break
            chosen = found
            jump = max(abs(found[1][l][i] - self.pose_true[l][i]) for l in range(lm.NUM_LEGS) for i in range(3))
            if jump <= step_max:
                break
        if chosen is None:                                       # 어떤 경우도 못 풂 → 지금 자세 유지
            self.stats["held"] += 1
            return lm.pose_to_firmware(self.pose_true)
        (self.feet, self.leg, self.u, self.p0, self.tgt, self.phase, height), joints, mk, used_scale = chosen
        if height != self.height:
            self._set_height(height)
        if used_scale < 1.0:
            self.stats["slowed"] += 1
        if mk < 1.0:
            self.stats["reduced"] += 1
            self.v, self.w = [x * mk for x in self.v], self.w * mk
        # 시간을 늦춰도 넘으면 마지막으로 관절 속도를 자른다 (서보에 무리한 점프를 안 보냄)
        self._glide(joints, JOINT_SPEED_MAX, dt)
        return lm.pose_to_firmware(self.pose_true)
