"""
역기구학(IK) 삼각 보행 — robot_main.py 의 TripodGait 와 같은 인터페이스 (update(vx, vy, wz, dt) → 18관절)

예전 TripodGait 는 관절 각도에 사인파를 얹어서 다리 길이·서보 범위를 몰랐다. 이 보행은
  - 발끝 궤적을 몸 좌표 [mm]로 만들고 leg_model.ik 로 관절을 푼다 (최종 CAD 치수)
  - 실제 관절 각도로 계산하고, 보낼 때 관절 보정(hexapod_pose_gui 와 같은 calibrated_joint)을 적용한다
  - 관절은 허용 범위(leg_model.JOINT_LIMITS = SDK 범위 ∩ 펌웨어 틱 한계) 밖으로 절대 안 보낸다
  - 보폭은 서기 높이에서 모든 다리가 닿는 만큼까지만 (시작할 때 계산), 넘으면 속도를 줄인다

걷는 방식
  - 시계(phase)가 돌면서 두 조(TRIPODS)가 번갈아 공중으로 (공중 시간 < 반 주기 → 바꿀 때 잠깐 6발이 다 바닥)
  - 바닥 다리: 몸이 가는 만큼 바닥에 붙은 채 반대로 밀림 (회전 포함)
  - 공중 다리: 들고 → '원래 자리 + 반 걸음 앞' 으로 옮기고 → 내려놓음. 내려놓을 자리는 공중에 있는 동안 명령을 따라 고침
  - 명령이 0 이면 남은 다리를 제자리로 옮긴 뒤 시계를 멈춘다 (이미 제자리인 다리는 안 듦)
  - 매 순간 18개 관절을 풀어서 하나라도 못 닿으면 몸 이동부터 줄이고, 관절이 너무 빨리 돌면 그 틱의 시간을 늦춘다
"""

import math

import leg_model as lm

# ───────────── 보행 파라미터 ─────────────
PERIOD = 0.9            # 한 주기 (두 조가 한 번씩 옮김) [s]
SWING_FRAC = 0.45       # 주기 중 한 다리가 공중에 있는 비율 (< 0.5)
LIFT_MM = 30.0          # 드는 높이 [mm]
STRIDE_USE = 0.8        # 계산한 최대 보폭 중 이만큼만 씀 (착지 오차 여유)
STRIDE_MAX = 120.0      # 보폭 상한 [mm] (이웃 발끼리 부딪히지 않게)
ACCEL = 250.0           # 몸 가속 한계 [mm/s²]
W_ACCEL = 1.5           # 회전 가속 한계 [rad/s²]
JOINT_SPEED_MAX = 300.0 # 관절 최고 속도 [도/s] (MG996R 무부하 ≈ 350). 넘는 틱은 시간을 늦춰 계산
SETTLE_TOL = 5.0        # 이 안이면 제자리로 본다 [mm]
LOOKAHEAD = 0.3         # 바닥 다리가 이 시간 뒤에 못 닿을 것 같으면 미리 줄인다 [s]

ST, SW = 0, 1


def _smoothstep(x):
    x = 0.0 if x < 0 else 1.0 if x > 1 else x
    return x * x * (3.0 - 2.0 * x)


def _rotz(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y


class IkTripodGait:
    def __init__(self, stand_pose=None):
        self.stand_pose = [row[:] for row in (stand_pose or lm.STAND_POSE)]
        self.home = [lm.fk(l, self.stand_pose[l]) for l in range(lm.NUM_LEGS)]
        bad = [l for l in range(lm.NUM_LEGS) if lm.ik(l, self.home[l]) is None]
        if bad:
            raise ValueError(f"서기 자세가 허용 범위 밖인 다리: {bad} — leg_model 의 캘리브레이션/한계를 확인할 것")
        self.ground = sum(h[2] for h in self.home) / lm.NUM_LEGS
        self.t_swing = PERIOD * SWING_FRAC
        self.t_stance = PERIOD * (1.0 - SWING_FRAC)
        self.stride = min(self._capacity(), STRIDE_MAX) * STRIDE_USE
        self.stats = {"ticks": 0, "slowed": 0, "reduced": 0, "held": 0}
        self.reset()

    # ───────────── 상태 ─────────────
    def reset(self):
        """서기 자세(모든 발 제자리)에서 다시 시작. 서기·비상정지·힘 빼기 뒤에 부른다."""
        self.feet = [h[:] for h in self.home]
        self.leg = [ST] * lm.NUM_LEGS
        self.u = [0.0] * lm.NUM_LEGS
        self.p0 = [h[:] for h in self.home]
        self.tgt = [h[:2] for h in self.home]
        self.phase = 0.0
        self.active = False
        self.v = [0.0, 0.0]       # 몸 속도 [mm/s]
        self.w = 0.0              # [rad/s]
        self.pose_true = [row[:] for row in self.stand_pose]   # 마지막으로 낸 자세 (실제 관절 각도)

    # ───────────── 보폭 한계 ─────────────
    def _capacity(self):
        """모든 다리가 원래 자리 ± 반 걸음까지 (어느 방향이든) 닿고, 옮기는 중간에 LIFT_MM 을 들 수 있는 보폭 [mm]."""
        dirs = [(math.cos(k * math.pi / 6), math.sin(k * math.pi / 6)) for k in range(12)]
        checks = ((0.5, 0.0), (0.25, LIFT_MM), (0.0, LIFT_MM))

        def ok(S):
            for l, h in enumerate(self.home):
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

    def stand_command(self):
        """서기 자세를 펌웨어로 보낼 각도 (캘리브레이션 적용). firmware_link.stand() 대신 이걸 보낸다."""
        return lm.pose_to_firmware(self.stand_pose)

    def walk_warning(self, min_stride=30.0):
        """걷기 어려우면 이유를 글로 (괜찮으면 None)."""
        if self.stride >= min_stride:
            return None
        tight = lm.stand_margins()
        why = ", ".join(f"다리 {l} {n} 범위 끝까지 {m:.0f}° ({src})" for l, n, m, src in tight) or "알 수 없음"
        fix = []
        if any(src.startswith("측정 1") for *_, src in tight):
            fix.append("측정점이 1개뿐인 관절이 있음 → calibrate_joints.py 로 ②③ 도 잴 것")
        if lm.USE_JOINT_POINTS and any(lm.calib_source(l, j) != "pose_gui" for l in range(6) for j in range(3)):
            fix.append("임시로 측정점 끄기: HEXAPOD_JOINT_POINTS=0")
        fix.append("측정해도 같으면 그 서보 혼을 몇 칸 돌려 다시 끼울 것 (서기 자세가 서보 끝에 붙어 있음)")
        return (f"보폭 {self.stride:.1f}mm — 걸을 수 없음 (서기 자세에서 다리를 {LIFT_MM:.0f}mm 들 여유가 없는 관절: {why})\n"
                + "\n".join("  → " + f for f in fix))

    def max_speed(self):
        """직진 최고 속도 [m/s] (보폭 ÷ 바닥 시간)."""
        return self.stride / self.t_stance / 1000.0

    def _limit_motion(self, v, w):
        """한 번 디딘 동안 발이 밀리는 거리(이동 + 회전)가 보폭을 넘지 않게 줄인다."""
        T = self.t_stance
        worst = 0.0
        for h in self.home:
            worst = max(worst, math.hypot(v[0] * T - w * T * h[1], v[1] * T + w * T * h[0]))
        k = 1.0 if worst <= self.stride or worst < 1e-9 else self.stride / worst
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
        h = self.home[l]
        x, y = _rotz(h[0], h[1], w * self.t_stance / 2)
        return [x + v[0] * self.t_stance / 2, y + v[1] * self.t_stance / 2]

    def _advance(self, dt, v, w, idle):
        """dt 만큼 진행한 새 상태 (feet, leg, u, p0, tgt, phase). self 는 바꾸지 않는다."""
        feet = [p[:] for p in self.feet]
        leg, u = self.leg[:], self.u[:]
        p0, tgt = [p[:] for p in self.p0], [t[:] for t in self.tgt]
        phase = self.phase

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
            sh = _smoothstep((u[l] - 0.12) / 0.76)               # 수평은 들고 난 뒤 시작, 내리기 전에 끝
            bump = math.sin(math.pi * u[l]) ** 2                # 뜰 때·닿을 때 수직 속도 0
            x = p0[l][0] + (tgt[l][0] - p0[l][0]) * sh
            y = p0[l][1] + (tgt[l][1] - p0[l][1]) * sh
            z = p0[l][2] + (self.ground - p0[l][2]) * _smoothstep(u[l]) + LIFT_MM * bump
            feet[l] = [x, y, z]
            if u[l] >= 1.0:
                leg[l] = ST
                feet[l] = [tgt[l][0], tgt[l][1], self.ground]

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
                    if idle and math.dist(feet[l][:2], self.home[l][:2]) < SETTLE_TOL:
                        continue                                # 이미 제자리 → 안 듦
                    leg[l], u[l], p0[l] = SW, 0.0, feet[l][:]
                    tgt[l] = self._land_target(l, v, w)
            phase = new % 1.0
        return feet, leg, u, p0, tgt, phase

    def _solve(self, feet):
        out = []
        for l in range(lm.NUM_LEGS):
            j = lm.ik(l, feet[l])
            if j is None:
                return None
            out.append(j)
        return out

    # ───────────── 매 제어 틱 ─────────────
    def update(self, vx, vy, wz, dt):
        """명령 (m/s, m/s, rad/s) → 펌웨어로 보낼 18관절 [도] (다리 6 × [hip, femur, tibia]).
        계산은 실제 관절 각도로 하고 (self.pose_true), 돌려줄 때 캘리브레이션(leg_model.to_firmware)을 적용한다."""
        self.stats["ticks"] += 1
        dt = max(0.0, min(dt, 0.1))
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

        # 시계를 돌릴지
        moving = math.hypot(*self.v) > 1.0 or abs(self.w) > 0.005
        far = max(math.dist(self.feet[l][:2], self.home[l][:2]) for l in range(lm.NUM_LEGS))
        if not idle or moving:
            self.active = True
        elif all(x == ST for x in self.leg) and far < SETTLE_TOL:
            self.active = False
            self.phase = 0.0

        # 계산 → 못 닿으면 몸 이동부터 줄이고, 관절이 너무 빨리 돌면 시간을 늦춘다
        step_max = JOINT_SPEED_MAX * max(dt, 1e-3)
        chosen = None
        for scale in (1.0, 0.5, 0.25, 0.125):
            found = None
            for mk in (1.0, 0.5, 0.25, 0.0):
                st = self._advance(dt * scale, [x * mk for x in self.v], self.w * mk, idle)
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
        (self.feet, self.leg, self.u, self.p0, self.tgt, self.phase), joints, mk, used_scale = chosen
        if used_scale < 1.0:
            self.stats["slowed"] += 1
        if mk < 1.0:
            self.stats["reduced"] += 1
            self.v, self.w = [x * mk for x in self.v], self.w * mk
        # 시간을 늦춰도 넘으면 마지막으로 관절 속도를 자른다 (서보에 무리한 점프를 안 보냄)
        out = []
        for l in range(lm.NUM_LEGS):
            row = []
            for i in range(3):
                d = joints[l][i] - self.pose_true[l][i]
                d = max(-step_max, min(step_max, d))
                row.append(self.pose_true[l][i] + d)
            out.append(row)
        self.pose_true = out
        return lm.pose_to_firmware(out)
