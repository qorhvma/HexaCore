"""
계단 오르기 — 정해진 순서 그대로 한 번 수행

  /usr/bin/python3 hexapod_stair_seq.py --robot AA:BB:CC:DD:EE:FF --pause     (단계마다 Enter)
  /usr/bin/python3 hexapod_stair_seq.py --robot AA:BB:CC:DD:EE:FF

다리 배치 (가는 방향 = 1·6번 사이, 150°):
    앞   = 1번(왼)  6번(오른)
    중간 = 2번(왼)  5번(오른)   ← 2·5번은 서로 일직선 (60° / -120°). '평행하게 접기' 의 기준
    뒤   = 3번(왼)  4번(오른)

순서 (★ 괄호 안은 설정 이름)
   1) 몸을 BODY_HEIGHT(230mm) 까지 들어 올림
   2) 앞 다리(1·6)를 조금 접음: 2·5번과 평행하게 다 접으면 발이 FOLD_MARGIN(10mm) 더 뒤로 갈 수 있을 만큼만
      (다리 하나씩: 살짝 들기 → hip 돌리기 → 닿을 때까지 내림)
   3) 그 상태로 FIRST_ADVANCE(200mm) 전진
   4) 앞 다리를 2·5번과 평행하게 접음 (하나씩)
   5) 앞 다리 하나를 femur FOLD_FEMUR(60) · tibia FOLD_TIBIA(30) 로 접어 올림
   6) 그 다리 hip 을 정면(가는 방향)으로 돌림
   7) 접지 버튼이 닿을 때까지 내려놓음
   8) 나머지 앞 다리도 5~7
   9) FRONT_ADVANCE(50mm) 전진 — 앞 다리는 계단 위에 둔 채 당겨서 hip 이 원래(평행) 각도 쪽으로 돌아옴
  10) 중간 다리 하나씩: hip 을 MID_BACK_DEG 만큼 뒤로 + femur·tibia 접어 올림
      → hip 을 MID_PLACE_HIP(170) 으로 (다른 다리와 LEG_CLEAR 보다 가까워지면 거기까지만) → 닿을 때까지 내려놓음
  11) 앞 다리를 로봇 앞뒤 폭(앞 발 ~ 뒤 발)이 SPAN(290mm) 이 되게 접음 (하나씩)
  12) 전진 — 3)·14) 처럼 3개씩 번갈아 옮기고 당김 (REAR_ADVANCE_KEEP_FRONT = True 면 9) 처럼 앞 다리는 둔 채)
      — 뒤 다리가 계단에 닿을 만큼 (REAR_ADVANCE_PRE, None = 자동)
  12-1) 앞·중간 다리 hip 을 하나씩 평행(2·5번 방향)으로 맞춤 (계단 밖으로 나가면 덜 돌림)
  13) 뒤 다리 하나씩: hip 을 REAR_BACK_DEG 만큼 뒤로 + 접어 올림 → 평행(2·5번 방향)으로 (다른 다리와 LEG_CLEAR 까지)
      → 닿을 때까지 내려놓음
  14) REAR_ADVANCE(50mm) 전진

수평: 시작할 때 낮게 선 채로 IMU 방향을 재고(몸을 ±3° 기울여 봄), 위 단계가 하나 끝날 때마다(5~7·10·13 은 다리마다)
  IMU 가 수평이 되게 몸을 돌려 맞춤 (발은 그대로). IMU 가 없으면 자동으로 안 함 (LEVEL = False 로 끌 수 있음)

전진 방법: 다리 3개(삼각형)를 하나씩 앞으로 옮겨 놓고(들기 → 앞으로 → 닿을 때까지 내림) 그 3개로 몸을 끌어당김.
  나머지 3개는 SKID_HOVER 만큼 살짝 떠서 따라옴 → 당긴 뒤 다시 내려놓음. 다른 3개로 바꿔서 반복.
  9) 에서는 앞 다리 2개는 계단 위에 둔 채 계속 당기고(안 듦), 나머지 4개가 2개씩 번갈아 옮겨짐.
Ctrl+C = 서기 자세로 돌아간 뒤 힘 빼기
"""

import argparse
import math
import select
import sys
import time

from hexapod_body import COXA_LEN, FEMUR_LEN, HIP_CENTER, HIP_SIGN, MOUNT, TIBIA_LEN, _mul_t, _rot, leg_fk, leg_ik
from hexapod_sdk import JOINT_LIMITS, NUM_JOINTS, NUM_LEGS, STAND_POSE, Hexapod, Link

# ═══════════════════════════ 설정 ═══════════════════════════
ROBOT = "AA:BB:CC:DD:EE:FF"   # 내 ESP32 블루투스 주소로 바꿀 것 (펌웨어가 켜질 때 출력)

# 다리 (1번부터 센 번호)
FRONT = [1, 6]
MIDDLE = [2, 5]
REAR = [3, 4]
PARALLEL_TO = 2         # 이 다리 방향(과 그 반대 방향)으로 '평행하게' 접음 (2번 = 60°, 5번 = -120°)
FORWARD_DEG = 150.0     # 가는 방향 [도, 몸 기준 반시계, 0 = 3번 쪽] (1·6번 사이)
TRIPODS = [[1, 3, 5], [2, 4, 6]]   # 전진할 때 번갈아 옮기고 당기는 3개 묶음

# 1) 몸 높이
START_HEIGHT = 70.0     # 처음 서는 높이 [mm]
BODY_HEIGHT = 230.0     # 들어 올릴 몸 높이: hip 축 ~ 발끝 [mm] (230 이면 발이 hip 에서 약 95mm 안쪽에만 닿음)
STANCE_REACH = 70.0     # 서 있을 때 hip 축 ~ 발끝 수평 거리 [mm]
RAISE_SPEED = 15.0*3      # 몸 올리는 속도 [mm/s]

# 2)·4)·11) 앞 다리 접기
FOLD_MARGIN = 10.0*3      # 2) 평행까지 다 접으면 발이 이만큼 더 뒤로 갈 수 있게 남겨 둠 [mm]
SHIFT_LIFT = 20.0       # 다리 자리를 옮길 때(접기) 살짝 드는 높이 [mm]
SPAN = 290.0            # 11) 로봇 앞뒤 폭 = 가는 방향으로 앞 발 ~ 뒤 발 거리 [mm]

# 3)·9)·12)·14) 전진 거리
FIRST_ADVANCE = 200.0/1.5
FRONT_ADVANCE = 200/1.5
REAR_ADVANCE_PRE = None # 12) 뒤 다리 들기 전에 전진하는 거리 [mm]. None = 뒤 다리를 앞으로 놓을 자리가
                        #   지금 중간 다리 발 자리(확실히 계단 위)보다 REAR_STEP_MARGIN 앞이 될 만큼
REAR_STEP_MARGIN = 20.0 # 12) [mm]
REAR_ADVANCE_MAX = 200.0  # 12) 이보다 많이는 안 감 [mm]
REAR_ADVANCE_KEEP_FRONT = False  # 12) True = 9) 처럼 앞 다리는 계단 위에 둔 채 당김 / False = 3)·14) 처럼 3개씩 번갈아 옮기고 당김
                        #   (앞 다리는 9)·11) 뒤라 hip 쪽으로 많이 당겨져 있어서 True 면 거의 못 감)
STEP_DROP_TOL = 15.0    # 12-1) 다리를 평행으로 옮겼는데 원래보다 이만큼 넘게 낮은 데 닿으면 계단 밖 → 덜 돌려서 다시 놓음 [mm]
REAR_ADVANCE = 50.0*2
STRIDE = 60.0*0.7           # 보폭: 한 번 끌어당기는 최대 거리 [mm]. 다리마다 닿는 범위 안에서 가운데로 놓고,
                        #   묶음 3개 중 하나라도 그만큼 못 가면 그 묶음은 갈 수 있는 만큼만 (몸 높이 230 이면 보통 30~60)
STRIDE_FIRST = None     # 단계별 보폭 [mm] (None = STRIDE): 3) 계단까지
STRIDE_FRONT = None     #   9) 앞 다리를 계단에 두고
STRIDE_REAR_PRE = None  #   12) 뒤 다리 올리기 전
STRIDE_LAST = None      #   14) 마지막
STRIDE_MIN = 10.0       # 이보다 작게밖에 못 가면 그 묶음은 이번에 안 옮김 (다른 묶음 차례로) [mm]
STEP_LIFT = 30.0        # 전진할 때 옮기는 다리를 드는 높이 [mm]
SKID_HOVER = 3.0        # 당기는 동안 안 당기는 다리를 띄우는 높이 [mm]
PULL_SPEED = 20.0*2       # 끌어당기는 몸 속도 [mm/s]

# 5)·10)·13) 접어 올리기
FOLD_FEMUR = 60       # 접어 올릴 때 femur [도] (작을수록 위로). ★ SDK 하한이 62 라 그보다 작으면 62
FOLD_TIBIA = 60.0       # 접어 올릴 때 tibia [도] (작을수록 많이 접힘)
FOLD_SPEED = 60.0       # 접고 펴는 관절 속도 [도/s]
PLACE_REACH = 120.0     # 7)·10)·13) 내려놓을 때 hip 축 ~ 발끝 수평 거리 [mm] (낮은 데까지 내려가면 닿는 만큼 안쪽으로 줄임)
MID_PLACE_HIP = 150.0   # 10) 중간 다리를 계단에 놓을 hip 각도 [도] (180 = 평행, 작을수록 앞쪽, 정면은 약 95)
MID_BACK_DEG = 10.0     # 10) 접어 올리기 전에 중간 다리 hip 을 뒤로 돌리는 각도 [도]
REAR_BACK_DEG = 10.0    # 13) 뒤 다리 hip 을 뒤로 살짝 돌리는 각도 [도]
LEG_CLEAR = 45.0        # 다리끼리 간섭 방지: 다리를 옮길 자리·접은 자세를 고를 때 다른 다리(무릎~발끝 전체)와 이만큼 띄움 [mm]
                        #   (femur·tibia 선분 사이 거리. 서보·다리 두께가 있으니 0 이 아니라 이 값으로)
LEG_CLEAR_STOP = 25.0   # 움직이는 중(옮기기·당기기)에 다른 다리와 이보다 가까워지면 그 자리에서 멈춤 [mm]
HIP_REACH_MAX = 85.0    # hip 이 180 에서 꺾여도 되는 최대 [도] (서보 범위 180 ± 90)

# 내려놓기SPEED
DESCEND_SPEED = 20.0*2    # 내리는 속도 [mm/s]
MAX_DOWN = 15.0         # 처음 바닥(몸 높이)보다 이만큼 더 내려도 안 닿으면 멈춤 [mm]
STAIR_HEIGHT = 160.0    # 오를 계단 높이 [mm] (재 보니 버튼은 약 169 에서 눌림 — 몸이 처지는 만큼 더 높게 읽힘)
STAIR_HEIGHT_TOL = 25.0 # 계단 높이 오차 [mm]
STAIR_MAX_H = STAIR_HEIGHT + STAIR_HEIGHT_TOL   # 이보다 높은 눌림은 '있을 수 없는 높이' 로 무시.
                        #   계단을 아직 못 찾았을 때 전진하며 옮기는 다리도 이보다 높이 듦
STAIR_DETECT_MM = STAIR_HEIGHT / 2   # 버튼이 처음 바닥보다 이만큼 넘게 위에서 눌리면 '계단 발견' [mm]
STOP_AT_STAIR = True    # 3) 전진 중 계단을 발견하면 FIRST_ADVANCE 를 다 안 가도 거기서 멈추고 4) 로
STEP_CLEAR_TOL = 10.0   # 옮기는 다리가 알려진 계단 윗면보다 이만큼 넘게 낮으면 계단 위 높이까지 들고 옮김 [mm]
STEP_HEIGHT_GUESS = STAIR_HEIGHT  # 버튼 고장 다리를 처음 계단에 놓을 때 쓰는 높이 [mm] (처음 바닥 기준, None = 모름)
DEAD_BUTTON_N = 3       # 끝까지 내려도 버튼이 이만큼 연달아 안 눌리면 고장으로 봄 (그 뒤엔 다른 다리가 닿은 계단 높이까지만 내림)
PRESS_TICKS = 3         # 버튼이 이만큼(틱, 50Hz 면 0.06초) 연달아 눌려야 '닿음' (튀는 값 거름)
EXPECT_TOL = 12.0       # 아는 바닥(이 다리 마지막 바닥·계단 윗면)으로 다시 내릴 때 그 높이 ± 이 안의 눌림만 믿음 [mm]
NEAR_GROUND_MM = 10.0   # 마지막 진짜 바닥 높이 + 이 안까지 내려오면 '떨어짐'을 못 봤어도 눌림을 믿음 [mm]
RELEASE_TICKS = 5       # 버튼이 이만큼(틱) 연달아 떨어진 걸 본 뒤에야 눌림을 믿음 (접혀서 눌린 버튼 거름)
LIFT_SPEED = 50.0*2*2       # 곧게 드는 속도 [mm/s]
MOVE_SPEED = 80.0*2       # 든 다리를 옮기는 발끝 속도 [mm/s]

# 수평 맞추기 (BNO086 IMU) — 단계마다 끝나면 몸을 돌려 IMU 가 0(수평 기준)이 되게 맞춤. 발은 그대로
LEVEL = True            # IMU 가 없거나 시작 측정이 이상하면 자동으로 꺼짐
IMU_CAL_DEG = 3.0       # 시작할 때(낮게 선 자세) IMU 방향을 재려고 몸을 기울여 보는 각도 [도]
IMU_CAL_MIN_GAIN = 0.3  # 명령 1° 에 IMU 가 이보다 덜 변하면 이상한 것으로 보고 끔
IMU_SETTLE = 0.4        # 몸을 움직인 뒤 IMU 를 읽기 전에 기다리는 시간 [초]
IMU_AVG_N = 20          # IMU 평균 낼 샘플 수
LEVEL_DEADBAND = 0.3    # 이보다 덜 기울었으면 안 맞춤 [도]
LEVEL_MAX_DEG = 10.0    # 몸을 이보다 많이 돌려서 맞추진 않음 [도]
LEVEL_ITERS = 6         # 한 번 맞출 때 최대 반복 (읽기 → 돌리기)
LEVEL_SPEED = 10.0*2      # 몸 돌리는 속도 [도/초]
LEVEL_FAIL_MAX = 3      # '맞췄더니 더 기울어짐' 이 연달아 이만큼 나면 수평 맞추기를 끔 (그 전엔 되돌리고 다음에 다시)
LEVEL_EACH_LEG = True   # 전진할 때 다리 하나를 놓을 때마다 수평 맞춤 (아래 LEVEL_WHILE_WALKING 은 당길 때마다)
LEVEL_WHILE_WALKING = True  # 전진할 때 한 번 당길 때마다(STRIDE) 수평 맞춤 (단계 끝에서도 맞춤)
# 꼼꼼하게 맞추기 — 뒤 다리를 들기 전(13), 12-1 뒤: 더 오래 기다리고·더 많이 평균 내고·두 번 읽어 값이 같아질 때까지
STRICT_SETTLE = 0.8     # 읽기 전에 기다리는 시간 [초]
STRICT_AVG_N = 50       # 평균 낼 샘플 수
STRICT_STABLE_DEG = 0.2 # 0.3초 간격으로 두 번 읽은 값이 이 안으로 같아야 '흔들림 멈춤'으로 봄 [도] (안 되면 최대 2초 더 기다림)
STRICT_DEADBAND = 0.2   # 이보다 덜 기울었을 때만 '수평' [도]
STRICT_ITERS = 10       # 최대 반복
STRICT_OK_DEG = 1.0     # 다 맞춰도 이보다 기울어 있으면 경고 (--pause 면 계속할지 물어봄) [도]

STOP_KEY = "s"          # 전진 중에 이 키를 누르면 그 전진을 멈추고 다음 단계로 (Enter 없이 바로). 옮기던 다리는 내려놓고 멈춤
START_SEC = 2.0
LOOP_HZ = 50
# ═══════════════════════════════════════════════════════════

L = lambda n: n - 1                                     # 1번부터 → 0번부터


class KeyStop:
    """with 블록 안에서 터미널을 '한 글자씩' 모드로 바꿔 STOP_KEY 가 눌렸는지 봄 (Enter 필요 없음, Ctrl+C 는 그대로).
    터미널이 아니면 아무것도 안 함."""

    def __init__(self):
        self.hit = False
        self.old = None

    def __enter__(self):
        try:
            import termios
            import tty
            if sys.stdin.isatty():
                self.old = termios.tcgetattr(sys.stdin.fileno())
                tty.setcbreak(sys.stdin.fileno())
        except Exception:
            self.old = None
        return self

    def pressed(self):
        if self.hit or self.old is None:
            return self.hit
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch.lower() == STOP_KEY.lower():
                self.hit = True
                print(f"\n[전진] '{STOP_KEY}' 키 — 지금 하던 다리까지만 하고 전진을 멈춤")
                break
        return self.hit

    def __exit__(self, *exc):
        if self.old is not None:
            import termios
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)    # 남은 키 버림 (다음 Enter 대기에 안 섞이게)
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.old)
        return False


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def unit(deg):
    return math.cos(math.radians(deg)), math.sin(math.radians(deg))


class Seq:
    def __init__(self, link, pause=False):
        self.link = link
        self.hx = Hexapod()
        self.dt = 1.0 / LOOP_HZ
        self.pause = pause
        self.contact_on = True
        self.feet = None
        self.fwd = unit(FORWARD_DEG)
        self.front, self.middle, self.rear = [L(n) for n in FRONT], [L(n) for n in MIDDLE], [L(n) for n in REAR]
        par = MOUNT[L(PARALLEL_TO)][2]
        self.side = []                                  # 다리마다 '평행' 방향 (몸 바깥쪽)
        for mx, my, _ in MOUNT:
            ux, uy = unit(par)
            self.side.append(par if mx * ux + my * uy >= 0 else wrap180(par + 180))
        self.ground_z = [-START_HEIGHT] * NUM_LEGS      # 다리마다 마지막으로 진짜 닿은 바닥 높이 (몸 기준 z)
        self.dead = set()                               # 버튼 고장으로 본 다리
        self.step_z = None                              # 버튼으로 찾은 계단 윗면 높이 (몸 기준 z)
        self.approach = False                           # 3) 계단 찾는 중
        self.keys = None                                # 전진 중 STOP_KEY 감시
        self.miss = [0] * NUM_LEGS                      # 연달아 안 닿은 횟수
        self.imu_on = False
        self.level_fails = 0                            # '맞출수록 더 기울어짐' 연달아 난 횟수
        self.imu_inv = None                             # IMU 변화 → 몸 돌림 명령 (2x2)
        self.tilt = [0.0, 0.0]                          # 지금까지 몸을 돌린 양 (roll, pitch)
        self.fold_femur = max(FOLD_FEMUR, max(JOINT_LIMITS[l][1][0] for l in range(NUM_LEGS)))
        if self.fold_femur != FOLD_FEMUR:
            print(f"[설정] FOLD_FEMUR {FOLD_FEMUR:.0f} 은 SDK 하한보다 작아서 {self.fold_femur:.0f} 로 접음")

    # ── 다리끼리 간섭 ──
    @staticmethod
    def leg_points(l, j):
        """관절 각도 → coxa 끝, 무릎, 발끝 (몸 기준)."""
        hip, fem, tib = j
        psi = math.radians((hip - HIP_CENTER) * HIP_SIGN[l])
        a, b = math.radians(180 - fem), math.radians(tib)
        ta = a - math.pi + b
        mx, my, th = MOUNT[l]
        c, s = math.cos(math.radians(th) + psi), math.sin(math.radians(th) + psi)
        kr, kz = COXA_LEN + FEMUR_LEN * math.cos(a), FEMUR_LEN * math.sin(a)
        fr, fz = kr + TIBIA_LEN * math.cos(ta), kz + TIBIA_LEN * math.sin(ta)
        return [(mx + COXA_LEN * c, my + COXA_LEN * s, 0.0), (mx + kr * c, my + kr * s, kz), (mx + fr * c, my + fr * s, fz)]

    @staticmethod
    def leg_samples(pts, n=8):
        return [tuple(a + (b - a) * i / n for a, b in zip(A, B)) for A, B in zip(pts, pts[1:]) for i in range(n + 1)]

    def clearance(self, l, j, poses=None):
        """다리 l 이 관절 j 일 때 다른 다리들과의 가장 가까운 거리 [mm] (femur·tibia 선분 기준).
        다른 다리 자세는 지금 자세, poses = {다리: 관절} 에 있으면 그 자세."""
        mine = self.leg_samples(self.leg_points(l, j))
        best = 1e9
        for n in range(NUM_LEGS):
            if n == l:
                continue
            jn = poses[n] if poses and n in poses else self.hx.cmd[n]
            for q in self.leg_samples(self.leg_points(n, jn)):
                for p in mine:
                    d = math.dist(p, q)
                    if d < best:
                        best = d
        return best

    def clear_ok(self, l, j):
        return j is not None and self.clearance(l, j) >= LEG_CLEAR

    # ── 기본 ──
    def send_joints(self, js):
        self.hx.set_pose(js)
        self.hx.move_update(self.link)
        if not self.hx.state_known:
            why = [m for m in self.link.log_queue if "전송" in m or "수신" in m or "형식" in m or "연결" in m]
            raise ConnectionError("로봇 전송 실패" + (f" — {why[-1]}" if why else ""))

    def send(self, feet):
        js = [leg_ik(l, f) for l, f in enumerate(feet)]
        if any(j is None for j in js):
            return False
        self.send_joints(js)
        self.feet = [list(f) for f in feet]
        return True

    def contact(self, l):
        return self.link.foot_fresh() and bool(self.link.foot[l])

    def wait(self, what):
        while self.link.log_queue:
            print("[로봇]", self.link.log_queue.popleft())
        if self.pause:
            input(f"\n   ⏸  {what} — Enter ")
        else:
            print(f"\n[순서] {what}")

    def along(self, p):
        return p[0] * self.fwd[0] + p[1] * self.fwd[1]

    def foot_at(self, l, deg, r, z):
        mx, my, _ = MOUNT[l]
        ux, uy = unit(deg)
        return [mx + r * ux, my + r * uy, z]

    def dir_reach(self, l, p=None):
        """발 위치의 hip 축 기준 방향 [도] 와 수평 거리."""
        p = self.feet[l] if p is None else p
        mx, my, _ = MOUNT[l]
        return math.degrees(math.atan2(p[1] - my, p[0] - mx)), math.hypot(p[0] - mx, p[1] - my)

    def clamp_dir(self, l, deg):
        off = wrap180(deg - MOUNT[l][2])
        return MOUNT[l][2] + max(-HIP_REACH_MAX, min(HIP_REACH_MAX, off))

    def hip_of(self, deg, l):
        return HIP_CENTER + HIP_SIGN[l] * wrap180(deg - MOUNT[l][2])

    def make_touched(self, l, expect=None):
        """내리는 동안 부를 '닿았나' 판정 함수.
        - 버튼이 RELEASE_TICKS 연달아 떨어진 걸 먼저 봐야 그다음 눌림을 믿음 (공중에서 접혀 눌린 버튼·눌린 채 남은 값 거름)
          단, 발이 이 다리의 마지막 진짜 바닥(ground_z) + NEAR_GROUND_MM 안까지 내려왔으면 떨어짐을 못 봤어도 믿음
          (살짝만 띄운 다리는 버튼이 안 떨어질 수 있어서)
        - 눌림은 PRESS_TICKS 연달아 있어야 '닿음' (한 번 튀는 값 거름)
        - 있을 수 없는 높이의 눌림은 무시: 이 다리 바닥(또는 아는 계단)보다 STAIR_MAX_H 넘게 위
          expect = [바닥 높이들] 을 주면 그 높이들 ± EXPECT_TOL 근처(또는 그보다 아래)에서만 믿음"""
        st = {"off": 0, "rel": False, "on": 0, "warned": False}

        def plausible(z):
            top = max(self.ground_z[l], self.step_z if self.step_z is not None else -1e9) + STAIR_MAX_H + EXPECT_TOL
            if z > top:
                return False
            if expect:
                return z <= min(e for e, _ in expect) + EXPECT_TOL or any(abs(z - e) <= t for e, t in expect)
            return True

        def touched():
            if not self.contact_on:
                return False
            pressed = self.contact(l)
            if not st["rel"]:
                st["off"] = 0 if pressed else st["off"] + 1
                if st["off"] >= RELEASE_TICKS:
                    st["rel"] = True
                elif self.feet[l][2] > self.ground_z[l] + NEAR_GROUND_MM:
                    st["on"] = 0
                    return False
            st["on"] = st["on"] + 1 if pressed else 0
            if st["on"] >= PRESS_TICKS:
                z = self.feet[l][2]
                if plausible(z):
                    return True
                if not st["warned"]:
                    st["warned"] = True
                    print(f"[접지] 다리 {l + 1} 공중(처음 바닥보다 {z + BODY_HEIGHT:+.0f}mm)에서 눌림 — 있을 수 없는 높이라 무시")
                st["on"] = 0
            return False
        return touched

    # ── 움직임 ──
    def move_feet(self, targets, speed):
        start = {l: list(self.feet[l]) for l in targets}
        dist = max([math.dist(start[l], targets[l]) for l in targets] + [0.0])
        n = max(1, round(dist / speed * LOOP_HZ))
        for k in range(1, n + 1):
            feet = [list(f) for f in self.feet]
            for l in targets:
                feet[l] = [a + (b - a) * k / n for a, b in zip(start[l], targets[l])]
            if not self.send(feet):
                return False
            time.sleep(self.dt)
        return True

    def move_joints(self, l, j1, deg_speed=FOLD_SPEED, touched=None, foot_speed=None):
        """다리 l 을 관절 공간으로 j1 까지. touched 가 참이 되면 멈추고 True."""
        j0 = list(self.hx.cmd[l])
        t = max(abs(a - b) for a, b in zip(j0, j1)) / deg_speed
        if foot_speed:
            pts = [leg_fk(l, [a + (b - a) * i / 20 for a, b in zip(j0, j1)]) for i in range(21)]
            t = max(t, sum(math.dist(pts[i], pts[i + 1]) for i in range(20)) / foot_speed)
        n = max(1, round(t * LOOP_HZ))
        prev = self.clearance(l, j0)
        for k in range(1, n + 1):
            if touched is not None and touched():
                return True
            js = [row[:] for row in self.hx.cmd]
            js[l] = [a + (b - a) * k / n for a, b in zip(j0, j1)]
            c = self.clearance(l, js[l])
            if c < LEG_CLEAR_STOP and c < prev:
                print(f"[다리 {l + 1}] 다른 다리와 {c:.0f}mm — 부딪힐 것 같아 여기서 멈춤")
                return touched is not None and touched()
            prev = c
            self.send_joints(js)
            self.feet[l] = leg_fk(l, js[l])
            time.sleep(self.dt)
        return touched is not None and touched()

    def straight_up(self, l, lift):
        """곧게 lift 만큼 (못 들면 들 수 있는 만큼) 들기."""
        f = self.feet[l]
        lo, hi = 0.0, lift
        if leg_ik(l, [f[0], f[1], f[2] + lift]) is None:
            for _ in range(20):
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if leg_ik(l, [f[0], f[1], f[2] + mid]) is not None else (lo, mid)
            hi = max(lo - 1.0, 0.0)
        self.move_feet({l: [f[0], f[1], f[2] + hi]}, LIFT_SPEED)

    def descend(self, l, touched, tag="", floor=None):
        """지금 방향 그대로 천천히 곧게 내리다 버튼이 눌리면 멈춤. 낮은 데서 안 닿으면 발을 hip 쪽으로 당기며 내림.
        floor(몸 기준 z) 까지 내려도 안 닿으면 거기서 멈춤 (None = 처음 바닥 - MAX_DOWN). 닿은 높이를 돌려줌.
        '버튼 고장'으로 본 다리도 버튼은 계속 봄 — 눌리면 거기서 멈추고 고장 표시를 지움.
        안 닿음은 floor 가 None(끝까지 내림)일 때만 고장 판단에 셈 (같은 바닥으로 옮기며 MAX_DOWN 만 더 내린 건 안 셈)."""
        count_miss = floor is None
        floor = -BODY_HEIGHT - MAX_DOWN if floor is None else floor
        deg, r = self.dir_reach(l)
        z = self.feet[l][2]
        while True:
            if self.contact_on and touched():
                self.miss[l] = 0
                if l in self.dead:
                    self.dead.discard(l)
                    print(f"[접지] 다리 {l + 1} 버튼이 다시 눌림 — 고장 표시 지움")
                print(f"[다리 {l + 1}] {tag}닿음 — 처음 바닥보다 {z + BODY_HEIGHT:+.0f}mm")
                self.saw_height(l, z)
                return z
            nz = z - DESCEND_SPEED * self.dt
            q = self.foot_at(l, deg, r, nz)
            while leg_ik(l, q) is None and r > COXA_LEN + 10:
                r -= 1.0
                q = self.foot_at(l, deg, r, nz)
            if nz < floor or leg_ik(l, q) is None:
                if l in self.dead or not self.contact_on:
                    print(f"[다리 {l + 1}] {tag}버튼 안 눌림 (고장으로 봄) — 처음 바닥보다 {z + BODY_HEIGHT:+.0f}mm 에 놓음")
                    return z
                print(f"[다리 {l + 1}] {tag}안 닿음 (처음 바닥보다 {z + BODY_HEIGHT:+.0f}mm 에서 멈춤)")
                if count_miss:
                    self.miss[l] += 1
                    if self.miss[l] >= DEAD_BUTTON_N:
                        self.dead.add(l)
                        print(f"[접지] ★ 다리 {l + 1} 버튼이 끝까지 내려도 {self.miss[l]}번 연달아 안 눌림 — 고장으로 봄"
                              " (버튼은 계속 보지만, 계단에 놓을 땐 다른 다리가 닿은 높이까지만 내림)")
                return z
            feet = [list(p) for p in self.feet]
            feet[l] = q
            self.send(feet)
            z = nz
            time.sleep(self.dt)

    def shift_leg(self, l, deg, r=None, lift=SHIFT_LIFT, tag="", allow_drop=False):
        """다리 하나 자리 옮기기: 살짝 들기 → hip 돌려 deg 방향(거리 r)으로 → 닿을 때까지 내림.
        같은 바닥으로 옮기는 거라 원래 높이 - MAX_DOWN 아래로는 안 내림 (버튼이 안 눌려도 다리를 다 펴지 않게).
        allow_drop = 계단 밖으로 나가 더 낮은 데 닿는 것도 봄 (처음 바닥 - MAX_DOWN 까지). 버튼 고장 다리는 원래 높이에 놓음."""
        _, r0 = self.dir_reach(l)
        r = r0 if r is None else r
        z_start = self.feet[l][2]
        clear = False                                   # 계단 위로 넘겨야 하는 다리
        g = self.ground_z[l]                            # 이 다리의 마지막 진짜 바닥 (공중에 멈춰 있어도 이 기준)
        if self.step_z is not None and g < self.step_z - STEP_CLEAR_TOL:
            # 계단보다 낮은 데 있는 다리 → 계단 윗면보다 lift 만큼 높이 들고 옮김 (안 그러면 계단 앞면에 발이 박힘)
            need = self.step_z + lift - z_start
            if need > lift:
                print(f"[다리 {l + 1}] 계단보다 낮은 데 있음 → 계단 위로 {need:.0f}mm 들고 옮김")
                lift, clear = need, True
        elif self.step_z is None and lift > STEP_LIFT:
            clear = True                                # 계단을 찾는 중 (높이 들어야 함)
        if l in self.dead or not self.contact_on:
            floor = g
        elif allow_drop:
            floor = None
        else:
            floor = g - MAX_DOWN
        touched = self.make_touched(l, None if allow_drop else self.known_heights(l))
        self.straight_up(l, lift)
        z = self.feet[l][2]
        reach_ok = any(leg_ik(l, self.foot_at(l, deg, rr, z)) is not None for rr in (r, r - 20, r - 40) if rr > COXA_LEN + 10)
        if clear and (z - z_start < lift - 5.0 or not reach_ok):   # 곧게는 계단 위까지 못 듦 / 그 높이에서 못 뻗음 → 접어 넘김
            why = (f"곧게는 {z - z_start:.0f}mm 밖에 못 들어서 (필요 {lift:.0f}mm)" if z - z_start < lift - 5.0
                   else "든 높이에서 놓을 자리까지 못 뻗어서")
            print(f"[다리 {l + 1}] {why} 접어 올려 넘김")
            self.fold_up(l)
            self.turn_folded(l, deg)
            toe = self.feet[l][2]
            jt = None
            for zz in [toe - 5.0 * i for i in range(80)]:
                rr = r
                jt = leg_ik(l, self.foot_at(l, deg, rr, zz))
                while jt is None and rr > COXA_LEN + 10:
                    rr -= 5.0
                    jt = leg_ik(l, self.foot_at(l, deg, rr, zz))
                if jt is not None:
                    break
            if jt is not None and self.move_joints(l, jt, touched=touched if self.contact_on else None,
                                                   foot_speed=DESCEND_SPEED * 2):
                print(f"[다리 {l + 1}] {tag}펴는 중에 닿음 — 처음 바닥보다 {self.feet[l][2] + BODY_HEIGHT:+.0f}mm")
                self.saw_height(l, self.feet[l][2])
                return self.feet[l][2]
            return self.descend(l, touched, tag, floor)
        j = leg_ik(l, self.foot_at(l, deg, r, z))
        while j is None and r > COXA_LEN + 10:
            r -= 2.0
            j = leg_ik(l, self.foot_at(l, deg, r, z))
        if j is not None and not self.clear_ok(l, j):    # 다른 다리와 가까우면 지금 자리 쪽으로 덜 옮김
            d0, r0 = self.dir_reach(l)
            span = wrap180(deg - d0)
            for i in range(19, -1, -1):
                u = i / 20
                jj = leg_ik(l, self.foot_at(l, d0 + span * u, r0 + (r - r0) * u, z))
                if self.clear_ok(l, jj) or i == 0:
                    print(f"[다리 {l + 1}] 다른 다리와 {LEG_CLEAR:.0f}mm 안으로 들어가서 {u * 100:.0f}% 만 옮김"
                          f" (hip {self.hip_of(d0 + span * u, l):.0f})")
                    j = jj
                    break
        if j is None:
            print(f"[다리 {l + 1}] {deg:.0f}° 방향으로 못 옮김 — 제자리에 내림")
        else:
            self.move_joints(l, j, foot_speed=MOVE_SPEED)
        return self.descend(l, touched, tag, floor)

    def fold_up(self, l, hip_deg=None):
        """femur·tibia 를 FOLD 각도로 접어 올림 (hip_deg 를 주면 hip 도 그 방향으로 같이)."""
        hip = self.hx.cmd[l][0] if hip_deg is None else self.hip_of(hip_deg, l)
        self.move_joints(l, [hip, self.fold_femur, FOLD_TIBIA])
        print(f"[다리 {l + 1}] femur {self.fold_femur:.0f} · tibia {FOLD_TIBIA:.0f} 로 접어 올림"
              f" (발끝이 hip 축보다 {self.feet[l][2]:+.0f}mm)")

    def turn_folded(self, l, deg):
        """접은 채 hip 만 deg 방향으로."""
        j = self.hx.cmd[l]
        self.move_joints(l, [self.hip_of(deg, l), j[1], j[2]])

    def put_down(self, l, deg, tag=""):
        """접힌 다리를 deg 방향 PLACE_REACH 위로 펴면서 내리고(닿으면 멈춤) → 닿을 때까지 곧게 내림.
        버튼 고장 다리는 다른 다리가 계단에 닿았던 높이(self.step_z)까지만 내림 (모르면 처음 바닥까지 — 다리가 다 펴질 수 있음)."""
        touched = self.make_touched(l)
        floor = None
        if l in self.dead or not self.contact_on:
            if self.step_z is not None:
                floor = self.step_z
                print(f"[다리 {l + 1}] 버튼 고장 — 다른 다리가 닿았던 계단 높이(처음 바닥보다 {self.step_z + BODY_HEIGHT:+.0f}mm)에 놓음")
            elif STEP_HEIGHT_GUESS is not None:
                floor = -BODY_HEIGHT + STEP_HEIGHT_GUESS
                print(f"[다리 {l + 1}] 버튼 고장 — STEP_HEIGHT_GUESS({STEP_HEIGHT_GUESS:.0f}mm) 높이에 놓음")
            else:
                print(f"[다리 {l + 1}] ★ 버튼 고장인데 계단 높이를 아직 모름 — 처음 바닥까지 내림 (STEP_HEIGHT_GUESS 를 정하면 그 높이에)")
        z_top = self.feet[l][2]
        r = PLACE_REACH
        j = None
        for z in [z_top - 5.0 * i for i in range(60)]:   # 접힌 발끝 높이에서 닿는 가장 높은 데
            j = leg_ik(l, self.foot_at(l, deg, r, z))
            if j is not None:
                break
        if j is None:
            print(f"[다리 {l + 1}] PLACE_REACH {r:.0f}mm 위로 못 폄 — 접힌 자리에서 곧게 내림")
            return self.descend(l, touched, tag, floor)
        if floor is not None and leg_fk(l, j)[2] < floor:  # 펴는 자세가 계단 높이보다 낮으면 계단 높이까지만
            j2 = leg_ik(l, self.foot_at(l, deg, r, floor))
            j = j2 if j2 is not None else j
        if self.move_joints(l, j, touched=touched if self.contact_on else None, foot_speed=DESCEND_SPEED * 2):
            print(f"[다리 {l + 1}] {tag}펴는 중에 닿음 — 처음 바닥보다 {self.feet[l][2] + BODY_HEIGHT:+.0f}mm")
            self.saw_height(l, self.feet[l][2])
            return self.feet[l][2]
        return self.descend(l, touched, tag, floor)

    def known_heights(self, l):
        """이 다리가 내려앉을 수 있는 높이 [(z, 허용 오차)]: 마지막 진짜 바닥, 아는 계단 윗면
        (계단을 아직 못 찾았으면 바닥 + STAIR_HEIGHT ± STAIR_HEIGHT_TOL)."""
        out = [(self.ground_z[l], EXPECT_TOL)]
        if self.step_z is not None:
            out.append((self.step_z, EXPECT_TOL))
        elif STAIR_HEIGHT:
            out.append((self.ground_z[l] + STAIR_HEIGHT, STAIR_HEIGHT_TOL))
        return out

    def saw_height(self, l, z):
        """버튼으로 닿은 높이 z → 이 다리의 진짜 바닥(ground_z) 갱신.
        z 가 처음 바닥보다 STAIR_DETECT_MM 넘게 위면 계단 → 계단 높이(step_z) 기억/갱신.
        (버튼 고장 다리를 놓을 때와, 계단보다 낮은 다리를 옮길 때 얼마나 들지 정할 때 씀)"""
        self.ground_z[l] = z
        if z + BODY_HEIGHT > STAIR_DETECT_MM and (self.step_z is None or z > self.step_z + STAIR_DETECT_MM / 2):
            first = self.step_z is None
            self.step_z = z
            print(f"[계단] {'계단 발견' if first else '더 높은 계단'} — 다리 {l + 1} 이 처음 바닥보다 {z + BODY_HEIGHT:+.0f}mm 에서 닿음")

    def pull(self, pullers, hover, dist, keep=(), drop_mode="hover", stop=None):
        """pullers 를 바닥에 둔 채 몸을 dist 만큼 앞으로. hover 는 SKID_HOVER 띄워 몸을 따라옴. 간 거리를 돌려줌.
        당기다 더 못 가거나 다른 다리와 너무 가까워지는 다리(keep 이 아닌)는 당기기에서 빼고 계속 감:
          drop_mode "hover" = 살짝 띄워 따라오게 (남은 당기는 다리 3개 이상일 때)
          drop_mode "drag"  = 바닥에 둔 채 끌려오게 (남은 당기는 다리 2개 이상일 때)"""
        min_left = 3 if drop_mode == "hover" else 2

        def let_go(drop, why):
            nonlocal pullers
            print(f"[전진] 다리 {[l + 1 for l in drop]} {why} ({gone:.0f}mm 에서) — "
                  + ("띄워서 따라오게" if drop_mode == "hover" else "당기기에서 빼고 끌려오게") + " 하고 계속")
            if drop_mode == "hover":
                self.move_feet({l: [self.feet[l][0], self.feet[l][1], self.feet[l][2] + SKID_HOVER] for l in drop},
                               LIFT_SPEED)
                hover.extend(drop)
            pullers = [l for l in pullers if l not in drop]

        pullers, hover = list(pullers), list(hover)
        if hover:
            ups = {}
            for l in hover:
                h = self.hover_lift(l)
                f = self.feet[l]
                while h > SKID_HOVER and leg_ik(l, [f[0], f[1], f[2] + h]) is None:
                    h -= 2.0
                ups[l] = [f[0], f[1], f[2] + h]
            self.move_feet(ups, LIFT_SPEED)
        gone, step = 0.0, PULL_SPEED * self.dt
        while gone < dist - 1e-6:
            if stop is not None and stop():
                break
            s = min(step, dist - gone)
            feet = [list(f) for f in self.feet]
            for l in pullers:
                feet[l] = [feet[l][0] - s * self.fwd[0], feet[l][1] - s * self.fwd[1], feet[l][2]]
            bad = [l for l in pullers if leg_ik(l, feet[l]) is None]
            if bad:
                drop = [l for l in bad if l not in keep]
                if len(drop) == len(bad) and len(pullers) - len(drop) >= min_left:
                    let_go(drop, "가 더 못 당김")
                    continue
                print(f"[전진] 다리 {[l + 1 for l in bad]} 가 더 못 당김 ({gone:.0f}mm 에서)")
                break
            close = []
            new = {l: leg_ik(l, feet[l]) for l in pullers}
            for l in pullers:                           # 당기다 다른 다리와 부딪힐 것 같으면 멈춤 (당기는 다리는 다 새 자세로)
                c = self.clearance(l, new[l], new)
                if c < LEG_CLEAR_STOP and c < self.clearance(l, self.hx.cmd[l]):
                    close.append((l + 1, round(c)))
            drop = [n - 1 for n, _ in close if n - 1 not in keep]
            if drop and len(pullers) - len(drop) >= min_left:
                let_go(drop, "가 다른 다리와 너무 가까워짐")
                continue
            if close:
                print(f"[전진] 다리 {close} (다리, mm) 가 다른 다리와 너무 가까워짐 ({gone:.0f}mm 에서) — 당기기 멈춤")
                break
            self.send(feet)
            gone += s
            time.sleep(self.dt)
        for l in hover:                                 # 띄운 다리 다시 내려놓기 (눌리면 바로 멈춤)
            self.descend(l, self.make_touched(l, self.known_heights(l)), "다시 ")
        return gone

    def step_lift(self):
        """전진할 때 옮기는 다리를 드는 높이: 계단을 아직 못 찾았으면 STAIR_MAX_H 보다 높이 (앞에 계단이 있어도 안 박히게)."""
        return STEP_LIFT if self.step_z is not None else max(STEP_LIFT, STAIR_MAX_H + STEP_LIFT / 2)

    def hover_lift(self, l):
        """당기는 동안 띄우는 높이: 계단보다 낮은 다리(또는 계단을 아직 모를 때)는 계단 윗면 위로 — 몸을 따라 앞으로 가다
        계단 앞면에 박히지 않게."""
        z = self.feet[l][2]
        if self.step_z is None:
            return max(SKID_HOVER, STAIR_MAX_H + SKID_HOVER) if self.approach else SKID_HOVER
        if z < self.step_z - STEP_CLEAR_TOL:
            return self.step_z - z + STEP_LIFT / 2
        return SKID_HOVER

    def advance(self, dist, keep=(), stop_at_stair=False, stride=None):
        """다리 3개를 하나씩 앞으로 옮겨 놓고 그 3개로 끌어당기기 → 다른 3개로 반복. dist 만큼.
        keep 다리는 옮기지 않고 늘 바닥에 둔 채 같이 당김. STOP_KEY 를 누르면 거기서 멈춤."""
        print(f"[전진] {dist:.0f}mm — 멈추려면 '{STOP_KEY}'")
        with KeyStop() as keys:
            self.keys = keys
            try:
                return self._advance(dist, keep, stop_at_stair, STRIDE if stride is None else stride)
            finally:
                self.keys = None

    def reach_span(self, l, p, z):
        """다리 l 의 발을 p(x, y) 에서 가는 방향으로 앞뒤로 얼마나 옮길 수 있나: (뒤 한계, 앞 한계) [mm, 앞 +]."""
        ok = [a for a in range(-160, 161, 2)
              if leg_ik(l, [p[0] + a * self.fwd[0], p[1] + a * self.fwd[1], z]) is not None]
        return (min(ok), max(ok)) if ok else (0.0, 0.0)

    def _advance(self, dist, keep, stop_at_stair, stride):
        posture = {l: list(self.feet[l]) for l in range(NUM_LEGS)}   # 몸 기준 기본 자리 (이 둘레로 ± 반 걸음)
        groups = [[L(n) for n in g if L(n) not in keep] for g in TRIPODS]
        gone, gi, restep, skipped = 0.0, 0, 0, 0
        print(f"[전진] 보폭 최대 {stride:.0f}mm")
        while gone < dist - 0.5:
            if self.keys.pressed():
                print(f"[전진] 키로 멈춤 ({gone:.0f} / {dist:.0f}mm)")
                break
            group = groups[gi]
            # 보폭: 묶음 다리마다 '기본 자리 앞뒤로 닿는 범위' 안에서 s 만큼 당길 수 있어야 함 (+ 계속 두는 다리는 지금 자리에서 뒤로 s)
            spans = {l: self.reach_span(l, posture[l], self.ground_z[l]) for l in group}
            room = [hi - lo - 4.0 for lo, hi in spans.values()]
            room += [-self.reach_span(l, self.feet[l], self.feet[l][2])[0] - 2.0 for l in keep]
            s = max(0.0, min([stride, dist - gone] + room))
            if s < STRIDE_MIN:
                skipped += 1
                print(f"[전진] 다리 {'·'.join(str(l + 1) for l in group)} 는 지금 {s:.0f}mm 밖에 못 감 — 이번엔 안 옮김")
                if skipped >= 2:
                    print("[전진] 두 묶음 다 못 감 — 멈춤")
                    break
                gi = 1 - gi
                continue
            skipped = 0
            for l in group:                             # 하나씩 앞으로: 닿는 범위 가운데 + 반 걸음 (당긴 뒤 가운데 - 반 걸음)
                lo, hi = spans[l]
                a = min(max((lo + hi) / 2 + s / 2, lo + s + 2.0), hi - 2.0)
                p = posture[l]
                deg, r = self.dir_reach(l, [p[0] + a * self.fwd[0], p[1] + a * self.fwd[1], p[2]])
                if LEVEL_EACH_LEG:
                    self.level(f"전진 다리 {l + 1} 들기 전")
                self.shift_leg(l, deg, r, self.step_lift())
            pullers = group + list(keep)
            hover = [l for l in range(NUM_LEGS) if l not in pullers]
            got = self.pull(pullers, hover, s, keep, stop=self.keys.pressed)
            gone += got
            print(f"[전진] 다리 {'·'.join(str(l + 1) for l in pullers)} 로 {got:.0f}mm (모두 {gone:.0f} / {dist:.0f}mm)")
            stuck = [l for l in keep if leg_ik(l, [self.feet[l][0] - 2 * self.fwd[0], self.feet[l][1] - 2 * self.fwd[1],
                                                    self.feet[l][2]]) is None]
            if stuck and got < s - 0.5:                 # 계속 바닥에 두던 다리가 더 못 당김 → 처음 자리로 다시 옮겨 놓고 계속
                for l in stuck:
                    p = posture[l]
                    print(f"[전진] 다리 {l + 1} 가 더 못 당겨서 처음 자리로 다시 옮겨 놓음")
                    deg, r = self.dir_reach(l, p)
                    self.shift_leg(l, deg, r, self.step_lift())
                    restep += 1
                if restep <= 4 * len(keep):
                    continue
            if self.keys.pressed():
                print(f"[전진] 키로 멈춤 ({gone:.0f} / {dist:.0f}mm)")
                break
            if got < 1.0:
                print("[전진] 더 못 감 — 멈춤")
                break
            if LEVEL_WHILE_WALKING:
                self.level(f"전진 {gone:.0f}mm 당긴 뒤")     # 한 번 당길 때마다 수평 맞춤
            if stop_at_stair and self.step_z is not None:
                print(f"[계단] 계단을 찾아서 전진 멈춤 ({gone:.0f} / {dist:.0f}mm)")
                break
            gi = 1 - gi
        return gone

    @staticmethod
    def dir_for_hip(l, hip):
        """hip 각도 → 다리 방향 [도, 몸 기준]."""
        return MOUNT[l][2] + (hip - HIP_CENTER) * HIP_SIGN[l]

    def forward_dir_limited(self, l, back_dir, goal=None):
        """back_dir 에서 정면(FORWARD_DEG) 쪽으로 hip 을 돌리되, 접은 자세로 돌아가는 길과 내려놓을 자세가
        다른 다리와 LEG_CLEAR 보다 안 가깝게, hip 한계 안에서."""
        goal = self.clamp_dir(l, FORWARD_DEG if goal is None else goal)
        span = wrap180(goal - back_dir)
        best = back_dir
        fold = self.hx.cmd[l][1:]
        for i in range(1, 101):
            deg = back_dir + span * i / 100
            if not self.clear_ok(l, [self.hip_of(deg, l)] + list(fold)):
                break
            down = None
            for z in (-BODY_HEIGHT + 60.0, -BODY_HEIGHT + 20.0, -BODY_HEIGHT):   # 내려놓을 자세 (계단 위 ~ 바닥)
                r = PLACE_REACH
                down = leg_ik(l, self.foot_at(l, deg, r, z))
                while down is None and r > COXA_LEN + 10:
                    r -= 5.0
                    down = leg_ik(l, self.foot_at(l, deg, r, z))
                if down is not None and not self.clear_ok(l, down):
                    break
            else:
                best = deg
                continue
            break
        if best != goal:
            print(f"[다리 {l + 1}] 다른 다리와 간섭 때문에 목표까지 안 돌림 (hip {self.hip_of(best, l):.0f},"
                  f" 목표 {self.hip_of(goal, l):.0f})")
        return best

    def safe_back_dir(self, l, d0, back):
        """d0 에서 뒤로 back 도 돌린 접은 자세가 다른 다리와 가까우면 덜 돌림."""
        sign = 1 if wrap180(FORWARD_DEG - d0) > 0 else -1
        for k in range(20, -1, -1):
            deg = self.clamp_dir(l, d0 - sign * back * k / 20)
            if self.clear_ok(l, [self.hip_of(deg, l), self.fold_femur, FOLD_TIBIA]) or k == 0:
                return deg

    # ── 수평 맞추기 ──
    def tilt_by(self, droll, dpitch):
        """발은 바닥에 둔 채 몸만 (droll, dpitch) 도 더 돌림 (몸 중심 기준)."""
        start = [list(f) for f in self.feet]

        def turned(u):
            R = _rot(droll * u, dpitch * u, 0.0)
            return [_mul_t(R, f) for f in start]
        target = turned(1.0)
        if any(leg_ik(l, f) is None for l, f in enumerate(target)):
            return False
        n = max(1, round(max(abs(droll), abs(dpitch)) / LEVEL_SPEED * LOOP_HZ))
        for k in range(1, n + 1):
            u = 0.5 - 0.5 * math.cos(math.pi * k / n)
            if not self.send(turned(u)):
                return False
            time.sleep(self.dt)
        for l in range(NUM_LEGS):                       # 몸을 돌리면 몸 기준 바닥 높이도 같이 바뀜
            self.ground_z[l] += target[l][2] - start[l][2]
        self.tilt = [self.tilt[0] + droll, self.tilt[1] + dpitch]
        return True

    def imu_read(self, strict=False):
        """흔들림이 가라앉게 기다린 뒤 최근 IMU 평균 (roll, pitch). strict 면 두 번 읽어 같아질 때까지. 없으면 None."""
        settle, n = (STRICT_SETTLE, STRICT_AVG_N) if strict else (IMU_SETTLE, IMU_AVG_N)
        time.sleep(settle)

        def once():
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                v = self.link.imu_recent(n)
                if v is not None:
                    return v
                time.sleep(0.02)
            return None
        v = once()
        if not strict or v is None:
            return v
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            time.sleep(0.3)
            w = once()
            if w is None:
                return None
            if max(abs(w[0] - v[0]), abs(w[1] - v[1])) < STRICT_STABLE_DEG:
                return w
            v = w
        print(f"[수평] IMU 값이 아직 흔들림 (roll {v[0]:+.1f}° pitch {v[1]:+.1f}°) — 그 값으로 맞춤")
        return v

    def imu_setup(self):
        """IMU 가 오는지 보고, 몸을 ±IMU_CAL_DEG 기울여 IMU 가 어떻게 변하나 잼 → 통과하면 수평 맞추기 켬."""
        end = time.monotonic() + 1.5
        while time.monotonic() < end and not self.link.imu_fresh() and not self.link.imu_err:
            time.sleep(0.05)
        if not self.link.imu_fresh():
            why = "펌웨어가 #imu,err (IMU 못 찾음)" if self.link.imu_err else "#imu 가 안 옴 (IMU 들어간 펌웨어인지)"
            print(f"[수평] {why} — 수평 맞추기 끔")
            return False
        base = self.imu_read()
        cols = []
        for axis in (0, 1):                             # 0 = roll, 1 = pitch 명령
            d = [IMU_CAL_DEG if i == axis else 0.0 for i in (0, 1)]
            ok = self.tilt_by(*d)
            m_plus = self.imu_read() if ok else None
            ok = ok and self.tilt_by(*[-2 * v for v in d])
            m_minus = self.imu_read() if ok else None
            self.tilt_by(*d)
            if base is None or m_plus is None or m_minus is None:
                print("[수평] 기울여 보는 중 실패 (다리가 안 닿거나 IMU 값이 끊김) — 수평 맞추기 끔")
                return False
            cols.append([(m_plus[i] - m_minus[i]) / (2 * IMU_CAL_DEG) for i in (0, 1)])
        a, c = cols[0]
        b, d = cols[1]
        det = a * d - b * c
        print(f"[수평] 명령 roll 1° → IMU roll {a:+.2f} pitch {c:+.2f} / 명령 pitch 1° → IMU roll {b:+.2f} pitch {d:+.2f}")
        if abs(det) < IMU_CAL_MIN_GAIN ** 2 or max(abs(a), abs(b)) < IMU_CAL_MIN_GAIN \
                or max(abs(c), abs(d)) < IMU_CAL_MIN_GAIN:
            print("[수평] 몸을 기울여도 IMU 가 거의 안 변함 — 수평 맞추기 끔")
            return False
        self.imu_inv = [[d / det, -b / det], [-c / det, a / det]]
        self.imu_on = True
        return True

    def level(self, tag, strict=False):
        """IMU 가 0(수평 기준)이 되게 몸을 돌림. 발은 바닥에 그대로. strict = 꼼꼼하게 (STRICT_ 설정)."""
        if not self.imu_on:
            if strict:
                print(f"[수평] {tag} ★ IMU 수평 맞추기가 꺼져 있어서 못 봄")
            return False
        deadband, iters = (STRICT_DEADBAND, STRICT_ITERS) if strict else (LEVEL_DEADBAND, LEVEL_ITERS)
        last, m = None, None
        for _ in range(iters):
            m = self.imu_read(strict)
            if m is None:
                print(f"[수평] {tag} IMU 값이 끊김 — 이번엔 안 맞춤")
                return False
            err = math.hypot(m[0], m[1])
            if last is not None and err > last[0] * 1.3 and err > 1.0:
                self.tilt_by(-last[1], -last[2])         # 맞출수록 더 기울어짐 → 되돌림
                self.level_fails += 1
                if self.level_fails >= LEVEL_FAIL_MAX:
                    self.imu_on = False
                    print(f"[수평] 맞출수록 더 기울어짐 ({last[0]:.1f}° → {err:.1f}°), {self.level_fails}번 연달아"
                          " — 되돌리고 수평 맞추기 끔 (IMU 방향이 반대로 재졌거나 값이 이상함)")
                else:
                    print(f"[수평] {tag} 맞췄더니 더 기울어짐 ({last[0]:.1f}° → {err:.1f}°) — 되돌리고 이번엔 그만"
                          f" ({self.level_fails}/{LEVEL_FAIL_MAX})")
                if strict:
                    self.not_level(tag, m)
                return False
            if max(abs(m[0]), abs(m[1])) < deadband:
                break
            inv = self.imu_inv
            cmd = [-(inv[0][0] * m[0] + inv[0][1] * m[1]), -(inv[1][0] * m[0] + inv[1][1] * m[1])]
            want = [min(max(self.tilt[i] + cmd[i], -LEVEL_MAX_DEG), LEVEL_MAX_DEG) for i in (0, 1)]
            dr, dp = want[0] - self.tilt[0], want[1] - self.tilt[1]
            if abs(dr) < 0.05 and abs(dp) < 0.05:
                print(f"[수평] {tag} 더 못 돌림 (한계 {LEVEL_MAX_DEG:.0f}°) — roll {m[0]:+.1f}° pitch {m[1]:+.1f}°")
                return False
            for frac in (1.0, 0.5, 0.25):               # 다리가 다 못 따라오면 되는 만큼만
                if self.tilt_by(dr * frac, dp * frac):
                    last = (err, dr * frac, dp * frac)
                    break
            else:
                print(f"[수평] {tag} 더 못 돌림 (다리가 안 닿음) — roll {m[0]:+.1f}° pitch {m[1]:+.1f}° 남음")
                return False
        else:
            m = self.imu_read(strict)
        if last is not None:
            self.level_fails = 0                        # 제대로 맞춰졌으면 연달아 실패 수 초기화
        if m is not None:
            print(f"[수평] {tag} 맞춤: IMU roll {m[0]:+.1f}° pitch {m[1]:+.1f}°"
                  f" (몸 돌림 roll {self.tilt[0]:+.1f}° pitch {self.tilt[1]:+.1f}°)")
        if strict and (m is None or max(abs(m[0]), abs(m[1])) > STRICT_OK_DEG):
            self.not_level(tag, m)
            return False
        return True

    def not_level(self, tag, m):
        """꼼꼼하게 맞췄는데도 STRICT_OK_DEG 넘게 기울어 있음 → 경고 (--pause 면 계속할지 물어봄)."""
        what = "IMU 값 없음" if m is None else f"roll {m[0]:+.1f}° pitch {m[1]:+.1f}°"
        print(f"[수평] {tag} ★ 아직 기울어 있음 ({what}, 허용 {STRICT_OK_DEG:.1f}°) — 다리가 더 못 따라오거나 발이 미끄러짐")
        if self.pause:
            if input("   계속할까? (y = 계속 / 다른 키 = 멈춤) ").strip().lower() != "y":
                raise KeyboardInterrupt

    def level_strict(self, tag):
        """꼼꼼하게: 맞추고 → 다시 읽어 확인 → 어긋나 있으면 한 번 더."""
        ok = self.level(tag, strict=True)
        if ok and self.imu_on:
            m = self.imu_read(strict=True)
            if m is not None and max(abs(m[0]), abs(m[1])) > STRICT_DEADBAND:
                print(f"[수평] {tag} 확인해 보니 다시 기울어짐 (roll {m[0]:+.1f}° pitch {m[1]:+.1f}°) — 한 번 더")
                ok = self.level(tag + " 다시", strict=True)
        return ok

    # ── 순서 ──
    def run(self):
        if not self.hx.sync(self.link):
            return False
        self.hx.max_speed = [[1e9] * NUM_JOINTS for _ in range(NUM_LEGS)]
        self.hx.timedelay = 0.0
        if LEVEL:
            self.link.imu_stream(True)                  # 미리 켜 둠 (일어서는 동안 값이 쌓임)
        low = [self.foot_at(l, MOUNT[l][2], STANCE_REACH, -START_HEIGHT) for l in range(NUM_LEGS)]
        high = [[f[0], f[1], -BODY_HEIGHT] for f in low]
        if any(leg_ik(l, f) is None for l, f in enumerate(low)) or any(leg_ik(l, f) is None for l, f in enumerate(high)):
            print("[설정] START_HEIGHT·BODY_HEIGHT·STANCE_REACH 로 못 섬")
            return False
        names = lambda ls: "·".join(str(l + 1) for l in ls)
        print(f"[배치] 가는 방향 {FORWARD_DEG:.0f}° → 앞 {names(self.front)} / 중간 {names(self.middle)} / 뒤 {names(self.rear)}")
        target = [leg_ik(l, f) for l, f in enumerate(low)]
        start = [row[:] for row in self.hx.cmd]
        n = max(1, round(START_SEC * LOOP_HZ))
        for k in range(1, n + 1):
            self.send_joints([[a + (b - a) * k / n for a, b in zip(s_, t)] for s_, t in zip(start, target)])
            time.sleep(self.dt)
        self.feet = low
        time.sleep(0.3)
        if not self.link.foot_fresh():
            print("[접지] 신호 없음 — 버튼을 안 보고 바닥 높이까지 내림 (계단 위에 못 놓음)")
            self.contact_on = False
        if LEVEL:
            print("[수평] 낮게 선 채로 IMU 방향 재는 중 (몸을 조금씩 기울여 봄)")
            if self.imu_setup():
                self.level("0)")

        self.wait(f"1) 몸을 {BODY_HEIGHT:.0f}mm 로 들어 올리기")
        self.move_feet({l: f for l, f in enumerate(high)}, RAISE_SPEED)
        self.ground_z = [f[2] for f in self.feet]
        self.level("1)")

        self.wait(f"2) 앞 다리 조금 접기 (평행까지 {FOLD_MARGIN:.0f}mm 남김)")
        for l in self.front:
            d0, r = self.dir_reach(l)
            par = self.side[l]
            a_par = self.along(self.foot_at(l, par, r, 0))
            span = wrap180(par - d0)
            lo, hi = 0.0, 1.0                           # d0 → par 사이에서 '평행보다 FOLD_MARGIN 앞' 인 방향
            for _ in range(30):
                mid = (lo + hi) / 2
                if self.along(self.foot_at(l, d0 + span * mid, r, 0)) - a_par > FOLD_MARGIN:
                    lo = mid
                else:
                    hi = mid
            deg = d0 + span * lo
            print(f"[다리 {l + 1}] hip {self.hip_of(d0, l):.0f} → {self.hip_of(deg, l):.0f} (평행 {self.hip_of(par, l):.0f})")
            self.shift_leg(l, deg)
        self.level("2)")

        self.wait(f"3) 그 상태로 {FIRST_ADVANCE:.0f}mm 전진")
        self.approach = True                            # 계단을 찾을 때까지 옮기는 다리·띄우는 다리를 높이 듦
        self.advance(FIRST_ADVANCE, stop_at_stair=STOP_AT_STAIR, stride=STRIDE_FIRST)
        self.approach = False
        if self.step_z is None:
            print("[계단] 3) 동안은 계단에 안 닿음 — 5)~7) 에서 앞 다리를 놓으며 찾음"
                  " (거기서도 못 찾으면 FIRST_ADVANCE 가 모자라거나 계단이 STAIR_DETECT_MM 보다 낮음)")
        self.level("3)")

        self.wait("4) 앞 다리를 2·5번과 평행하게 접기")
        for l in self.front:
            self.shift_leg(l, self.side[l])
        self.level("4)")

        for l in self.front:
            self.wait(f"5)~7) 앞 다리 {l + 1}: 접어 올리기 → hip 정면 → 닿을 때까지 내려놓기")
            self.fold_up(l)
            face = self.clamp_dir(l, FORWARD_DEG)
            self.turn_folded(l, face)
            self.put_down(l, face)
            self.level(f"5)~7) 다리 {l + 1}")

        self.wait(f"9) {FRONT_ADVANCE:.0f}mm 전진 (앞 다리는 계단 위에 두고 당김 → hip 이 원래 각도 쪽으로)")
        self.advance(FRONT_ADVANCE, keep=self.front, stride=STRIDE_FRONT)
        for l in self.front:
            d, _ = self.dir_reach(l)
            print(f"[다리 {l + 1}] hip {self.hip_of(d, l):.0f} (평행 {self.hip_of(self.side[l], l):.0f})")
        self.level("9)")

        for l in self.middle:
            self.wait(f"10) 중간 다리 {l + 1}: 뒤로 접어 올리기 → hip {MID_PLACE_HIP:.0f} 으로 → 닿을 때까지 내려놓기")
            d0, _ = self.dir_reach(l)
            back = self.safe_back_dir(l, d0, MID_BACK_DEG)
            self.fold_up(l, back)
            face = self.forward_dir_limited(l, back, self.dir_for_hip(l, MID_PLACE_HIP))
            self.turn_folded(l, face)
            self.put_down(l, face)
            self.level(f"10) 다리 {l + 1}")

        self.wait(f"11) 앞 다리를 앞뒤 폭 {SPAN:.0f}mm 로 접기")
        rear_along = min(self.along(self.feet[l]) for l in self.rear)
        goal = rear_along + SPAN
        for l in self.front:
            d0, r = self.dir_reach(l)
            par = self.side[l]
            if self.along(self.feet[l]) <= goal:
                print(f"[다리 {l + 1}] 이미 폭 안 ({self.along(self.feet[l]) - rear_along:.0f}mm)")
                continue
            span = wrap180(par - d0)
            lo, hi = 0.0, 1.0
            for _ in range(30):
                mid = (lo + hi) / 2
                if self.along(self.foot_at(l, d0 + span * mid, r, 0)) > goal:
                    lo = mid
                else:
                    hi = mid
            self.shift_leg(l, d0 + span * hi)
        span_now = max(self.along(self.feet[l]) for l in self.front) - rear_along
        print(f"[폭] 앞뒤 폭 {span_now:.0f}mm")
        self.level("11)")

        # 12) 전진 — 3)·14) 처럼 3개씩 번갈아 (REAR_ADVANCE_KEEP_FRONT 면 9) 처럼 앞 다리는 계단 위에 둔 채)
        #     거리 = 뒤 다리를 앞으로 놓을 자리가 지금 중간 다리 발 자리(확실히 계단 위)보다 REAR_STEP_MARGIN 앞이 될 만큼
        mid_along = min(self.along(self.feet[l]) for l in self.middle)
        rear_land = min(self.along(self.foot_at(l, self.side[l], PLACE_REACH, 0)) for l in self.rear)   # 13) 에서 평행으로 놓음
        dist = max(0.0, mid_along + REAR_STEP_MARGIN - rear_land) if REAR_ADVANCE_PRE is None else REAR_ADVANCE_PRE
        dist = min(dist, REAR_ADVANCE_MAX)
        self.wait(f"12) {dist:.0f}mm 전진 ("
                  + ("9) 처럼 앞 다리는 계단 위에 두고" if REAR_ADVANCE_KEEP_FRONT else "3개씩 번갈아 옮기고 당기기")
                  + ") — 뒤 다리가 계단에 닿을 만큼")
        if dist > 0.5:
            got = self.advance(dist, keep=self.front if REAR_ADVANCE_KEEP_FRONT else (), stride=STRIDE_REAR_PRE)
            if got < dist - 0.5:
                print(f"[전진] ★ {dist - got:.0f}mm 덜 감 — 뒤 다리가 계단에 못 닿을 수 있음")
        self.level("12) 전진")

        # 12-1) 앞·중간 다리 hip 을 평행(2·5번 방향)으로 맞춤 (하나씩: 살짝 들기 → 돌리기 → 닿을 때까지 내림)
        #       옮긴 발이 원래보다 STEP_DROP_TOL 넘게 낮은 데 닿으면 계단 밖 → 덜 돌려서 다시
        self.wait("12-1) 앞·중간 다리 hip 을 평행으로 맞추기")
        for l in self.front + self.middle:
            d0, r0 = self.dir_reach(l)
            z0 = self.feet[l][2]
            r = max(r0, STANCE_REACH)                   # 발이 hip 바로 아래에 있으면 방향이 의미 없으니 조금 바깥으로
            span = wrap180(self.side[l] - d0)
            for u in (1.0, 0.75, 0.5, 0.25, 0.0):
                deg = d0 + span * u
                print(f"[다리 {l + 1}] hip {self.hip_of(d0, l):.0f} → {self.hip_of(deg, l):.0f} (평행 {self.hip_of(self.side[l], l):.0f})")
                z = self.shift_leg(l, deg, r, lift=max(SHIFT_LIFT, z0 - self.feet[l][2] + SHIFT_LIFT), allow_drop=True)
                if z >= z0 - STEP_DROP_TOL:
                    break
                print(f"[다리 {l + 1}] {z0 - z:.0f}mm 낮은 데 닿음 = 계단 밖으로 나감 → 덜 돌려서 다시")
        self.level_strict("12)")                         # 다음이 뒤 다리 들기 → 꼼꼼하게

        for l in self.rear:
            self.wait(f"13) 뒤 다리 {l + 1}: hip 뒤로 살짝 → 접어 올리기 → 평행으로 → 닿을 때까지 내려놓기")
            self.level_strict(f"13) 다리 {l + 1} 들기 전")      # 뒤 다리를 들면 받침이 줄어드니 먼저 꼼꼼하게
            d0, _ = self.dir_reach(l)
            back = self.safe_back_dir(l, d0, REAR_BACK_DEG)
            self.fold_up(l, back)
            face = self.forward_dir_limited(l, back, self.side[l])     # 정면(hip 95)이 아니라 평행으로
            self.turn_folded(l, face)
            self.put_down(l, face)
            self.level_strict(f"13) 다리 {l + 1}")

        self.wait(f"14) {REAR_ADVANCE:.0f}mm 전진")
        self.advance(REAR_ADVANCE, stride=STRIDE_LAST)
        self.level("14)")
        print("\n[순서] 끝")
        return True

    def finish(self):
        print("[순서] 서기 자세로 돌아가는 중")
        start = [row[:] for row in self.hx.cmd]
        n = max(1, round(START_SEC * LOOP_HZ))
        for k in range(1, n + 1):
            self.send_joints([[a + (b - a) * k / n for a, b in zip(s_, t)] for s_, t in zip(start, STAND_POSE)])
            time.sleep(self.dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default=ROBOT, help="로봇 블루투스 주소 또는 포트")
    ap.add_argument("--pause", action="store_true", help="단계마다 Enter 를 기다림")
    a = ap.parse_args()
    link = Link(a.robot)
    if not link.connect():
        print("\n".join(link.log_queue))
        return
    seq = Seq(link, a.pause)
    try:
        seq.run()
    except (KeyboardInterrupt, ConnectionError) as e:
        print("\n중단:", e or "Ctrl+C")
        while link.log_queue:
            print("[로그]", link.log_queue.popleft())
    if seq.hx.state_known:
        try:
            seq.finish()
        except (KeyboardInterrupt, ConnectionError):
            pass
    if LEVEL:
        link.imu_stream(False)
    seq.hx.poweroff(link)
    time.sleep(0.2)
    link.close()


if __name__ == "__main__":
    main()
