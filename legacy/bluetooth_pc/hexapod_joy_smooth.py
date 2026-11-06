"""
자연스러운 조이스틱 보행 — 쉬지 않고 이어지는 걸음 + 버튼 높이·회전 + 안전 모드
오렌지보드(우노) + 조이스틱 쉴드 → 컴퓨터 → 블루투스 → 로봇

필요한 파일 (같은 폴더): hexapod_sdk.py (통신·접지·IMU), hexapod_body.py (leg_fk / leg_ik / MOUNT)
우노: joystick_controller.ino (<x,y,버튼,볼륨> 을 계속 보냄 — hexapod_joy_walk2.py 와 같은 것)

실행
  /usr/bin/python3 hexapod_joy_smooth.py --uno /dev/ttyACM0 --robot AA:BB:CC:DD:EE:FF
  /usr/bin/python3 hexapod_joy_smooth.py --uno /dev/ttyACM0 --safe       # 안전 모드로 시작
  /usr/bin/python3 hexapod_joy_smooth.py --uno /dev/ttyACM0 --joytest    # 버튼 번호 확인 (로봇 없이)
  /usr/bin/python3 hexapod_joy_smooth.py --sim                           # 로봇·조종기 없이 시뮬레이션 점검
  ★ 로봇 전원을 켠 직후(펌웨어 부팅 자세 = 서기 자세)에 실행할 것

조작
  조이스틱        : 기울인 방향으로 걷기. 많이 기울일수록 빠름. 가속·감속이 부드럽게 붙음
  Left / Right   : 누르고 있는 동안 왼쪽 / 오른쪽으로 회전 (조이스틱과 같이 쓰면 걸으면서 돎)
  노랑 / 파랑     : 몸 높이 올리기 / 내리기. 한 번 누르면 HEIGHT_STEP, 누르고 있으면 계속 (걷는 중에도 됨)
  조이스틱 누름   : 높이를 기본값(DEFAULT_HEIGHT)으로
  BTN_SAFE       : 안전 모드 켜기 / 끄기 (모든 다리가 바닥에 있을 때 바뀜)
  볼륨           : USE_POT_SPEED = True 면 최고 속도 조절
  Ctrl+C         : 멈추고 → 낮춘 뒤 → 힘 빼기  (한 번 더 Ctrl+C = 바로 힘 빼기)

걷는 방식 (공통)
  - 걸음을 하나씩 끊지 않고, 시계(phase)가 계속 돌면서 다리마다 차례가 오면 들어서 옮김
  - 바닥 다리: 몸이 가는 만큼 바닥에 붙은 채로 뒤로 밀림 (회전도 같이)
  - 공중 다리: 들어 올린 뒤 '원래 자리 + 반 걸음 앞' 에 내려놓음. 내려놓을 자리는 공중에 있는 동안
    조이스틱을 따라 계속 고쳐서, 방향을 바꿔도 바로 따라감
  - 조이스틱을 놓으면 천천히 서고, 발이 원래 자리로 다 돌아오면 시계도 멈춤 (이미 제자리인 다리는 안 듦)
  - 보폭은 그 높이에서 다리가 닿는 만큼까지만 (시작할 때 높이별로 계산). 넘으면 속도를 자동으로 줄임
  - 매 순간 18개 관절을 계산해서 하나라도 못 닿으면 그 순간의 몸 이동·높이 변경을 버리고 다리만 움직임
    (→ 범위 밖 명령은 절대 안 보냄)

일반 모드 (NORMAL) — 트라이포드: 3개씩 번갈아, 빠르고 자연스러움. 접지 버튼·IMU 안 씀

안전 모드 (SAFE) — 웨이브: 한 번에 한 다리만 들어서 늘 5개가 바닥을 받침. 높은 자세에서도 쓰도록:
  - 접지: 다리를 예상 바닥 LAND_NEAR 위까지 내린 뒤 LAND_SPEED 로 천천히 내리며 접지 버튼이 눌리면 거기 놓음
          (턱이 있어 일찍 닿으면 그 자리, 낮으면 MAX_DOWN 까지 더 내림). 다리가 닿을 때까지 다음 다리는 안 듦
  - 접지 유지: 바닥에 있어야 할 다리의 버튼이 PRESS_DELAY 넘게 떨어져 있으면 PRESS_MAX 까지 눌러 내림
  - 수평: IMU 로 몸 기울기를 계속 0 으로 맞춤 (시작할 때 몸을 살짝 기울여 IMU 방향을 자동으로 잼)
  - 높이 유지: 바닥 다리들의 평균 높이를 원하는 몸 높이로 천천히 맞춤 (울퉁불퉁한 바닥에서 몸이 따라 뜨거나 가라앉지 않게)
  - 안정: 다리를 들기 전에 '남은 발들이 만드는 다각형 안에 무게중심이 STAB_MARGIN 이상 안쪽' 인지 확인.
          모자라면 기다리면서 몸을 다각형 가운데로 옮긴 다음 듦 (몸이 높아도 넘어지지 않게)
  - 발 자리를 일반 모드보다 SAFE_SPREAD_MM 넓게 → 받침 다각형이 커짐. 속도·가속도 낮음
  ★ 접지 버튼이 없거나 신호가 안 오면 접지 기능은 끄고 걸음만 웨이브로 (시작 때 알림)
  ★ IMU 가 없거나 시작 측정이 이상하면 수평 기능만 끔

높이 = hip 축에서 발끝까지 수직 거리 [mm] (서기 자세 ≈ 66, 다리를 다 펴면 ≈ 237)
  쓸 수 있는 범위는 모드마다 '그 높이에서 MIN_STRIDE 이상 걸을 수 있는 높이' 로 시작할 때 계산해서 알려줌
"""

import argparse
import json
import os
import math
import random
import threading
import time
from collections import deque
from dataclasses import dataclass, replace

try:
    import serial
except ImportError:
    serial = None

from hexapod_body import COXA_LEN, FEMUR_LEN, MOUNT, TIBIA_LEN, _mul_t, _rot, leg_fk, leg_ik
from hexapod_sdk import NUM_JOINTS, NUM_LEGS, STAND_POSE, Hexapod, Link

# ═══════════════════════════ 설정 ═══════════════════════════
ROBOT = "AA:BB:CC:DD:EE:FF"   # 내 ESP32 블루투스 주소로 바꿀 것 (펌웨어가 켜질 때 출력)
START_SEC = 2.0             # 처음 서기 자세로 맞추는 시간 [초]

# ───────────── 속도 파라미터 (속도는 여기서만 맞추면 됨) ─────────────
# 걷는 속도 = 보폭 ÷ 바닥 시간
#   보폭      : 높이마다 다리가 닿는 만큼 (자동 계산, stride_max 까지) × STRIDE_USE
#   바닥 시간 : period × (1 - swing_frac)   ← period 를 줄이면 다리를 더 자주 옮겨서 빨라짐
#   공중 시간 : period × swing_frac         ← 너무 짧으면 관절이 JOINT_SPEED_MAX 를 넘음
#               → 그 순간은 시간을 늦춰서 계산하므로 걸음이 고르지 않게 됨 (실제 로봇은 흔들림도 커짐)
#   v_max     : 조이스틱 끝까지 밀었을 때 속도 상한 (보폭 ÷ 바닥 시간보다 크게 줘도 그 이상은 안 나옴)
#   accel     : 출발·정지 부드러움 (v_max 까지 걸리는 시간 ≈ v_max ÷ accel)
#   swing_frac 상한: 트라이포드 0.5 미만, 웨이브 1/6(0.167) 미만 (다리 바꿀 때 잠깐 다 바닥에 있어야 함)
# 최적값 찾기
#   1) python3 hexapod_joy_smooth.py --tune --sim [--save]
#        시뮬레이션: period·swing_frac 조합마다 '실제로 나오는 속도'와 관절 한계를 재서, 가장 빠르면서 걸음이 고른 값 추천
#   2) /usr/bin/python3 hexapod_joy_smooth.py --tune --uno /dev/ttyUSB1
#        실제 로봇: 노랑/파랑으로 속도 단계를 올리고 내리며 걷고, 단계마다 흔들림(IMU)·접지 끊김·관절 한계를 잼
#        → 기준을 넘지 않는 가장 빠른 단계를 추천하고, 끝날 때 저장할지 물어봄
#   저장하면 TUNED_FILE 에 기록 → 다음 실행부터 자동 적용 (--no-tuned 로 무시)
SPEED = {
    "normal": dict(period=1.0/2, swing_frac=0.45, v_max=110.0, w_max=35.0, accel=250.0, w_accel=120.0),
    "safe":   dict(period=3.6, swing_frac=0.13, v_max=35.0,  w_max=12.0, accel=80.0,  w_accel=40.0),
}
JOINT_SPEED_MAX = 250.0     # 관절 최고 속도 [도/s] (MG996R 무부하 ≈ 350). 넘는 순간은 시간을 늦춰서 계산
HEIGHT_SPEED = 35.0         # 몸이 올라가고 내려가는 최고 속도 [mm/s]
HEIGHT_RATE = 40.0          # 높이 버튼을 누르고 있을 때 목표가 바뀌는 속도 [mm/s]
TARGET_SPEED = 300.0        # 공중 다리의 내려놓을 자리가 조이스틱을 따라 옮겨 가는 최고 속도 [mm/s]
LAND_SPEED = 30.0           # 안전 모드: 접지 버튼을 찾으며 내리는 속도 [mm/s] (반응 지연 ~60ms × 속도 = 넘침)
PRESS_SPEED = 15.0          # 안전 모드: 뜬 바닥 다리를 눌러 내리는 속도 [mm/s]
SWAY_SPEED = 30.0           # 안전 모드: 무게중심을 받침 가운데로 옮기는 속도 [mm/s]
IMU_RATE = 6.0              # 안전 모드: 수평 맞추는 최고 속도 [도/s]
LOOP_HZ = 50                # 계산·전송 횟수 [회/초]
TUNED_FILE = "speed_tuned.json"     # 튜닝 결과 (이 파일과 같은 폴더)

# 튜닝 기준 (실제 로봇)
TUNE_LEVELS = [0.5, 0.65, 0.8, 0.9, 1.0, 1.1, 1.25, 1.4]   # 지금 속도의 몇 배로 걸어 볼지
TUNE_WOBBLE_MAX = {"normal": 1.5, "safe": 1.0}             # 걸을 때 몸 흔들림(IMU roll·pitch, 1초 평균에서 벗어난 RMS) 상한 [도]
TUNE_CONTACT_LOSS_MAX = 0.10    # 바닥에 있어야 할 다리 접지 버튼이 떨어져 있던 비율 상한
TUNE_SLOW_MAX = 0.05            # 관절 한계 때문에 시간을 늦춘 틱 비율 상한
TUNE_FALLBACK_MAX = 0.03        # 다리가 끝까지 닿아 몸 이동을 줄인 틱 비율 상한
TUNE_MIN_SEC = 4.0              # 한 단계에서 이만큼은 걸어야 판정 [초]
# ───────────────────────────────────────────────────────────


@dataclass
class Gait:
    name: str
    key: str                # "normal" / "safe" (SPEED·튜닝 파일의 이름)
    order: list             # 드는 차례. 같은 묶음은 같이 듦 (코드 번호: 다리 1 = 0)
    lift: float             # 드는 높이 [mm] (못 들면 그 다리만 들 수 있는 만큼)
    spread_mm: float        # 발 자리를 tibia 수직 자리보다 이만큼 바깥으로 [mm]
    stride_max: float       # 보폭 상한 [mm] — 다리가 더 닿아도 이웃 발끼리 부딪히지 않게
    safe: bool              # 안전 기능 (접지·수평·안정 확인) 켜기
    period: float           # ↓ 속도 파라미터 (위 SPEED 설명)
    swing_frac: float
    v_max: float
    w_max: float            # [도/s]
    accel: float            # [mm/s²]
    w_accel: float          # [도/s²]


SPEED_KEYS = ("period", "swing_frac", "v_max", "w_max", "accel", "w_accel")
SWING_FRAC_MAX = {"normal": 0.49, "safe": 0.16}


def make_gaits():
    """SPEED 로 두 보행을 만듦. 다리 배치 (위에서 봄): 3 맨앞 / 2 왼앞, 4 오른앞 / 1 왼뒤, 5 오른뒤 / 6 맨뒤"""
    normal = Gait("일반(트라이포드)", "normal", order=[[0, 2, 4], [1, 3, 5]], lift=35.0, spread_mm=0.0,
                  stride_max=110.0, safe=False, **SPEED["normal"])
    safe = Gait("안전(웨이브)", "safe", order=[[5], [0], [4], [1], [3], [2]], lift=30.0, spread_mm=25.0,
                stride_max=80.0, safe=True, **SPEED["safe"])
    return normal, safe


NORMAL, SAFE = make_gaits()


def scaled(g, k):
    """보행 g 를 k 배 빠르게 (주기 ÷ k, 속도·가속도 × k)."""
    return replace(g, period=g.period / k, v_max=g.v_max * k, w_max=g.w_max * k,
                   accel=g.accel * k, w_accel=g.w_accel * k)


def tuned_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), TUNED_FILE)


def load_tuned():
    """TUNED_FILE 이 있으면 SPEED 에 덮어쓰고 보행을 다시 만듦."""
    global NORMAL, SAFE
    try:
        with open(tuned_path(), encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        return False
    except (OSError, ValueError) as e:
        print(f"[속도] {TUNED_FILE} 를 못 읽음 ({e}) — 기본값 사용")
        return False
    used = []
    for key in ("normal", "safe"):
        vals = data.get(key) or {}
        for k in SPEED_KEYS:
            v = vals.get(k)
            if not isinstance(v, (int, float)) or v <= 0:
                continue
            if k == "swing_frac" and v >= SWING_FRAC_MAX[key]:
                continue
            SPEED[key][k] = float(v)
            used.append(f"{key}.{k}")
    NORMAL, SAFE = make_gaits()
    if used:
        print(f"[속도] {TUNED_FILE} 적용: " + ", ".join(f"{g.name} 주기 {g.period:.2f}s 최고 {g.v_max:.0f}mm/s"
                                                     for g in (NORMAL, SAFE)))
    return bool(used)


def save_tuned(results, how):
    """results = {"normal": {속도 파라미터}, ...}. 있던 파일에 합쳐서 저장."""
    path = tuned_path()
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        data = {}
    for key, vals in results.items():
        data[key] = {k: round(float(vals[k]), 3) for k in SPEED_KEYS}
        data[key]["_how"] = how
        data[key]["_date"] = time.strftime("%Y-%m-%d %H:%M")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"[속도] 저장함: {path}")


DEFAULT_HEIGHT = 120.0      # 기본 몸 높이 [mm] (모드 범위 밖이면 범위 안으로)
HEIGHT_STEP = 10.0          # 버튼 한 번 [mm]
HEIGHT_HOLD = 0.35          # 이보다 오래 누르면 '누르고 있음' [초]

MIN_STRIDE = 30.0           # 이보다 작게밖에 못 걷는 높이는 안 씀 [mm]
MIN_LIFT = 15.0             # 높이 범위를 정할 때 최소한 이만큼은 들 수 있어야 함 [mm]
STRIDE_USE = 0.8            # 계산한 최대 보폭의 이만큼만 씀 (착지 오차·바닥 높낮이 여유)
REACH_MARGIN = 15.0         # 다리를 다 펴기 직전(특이점)은 안 씀 [mm]
SAFE_SPREAD_MAX = 0.5       # spread_mm 은 '닿는 끝까지 남은 거리' 의 이 비율까지만
LOOKAHEAD = 0.4             # 바닥 다리가 이 시간 [초] (또는 멈추는 데 걸리는 시간) 뒤에 못 닿을 것 같으면 미리 천천히
SETTLE_TOL = 8.0            # 멈췄을 때 원래 자리에서 이 안이면 제자리로 봄 [mm]
REHOME_TOL = 20.0           # 서 있는데 원래 자리에서 이보다 벗어난 발이 있으면 다시 디딤 [mm] (높이·모드 바꾼 뒤)

# 안전 모드
LAND_NEAR = 8.0             # 예상 바닥보다 이만큼 위까지는 빨리 내리고 [mm]
MAX_DOWN = 25.0             # 예상 바닥보다 이만큼 더 내려도 안 닿으면 거기 둠 [mm]
FOOT_DEAD_MISSES = 2        # 바닥을 찾다 MAX_DOWN 까지 가도 안 눌린 게 이만큼 연속이면 그 다리 버튼은 고장으로 보고 뺌
FOOT_REVIVE_SEC = 2.0       # 뺀 버튼이 바닥에 있는 동안 이만큼 계속 눌려 있으면 다시 씀 [초]
PRESS_DELAY = 0.25          # 바닥 다리 버튼이 이만큼 떨어져 있으면 눌러 내림 [초]
PRESS_MAX = 15.0            # 한 번 디딘 뒤 최대로 더 누르는 거리 [mm]
HEIGHT_FIX_GAIN = 1.5       # 바닥 다리 평균 높이 → 몸 높이 맞추는 세기 [1/s]
HEIGHT_FIX_SPEED = 10.0     # 그 최고 속도 [mm/s]
STAB_MARGIN = 25.0          # 다리를 들기 전 무게중심이 받침 다각형 안쪽으로 이만큼은 있어야 함 [mm]
STAB_HARD = 8.0             # 이보다 가까워지는 몸 이동은 막음 [mm]
STAB_RELAX_SEC = 1.5        # 이만큼 기다려도 안 되면 STAB_HARD 로 낮춰서 듦 [초]
COM_OFFSET = (0.0, 0.0)     # 무게중심이 몸 중심에서 (앞, 왼쪽) 으로 벗어난 거리 [mm] (배터리 위치 등)
SEARCH_SLOW = 0.5           # 다리가 바닥을 찾거나 들기를 기다리는 동안 몸 속도 비율
SAFE_STANCE_FACTOR = 1.5    # 안전 모드는 바닥 찾느라 시계가 멈추는 만큼 바닥 시간이 길어짐 → 보폭 계산에 반영

# IMU 수평 (안전 모드에서만)
IMU_LEVEL = True            # --no-imu 로 끔
IMU_CAL_DEG = 3.0           # 시작할 때 IMU 방향 재려고 몸을 기울이는 각도 [도]
IMU_CAL_MIN_GAIN = 0.3      # 명령 1° 에 IMU 가 이보다 덜 변하면 이상한 것으로 보고 끔
IMU_SETTLE = 0.4            # 몸을 움직인 뒤 IMU 를 읽기 전에 기다리는 시간 [초]
IMU_AVG_N = 20              # IMU 평균 샘플 수
IMU_DEADBAND = 0.4          # 이보다 덜 기울었으면 안 고침 [도]
IMU_GAIN = 1.2              # 기울기 1° 당 고치는 속도 [도/s per 도]
IMU_MAX = 10.0              # 몸을 이보다 많이 돌려서 맞추진 않음 [도]

# 조종기
STICK_FWD_SIGN = -1         # 위로 밀었는데 뒤로 가면 +1
STICK_SIDE_SIGN = +1        # 왼쪽으로 밀었는데 오른쪽으로 가면 -1
DEADZONE = 0.12             # 가운데 무시 구간 (0~1)
STICK_EXPO = 0.5            # 0 = 그대로, 1 = 가운데 근처를 아주 섬세하게
UNO_TIMEOUT = 0.5           # 조종기 신호가 이만큼[초] 없으면 멈춤
USE_POT_SPEED = False       # 볼륨으로 최고 속도 조절 (0 → POT_MIN 배, 끝 → 1 배)
POT_MIN = 0.3
# 버튼 번호 = 우노가 보내는 버튼 값의 비트. --joytest 로 눌러 보고 맞출 것
BTN_UP, BTN_DOWN, BTN_SAFE, BTN_LEFT, BTN_RIGHT, BTN_RESET = 0, 1, 2, 4, 5, 6
# ═══════════════════════════════════════════════════════════

ST, SW, SE = 0, 1, 2        # 다리 상태: 바닥 / 공중 / 바닥 찾는 중


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def smoothstep(x):
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def rotz(p, a):
    """(x, y) 를 a [rad] 만큼 반시계로 돌림. z 가 있으면 그대로 붙임."""
    c, s = math.cos(a), math.sin(a)
    return [c * p[0] - s * p[1], s * p[0] + c * p[1]] + list(p[2:])


def convex_hull(pts):
    pts = sorted(set((round(p[0], 3), round(p[1], 3)) for p in pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]          # 반시계


def stab_margin(pts, com=COM_OFFSET):
    """무게중심이 발들(xy)이 만드는 다각형 안쪽으로 얼마나 들어가 있나 [mm]. 밖이면 음수."""
    hull = convex_hull(pts)
    if len(hull) < 3:
        return -1e9
    m = 1e9
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey)
        if L < 1e-9:
            continue
        m = min(m, (ex * (com[1] - a[1]) - ey * (com[0] - a[0])) / L)
    return m


# ───────────────────────── 조종기 ─────────────────────────
class Joystick:
    def __init__(self, port):
        if serial is None:
            raise RuntimeError("pyserial 필요: pip install pyserial")
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
        except serial.SerialException as e:
            raise SystemExit(self._open_help(port, e))
        time.sleep(2)                       # 우노는 포트를 열면 리셋됨
        self.x = self.y = 512
        self.pot = 1023
        self.mask = 0
        self.pressed = 0                    # 새로 눌린 버튼 (take_pressed 로 읽으면 지워짐)
        self.last = 0.0
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    @staticmethod
    def _open_help(port, e):
        lines = [f"[우노] {port} 를 못 열었음: {e}"]
        try:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
        except ImportError:
            ports = []
        ports = [p for p in ports if "USB" in p.device or "ACM" in p.device]
        if ports:
            lines.append("  지금 연결된 USB 시리얼 포트:")
            lines += [f"    {p.device}  ({p.description})" for p in ports]
            lines.append("  → --uno 에 위 포트 중 조종기를 넣을 것 (우노를 뺐다 꽂아서 사라지는 쪽이 조종기)")
        else:
            lines.append("  USB 시리얼 포트가 하나도 없음 → 조종기 USB 케이블을 확인할 것")
        if getattr(e, "errno", None) == 13:
            lines.append("  권한 없음 → 'sudo usermod -aG dialout $USER' 한 뒤 로그아웃·로그인 (한 번만)")
        return "\n".join(lines)

    def _run(self):
        buf = b""
        while True:
            try:
                buf += self.ser.read(128)
            except serial.SerialException as e:
                print("[우노] 끊김:", e)
                time.sleep(1)
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not (line.startswith(b"<") and line.endswith(b">")):
                    continue
                try:
                    v = [int(t) for t in line[1:-1].split(b",")]
                except ValueError:
                    continue
                if len(v) < 3:
                    continue
                with self.lock:
                    self.pressed |= v[2] & ~self.mask
                    self.x, self.y, self.mask = v[0], v[1], v[2]
                    if len(v) > 3:
                        self.pot = v[3]
                    self.last = time.monotonic()
            if len(buf) > 1024:
                buf = b""

    def alive(self):
        return time.monotonic() - self.last < UNO_TIMEOUT

    def take_pressed(self):
        with self.lock:
            p, self.pressed = self.pressed, 0
        return p

    def held(self, bit):
        return bool(self.mask & (1 << bit))

    def speed_scale(self):
        if not USE_POT_SPEED:
            return 1.0
        return POT_MIN + (1.0 - POT_MIN) * clamp(self.pot / 1023.0, 0.0, 1.0)

    def axes(self):
        """(앞뒤, 좌우) -1~1. 좌우 + = 왼쪽."""
        def ax(v, sign):
            a = (v - 512) / 512.0 * sign
            if abs(a) < DEADZONE:
                return 0.0
            a = clamp((a - math.copysign(DEADZONE, a)) / (1 - DEADZONE), -1.0, 1.0)
            return a * (1 - STICK_EXPO) + a * abs(a) * STICK_EXPO
        return ax(self.y, STICK_FWD_SIGN), ax(self.x, STICK_SIDE_SIGN)


# ───────────────────────── 보행 상태 ─────────────────────────
class State:
    """한 순간의 보행 상태. 발 위치는 '수평 몸 좌표' (몸 중심 원점, x 앞, y 왼쪽, z 위, 바닥과 수평)."""
    __slots__ = ("evt", "phi", "feet", "p0", "tgt", "leg", "u", "lift_k", "press", "noc", "saw_off", "H", "tilt", "blocked_t")

    def copy(self):
        n = State.__new__(State)
        n.evt = [None] * NUM_LEGS            # 이번 틱 착지 사건 (다리별 "contact" / "floor") — 복사하면 비움
        n.phi = self.phi
        n.feet = [p[:] for p in self.feet]
        n.p0 = [p[:] for p in self.p0]
        n.tgt = [p[:] for p in self.tgt]
        n.leg = self.leg[:]
        n.u = self.u[:]
        n.lift_k = self.lift_k[:]
        n.press = self.press[:]
        n.noc = self.noc[:]
        n.saw_off = self.saw_off[:]
        n.H = self.H
        n.tilt = self.tilt[:]
        n.blocked_t = self.blocked_t
        return n


class Walker:
    def __init__(self, link, joy, start_safe=False, use_imu=IMU_LEVEL, clock=time.monotonic, sleep=time.sleep):
        self.link = link
        self.joy = joy
        self.clock = clock
        self.sleep = sleep
        self.hx = Hexapod()
        self.dt = 1.0 / LOOP_HZ
        self.gait = SAFE if start_safe else NORMAL
        self.pending = None                 # 바꾸기로 한 모드 (다리가 다 바닥에 있을 때 바뀜)
        self.use_imu = use_imu
        self.imu_inv = None                 # IMU 변화 → 몸 기울기 명령 (2x2). None = 수평 기능 꺼짐
        self.foot_ok = [True] * NUM_LEGS     # 다리별 접지 버튼을 믿을 수 있나 (안 되면 그 다리만 버튼 없이)
        self.foot_miss = [0] * NUM_LEGS
        self.foot_revive = [0.0] * NUM_LEGS
        self.s = None
        self.v = [0.0, 0.0]                 # 지금 몸 속도 [mm/s] (몸 좌표)
        self.w = 0.0                        # 지금 회전 속도 [rad/s]
        self.H_target = 0.0
        self.active = False                 # 걸음 시계가 도는 중
        self.idle = True                    # 조종 입력이 없음
        self.quiet = False                  # True 면 조종기 입력 무시 (끝낼 때)
        self.hold_since = None              # 높이 버튼을 누르기 시작한 시각
        self.h_still_since = 0.0            # 높이가 마지막으로 바뀐 시각
        self.last_print = {}
        self.stats = {"fallback": 0, "hold": 0, "ticks": 0}
        self.tuning = False                 # 튜닝 중: 노랑/파랑 = 속도 단계 (높이 대신)
        self.tune_step = 0
        self.stick_mag, self.turn_in = 0.0, 0
        self.tick_info = None               # 지난 틱 지표 (튜닝용)
        self._build_tables()

    # ── 출력 ──
    def say(self, key, msg, every=2.0):
        now = self.clock()
        if now - self.last_print.get(key, -1e9) >= every:
            self.last_print[key] = now
            print(msg)

    # ── 기하 ──
    def homes(self, H, g=None):
        """몸 높이 H 일 때 발 '원래 자리' 6개 (수평 몸 좌표). 높이가 범위 밖이면 None."""
        g = g or self.gait
        s = (TIBIA_LEN - H) / FEMUR_LEN
        if not -1.0 < s < 1.0:
            return None
        r_tv = FEMUR_LEN * math.sqrt(1 - s * s)                 # tibia 가 바닥에 수직인 거리 (femur 축 기준)
        far = math.sqrt(max(0.0, (FEMUR_LEN + TIBIA_LEN - REACH_MARGIN) ** 2 - H * H))
        r = r_tv + min(g.spread_mm, max(0.0, far - r_tv) * SAFE_SPREAD_MAX)
        out = []
        for mx, my, th in MOUNT:
            rho = COXA_LEN + r
            out.append([mx + rho * math.cos(math.radians(th)), my + rho * math.sin(math.radians(th)), -H])
        return out

    def _capacity(self, H, g):
        """높이 H 에서 어느 방향으로든 걸을 수 있는 보폭 [mm] (다리가 원래 자리 ± 반 걸음까지 닿고 들 수 있음)."""
        hs = self.homes(H, g)
        if hs is None:
            return 0.0
        dirs = [(math.cos(k * math.pi / 6), math.sin(k * math.pi / 6)) for k in range(12)]
        checks = [(0.5, 0.0), (0.25, MIN_LIFT), (0.0, MIN_LIFT)]
        if g.safe:
            checks.append((0.5, -10.0))                         # 바닥이 조금 낮아도 찾아 내려갈 여유
        def ok(S):
            for l, h in enumerate(hs):
                for dx, dy in dirs:
                    for frac, dz in checks:
                        if leg_ik(l, [h[0] + dx * S * frac, h[1] + dy * S * frac, h[2] + dz]) is None:
                            return False
            return True
        if not ok(0.0):
            return 0.0
        lo, hi = 0.0, 240.0
        for _ in range(10):
            mid = (lo + hi) / 2
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo

    def _build_tables(self):
        self.cap = {}
        self.hrange = {}
        for g in (NORMAL, SAFE):
            tab = {h: self._capacity(float(h), g) for h in range(40, 237, 3)}
            good = [h for h, S in tab.items() if S >= MIN_STRIDE]
            self.cap[g.name] = tab
            self.hrange[g.name] = (float(min(good)), float(max(good))) if good else (None, None)

    def capacity(self, H, g=None):
        g = g or self.gait
        tab = self.cap[g.name]
        keys = sorted(tab)
        if H <= keys[0]:
            S = tab[keys[0]]
        elif H >= keys[-1]:
            S = tab[keys[-1]]
        else:
            k = min(int((H - keys[0]) // (keys[1] - keys[0])), len(keys) - 2)
            a, b = keys[k], keys[k + 1]
            S = tab[a] + (tab[b] - tab[a]) * (H - a) / (b - a)
        return min(S, g.stride_max)

    def h_range(self, g=None):
        return self.hrange[(g or self.gait).name]

    def stance_plan(self, g=None):
        g = g or self.gait
        return g.period * (1 - g.swing_frac) * (SAFE_STANCE_FACTOR if g.safe else 1.0)

    def offsets(self, g=None):
        g = g or self.gait
        off = [0.0] * NUM_LEGS
        for k, grp in enumerate(g.order):
            for l in grp:
                off[l] = k / len(g.order)
        return off

    # ── 센서 ──
    def contact_ok(self):
        return self.link.foot_fresh()

    def foot_raw(self, l):
        """버튼 값 그대로 (고장으로 뺀 다리도). 신호가 없으면 None."""
        if not self.link.foot_fresh():
            return None
        f = self.link.foot
        return None if f is None else bool(f[l])

    def contact(self, l):
        """믿을 수 있는 버튼만. 뺀 다리는 None (= 버튼 없음으로 다룸)."""
        return self.foot_raw(l) if self.foot_ok[l] else None

    def check_feet(self):
        """서 있을 때 (발이 다 바닥) 안 눌린 버튼은 고장으로 보고 뺌."""
        end = self.clock() + 1.0
        while self.clock() < end and not self.contact_ok():
            self.sleep(0.05)
        if not self.contact_ok():
            return
        self.sleep(0.3)
        raw = [self.foot_raw(l) for l in range(NUM_LEGS)]
        dead = [l for l in range(NUM_LEGS) if raw[l] is False]
        for l in dead:
            self.foot_ok[l] = False
        if not dead:
            print("[접지] 서 있을 때 6개 버튼 모두 눌림 — 정상")
        elif len(dead) == NUM_LEGS:
            print("[접지] 서 있는데 버튼이 하나도 안 눌림 — 배선·펌웨어 확인 (펌웨어는 LOW = 닿음). 접지 기능 없이 걸음")
        else:
            print(f"[접지] 서 있는데 다리 {[l + 1 for l in dead]} 버튼이 안 눌림 → 고장/배선 빠짐으로 보고 그 다리만 버튼 없이 "
                  f"(눌리는 다리: {[l + 1 for l in range(NUM_LEGS) if l not in dead]})")

    def _update_feet_health(self, s, dt):
        """착지 결과로 버튼 고장을 판단하고, 고장으로 뺀 버튼이 다시 잘 눌리면 되살림."""
        for l in range(NUM_LEGS):
            if s.evt[l] == "contact":
                self.foot_miss[l] = 0
            elif s.evt[l] == "floor":
                self.foot_miss[l] += 1
                if self.foot_ok[l] and self.foot_miss[l] >= FOOT_DEAD_MISSES:
                    self.foot_ok[l] = False
                    print(f"[접지] 다리 {l + 1}: {FOOT_DEAD_MISSES}번 연속 바닥을 못 찾음 → 버튼 고장으로 보고 뺌 (버튼 없이 걸음)")
            if not self.foot_ok[l]:
                if s.leg[l] == ST and self.foot_raw(l):
                    self.foot_revive[l] += dt
                    if self.foot_revive[l] >= FOOT_REVIVE_SEC:
                        self.foot_ok[l], self.foot_miss[l] = True, 0
                        print(f"[접지] 다리 {l + 1}: 버튼이 다시 잘 눌림 → 다시 씀")
                else:
                    self.foot_revive[l] = 0.0

    # ── 관절 ──
    def _joints(self, s):
        R = _rot(s.tilt[0], s.tilt[1], 0.0)
        out = []
        for l in range(NUM_LEGS):
            j = leg_ik(l, _mul_t(R, s.feet[l]))
            if j is None:
                return None
            out.append(j)
        return out

    def _ik_ok(self, l, p, tilt):
        return leg_ik(l, _mul_t(_rot(tilt[0], tilt[1], 0.0), p)) is not None

    def _send_joints(self, joints):
        self.hx.set_pose(joints)
        self.hx.move_update(self.link)
        if not self.hx.state_known:
            raise ConnectionError("로봇 전송 실패")

    # ── 보행 한 순간 ──
    def _land_target(self, h, v, w, Tp):
        q = rotz(h, w * Tp / 2)
        return [q[0] + v[0] * Tp / 2, q[1] + v[1] * Tp / 2]

    def _swing_pos(self, s, l, target, end_z, home):
        """공중 다리 위치: 먼저 들고 → 옮기고 → 수직으로 내림.
        못 닿으면 드는 높이부터 줄이고, 그래도 안 되면 내려놓을 자리를 원래 자리 쪽으로 당김."""
        u, p0, g = s.u[l], s.p0[l], self.gait
        sh = smoothstep((u - 0.12) / 0.76)
        zb = p0[2] + (end_z - p0[2]) * smoothstep(u)
        bump = math.sin(math.pi * u) ** 2                       # 뜰 때·닿을 때 속도 0 → 부드럽고 충격 적음
        for pull in (0.0, 0.25, 0.5, 0.75, 1.0):
            tx = target[0] + (home[0] - target[0]) * pull
            ty = target[1] + (home[1] - target[1]) * pull
            x = p0[0] + (tx - p0[0]) * sh
            y = p0[1] + (ty - p0[1]) * sh
            k0 = s.lift_k[l]
            for k in (k0, k0 * 0.6, k0 * 0.3, 0.0):
                p = [x, y, zb + g.lift * k * bump]
                if self._ik_ok(l, p, s.tilt):
                    s.lift_k[l] = k
                    if pull:
                        s.tgt[l] = [tx, ty]
                    return p
        return [x, y, zb]

    @staticmethod
    def _shift_z(s, d):
        for l in range(NUM_LEGS):
            s.feet[l][2] += d
            s.p0[l][2] += d

    def _advance(self, s0, dt, v, w, dH, move, adjust):
        s = s0.copy()
        g = self.gait
        Tp = self.stance_plan()
        Tsw = g.period * g.swing_frac
        use_ct = g.safe and self.contact_ok()
        ct = [self.contact(l) for l in range(NUM_LEGS)] if use_ct else [None] * NUM_LEGS

        # 0) 높이: 몸이 dH 올라가면 발은 몸 기준 dH 내려감
        if dH:
            s.H += dH
            self._shift_z(s, -dH)
        hs = self.homes(s.H)
        if hs is None:
            return None
        ground = -s.H

        # 1) 몸 이동 → 바닥에 있는 발(바닥·바닥 찾는 중)은 반대로 밀림
        if move and (v[0] or v[1] or w):
            a = -w * dt
            for l in range(NUM_LEGS):
                if s.leg[l] != SW:
                    p = s.feet[l]
                    q = rotz([p[0] - v[0] * dt, p[1] - v[1] * dt], a)
                    s.feet[l] = [q[0], q[1], p[2]]

        # 2) 공중 다리
        for l in range(NUM_LEGS):
            if s.leg[l] != SW:
                continue
            s.u[l] = min(1.0, s.u[l] + dt / Tsw)
            if ct[l] is False:
                s.saw_off[l] = True
            want = self._land_target(hs[l], v, w, Tp)
            d = math.dist(want, s.tgt[l])
            lim = TARGET_SPEED * dt
            if d > lim:
                want = [s.tgt[l][i] + (want[i] - s.tgt[l][i]) * lim / d for i in (0, 1)]
            s.tgt[l] = want
            target = want
            end_z = ground + (LAND_NEAR if ct[l] is not None else 0.0)
            s.feet[l] = self._swing_pos(s, l, target, end_z, hs[l])
            if s.saw_off[l] and s.u[l] > 0.55 and ct[l]:
                s.leg[l] = ST                                   # 턱 위에 일찍 닿음 → 거기 놓음
                s.noc[l] = 0.0
                s.evt[l] = "contact"
            elif s.u[l] >= 1.0:
                s.leg[l] = SE if ct[l] is not None else ST      # 버튼 없는(뺀) 다리는 예상 바닥에 그냥 놓음
                s.noc[l] = 0.0

        # 3) 바닥 찾는 다리: 천천히 내리다 버튼이 눌리면 놓음
        for l in range(NUM_LEGS):
            if s.leg[l] != SE:
                continue
            if ct[l] is False:
                s.saw_off[l] = True
            if ct[l] is None:
                s.leg[l] = ST
                continue
            if s.saw_off[l] and ct[l]:
                s.leg[l] = ST
                s.evt[l] = "contact"
                continue
            floor = ground - (MAX_DOWN if s.saw_off[l] else 0.0)   # 버튼이 한 번도 안 떨어졌으면 고장으로 보고 예상 바닥까지만
            nz = s.feet[l][2] - LAND_SPEED * dt
            if nz <= floor:
                s.feet[l][2] = floor
                s.leg[l] = ST
                if s.saw_off[l]:
                    s.evt[l] = "floor"
                    self.say(f"down{l}", f"[접지] 다리 {l + 1}: {MAX_DOWN:.0f}mm 더 내려도 안 닿음 — 거기 둠")
            else:
                s.feet[l][2] = nz

        # 4) 시계 → 차례가 온 다리 들기 (안전 모드: 바닥 찾는 다리가 있으면 기다림)
        if self.active and SE not in s.leg:
            phi_new = s.phi + dt / g.period
            off = self.offsets()
            lifting = []
            for l in range(NUM_LEGS):
                if s.leg[l] != ST or (off[l] - s.phi) % 1.0 >= phi_new - s.phi:
                    continue
                if self.idle and math.dist(s.feet[l][:2], hs[l][:2]) < SETTLE_TOL \
                        and abs(s.feet[l][2] - ground) < SETTLE_TOL:
                    continue                                    # 이미 제자리 → 안 듦
                lifting.append(l)
            blocked = False
            if g.safe and lifting:
                support = [s.feet[k][:2] for k in range(NUM_LEGS) if s.leg[k] == ST and k not in lifting]
                need = STAB_HARD if s.blocked_t > STAB_RELAX_SEC else STAB_MARGIN
                if stab_margin(support) < need:
                    blocked = True
                    if adjust and support:                      # 몸 이동이 막힌 순간에도 받침 가운데로는 옮김
                        cx = sum(p[0] for p in support) / len(support) - COM_OFFSET[0]
                        cy = sum(p[1] for p in support) / len(support) - COM_OFFSET[1]
                        d = math.hypot(cx, cy)
                        if d > 0.5:
                            step = min(d, SWAY_SPEED * dt)
                            for k in range(NUM_LEGS):
                                if s.leg[k] != SW:
                                    s.feet[k][0] -= cx / d * step
                                    s.feet[k][1] -= cy / d * step
            if blocked:
                s.blocked_t += dt
                if s.blocked_t > 0.6:
                    self.say("stab", f"[안정] 다리 {[l + 1 for l in lifting]} 들면 무게중심이 받침 밖에 가까움 — 몸을 옮기며 기다림")
            else:
                s.blocked_t = 0.0
                s.phi = phi_new % 1.0
                for l in lifting:
                    s.leg[l] = SW
                    s.u[l] = 0.0
                    s.p0[l] = s.feet[l][:]
                    s.tgt[l] = self._land_target(hs[l], v if move else [0.0, 0.0], w if move else 0.0, self.stance_plan())
                    s.lift_k[l] = 1.0
                    s.press[l] = 0.0
                    s.saw_off[l] = False

        # 5) 안전 모드 보정
        if adjust:
            if use_ct:                                          # 바닥 다리 접지 유지
                for l in range(NUM_LEGS):
                    if s.leg[l] != ST:
                        continue
                    if ct[l] is False:
                        s.noc[l] += dt
                        if s.noc[l] > PRESS_DELAY and s.press[l] < PRESS_MAX:
                            d = min(PRESS_SPEED * dt, PRESS_MAX - s.press[l])
                            s.feet[l][2] -= d
                            s.press[l] += d
                    else:
                        s.noc[l] = 0.0
            if g.safe:                                          # 바닥 다리 평균 높이 = 몸 높이
                zs = [s.feet[l][2] for l in range(NUM_LEGS) if s.leg[l] == ST]
                if len(zs) >= 3:
                    err = ground - sum(zs) / len(zs)
                    lim = HEIGHT_FIX_SPEED * dt
                    self._shift_z(s, clamp(err * HEIGHT_FIX_GAIN * dt, -lim, lim))
            self._update_tilt(s, dt)
        return s

    def _update_tilt(self, s, dt):
        lim = IMU_RATE * dt
        m = self.link.imu_recent(IMU_AVG_N) if (self.gait.safe and self.imu_inv) else None
        if m is None:
            for i in (0, 1):                                    # 수평 기능을 안 쓰면 천천히 0 으로
                s.tilt[i] -= clamp(s.tilt[i], -lim, lim)
            return
        e = [0.0 if abs(x) < IMU_DEADBAND else x - math.copysign(IMU_DEADBAND, x) for x in m]
        inv = self.imu_inv
        cmd = [-(inv[0][0] * e[0] + inv[0][1] * e[1]), -(inv[1][0] * e[0] + inv[1][1] * e[1])]
        for i in (0, 1):
            s.tilt[i] = clamp(s.tilt[i] + clamp(IMU_GAIN * cmd[i] * dt, -lim, lim), -IMU_MAX, IMU_MAX)

    def _stable_ok(self, s0, s):
        """안전 모드: 받침 다각형 가장자리로 무게중심이 더 가까워지는 움직임을 막음."""
        if not self.gait.safe:
            return True
        legs = [l for l in range(NUM_LEGS) if s.leg[l] == ST]
        if len(legs) == NUM_LEGS:
            return True
        new = stab_margin([s.feet[l][:2] for l in legs])
        if new >= STAB_HARD:
            return True
        old = stab_margin([s0.feet[l][:2] for l in legs])
        return new >= old - 0.01

    # ── 조종 입력 ──
    def _read_inputs(self, dt):
        j = self.joy
        if self.quiet or j is None or not j.alive():
            self.hold_since = None
            return 0.0, 0.0, 0, 0
        pressed = j.take_pressed()
        if pressed & (1 << BTN_SAFE):
            nxt = NORMAL if (self.pending or self.gait).safe else SAFE
            self.pending = None if nxt is self.gait else nxt
            print(f"[모드] {nxt.name} 로 바꿈 (다리가 다 바닥에 있을 때 바뀜)")
        if pressed & (1 << BTN_RESET):
            self.H_target = DEFAULT_HEIGHT
        hdir = (1 if j.held(BTN_UP) else 0) - (1 if j.held(BTN_DOWN) else 0)
        if self.tuning:                                         # 튜닝: 노랑/파랑 = 속도 단계 올리기/내리기
            self.tune_step += (1 if pressed & (1 << BTN_UP) else 0) - (1 if pressed & (1 << BTN_DOWN) else 0)
            pressed &= ~((1 << BTN_UP) | (1 << BTN_DOWN))
            hdir = 0
        if pressed & (1 << BTN_UP):
            self.H_target += HEIGHT_STEP
        if pressed & (1 << BTN_DOWN):
            self.H_target -= HEIGHT_STEP
        if hdir:
            now = self.clock()
            if self.hold_since is None:
                self.hold_since = now
            elif now - self.hold_since > HEIGHT_HOLD:
                self.H_target += hdir * HEIGHT_RATE * dt
        else:
            self.hold_since = None
        fwd, side = j.axes()
        turn = (1 if j.held(BTN_LEFT) else 0) - (1 if j.held(BTN_RIGHT) else 0)
        k = j.speed_scale()
        return fwd * k, side * k, turn * k, hdir

    def _limit_motion(self, v, w, H):
        """보폭이 그 높이에서 닿는 만큼을 넘지 않게 (이동 + 회전 합쳐서) 줄임."""
        S = self.capacity(H) * STRIDE_USE
        Tp = self.stance_plan()
        hs = self.homes(H)
        if hs is None:
            return [0.0, 0.0], 0.0
        worst = 0.0
        for h in hs:
            # 회전은 호 길이로 (끝점 사이 거리로 재면 한 바퀴 가까이 돌 때 짧게 보여서 한계를 못 막음)
            worst = max(worst, math.hypot(v[0] * Tp - w * Tp * h[1], v[1] * Tp + w * Tp * h[0]))
        k = 1.0 if worst <= S or worst < 1e-9 else S / worst
        return [v[0] * k, v[1] * k], w * k

    def _headroom(self, v, w):
        """지금 바닥 다리들이 (v, w) 로 조금 더 가도 닿는가 → 속도 비율 (1, 0.5, 0.25, 0)."""
        s = self.s
        sp = math.hypot(*v)
        if sp < 1e-6 and abs(w) < 1e-6:
            return 1.0
        T = max(LOOKAHEAD, 1.5 * sp / self.gait.accel, 1.5 * abs(w) / math.radians(self.gait.w_accel))
        R = _rot(s.tilt[0], s.tilt[1], 0.0)
        for k in (1.0, 0.5, 0.25):
            ok = True
            for l in range(NUM_LEGS):
                if s.leg[l] == SW:
                    continue
                p = s.feet[l]
                q = rotz([p[0] - v[0] * k * T, p[1] - v[1] * k * T], -w * k * T)
                if leg_ik(l, _mul_t(R, [q[0], q[1], p[2]])) is None:
                    ok = False
                    break
            if ok:
                return k
        return 0.0

    def _apply_pending_mode(self):
        s = self.s
        if self.pending is None or any(x != ST for x in s.leg):
            return
        self.gait, self.pending = self.pending, None
        lo, hi = self.h_range()
        self.H_target = clamp(self.H_target, lo, hi)
        s.phi = (-1e-3) % 1.0
        s.blocked_t = 0.0
        extra = ""
        if self.gait.safe:
            if not self.contact_ok():
                extra += " / 접지 신호 없음 → 접지 기능 없이"
            if not self.imu_inv:
                extra += " / IMU 수평 꺼짐"
        print(f"[모드] {self.gait.name} — 높이 {lo:.0f}~{hi:.0f}mm, 최고 {self.gait.v_max:.0f}mm/s{extra}")

    # ── 한 틱 ──
    def tick(self, dt):
        s = self.s
        g = self.gait
        self._apply_pending_mode()
        fwd, side, turn, hdir = self._read_inputs(dt)
        self.stick_mag, self.turn_in = min(1.0, math.hypot(fwd, side)), turn
        lo, hi = self.h_range()
        self.H_target = clamp(self.H_target, lo, hi)

        # 높이: 목표로 부드럽게 (끝에서 천천히)
        err = self.H_target - s.H
        dH = clamp(err * 4.0 * dt, -HEIGHT_SPEED * dt, HEIGHT_SPEED * dt) if abs(err) > 0.05 else 0.0
        if dH:
            self.h_still_since = self.clock()

        # 속도: 조이스틱 → 원하는 속도 → 보폭 한계 → 가속도 한계
        mag = math.hypot(fwd, side)
        if mag > 1.0:
            fwd, side = fwd / mag, side / mag
        v_des = [fwd * g.v_max, side * g.v_max]
        w_des = turn * math.radians(g.w_max)
        if g.safe and s.blocked_t > 0:
            v_des, w_des = [0.0, 0.0], 0.0                      # 다리를 못 들고 기다리는 중 → 서서 몸만 받침 가운데로
        elif g.safe and SE in s.leg:
            v_des = [x * SEARCH_SLOW for x in v_des]
            w_des *= SEARCH_SLOW
        v_des, w_des = self._limit_motion(v_des, w_des, s.H)
        k = self._headroom(v_des, w_des)                        # 다리 끝에 가까우면 미리 천천히 (다리를 옮기면 다시 빨라짐)
        v_des, w_des = [x * k for x in v_des], w_des * k
        dv = [v_des[0] - self.v[0], v_des[1] - self.v[1]]
        n = math.hypot(*dv)
        a = g.accel * dt
        if n > a:
            dv = [dv[0] / n * a, dv[1] / n * a]
        self.v = [self.v[0] + dv[0], self.v[1] + dv[1]]
        self.w += clamp(w_des - self.w, -math.radians(g.w_accel) * dt, math.radians(g.w_accel) * dt)
        self.v, self.w = self._limit_motion(self.v, self.w, s.H)

        # 걸음 시계를 돌릴지
        self.idle = mag == 0 and turn == 0
        moving = math.hypot(*self.v) > 1.0 or abs(self.w) > 0.01
        hs = self.homes(s.H)
        far = max(math.dist(s.feet[l][:2], hs[l][:2]) for l in range(NUM_LEGS)) if hs else 0.0
        all_down = all(x == ST for x in s.leg)
        if not self.idle or moving:
            self.active = True
        elif all_down and far < SETTLE_TOL:
            self.active = False
        elif far > REHOME_TOL and self.clock() - self.h_still_since > 0.3:
            self.active = True
        if self.pending is not None:
            self.active = True                                  # 모드를 바꾸려면 걸음 사이에 다리가 다 바닥에 있어야 함

        # 계산 → 못 닿으면 몸 이동부터 하나씩 빼고 다시. 관절이 너무 빨리 돌면 이번 틱의 시간을 늦춤
        new = None
        step_max = JOINT_SPEED_MAX * max(dt, self.dt)          # 실제 틱 간격 기준 (블루투스가 느려 50Hz 가 안 나와도 너무 빡빡하지 않게)
        for scale in (1.0, 0.5, 0.25, 0.125):
            found = None
            for mk, height, adjust in ((1.0, True, True), (0.5, True, True), (0.25, True, True),
                                       (0.0, True, True), (0.0, False, True), (0.0, False, False)):
                cand = self._advance(s, dt * scale, [x * mk for x in self.v], self.w * mk,
                                     dH * scale if height else 0.0, mk > 0, adjust)
                if cand is None:
                    continue
                j = self._joints(cand)
                if j is not None and self._stable_ok(s, cand):
                    found = (cand, j, mk, height, adjust)
                    break
            if found is None:
                break
            new, joints, mk, height, adjust = found
            jump = max(abs(j[i] - self.hx.target[l][i]) for l, j in enumerate(joints) for i in range(NUM_JOINTS))
            if jump <= step_max:
                break
        if new is not None and scale < 1.0:
            self.stats["slow"] = self.stats.get("slow", 0) + 1
        self.stats["ticks"] += 1
        if new is None:
            self.stats["hold"] += 1
            self.v, self.w = [0.0, 0.0], 0.0
            self.tick_info = {"dt": dt, "v": 0.0, "w": 0.0, "slow": 0, "fallback": 1, "jump": 0.0}
            self.say("hold", "[보행] 이번 순간은 어떤 움직임도 다리가 안 닿음 — 자세 유지")
            return
        if mk < 1.0 or not (height and adjust):
            self.stats["fallback"] += 1
            if mk < 1.0:
                self.v, self.w = [x * mk for x in self.v], self.w * mk   # 반씩 줄여서 부드럽게 멈춤
                if mk == 0.0 and moving:
                    self.say("reach", "[보행] 다리가 끝까지 닿아서 몸을 멈춤 (다리를 옮긴 뒤 계속)")
            if not height:
                self.H_target = s.H
                self.say("hreach", f"[높이] {s.H:.0f}mm 에서 더는 다리가 안 닿음 — 여기서 멈춤")
        if abs(new.H - self.H_target) < 0.5 and abs(s.H - self.H_target) >= 0.5:
            print(f"[높이] {new.H:.0f}mm")
        jump = max(abs(j[i] - self.hx.target[l][i]) for l, j in enumerate(joints) for i in range(NUM_JOINTS))
        self.tick_info = {"dt": dt, "v": math.hypot(*self.v) * scale, "w": self.w * scale,   # 실제로 나온 속도
                          "slow": int(scale < 1.0), "fallback": int(mk < 1.0), "jump": jump}
        self.s = new
        self._update_feet_health(new, dt)
        self._send_joints(joints)

    def run_for(self, sec, until=None, on_tick=None):
        """sec 초 동안 (또는 until() 이 True 가 될 때까지) 틱을 돌림. on_tick() = 매 틱 뒤에 부름."""
        end = self.clock() + sec
        last = self.clock()
        while self.clock() < end:
            t0 = self.clock()
            dt = clamp(t0 - last, 1e-3, 2.0 / LOOP_HZ)
            last = t0
            while self.link.log_queue:
                print("[로봇]", self.link.log_queue.popleft())
            self.tick(dt)
            if on_tick is not None:
                on_tick()
            if until is not None and until():
                return True
            self.sleep(max(0.0, self.dt - (self.clock() - t0)))
        return False

    def settled(self):
        return not self.active and all(x == ST for x in self.s.leg)

    # ── 시작 ──
    def start(self):
        if not self.hx.sync(self.link):
            return False
        self.link.imu_stream(True)
        self.hx.max_speed = [[1e9] * NUM_JOINTS for _ in range(NUM_LEGS)]   # SDK 보간 끔 → 계산한 각도 그대로
        self.hx.timedelay = 0.0
        start = [row[:] for row in self.hx.cmd]
        n = max(1, round(START_SEC * LOOP_HZ))
        for k in range(1, n + 1):
            u = smoothstep(k / n)
            self._send_joints([[a + (b - a) * u for a, b in zip(sj, tj)] for sj, tj in zip(start, STAND_POSE)])
            self.sleep(self.dt)

        s = State.__new__(State)
        s.evt = [None] * NUM_LEGS
        s.feet = [leg_fk(l, STAND_POSE[l]) for l in range(NUM_LEGS)]
        s.p0 = [p[:] for p in s.feet]
        s.tgt = [p[:2] for p in s.feet]
        s.leg = [ST] * NUM_LEGS
        s.u = [0.0] * NUM_LEGS
        s.lift_k = [1.0] * NUM_LEGS
        s.press = [0.0] * NUM_LEGS
        s.noc = [0.0] * NUM_LEGS
        s.saw_off = [False] * NUM_LEGS
        s.H = -sum(p[2] for p in s.feet) / NUM_LEGS
        s.tilt = [0.0, 0.0]
        s.phi = (-1e-3) % 1.0
        s.blocked_t = 0.0
        self.s = s
        self.rest_H = s.H
        self.H_target = s.H
        self.h_still_since = self.clock()

        for g in (NORMAL, SAFE):
            lo, hi = self.h_range(g)
            if lo is None:
                print(f"[높이] {g.name}: 걸을 수 있는 높이가 없음 — MIN_STRIDE / 다리 길이를 확인할 것")
                return False
            print(f"[높이] {g.name}: {lo:.0f} ~ {hi:.0f}mm "
                  f"(보폭 {self.capacity(lo, g):.0f}~{max(self.capacity(h, g) for h in range(int(lo), int(hi) + 1)):.0f}mm)")
        lo, hi = self.h_range()
        if not lo <= s.H <= hi:
            print(f"[높이] 서기 높이 {s.H:.0f}mm 가 {self.gait.name} 범위 밖 — 범위 안으로 옮김")

        # 시작 높이에서 먼저 발을 원래 자리로 (모드에 따라 발 자리가 서기 자세와 다를 수 있음)
        self.H_target = clamp(s.H, lo, hi)
        self.quiet = True
        self.active = True
        self.run_for(15.0, until=lambda: self.settled() and abs(self.s.H - self.H_target) < 0.5)

        # IMU 방향 재기 (낮은 자세라 몸을 충분히 기울여 볼 수 있음)
        if self.use_imu:
            self.imu_setup()
        if not self.contact_ok():
            print("[접지] 접지 버튼 신호(#ft)가 안 옴 — 안전 모드에서도 접지 기능은 꺼짐")
        else:
            self.check_feet()
        self.quiet = False
        self.H_target = clamp(DEFAULT_HEIGHT, lo, hi)
        print(f"[시작] {self.gait.name} / 높이 {self.H_target:.0f}mm 로 올림")
        return True

    def reset_sim(self, g, H):
        """(시뮬레이션) 보행 g, 높이 H 의 원래 자리에 바로 선 상태로."""
        hs = self.homes(H, g)
        s = State.__new__(State)
        s.evt = [None] * NUM_LEGS
        s.feet = [p[:] for p in hs]
        s.p0 = [p[:] for p in hs]
        s.tgt = [p[:2] for p in hs]
        s.leg = [ST] * NUM_LEGS
        s.u, s.lift_k, s.press, s.noc = [0.0] * NUM_LEGS, [1.0] * NUM_LEGS, [0.0] * NUM_LEGS, [0.0] * NUM_LEGS
        s.saw_off = [False] * NUM_LEGS
        s.H, s.tilt, s.phi, s.blocked_t = H, [0.0, 0.0], (-1e-3) % 1.0, 0.0
        self.gait, self.pending, self.s = g, None, s
        self.v, self.w, self.H_target, self.active, self.quiet = [0.0, 0.0], 0.0, H, False, False
        joints = self._joints(s)
        self.hx.cmd = [j[:] for j in joints]
        self.hx.target = [j[:] for j in joints]
        self.hx.state_known = True

    # ── IMU ──
    def _tilt_to(self, roll, pitch, sec=0.6):
        s0 = self.s
        a = s0.tilt[:]
        n = max(1, round(sec * LOOP_HZ))
        for k in range(1, n + 1):
            u = smoothstep(k / n)
            s = s0.copy()
            s.tilt = [a[0] + (roll - a[0]) * u, a[1] + (pitch - a[1]) * u]
            j = self._joints(s)
            if j is None:
                return False
            self._send_joints(j)
            self.s = s
            self.sleep(self.dt)
        return True

    def _imu_read(self):
        self.sleep(IMU_SETTLE)
        end = self.clock() + 0.5
        while self.clock() < end:
            v = self.link.imu_recent(IMU_AVG_N)
            if v is not None:
                return v
            self.sleep(0.02)
        return None

    def imu_setup(self):
        end = self.clock() + 1.5
        while self.clock() < end and not self.link.imu_fresh() and not self.link.imu_err:
            self.sleep(0.05)
        if not self.link.imu_fresh():
            why = "펌웨어가 #imu,err (IMU 못 찾음)" if self.link.imu_err else "#imu 가 안 옴"
            print(f"[IMU] {why} — 수평 기능 끔")
            return False
        cols = []
        for axis in (0, 1):
            d = [IMU_CAL_DEG if i == axis else 0.0 for i in (0, 1)]
            ok = self._tilt_to(*d)
            mp = self._imu_read() if ok else None
            ok = ok and self._tilt_to(*[-x for x in d])
            mm = self._imu_read() if ok else None
            self._tilt_to(0.0, 0.0)
            if mp is None or mm is None:
                print("[IMU] 기울여 보는 중 실패 (다리가 안 닿거나 IMU 값이 끊김) — 수평 기능 끔")
                return False
            cols.append([(mp[i] - mm[i]) / (2 * IMU_CAL_DEG) for i in (0, 1)])
        a, c = cols[0]
        b, d = cols[1]
        det = a * d - b * c
        print(f"[IMU] 명령 roll 1° → IMU roll {a:+.2f} pitch {c:+.2f} / 명령 pitch 1° → IMU roll {b:+.2f} pitch {d:+.2f}")
        if abs(det) < IMU_CAL_MIN_GAIN ** 2 or max(abs(a), abs(b)) < IMU_CAL_MIN_GAIN \
                or max(abs(c), abs(d)) < IMU_CAL_MIN_GAIN:
            print("[IMU] 몸을 기울여도 IMU 가 거의 안 변함 — 수평 기능 끔")
            return False
        self.imu_inv = [[d / det, -b / det], [-c / det, a / det]]
        print(f"[IMU] 방향 확인 ({'roll·pitch 축이 바뀌어 붙어 있음' if abs(b) > abs(a) else '축 그대로'}) — 안전 모드에서 수평 유지")
        return True

    # ── 끝 ──
    def shutdown(self):
        print("[끝] 멈추고 낮추는 중… (한 번 더 Ctrl+C 면 바로 힘 뺌)")
        self.quiet = True
        self.pending = None
        self.run_for(6.0, until=self.settled)
        lo, _ = self.h_range()
        self.H_target = max(lo, min(self.rest_H, self.s.H))
        self.run_for(8.0, until=lambda: abs(self.s.H - self.H_target) < 0.5 and self.settled())


# ───────────────────────── 버튼 확인 ─────────────────────────
def joytest(port):
    joy = Joystick(port)
    print("버튼을 하나씩 눌러 보세요 (Ctrl+C = 끝). 지금 설정:",
          f"UP={BTN_UP} DOWN={BTN_DOWN} SAFE={BTN_SAFE} LEFT={BTN_LEFT} RIGHT={BTN_RIGHT} RESET={BTN_RESET}")
    try:
        while True:
            p = joy.take_pressed()
            for b in range(16):
                if p & (1 << b):
                    print(f"  버튼 비트 {b} 눌림")
            f, sd = joy.axes()
            print(f"\r  앞뒤 {f:+.2f} 좌우 {sd:+.2f} 볼륨 {joy.pot:4d} {'연결됨' if joy.alive() else '신호 없음'}   ",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


# ───────────────────────── 속도 튜닝 ─────────────────────────
class Meter:
    """조이스틱을 끝까지 민 채 (또는 회전 버튼만 누른 채) 걷는 동안의 지표."""

    def __init__(self, tick_sec):
        self.tick_sec = tick_sec
        self.t = self.t_move = self.dist = self.t_turn = self.turn = 0.0
        self.n = self.slow = self.fb = 0
        self.peak = 0.0
        self.win = deque(maxlen=LOOP_HZ)          # 최근 1초 IMU
        self.wob_sq, self.wob_n = 0.0, 0
        self.st_n = [0] * NUM_LEGS                # 다리별: 바닥에 있던 틱 / 그중 버튼이 떨어져 있던 틱
        self.st_off = [0] * NUM_LEGS
        self.wall = 0.0                           # 실제 걸린 시간 (루프 주기 계산)
        self.last_clock = None
        self.last_print = 0.0

    def add(self, w):
        info = w.tick_info
        full = w.stick_mag >= 0.9
        turning = w.turn_in != 0 and w.stick_mag < 0.1
        if info is None or not (full or turning):
            self.win.clear()
            self.last_clock = None
            return False
        now = w.clock()
        if self.last_clock is not None and now - self.last_clock < 0.5:
            self.wall += now - self.last_clock
        self.last_clock = now
        dt = info["dt"]
        self.n += 1
        self.t += dt
        if full:
            self.dist += info["v"] * dt
            self.t_move += dt
        else:
            self.turn += abs(info["w"]) * dt
            self.t_turn += dt
        self.slow += info["slow"]
        self.fb += info["fallback"]
        self.peak = max(self.peak, info["jump"] / max(dt, self.tick_sec))
        m = w.link.imu_recent(1)
        if m is not None:
            self.win.append(m)
            if len(self.win) >= 10:
                mr = sum(x[0] for x in self.win) / len(self.win)
                mp = sum(x[1] for x in self.win) / len(self.win)
                self.wob_sq += (m[0] - mr) ** 2 + (m[1] - mp) ** 2
                self.wob_n += 1
        for l in range(NUM_LEGS):
            v = w.foot_raw(l)
            if v is not None and w.s.leg[l] == ST:
                self.st_n[l] += 1
                self.st_off[l] += not v
        self.foot_ok = w.foot_ok[:]
        return True

    def leg_loss(self):
        """다리별 접지 끊김 비율 (기록 없으면 None)."""
        return [self.st_off[l] / self.st_n[l] if self.st_n[l] > 20 else None for l in range(NUM_LEGS)]

    def result(self):
        n = max(1, self.n)
        return {
            "sec": self.t,
            "speed": self.dist / self.t_move if self.t_move > 0.5 else None,
            "turn": math.degrees(self.turn / self.t_turn) if self.t_turn > 0.5 else None,
            "slow": self.slow / n,
            "fallback": self.fb / n,
            "peak": self.peak,
            "wobble": math.sqrt(self.wob_sq / self.wob_n) if self.wob_n > 20 else None,
            "contact_loss": self._ok_loss(),
            "hz": (self.n - 1) / self.wall if self.wall > 1.0 else None,
        }

    def _ok_loss(self):
        """버튼이 살아 있는 다리만의 접지 끊김 비율 (고장으로 뺀 다리는 판정에서 뺌)."""
        ok = getattr(self, "foot_ok", [True] * NUM_LEGS)
        n = sum(self.st_n[l] for l in range(NUM_LEGS) if ok[l])
        off = sum(self.st_off[l] for l in range(NUM_LEGS) if ok[l])
        return off / n if n > 50 else None


def _fmt(v, f, none="  -"):
    return none if v is None else format(v, f)


def _recommend_from(base, k_period, k_sf, fwd, side, turn):
    """측정한 실제 속도로 SPEED 값 만들기. 가속은 지금과 같은 시간에 최고 속도에 닿게."""
    v_max = max(5.0, math.floor(max(fwd or 0.0, side or 0.0) / 5) * 5)
    w_max = max(2.0, math.floor(turn or base.w_max))
    return {"period": round(k_period, 3), "swing_frac": k_sf, "v_max": v_max, "w_max": w_max,
            "accel": round(v_max * base.accel / base.v_max), "w_accel": round(w_max * base.w_accel / base.w_max)}


def tune_sim(height, save):
    """시뮬레이션: period·swing_frac 마다 실제로 나오는 속도와 관절 한계를 재서 최적값 추천."""
    clk = SimClock()
    link = FakeLink(clk)
    joy = FakeJoy(clk, [])
    w = Walker(link, joy, clock=clk.now, sleep=clk.sleep)
    link.walker = w
    link.IMU_BIAS = [0.0, 0.0]
    grids = {
        "normal": ([round(0.5 + 0.1 * i, 2) for i in range(12)], [0.40, 0.45]),
        "safe": ([round(2.0 + 0.4 * i, 2) for i in range(9)], [0.11, 0.13, 0.15]),
    }
    tests = [("fwd", {"fwd": 1.0}), ("side", {"side": 1.0}), ("turn", {"held": [BTN_LEFT]})]

    def measure(g, H, cmd):
        w.reset_sim(g, H)
        joy.script = [(0.0, 1e9, cmd)]
        joy.tapped.clear()
        joy.t0 = clk.now()
        warm, sec = max(1.5, 1.2 * g.period), max(4.0, 2.5 * g.period)
        n_warm, n = round(warm * LOOP_HZ), round(sec * LOOP_HZ)
        meter = Meter(w.dt)
        for k in range(n_warm + n):
            w.tick(w.dt)
            clk.sleep(w.dt)
            if k >= n_warm:
                meter.add(w)
        return meter.result()

    results = {}
    for base in (NORMAL, SAFE):
        lo, hi = w.h_range(base)
        H = clamp(height, lo, hi)
        periods, fracs = grids[base.key]
        print(f"\n── {base.name}, 높이 {H:.0f}mm (지금 설정: 주기 {base.period:.2f}s, 공중 비율 {base.swing_frac:.2f}, "
              f"최고 {base.v_max:.0f}mm/s) ──")
        print("   주기  공중   앞mm/s  옆mm/s  회전°/s  관절한계로늦춤  다리끝줄임  관절최고°/s")
        rows = []
        for sf in fracs:
            for per in periods:
                g = replace(base, period=per, swing_frac=sf, v_max=2000.0, w_max=360.0, accel=5000.0, w_accel=3000.0)
                r = {name: measure(g, H, cmd) for name, cmd in tests}
                row = {"period": per, "sf": sf, "fwd": r["fwd"]["speed"], "side": r["side"]["speed"],
                       "turn": r["turn"]["turn"], "slow": max(x["slow"] for x in r.values()),
                       "fallback": max(x["fallback"] for x in r.values()), "peak": max(x["peak"] for x in r.values())}
                rows.append(row)
                print(f"  {per:5.2f}  {sf:4.2f}  {_fmt(row['fwd'], '6.0f')}  {_fmt(row['side'], '6.0f')}  "
                      f"{_fmt(row['turn'], '6.1f')}   {row['slow'] * 100:8.1f}%     {row['fallback'] * 100:6.1f}%   "
                      f"{row['peak']:7.0f}", flush=True)
        clean = [r for r in rows if r["slow"] <= 0.03 and r["fallback"] <= 0.01 and r["fwd"]]
        if not clean:
            print("  → 걸음이 고른 조합이 없음 (모두 관절 한계에 걸림) — JOINT_SPEED_MAX / 높이를 확인할 것")
            continue
        top = max(r["fwd"] for r in clean)
        best = max((r for r in clean if r["fwd"] >= 0.97 * top), key=lambda r: (r["period"], r["sf"]))
        rec = _recommend_from(base, best["period"], best["sf"], best["fwd"], best["side"], best["turn"])
        results[base.key] = rec
        print(f"  ★ 추천: 주기 {rec['period']}s, 공중 비율 {rec['swing_frac']} → 앞 {best['fwd']:.0f}mm/s, "
              f"옆 {best['side']:.0f}mm/s, 회전 {best['turn']:.1f}°/s (관절 최고 {best['peak']:.0f}°/s)")
        print("     (가장 빠른 값의 97% 안에서 주기가 가장 긴 것 = 같은 속도면 다리를 덜 자주 옮기는 쪽)")
        g = replace(base, **rec)
        line = []
        for h in (lo + 0.15 * (hi - lo), lo + 0.5 * (hi - lo), lo + 0.85 * (hi - lo)):
            line.append(f"{h:.0f}mm → {_fmt(measure(g, h, {'fwd': 1.0})['speed'], '.0f')}mm/s")
        print("     추천값으로 높이별 앞 속도: " + ", ".join(line))

    if not results:
        return
    print("\n추천 SPEED (시뮬레이션은 관절 각도·속도 한계만 봄 — 흔들림·미끄러짐은 실제 로봇 --tune 으로 확인):")
    for key, rec in results.items():
        print(f'    "{key}": dict(' + ", ".join(f"{k}={rec[k]}" for k in SPEED_KEYS) + "),")
    if save:
        save_tuned(results, "sim")
    else:
        print(f"→ 저장해서 바로 쓰려면 --save 를 붙여 다시 실행 ({TUNED_FILE})")


class RobotTuner:
    """실제 로봇: 노랑/파랑으로 속도 단계를 바꾸며 걷고, 단계마다 지표를 모음."""

    def __init__(self, w):
        self.w = w
        self.bases = {"normal": NORMAL, "safe": SAFE}
        self.idx = {"normal": 0, "safe": 0}
        self.meters = {}
        w.tuning = True
        self._apply()

    def _apply(self):
        key = self.w.gait.key
        k = TUNE_LEVELS[self.idx[key]]
        self.w.gait = scaled(self.bases[key], k)
        g = self.w.gait
        print(f"[튜닝] {g.name} ×{k:.2f} 단계 ({self.idx[key] + 1}/{len(TUNE_LEVELS)}): 주기 {g.period:.2f}s, "
              f"최고 {g.v_max:.0f}mm/s, 회전 {g.w_max:.0f}°/s — 조이스틱을 끝까지 밀고 {TUNE_MIN_SEC:.0f}초 넘게 걸어 보세요")

    def on_tick(self):
        w = self.w
        key = w.gait.key
        if w.gait is self.bases[key] or w.tune_step:            # 모드가 바뀌었거나 노랑/파랑
            self.idx[key] = int(clamp(self.idx[key] + w.tune_step, 0, len(TUNE_LEVELS) - 1))
            w.tune_step = 0
            self._apply()
        k = TUNE_LEVELS[self.idx[key]]
        m = self.meters.setdefault((key, k), Meter(w.dt))
        if m.add(w) and m.t - m.last_print >= 3.0:
            m.last_print = m.t
            r = m.result()
            print(f"[튜닝] {w.gait.name} ×{k:.2f}: 속도 {_fmt(r['speed'], '.0f')}mm/s, 회전 {_fmt(r['turn'], '.1f')}°/s, "
                  f"흔들림 {_fmt(r['wobble'], '.2f')}°, 접지 끊김 {_fmt(r['contact_loss'] and r['contact_loss'] * 100, '.0f')}%, "
                  f"관절 한계 {r['slow'] * 100:.0f}%, 다리 끝 {r['fallback'] * 100:.0f}%, 루프 {_fmt(r['hz'], '.0f')}Hz "
                  f"(걸은 시간 {r['sec']:.0f}s)")

    def report_and_save(self, ask=True):
        if not self.meters:
            print("[튜닝] 모은 기록 없음")
            return
        results = {}
        for key, base in self.bases.items():
            rows = sorted((k, m.result()) for (kk, k), m in self.meters.items() if kk == key)
            rows = [(k, r) for k, r in rows if r["sec"] >= 1.0]
            if not rows:
                continue
            wmax = TUNE_WOBBLE_MAX[key]
            print(f"\n── {base.name} (기준: 흔들림 ≤ {wmax}°, 접지 끊김 ≤ {TUNE_CONTACT_LOSS_MAX * 100:.0f}%, "
                  f"관절 한계 ≤ {TUNE_SLOW_MAX * 100:.0f}%, 다리 끝 ≤ {TUNE_FALLBACK_MAX * 100:.0f}%) ──")
            print("   배율  걸은초  속도mm/s  회전°/s  흔들림°  접지끊김  관절한계  다리끝  루프Hz  판정")
            good = []
            for k, r in rows:
                why = []
                if r["sec"] < TUNE_MIN_SEC:
                    why.append("짧음")
                if r["wobble"] is not None and r["wobble"] > wmax:
                    why.append("흔들림")
                if r["contact_loss"] is not None and r["contact_loss"] > TUNE_CONTACT_LOSS_MAX:
                    why.append("접지")
                if r["slow"] > TUNE_SLOW_MAX:
                    why.append("관절")
                if r["fallback"] > TUNE_FALLBACK_MAX:
                    why.append("다리끝")
                if not why:
                    good.append((k, r))
                cl = r["contact_loss"]
                print(f"  ×{k:4.2f}  {r['sec']:5.0f}   {_fmt(r['speed'], '6.0f')}   {_fmt(r['turn'], '6.1f')}   "
                      f"{_fmt(r['wobble'], '5.2f')}   {_fmt(cl and cl * 100, '5.0f')}%   {r['slow'] * 100:5.0f}%  "
                      f"{r['fallback'] * 100:5.0f}%  {_fmt(r['hz'], '5.0f')}   {'OK' if not why else ', '.join(why)}")
            if any(r["wobble"] is None for _, r in rows):
                print("  (흔들림 '-' = IMU 값이 안 옴 → 흔들림은 판정에서 빠짐)")
            n = [0] * NUM_LEGS
            off = [0] * NUM_LEGS
            ok = [True] * NUM_LEGS
            for (kk, _), m in self.meters.items():
                if kk == key:
                    for l in range(NUM_LEGS):
                        n[l] += m.st_n[l]
                        off[l] += m.st_off[l]
                    ok = getattr(m, "foot_ok", ok)
            if sum(n):
                parts = []
                for l in range(NUM_LEGS):
                    if n[l] < 20:
                        parts.append(f"{l + 1}: -")
                    else:
                        parts.append(f"{l + 1}: {off[l] / n[l] * 100:.0f}%" + ("" if ok[l] else "(뺌)"))
                print("  다리별 접지 끊김: " + ", ".join(parts))
                print("  ('뺌' = 버튼 고장으로 보고 판정·접지 기능에서 뺀 다리. 100% 가까우면 그 다리 버튼·배선을 확인)")
            hz = [r["hz"] for _, r in rows if r["hz"]]
            if hz and min(hz) < LOOP_HZ * 0.8:
                print(f"  ※ 루프가 {min(hz):.0f}Hz 까지 떨어짐 (목표 {LOOP_HZ}Hz) — 블루투스 전송이 밀리는 중일 수 있음")
            if not good:
                print("  → 기준을 지킨 단계가 없음. 가장 느린 단계보다 더 느리게 해야 함 (TUNE_LEVELS 에 더 작은 배율 추가)")
                continue
            k, r = good[-1]
            g = scaled(base, k)
            rec = {kk: getattr(g, kk) for kk in SPEED_KEYS}
            # 실제로 나온 속도가 상한보다 낮으면 그 속도를 상한으로 (조이스틱 끝 근처가 헛돌지 않게)
            if r["speed"]:
                rec["v_max"] = min(rec["v_max"], max(5.0, math.ceil(r["speed"] / 5) * 5))
            if r["turn"]:
                rec["w_max"] = min(rec["w_max"], max(2.0, math.ceil(r["turn"])))
            rec["accel"] *= rec["v_max"] / g.v_max
            rec["w_accel"] *= rec["w_max"] / g.w_max
            rec["v_max"] = round(rec["v_max"])
            rec["w_max"] = round(rec["w_max"], 1)
            results[key] = rec
            print(f"  ★ 추천: ×{k:.2f} → 주기 {g.period:.2f}s, 최고 {rec['v_max']:.0f}mm/s, 회전 {rec['w_max']:.0f}°/s "
                  f"(실제 측정 속도 {_fmt(r['speed'], '.0f')}mm/s)")
            if k == TUNE_LEVELS[-1]:
                print("  (가장 빠른 단계까지 괜찮았음 → TUNE_LEVELS 에 더 큰 배율을 넣고 다시 해 볼 수 있음)")
        if not results:
            return
        if ask:
            try:
                yes = input(f"\n추천값을 {TUNED_FILE} 에 저장할까요? (다음 실행부터 적용) [y/N] ").strip().lower() == "y"
            except EOFError:
                yes = False
            if yes:
                save_tuned(results, "robot")


# ───────────────────────── 시뮬레이션 (로봇 없이 점검) ─────────────────────────
class SimClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, d):
        self.t += max(d, 1e-4)


class FakeLink:
    """로봇 대신: 명령을 받기만 하고, 접지·IMU 는 걷는 상태로부터 흉내 냄."""
    IMU_A = [[0.0, -1.0], [1.0, 0.0]]       # 몸 기울기 명령 → IMU 변화 (일부러 축을 바꿔 붙인 것처럼)
    IMU_BIAS = [2.5, -1.5]                  # 바닥 기울기 [도]

    def __init__(self, clock):
        self.clock = clock
        self.log_queue = deque(maxlen=500)
        self.is_open = True
        self.imu_err = False
        self.walker = None
        self.terrain = [0.0] * NUM_LEGS     # 다리별 바닥 높낮이 [mm] (+ = 턱)
        self.sent = 0

    def log(self, msg):
        self.log_queue.append(msg)

    def send(self, data):
        self.sent += 1
        return True

    def close(self):
        self.is_open = False

    def imu_stream(self, on=True):
        return True

    def imu_fresh(self, max_age=None):
        return True

    def imu_recent(self, n):
        w = self.walker
        if w is None or w.s is None:
            return None
        t = w.s.tilt
        A = self.IMU_A
        return tuple(A[i][0] * t[0] + A[i][1] * t[1] + self.IMU_BIAS[i] + random.gauss(0, 0.1) for i in (0, 1))

    def foot_fresh(self, max_age=None):
        return True

    @property
    def foot(self):
        w = self.walker
        if w is None or w.s is None:
            return [True] * NUM_LEGS
        s = w.s
        out = []
        for l in range(NUM_LEGS):
            others = [s.feet[k][2] for k in range(NUM_LEGS) if k != l and s.leg[k] == ST]
            floor = (sum(others) / len(others) if others else -s.H) + self.terrain[l]
            out.append(s.feet[l][2] <= floor + 0.3)
        return out


class FakeJoy:
    def __init__(self, clock, script):
        self.clock = clock
        self.script = script                # [(시작, 끝, {"fwd", "side", "held": [비트], "tap": [비트]})]
        self.tapped = set()
        self.pot = 1023
        self.t0 = 1e9                       # start() 가 끝난 뒤에 시작

    def _now(self):
        t = self.clock.now() - self.t0
        for i, (a, b, cmd) in enumerate(self.script):
            if a <= t < b:
                return i, cmd
        return None, {}

    def alive(self):
        return True

    def take_pressed(self):
        i, cmd = self._now()
        if i is None or i in self.tapped:
            return 0
        self.tapped.add(i)
        m = 0
        for b in cmd.get("tap", []):
            m |= 1 << b
        return m

    def held(self, bit):
        return bit in self._now()[1].get("held", [])

    def axes(self):
        c = self._now()[1]
        return c.get("fwd", 0.0), c.get("side", 0.0)

    def speed_scale(self):
        return 1.0


def simulate():
    random.seed(1)
    clk = SimClock()
    script = [
        (0.0, 4.0, {"fwd": 1.0}),                               # 일반: 앞으로
        (4.0, 7.0, {"fwd": 0.6, "held": [BTN_LEFT]}),           # 걸으면서 왼쪽으로 돎
        (7.0, 9.0, {"side": -1.0}),                             # 오른쪽 옆걸음
        (9.0, 11.0, {}),                                        # 놓음 → 제자리
        (11.0, 13.5, {"held": [BTN_UP], "tap": [BTN_UP]}),      # 높이 올림 (누르고 있음)
        (13.5, 16.0, {"fwd": 1.0}),                             # 높은 자세로 걷기
        (16.0, 16.1, {"tap": [BTN_SAFE]}),                      # 안전 모드
        (16.1, 20.0, {}),
        (20.0, 22.5, {"held": [BTN_UP], "tap": [BTN_UP]}),      # 안전 모드에서도 높게
        (22.5, 40.0, {"fwd": 1.0}),                             # 안전 모드 앞으로 (중간에 턱)
        (40.0, 48.0, {"fwd": 0.5, "side": 0.5, "held": [BTN_RIGHT]}),
        (48.0, 52.0, {}),
        (52.0, 52.1, {"tap": [BTN_SAFE]}),                      # 일반으로
        (52.1, 56.0, {"fwd": -1.0}),
        (56.0, 60.0, {"held": [BTN_DOWN], "tap": [BTN_DOWN]}),
    ]
    link = FakeLink(clk)
    joy = FakeJoy(clk, script)
    w = Walker(link, joy, clock=clk.now, sleep=clk.sleep)
    link.walker = w
    if not w.start():
        print("시작 실패")
        return False

    # 점검 지표
    prev = [row[:] for row in w.hx.target]
    max_jump = 0.0
    min_margin = 1e9
    min_down = NUM_LEGS
    max_tilt_err = None
    pose = [0.0, 0.0, 0.0]                  # 바닥 기준 몸 위치 (x, y, yaw)
    hmax = {"일반(트라이포드)": 0.0, "안전(웨이브)": 0.0}
    t_end = 62.0
    t0 = clk.now()
    joy.t0 = t0
    worst = None
    while clk.now() - t0 < t_end:
        t = clk.now() - t0
        link.terrain = [0.0, 0.0, 12.0, 0.0, -10.0, 0.0] if 28.0 < t < 36.0 else [0.0] * NUM_LEGS
        w.tick(w.dt)
        clk.sleep(w.dt)
        cur = w.hx.target
        jump, jl, jj = max((abs(cur[l][j] - prev[l][j]), l, j) for l in range(NUM_LEGS) for j in range(3))
        if jump > max_jump:
            max_jump = jump
            worst = (t, jl, jj, w.s.leg[jl], round(w.s.u[jl], 2), w.gait.name, round(w.s.H))
        prev = [row[:] for row in cur]
        pose[2] += w.w * w.dt
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        pose[0] += (c * w.v[0] - s_ * w.v[1]) * w.dt
        pose[1] += (s_ * w.v[0] + c * w.v[1]) * w.dt
        down = [l for l in range(NUM_LEGS) if w.s.leg[l] == ST]
        hmax[w.gait.name] = max(hmax[w.gait.name], w.s.H)
        if w.gait.safe:
            min_down = min(min_down, len(down))
            if len(down) < NUM_LEGS:
                min_margin = min(min_margin, stab_margin([w.s.feet[l][:2] for l in down]))
            if t > 30:
                m = link.imu_recent(1)
                e = math.hypot(*m) - 0.3
                max_tilt_err = e if max_tilt_err is None else max(max_tilt_err, e)
    w.shutdown()
    print("\n── 시뮬레이션 결과 ──")
    print(f"  틱 {w.stats['ticks']} / 몸 이동·높이를 뺀 틱 {w.stats['fallback']} / 자세 유지 {w.stats['hold']} "
          f"/ 관절 속도 때문에 늦춘 틱 {w.stats.get('slow', 0)}")
    print(f"  한 틱 최대 관절 변화 {max_jump:.1f}° (50Hz) — 시각·다리·관절·상태·u·모드·높이 {worst}")
    print(f"  최고 높이: 일반 {hmax['일반(트라이포드)']:.0f}mm, 안전 {hmax['안전(웨이브)']:.0f}mm")
    print(f"  안전 모드: 바닥 다리 최소 {min_down}개, 받침 여유 최소 {min_margin:.1f}mm, "
          f"수평 오차(30초 뒤) 최대 {max_tilt_err if max_tilt_err is not None else float('nan'):.2f}°")
    print(f"  몸 이동 (바닥 기준): x {pose[0]:.0f}mm, y {pose[1]:.0f}mm, 회전 {math.degrees(pose[2]):.0f}°")
    print(f"  끝 높이 {w.s.H:.0f}mm, 모든 다리 바닥: {all(x == ST for x in w.s.leg)}")
    return True


# ───────────────────────── 실행 ─────────────────────────
def main():
    global DEFAULT_HEIGHT
    ap = argparse.ArgumentParser()
    ap.add_argument("--uno", help="조이스틱 우노 포트 (예: /dev/ttyACM0)")
    ap.add_argument("--robot", default=ROBOT, help="로봇 블루투스 주소 또는 포트")
    ap.add_argument("--safe", action="store_true", help="안전 모드로 시작")
    ap.add_argument("--no-imu", action="store_true", help="IMU 수평 끄기")
    ap.add_argument("--joytest", action="store_true", help="버튼 번호 확인 (로봇 없이)")
    ap.add_argument("--sim", action="store_true", help="로봇·조종기 없이 시뮬레이션 점검")
    ap.add_argument("--tune", action="store_true",
                    help="속도 튜닝 (--sim 과 같이 = 시뮬레이션으로 최적값 찾기, --uno 와 같이 = 실제 로봇에서 단계별로 재기)")
    ap.add_argument("--save", action="store_true", help="--tune --sim 결과를 바로 저장")
    ap.add_argument("--height", type=float, help=f"기본 몸 높이 [mm] (기본 {DEFAULT_HEIGHT:.0f}). 튜닝도 이 높이에서")
    ap.add_argument("--no-tuned", action="store_true", help=f"{TUNED_FILE} 무시하고 위 SPEED 그대로")
    a = ap.parse_args()

    if a.height:
        DEFAULT_HEIGHT = a.height
    if not a.no_tuned and not a.joytest:
        load_tuned()
    if a.sim and a.tune:
        tune_sim(DEFAULT_HEIGHT, a.save)
        return
    if a.sim:
        simulate()
        return
    if not a.uno:
        ap.error("--uno 가 필요함 (또는 --sim)")
    if a.joytest:
        joytest(a.uno)
        return

    joy = Joystick(a.uno)
    link = Link(a.robot)
    if not link.connect():
        print("\n".join(link.log_queue))
        return
    w = Walker(link, joy, start_safe=a.safe, use_imu=IMU_LEVEL and not a.no_imu)
    tuner = None
    try:
        if not w.start():
            print("\n".join(link.log_queue))
            return
        if a.tune:
            tuner = RobotTuner(w)
            print("튜닝: 조이스틱을 끝까지 밀고 걷기 (끝까지 민 동안만 잼) / Left·Right 만 = 회전 재기 / "
                  "노랑·파랑 = 속도 단계 ↑↓ / 버튼", BTN_SAFE, "= 안전 모드 / Ctrl+C = 끝내고 결과")
            print("★ 처음엔 가장 느린 단계. 넘어질 것 같으면 바로 조이스틱을 놓을 것")
        else:
            print("조작: 조이스틱 = 걷기 / Left·Right = 회전 / 노랑·파랑 = 높이 (누르고 있으면 계속) / "
                  "조이스틱 누름 = 기본 높이 / 버튼", BTN_SAFE, "= 안전 모드 / Ctrl+C = 끝")
        while True:
            w.run_for(3600.0, on_tick=tuner.on_tick if tuner else None)
    except (KeyboardInterrupt, ConnectionError) as e:
        print("\n중단:", e or "Ctrl+C")
        if w.s is not None and w.hx.state_known:
            try:
                w.shutdown()
            except (KeyboardInterrupt, ConnectionError):
                pass
    w.hx.poweroff(link)
    time.sleep(0.2)
    link.close()
    if tuner is not None:
        tuner.report_and_save()


if __name__ == "__main__":
    main()
