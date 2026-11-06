"""
계단 오르기 - 발판 조합 브루트포스 + 사이클 탐색
2D 측면도 (앞/중간/뒤 다리쌍)
"""
import numpy as np
from itertools import product

# ===== 설정 =====
FEMUR, TIBIA = 7.7, 10.0
GRID = 0.5

MOUNT = [13.0, 0.0, -13.0]      # 앞, 중간, 뒤 coxa 오프셋
LEG_N = 3

# 서보 가동범위 (도) - 180도 제한 반영
FEMUR_MIN, FEMUR_MAX = -150, 0     # 스팬 90
KNEE_MIN,  KNEE_MAX  = -160,  0     # 스팬 150  (둘 다 180 이내)
STEP = 0.5

# 계단
RISE, TREAD = 6.0, 13.0
OFFSETS = [3.0, 5.0, 7.0, 9.0, 11.0]   # 디딤면 위 발판 후보
STEPS   = [-1, 0, 1, 2, 3]             # 고려할 단 번호

MARGIN = 1.0     # 안정성 여유 (cm)


def cell(x, y):
    return (int(round(x / GRID)), int(round(y / GRID)))


def make_W():
    W = set()
    for d1 in np.arange(FEMUR_MIN, FEMUR_MAX + STEP, STEP):
        a1 = np.radians(d1)
        kx, ky = FEMUR * np.cos(a1), FEMUR * np.sin(a1)
        for d2 in np.arange(KNEE_MIN, KNEE_MAX + STEP, STEP):
            a2 = a1 + np.radians(d2)
            W.add(cell(kx + TIBIA * np.cos(a2), ky + TIBIA * np.sin(a2)))
    return W


def foot_pos(step, off):
    """단 번호 + 오프셋 -> 실제 좌표"""
    return (step * TREAD + off, step * RISE)


def pos_body(state, W):
    """state = ((step,off), (step,off), (step,off))"""
    B = None
    for i, (s, o) in enumerate(state):
        fx, fy = foot_pos(s, o)
        c = cell(fx - MOUNT[i], fy)
        tmp = {(c[0] - wx, c[1] - wy) for (wx, wy) in W}
        B = tmp if B is None else (B & tmp)
        if not B:
            return set()
    return B


def stable(state, swing_leg, B):
    """swing_leg를 들었을 때, B 중에서 안정한 몸통 위치만"""
    sup = [foot_pos(s, o)[0] for i, (s, o) in enumerate(state)
           if i != swing_leg]
    lo, hi = min(sup) + MARGIN, max(sup) - MARGIN
    if lo > hi:
        return set()
    return {c for c in B if lo <= c[0] * GRID <= hi}


# ===== 1. 유효 노드 브루트포스 =====
def enumerate_nodes(W):
    nodes = {}
    cand = [(s, o) for s in STEPS for o in OFFSETS]
    total = 0
    for combo in product(cand, repeat=LEG_N):
        total += 1
        f, m, r = combo
        # 물리적 순서: 앞다리 >= 중간 >= 뒤
        if not (f[0] >= m[0] >= r[0]):
            continue
        if f[0] - r[0] > 3:          # 너무 벌어진 배치 제외
            continue
        B = pos_body(combo, W)
        if B:
            nodes[combo] = B
    return nodes, total


# ===== 2. 간선 =====
def edges_from(state, nodes):
    out = []
    for leg in range(LEG_N):
        for s in STEPS:
            for o in OFFSETS:
                if (s, o) == state[leg]:
                    continue
                nxt = list(state)
                nxt[leg] = (s, o)
                nxt = tuple(nxt)
                if nxt not in nodes:
                    continue
                door = nodes[state] & nodes[nxt]
                door = stable(state, leg, door)
                door = stable(nxt, leg, door)
                if door:
                    out.append((leg, nxt, door))
    return out


# ===== 3. 사이클 탐색 =====
def shift(state, k=1):
    return tuple((s + k, o) for (s, o) in state)


def find_cycle(start, nodes, max_depth=3):
    goal = shift(start, 1)
    stack = [(start, [])]
    while stack:
        cur, path = stack.pop()
        if len(path) > max_depth:
            continue
        if cur == goal and path:
            return path
        for leg, nxt, door in edges_from(cur, nodes):
            if any(p[0] == leg for p in path):    # 다리당 1회
                continue
            stack.append((nxt, path + [(leg, nxt, door)]))
    return None


if __name__ == "__main__":
    W = make_W()
    print(f"W 격자 : {len(W)}\n")

    nodes, total = enumerate_nodes(W)
    print(f"전체 조합      : {total}")
    print(f"유효 노드      : {len(nodes)}\n")

    # 시작 노드 후보들에서 사이클 탐색
    starts = [s for s in nodes if s[0][0] == 1 and s[1][0] == 0 and s[2][0] == -1]
    print(f"시작 후보(계단식 배치) : {len(starts)}\n")

    found = None
    for st in starts:
        cyc = find_cycle(st, nodes)
        if cyc:
            found = (st, cyc)
            break

    if found:
        st, cyc = found
        print("=== 사이클 발견 ===")
        names = ["앞", "중간", "뒤"]
        print(f"시작: {[(s,o) for s,o in st]}")
        cur = st
        for leg, nxt, door in cyc:
            dx = [c[0] * GRID for c in door]
            dy = [c[1] * GRID for c in door]
            print(f"  {names[leg]}다리 → 단{nxt[leg][0]} @{nxt[leg][1]}cm  |"
                  f"  문 크기 {len(door)}칸,"
                  f" x {min(dx):.1f}~{max(dx):.1f}, y {min(dy):.1f}~{max(dy):.1f}")
            cur = nxt
        print(f"도착: {[(s,o) for s,o in cur]}")
        print(f"시작+1단 == 도착 ? {cur == shift(st,1)}")
    else:
        print("사이클 없음")
