"""
6족 로봇 - 다리 작업공간(W)과 몸통 가능위치(B) 계산
2D 측면도 (x=앞뒤, y=위아래)
"""
import numpy as np
import matplotlib.pyplot as plt

# ===== 1. 설정 =====
FEMUR = 7.7
TIBIA = 10.0
GRID  = 0.5          # 격자 크기(cm). 부동소수점 문제 해결용

# 몸통 중심에서 각 다리 coxa까지의 x 오프셋
MOUNT = [13.0, 0.0, -13.0]     # 앞, 중간, 뒤
LEG_N = len(MOUNT)

# 서보 가동범위 (도)
FEMUR_MIN, FEMUR_MAX = -150, -30    # femur 절대각 (아래쪽 향함)
KNEE_MIN,  KNEE_MAX  = -140, 0      # 무릎 상대각 (한 방향만)
STEP = 0.5


def cell(x, y):
    """실제 좌표(cm) -> 격자 인덱스"""
    return (int(round(x / GRID)), int(round(y / GRID)))


def to_cm(c):
    """격자 인덱스 -> 실제 좌표(cm)"""
    return (c[0] * GRID, c[1] * GRID)


# ===== 2. 작업공간 W 만들기 =====
def make_workspace():
    """
    coxa 관절을 원점으로 했을 때, 발이 갈 수 있는 모든 위치.
    반환: 격자 인덱스 집합, 그리고 {격자: (femur각, 무릎각)} 조회표
    """
    W = set()
    lookup = {}

    a1_list = np.arange(FEMUR_MIN, FEMUR_MAX + STEP, STEP)
    a2_list = np.arange(KNEE_MIN, KNEE_MAX + STEP, STEP)

    for d1 in a1_list:
        a1 = np.radians(d1)
        # femur 끝(무릎) 위치
        kx = FEMUR * np.cos(a1)
        ky = FEMUR * np.sin(a1)

        for d2 in a2_list:
            a2 = a1 + np.radians(d2)       # 각도 누적
            px = kx + TIBIA * np.cos(a2)
            py = ky + TIBIA * np.sin(a2)

            c = cell(px, py)
            W.add(c)
            lookup.setdefault(c, (d1, d2))

    return W, lookup


# ===== 3. 몸통 가능위치 B 계산 =====
def pos_body(footholds, W):
    """
    footholds: [(x,y), ...] 다리별 발 위치 (몸통 좌표계와 같은 절대 좌표)
    반환: 몸통 중심이 있을 수 있는 격자 위치 집합
    """
    B = None
    for i, foot in enumerate(footholds):
        # 이 다리의 coxa 기준 목표점을 격자로
        fx, fy = cell(foot[0] - MOUNT[i], foot[1])

        # b = f - w  (역작업공간)
        tmp = {(fx - wx, fy - wy) for (wx, wy) in W}

        B = tmp if B is None else (B & tmp)

        if not B:                 # 중간에 비면 즉시 종료
            return set()
    return B


# ===== 4. 실행 =====
if __name__ == "__main__":
    W, lookup = make_workspace()

    # --- W 검증 ---
    pts = np.array([to_cm(c) for c in W])
    r = np.hypot(pts[:, 0], pts[:, 1])
    print(f"W 격자 개수 : {len(W)}")
    print(f"최소 반지름  : {r.min():.2f}")
    print(f"최대 반지름  : {r.max():.2f}  (이론값 {FEMUR+TIBIA})")

    # --- 발 배치 예시 (평지, 몸통 높이 12cm 목표) ---
    H = -12.0
    footholds = [(13.0, H), (0.0, H), (-13.0, H)]

    B = pos_body(footholds, W)
    print(f"\n몸통 가능위치 개수 : {len(B)}")

    if B:
        bpts = np.array([to_cm(c) for c in B])
        print(f"  x 범위 : {bpts[:,0].min():.1f} ~ {bpts[:,0].max():.1f}")
        print(f"  y 범위 : {bpts[:,1].min():.1f} ~ {bpts[:,1].max():.1f}")

    # --- 그림 ---
    fig, ax = plt.subplots(1, 2, figsize=(13, 6))

    ax[0].scatter(pts[:, 0], pts[:, 1], s=1, c='red')
    ax[0].plot(0, 0, 'ko', ms=8)
    ax[0].set_title("W : leg workspace (coxa at origin)")
    ax[0].set_aspect('equal'); ax[0].grid(alpha=.3)

    if B:
        ax[1].scatter(bpts[:, 0], bpts[:, 1], s=3, c='blue',
                      label='body positions')
    fx = [f[0] for f in footholds]
    fy = [f[1] for f in footholds]
    ax[1].scatter(fx, fy, s=80, c='red', marker='x', label='foot')
    ax[1].set_title("B : body positions")
    ax[1].set_aspect('equal'); ax[1].grid(alpha=.3); ax[1].legend()

    plt.tight_layout()
    plt.savefig('result.png', dpi=100)
    print("\n그림 저장 완료")

    # ===== 5. 계단 테스트 =====
    print("\n--- 계단 (단높이 6, 단너비 13) ---")
    stair = [(13.0, -12.0 + 6), (0.0, -12.0), (-13.0, -12.0 - 6)]
    Bs = pos_body(stair, W)
    print(f"몸통 가능위치 개수 : {len(Bs)}")
    if Bs:
        sp = np.array([to_cm(c) for c in Bs])
        print(f"  x 범위 : {sp[:,0].min():.1f} ~ {sp[:,0].max():.1f}")
        print(f"  y 범위 : {sp[:,1].min():.1f} ~ {sp[:,1].max():.1f}")
