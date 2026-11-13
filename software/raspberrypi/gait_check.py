"""
IK 보행 점검

    python3 gait_check.py                          # 로봇 없이: 범위표 + 여러 명령으로 시뮬레이션해서 안전 조건 확인
    python3 gait_check.py --identify --port /dev/ttyUSB0
        # 실물: 다리 번호·방향 가정 확인. ★ 몸통을 받침에 올려 다리가 바닥에 안 닿게 하고 할 것
        #   다리를 하나씩 '앞으로 돌리기 → 들기' 한다. 화면에 적힌 위치의 다리가, 적힌 방향으로 움직여야 한다.
        #   다르면 leg_model.LAYOUT (위치·순서) / hip 부호를 고친다.

로봇 없이 확인하는 것 (하나라도 어기면 종료 코드 1)
    - 모든 관절(실제 각도)이 허용 범위 안, 펌웨어로 보내는 각도(관절 보정 적용)의 틱이 펌웨어 한계 안
    - 한 틱(20ms)에 관절이 JOINT_SPEED_MAX 이상 안 돎
    - 바닥에 있는 다리는 바닥 높이 그대로 (미끄러지지 않게 발끝이 몸 기준으로만 움직임)
    - 무게중심(몸 중심)이 바닥 다리들이 만드는 다각형 안 (정적 안정 여유 [mm])
    - 명령을 0 으로 하면 몇 초 안에 모든 발이 제자리(SETTLE_TOL 안)로 돌아오고 시계가 멈춤
"""

import argparse
import math
import random
import sys
import time

import ik_gait as g
import leg_model as lm

DT = 0.02


def _hull(pts):
    pts = sorted(set((round(p[0], 3), round(p[1], 3)) for p in pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def stab_margin(pts):
    """원점(몸 중심)이 다각형 안쪽으로 들어간 거리 [mm]. 밖이면 음수."""
    h = _hull(pts)
    if len(h) < 3:
        return -1e9
    m = 1e9
    for i in range(len(h)):
        a, b = h[i], h[(i + 1) % len(h)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey)
        if L > 1e-9:
            m = min(m, (ex * (0 - a[1]) - ey * (0 - a[0])) / L)
    return m


def run(gait, cmds, label):
    """cmds = [(초, vx, vy, wz), ...]. 위반 목록과 요약을 돌려준다."""
    errs, worst_margin, max_jump = [], 1e9, 0.0
    prev = [row[:] for row in gait.pose_true]
    dist = [0.0, 0.0]
    yaw = 0.0
    t = 0.0
    for secs, vx, vy, wz in cmds:
        for _ in range(int(round(secs / DT))):
            fw_pose = gait.update(vx, vy, wz, DT)      # 펌웨어로 보내는 값 (캘리브레이션 적용)
            pose = gait.pose_true                      # 실제 관절 각도
            t += DT
            # 몸 이동 적분 (몸 좌표 → 바닥 좌표)
            c, s = math.cos(yaw), math.sin(yaw)
            dist[0] += (c * gait.v[0] - s * gait.v[1]) * DT
            dist[1] += (s * gait.v[0] + c * gait.v[1]) * DT
            yaw += gait.w * DT
            for l in range(lm.NUM_LEGS):
                if not lm.in_limits(l, pose[l]):
                    errs.append(f"{t:.2f}s 다리 {l} 범위 밖 {[round(v, 1) for v in pose[l]]}")
                for i in range(3):
                    tick = lm.fw_ticks(l, i, fw_pose[l][i])
                    if not lm.TICK_HARD_MIN - 1e-6 <= tick <= lm.TICK_HARD_MAX + 1e-6:
                        errs.append(f"{t:.2f}s 다리 {l} 관절 {i} 틱 {tick:.0f} 이 한계 밖 (펌웨어가 잘라 버림)")
                for i in range(3):
                    jump = abs(pose[l][i] - prev[l][i])
                    max_jump = max(max_jump, jump / DT)
                    if jump > g.JOINT_SPEED_MAX * DT + 1e-6:
                        errs.append(f"{t:.2f}s 다리 {l} 관절 {i} 너무 빠름 {jump / DT:.0f}°/s")
                if gait.leg[l] == g.ST and abs(gait.feet[l][2] - gait.ground) > 0.5:
                    errs.append(f"{t:.2f}s 다리 {l} 바닥인데 높이 {gait.feet[l][2]:.1f}")
            # 관절 속도를 자른 틱이 아니면, 보낸 관절의 FK = 바닥 발 위치
            support = [lm.fk(l, pose[l])[:2] for l in range(lm.NUM_LEGS) if gait.leg[l] == g.ST]
            if len(support) < 3:
                errs.append(f"{t:.2f}s 바닥 다리 {len(support)}개")
            else:
                worst_margin = min(worst_margin, stab_margin(support))
            prev = pose
    return errs, {"label": label, "time": t, "dist": dist, "yaw": math.degrees(yaw),
                  "margin": worst_margin, "max_joint_speed": max_jump}


def _table_matches_sources():
    """표가 펌웨어·SDK 원본과 같은지 (data/calibration/check_calibration.py). 원본 파일이 없으면 건너뜀."""
    import importlib.util
    import os
    path = os.path.join(os.path.dirname(lm.TABLE_PATH), "check_calibration.py")
    if not os.path.exists(path):
        print("  (check_calibration.py 없음 → 원본 대조 건너뜀)")
        return True
    spec = importlib.util.spec_from_file_location("_check_calibration", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    try:
        diffs = mod._diff(lm.CAL, mod.build_table())
    except (OSError, ValueError) as e:
        print(f"  원본을 못 읽어 대조 건너뜀: {e}")
        return True
    if diffs:
        print("  ✗ 표가 원본(firmware.ino / hexapod_sdk.py)과 다름:")
        for d in diffs[:10]:
            print("     ", d)
        return False
    print("  표 = firmware.ino + hexapod_sdk.py + hexapod_pose_gui.py (일치)")
    return True


def offline():
    print(f"캘리브레이션 표: {lm.TABLE_PATH}")
    print(f"  다리 길이 coxa {lm.COXA_LEN} / femur {lm.FEMUR_LEN} / tibia {lm.TIBIA_LEN} mm, "
          f"hip 부호 {lm.HIP_SIGN}, 서기 {lm.STAND_POSE[0]}")
    table_ok = _table_matches_sources()
    print("관절 허용 범위 [실제 관절 도] = 표의 joint_limits ∩ 펌웨어 틱 %d~%d  |  서기 자세 → 펌웨어 각도 (관절 보정 적용)"
          % (lm.TICK_HARD_MIN, lm.TICK_HARD_MAX))
    for l in range(lm.NUM_LEGS):
        lims = "  ".join(f"{n} {a:5.1f}~{b:5.1f}" for n, (a, b) in zip(("hip", "femur", "tibia"), lm.JOINT_LIMITS[l]))
        fw = [round(lm.to_firmware(l, j, lm.STAND_POSE[l][j]), 1) for j in range(3)]
        ticks = [round(lm.tick_of(l, j, lm.STAND_POSE[l][j])) for j in range(3)]
        print(f"  다리 {l} {lm.LEG_NAMES[l]:<5} {lims}  |  {fw} 틱 {ticks}")
    gait = g.IkTripodGait()
    sp = gait.stand_pose[0]
    print(f"서기 자세 (실제) hip {sp[0]:.0f} femur {sp[1]:.0f} tibia {sp[2]:.0f} — {gait.stand_note}")
    print(f"서기 발 높이 {gait.ground:.1f}mm (몸통 원점 기준), 보폭 {gait.stride:.1f}mm, 직진 최고 {gait.max_speed():.3f} m/s")
    if not lm.USE_JOINT_POINTS:
        print("  (HEXAPOD_JOINT_POINTS=0 → 측정점 무시, pose_gui 보정만)")
    warn = gait.walk_warning()
    if warn:
        print("★ " + warn)
        print("결과: 실패 (걸을 수 없음)")
        return 1

    scen = [
        ("전진 최고", [(4, 0.12, 0, 0), (2, 0, 0, 0)]),
        ("후진", [(4, -0.12, 0, 0), (2, 0, 0, 0)]),
        ("옆 (왼쪽)", [(4, 0, 0.08, 0), (2, 0, 0, 0)]),
        ("제자리 회전", [(4, 0, 0, 0.6), (2, 0, 0, 0)]),
        ("걸으며 회전", [(4, 0.08, 0, 0.4), (2, 0, 0, 0)]),
        ("방향 급변", [(1, 0.12, 0, 0), (1, -0.12, 0, 0), (1, 0, 0.08, -0.6), (1, 0, -0.08, 0.6), (2, 0, 0, 0)]),
    ]
    rng = random.Random(0)
    rand = [(rng.uniform(0.1, 1.0), rng.uniform(-0.12, 0.12), rng.uniform(-0.08, 0.08), rng.uniform(-0.6, 0.6))
            for _ in range(80)] + [(3, 0, 0, 0)]
    scen.append(("무작위", rand))
    bad = False
    print(f"\n{'시나리오':<12} {'시간':>5} {'이동 x,y [mm]':>16} {'회전':>7} {'안정 여유':>9} {'관절 최고 속도':>12}")
    for label, cmds in scen:
        gait.reset()
        errs, s = run(gait, cmds, label)
        far = max(math.dist(gait.feet[l][:2], gait.home[l][:2]) for l in range(lm.NUM_LEGS))
        if gait.active or far > g.SETTLE_TOL:
            errs.append(f"멈춘 뒤 제자리로 안 돌아옴 (발이 최대 {far:.1f}mm 벗어남)")
        print(f"{label:<12} {s['time']:5.1f} {s['dist'][0]:7.0f},{s['dist'][1]:7.0f} {s['yaw']:6.0f}° "
              f"{s['margin']:8.1f}mm {s['max_joint_speed']:9.0f}°/s" + ("" if not errs else f"  ✗ {len(errs)}건"))
        for e in errs[:5]:
            print("    ", e)
        bad |= bool(errs)
    print("\n통계:", gait.stats)
    bad |= not table_ok
    print("결과:", "실패" if bad else "통과")
    return 1 if bad else 0


def identify(port):
    from firmware_link import FirmwareLink
    print("★ 몸통을 받침에 올려 다리가 바닥에 안 닿게 했는지 확인. Enter = 시작, Ctrl+C = 중단")
    input()
    fw = FirmwareLink(port)
    fw.connect()
    if fw.simulate:
        sys.exit(f"{port} 에 ESP32 가 없어 시뮬레이션으로 열림 → --port 를 확인할 것")
    stand = g.IkTripodGait().stand_pose          # 보행과 같은 서기 자세 (자동 선택 포함)
    pose = [row[:] for row in stand]

    def glide(target, secs=0.8):
        """실제 관절 각도로 부드럽게 옮긴다 (보낼 때 관절 보정 적용)."""
        start = [row[:] for row in pose]
        n = max(1, int(secs / DT))
        for k in range(1, n + 1):
            u = 0.5 - 0.5 * math.cos(math.pi * k / n)
            for l in range(6):
                for j in range(3):
                    pose[l][j] = start[l][j] + (target[l][j] - start[l][j]) * u
            fw.send_pose(lm.pose_to_firmware(pose))
            time.sleep(DT)

    try:
        fw.send_pose(lm.pose_to_firmware(pose))     # 처음엔 서기 자세로 바로
        time.sleep(1.5)
        for l in range(lm.NUM_LEGS):
            print(f"\n다리 {l} → '{lm.LEG_NAMES[l]}' 다리여야 함: 1) 앞쪽으로 돌고  2) 위로 들린다")
            t = [row[:] for row in stand]
            t[l][0] += 15.0                     # hip + = 앞쪽 (가정)
            glide(t)
            time.sleep(0.5)
            t[l][1] -= 25.0                     # femur - = 위로
            glide(t)
            time.sleep(0.8)
            glide(stand)
            ans = input("  맞으면 Enter, 다르면 무엇이 달랐는지 적고 Enter: ").strip()
            if ans:
                print(f"  기록: 다리 {l}: {ans}")
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        glide(stand, 0.8)
        fw.close()


def main():
    ap = argparse.ArgumentParser(description="IK 보행 점검")
    ap.add_argument("--identify", action="store_true", help="실물: 다리 번호·방향 확인 (다리를 띄우고)")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="ESP32 시리얼 포트")
    a = ap.parse_args()
    if a.identify:
        identify(a.port)
    else:
        sys.exit(offline())


if __name__ == "__main__":
    main()
