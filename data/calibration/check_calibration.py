#!/usr/bin/env python3
"""
캘리브레이션 표(calibration.json) ↔ 원본(펌웨어·SDK) 대조

    python3 data/calibration/check_calibration.py            # 표와 원본이 같은지 검사 (다르면 종료 코드 1)
    python3 data/calibration/check_calibration.py --update   # 원본에서 값을 읽어 표를 다시 씀 (재캘리브레이션 뒤)

원본
    firmware/firmware/firmware.ino          서보별 tCal·dir·채널, JOINT_OFS, 틱 한계, 변환 상수 (ESP32 에 올라가는 값)
    legacy/bluetooth_pc/hexapod_sdk.py      다리 길이, 관절 범위, 좌우 부호, 서기 자세
    legacy/bluetooth_pc/hexapod_pose_gui.py 실제 관절 각도 → 펌웨어 각도 보정 (가변저항 곡선, 일자 자세, 90° 이동량)
표에만 있는 값 (원본 없음): 다리 장착 위치·방향 (최종 CAD URDF), femur 축 높이
"""

import argparse
import importlib.util
import json
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
TABLE = os.path.join(ROOT, "data", "calibration", "calibration.json")
FIRMWARE = os.path.join(ROOT, "firmware", "firmware", "firmware.ino")
SDK = os.path.join(ROOT, "legacy", "bluetooth_pc", "hexapod_sdk.py")
POSE_GUI = os.path.join(ROOT, "legacy", "bluetooth_pc", "hexapod_pose_gui.py")
JOINTS = ("hip", "femur", "tibia")

# 원본이 없는 값 (최종 CAD URDF, 2026-11-07). 펌웨어 다리 번호 순서 (0 왼앞 … 5 오른앞)
MOUNTS = [
    {"name": "왼앞", "urdf": "L3", "x": 90.9, "y": 54.7, "heading_deg": 35.0},
    {"name": "왼가운데", "urdf": "L2", "x": 0.0, "y": 99.0, "heading_deg": 90.0},
    {"name": "왼뒤", "urdf": "L1", "x": -90.9, "y": 54.8, "heading_deg": 145.2},
    {"name": "오른뒤", "urdf": "L6", "x": -90.9, "y": -54.7, "heading_deg": -144.0},
    {"name": "오른가운데", "urdf": "L5", "x": 0.0, "y": -99.0, "heading_deg": -90.0},
    {"name": "오른앞", "urdf": "L4", "x": 90.9, "y": -54.7, "heading_deg": -34.8},
]


def _strip_comments(src):
    return re.sub(r"//[^\n]*", "", src)


def read_firmware(path=FIRMWARE):
    src = open(path, encoding="utf-8").read()
    code = _strip_comments(src)

    def num(name):
        m = re.search(rf"\b{name}\s*=\s*([-0-9.]+)f?\s*;", code)
        if not m:
            raise ValueError(f"firmware.ino 에서 {name} 를 못 찾음")
        return float(m.group(1))

    def arr3(name):
        m = re.search(rf"\b{name}\s*\[3\]\s*=\s*\{{([^}}]*)\}}", code)
        if not m:
            raise ValueError(f"firmware.ino 에서 {name}[3] 을 못 찾음")
        return [float(v) for v in m.group(1).split(",")]

    m = re.search(r"TICKS_PER_DEG\s*=\s*([0-9.]+)f?\s*/\s*([0-9.]+)f?", code)
    tpd = float(m.group(1)) / float(m.group(2))

    sv_block = re.search(r"Sv\s+SV\[6\]\[3\]\s*=\s*\{(.*?)\};", code, re.S).group(1)
    entries = re.findall(r"\{\s*&(pwmU\d)\s*,\s*(\d+)\s*,\s*\"([^\"]+)\"\s*,\s*(\d+)\s*,\s*([+-]?\d+)\s*\}", sv_block)
    if len(entries) != 18:
        raise ValueError(f"SV 표에서 서보 {len(entries)}개를 찾음 (18개여야 함)")
    drivers = dict(re.findall(r"Adafruit_PWMServoDriver\s+(pwmU\d)\((0x[0-9A-Fa-f]+)\)", code))

    ofs_block = re.search(r"JOINT_OFS\[6\]\[3\]\s*=\s*\{(.*?)\};", code, re.S).group(1)
    ofs_rows = re.findall(r"\{\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*\}", ofs_block)
    if len(ofs_rows) != 6:
        raise ValueError(f"JOINT_OFS 에서 {len(ofs_rows)}줄을 찾음 (6줄이어야 함)")

    servos = []
    for i, (drv, ch, name, tcal, d) in enumerate(entries):
        leg, j = divmod(i, 3)
        servos.append({"leg": leg, "joint": JOINTS[j], "driver": drivers.get(drv, drv), "channel": int(ch),
                       "name": name, "t_cal": int(tcal), "dir": int(d), "joint_ofs_deg": float(ofs_rows[leg][j])})
    return {
        "tick_hard_min": int(num("TICK_HARD_MIN")),
        "tick_hard_max": int(num("TICK_HARD_MAX")),
        "ref_ang_deg": arr3("REF_ANG"),
        "cal_ang_deg": arr3("CAL_ANG"),
        "angle_gain": num("ANGLE_GAIN"),
        "ticks_per_deg": tpd,
        "tick_90_nominal": int(num("TICK_90_NOMINAL")),
        "servo_freq_hz": int(re.search(r"#define\s+SERVO_FREQ\s+(\d+)", src).group(1)),
    }, servos


def read_sdk(path=SDK):
    spec = importlib.util.spec_from_file_location("_hexapod_sdk_check", path)
    sdk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sdk)
    lims = [[list(map(float, sdk.JOINT_LIMITS[l][j])) for j in range(3)] for l in range(6)]
    return {
        "coxa_len_mm": float(sdk.COXA_LEN),
        "femur_len_mm": float(sdk.FEMUR_LEN),
        "tibia_len_mm": float(sdk.TIBIA_LEN),
        "hip_sign": [int(v) for v in sdk.SIDE],
        "stand_pose_deg": [list(map(float, p)) for p in sdk.STAND_POSE],
        "joint_limits_deg": lims,
    }


def _literals(path, names):
    """파이썬 파일의 맨 위 '이름 = 리터럴' 대입을 실행하지 않고 읽는다 (tkinter 등 import 없이)."""
    import ast
    tree = ast.parse(open(path, encoding="utf-8").read())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            n = node.targets[0].id
            if n in names:
                try:
                    out[n] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    missing = set(names) - set(out)
    if missing:
        raise ValueError(f"{os.path.basename(path)} 에서 못 찾음: {sorted(missing)}")
    return out


def read_pose_gui(path=POSE_GUI):
    v = _literals(path, ["CALIB_TICK", "CALIB_POT", "CALIB_STRAIGHT", "EXTENDED_ANG", "EXTENDED_RAW",
                         "SLIDER_PER_90", "FW_TCAL", "FW_DIR", "FW_OFS", "ANGLE_GAIN", "CAL_ANG", "REF_ANG"])
    if len(v["CALIB_POT"]) != 18:
        raise ValueError("CALIB_POT 는 모터 18개여야 함")
    return {
        "_about": ("hexapod_pose_gui.calibrated_joint 와 같은 변환. 실제 관절 각도(180 = 다리를 수평 일자로 편 자세) → "
                   "펌웨어로 보낼 각도. 모터 번호 = 다리×3 + 채널 (채널 0 tibia, 1 femur, 2 hip)."),
        "calib_tick": [int(t) for t in v["CALIB_TICK"]],
        "calib_pot": [[int(p) for p in row] for row in v["CALIB_POT"]],
        "calib_straight_motors": sorted(int(m) for m in v["CALIB_STRAIGHT"]),   # 모터 번호 (1부터)
        "extended_ang_deg": [float(a) for a in v["EXTENDED_ANG"]],
        "extended_raw_deg": [[float(a) for a in row] for row in v["EXTENDED_RAW"]],
        "slider_per_90": [[float(a) for a in row] for row in v["SLIDER_PER_90"]],
    }, v


def pose_gui_firmware_mismatch(fw, servos, v):
    """hexapod_pose_gui 안에 복사된 펌웨어 값이 firmware.ino 와 같은지."""
    out = []
    for s in servos:
        l, j = s["leg"], JOINTS.index(s["joint"])
        for name, mine in (("FW_TCAL", s["t_cal"]), ("FW_DIR", s["dir"]), ("FW_OFS", s["joint_ofs_deg"])):
            if abs(v[name][l][j] - mine) > 1e-6:
                out.append(f"hexapod_pose_gui.{name}[{l}][{j}] = {v[name][l][j]} ≠ firmware.ino {mine}")
    if abs(v["ANGLE_GAIN"] - fw["angle_gain"]) > 1e-6 or list(map(float, v["CAL_ANG"])) != fw["cal_ang_deg"] \
            or list(map(float, v["REF_ANG"])) != fw["ref_ang_deg"]:
        out.append("hexapod_pose_gui 의 ANGLE_GAIN/CAL_ANG/REF_ANG 이 firmware.ino 와 다름")
    return out


def build_table():
    fw, servos = read_firmware()
    kin = read_sdk()
    pose, _ = read_pose_gui()
    kin.update({"coxa_dz_mm": -11.4, "hip_z_mm": 4.0, "mounts": MOUNTS})
    return {
        "_about": ("헥사포드 캘리브레이션 표. 로봇 소프트웨어(software/raspberrypi/leg_model.py)가 읽는다. "
                   "firmware 와 servos 는 firmware/firmware/firmware.ino, kinematics 의 길이·범위·부호·서기 자세는 "
                   "legacy/bluetooth_pc/hexapod_sdk.py 에서 check_calibration.py --update 로 가져온다. "
                   "kinematics.mounts / coxa_dz_mm / hip_z_mm 는 최종 CAD URDF(2026-11-07) 값. "
                   "pose_calibration 은 legacy/bluetooth_pc/hexapod_pose_gui.py 의 관절 보정 (실제 각도 → 펌웨어 각도)."),
        "units": {"angle": "펌웨어 관절 각도 [도] (180 = 다리를 쭉 편 상태)", "length": "mm", "tick": "PCA9685 12비트 틱"},
        "firmware": fw,
        "servos": servos,
        "kinematics": kin,
        "pose_calibration": pose,
    }


def _diff(a, b, path=""):
    out = []
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k.startswith("_"):
                continue
            if k not in a or k not in b:
                out.append(f"{path}{k}: {'표에 없음' if k not in a else '원본에 없음'}")
            else:
                out += _diff(a[k], b[k], f"{path}{k}.")
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append(f"{path[:-1]}: 길이 {len(a)} ≠ {len(b)}")
        else:
            for i, (x, y) in enumerate(zip(a, b)):
                out += _diff(x, y, f"{path}{i}.")
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if abs(a - b) > 1e-6:
            out.append(f"{path[:-1]}: 표 {a} ≠ 원본 {b}")
    elif a != b:
        out.append(f"{path[:-1]}: 표 {a!r} ≠ 원본 {b!r}")
    return out


def main():
    ap = argparse.ArgumentParser(description="캘리브레이션 표 ↔ 펌웨어·SDK 대조")
    ap.add_argument("--update", action="store_true", help="원본에서 읽어 calibration.json 을 다시 씀")
    a = ap.parse_args()
    fresh = build_table()
    fw, servos = read_firmware()
    gui_bad = pose_gui_firmware_mismatch(fw, servos, read_pose_gui()[1])
    for d in gui_bad:
        print("  ✗", d)
    if a.update:
        with open(TABLE, "w", encoding="utf-8") as f:
            json.dump(fresh, f, ensure_ascii=False, indent=2)
            f.write("\n")
        print(f"썼음: {os.path.relpath(TABLE, ROOT)}")
        return 0
    table = json.load(open(TABLE, encoding="utf-8"))
    diffs = _diff(table, fresh)
    if diffs:
        print("표와 원본이 다름 (원본을 고쳤으면 --update, 표를 고쳤으면 원본도 같이 고칠 것):")
        for d in diffs:
            print("  ", d)
        return 1
    print("calibration.json = firmware.ino + hexapod_sdk.py + hexapod_pose_gui.py (일치)")
    return 1 if gui_bad else 0


if __name__ == "__main__":
    sys.exit(main())
