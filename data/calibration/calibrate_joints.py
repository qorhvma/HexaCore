#!/usr/bin/env python3
"""
관절 캘리브레이션: 다리마다 정해진 자세를 손으로(버튼으로) 맞추고, 그때의 펌웨어 각도를 기록한다.

    python3 data/calibration/calibrate_joints.py --bt                      # 컴퓨터에서 블루투스 (이름으로 ESP32 를 찾음)
    python3 data/calibration/calibrate_joints.py --robot AA:BB:CC:DD:EE:FF  # 블루투스 주소를 직접
    python3 data/calibration/calibrate_joints.py --robot /dev/ttyAMA0      # 파이 화면에서 (robot_main.py 는 끄고)
    python3 data/calibration/calibrate_joints.py --fake                    # 로봇 없이 화면만

블루투스: ESP32 펌웨어가 'Hyperism-HEXAPODv1' 로 켜진다. 파이의 robot_main.py 는 끌 것 (UART 로 같이 명령하면 서로 덮어씀).
  파이썬에 블루투스 소켓이 없으면 (anaconda 등) /usr/bin/python3 로 알아서 다시 실행한다.

★ 몸통을 받침에 올려 다리가 바닥·받침에 닿지 않게 하고 할 것. 처음 명령에 18개 서보가 한꺼번에 켜진다.

재는 자세 (실제 관절 각도. 180 = 쭉 편 상태)
    ① 쫙 편 자세        hip 180 (중립 방향), femur 180 (수평), tibia 180 (femur 와 일직선)
    ② tibia 90° 아래    femur 는 수평 그대로, tibia 만 수직 아래            → tibia 90
    ③ femur 90° 아래    tibia 는 편 채로, femur 를 수직 아래                → femur 270
    ④ hip 앞으로 90°    다리가 중립 방향에서 앞쪽으로 90°                  → hip 270
    ⑤ hip 뒤로 90°      다리가 중립 방향에서 뒤쪽으로 90°                  → hip 90
  ④⑤는 이웃 다리와 부딪히면 건너뛰어도 된다 (그러면 hip 은 pose_gui 기울기를 쓴다).

결과: data/calibration/joint_points.json (기록할 때마다 저장) → calibration.json 도 같이 갱신
  보행(software/raspberrypi/leg_model.py)은 측정점이 있는 관절은 측정점으로, 없는 관절은 예전 pose_gui 보정으로 바꾼다.
"""

import argparse
import datetime
import importlib.util
import json
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time

BT_NAME = "Hyperism-HEXAPODv1"          # firmware/firmware/firmware.ino 의 BT_NAME
SYSTEM_PYTHON = "/usr/bin/python3"
MAC_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


def _wants_bluetooth(argv):
    return "--bt" in argv or any(MAC_RE.match(a) for a in argv)


# 블루투스가 필요한데 이 파이썬에 AF_BLUETOOTH 가 없으면 (anaconda 등) 시스템 파이썬으로 다시 실행
if (_wants_bluetooth(sys.argv[1:]) and not hasattr(socket, "AF_BLUETOOTH") and os.path.exists(SYSTEM_PYTHON)
        and os.path.realpath(sys.executable) != os.path.realpath(SYSTEM_PYTHON)):
    print(f"이 파이썬({sys.executable})은 블루투스 소켓이 없어 {SYSTEM_PYTHON} 로 다시 실행", flush=True)
    os.execv(SYSTEM_PYTHON, [SYSTEM_PYTHON, os.path.abspath(__file__), *sys.argv[1:]])

import tkinter as tk  # noqa: E402
from tkinter import messagebox  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
POINTS_PATH = os.path.join(HERE, "joint_points.json")
sys.path.insert(0, os.path.join(ROOT, "software", "raspberrypi"))
import leg_model as lm  # noqa: E402

JOINTS = ("hip", "femur", "tibia")
STEPS = [
    {"key": "straight", "title": "① 쫙 편 자세", "short": "① 편 자세 h/f/t", "target": {"hip": 180, "femur": 180, "tibia": 180},
     "base": {}, "how": "hip: 다리가 중립 방향(CAD 기준, 몸에서 똑바로 바깥)으로\n"
                         "femur: 수평 (수평계·직각자로)\ntibia: femur 와 일직선으로 쭉"},
    {"key": "tibia90", "title": "② tibia 90° 아래", "short": "② tibia90", "target": {"tibia": 90},
     "base": {"hip": 180, "femur": 180}, "how": "femur 는 ① 의 수평 그대로 두고\ntibia 만 수직 아래로 (femur 와 직각)"},
    {"key": "femur90", "title": "③ femur 90° 아래", "short": "③ femur270", "target": {"femur": 270},
     "base": {"hip": 180, "tibia": 180}, "how": "tibia 는 femur 와 일직선(편 채)으로 두고\nfemur 를 수직 아래로"},
    {"key": "hip_fwd", "title": "④ hip 앞으로 90°", "short": "④ hip270", "target": {"hip": 270},
     "base": {"femur": 180, "tibia": 90}, "how": "다리가 ① 의 방향에서 앞쪽(라이다 쪽)으로 90°\n"
                                                "이웃 다리와 부딪히면 건너뛸 것"},
    {"key": "hip_back", "title": "⑤ hip 뒤로 90°", "short": "⑤ hip90", "target": {"hip": 90},
     "base": {"femur": 180, "tibia": 90}, "how": "다리가 ① 의 방향에서 뒤쪽으로 90°\n이웃 다리와 부딪히면 건너뛸 것"},
]
REST_TRUE = [180.0, 135.0, 45.0]   # 고르지 않은 다리는 서기 자세로 접어 둔다
STEPS_BTN = (-5, -1, -0.2, 0.2, 1, 5)
GLIDE_SEC = 0.6
SEND_HZ = 30


def load_points(path=POINTS_PATH):
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        legs = data.get("legs") or []
    except FileNotFoundError:
        legs = []
    legs = [dict((j, dict(v)) for j, v in (leg or {}).items()) for leg in legs]
    while len(legs) < lm.NUM_LEGS:
        legs.append({})
    for leg in legs:
        for j in JOINTS:
            leg.setdefault(j, {})
    return legs


def save_points(legs, robot, path=POINTS_PATH):
    data = {
        "_about": ("관절 캘리브레이션 측정점: legs[다리][관절][실제 각도] = 그 자세를 만든 펌웨어 각도. "
                   "data/calibration/calibrate_joints.py 가 쓴다. 실제 각도 180 = 쭉 편 상태, "
                   "tibia 90 = femur 와 직각 아래, femur 270 = 수직 아래, hip 270/90 = 앞/뒤로 90°."),
        "updated": datetime.datetime.now().isoformat(timespec="seconds"),
        "robot": robot,
        "legs": [{j: {k: round(float(v), 2) for k, v in sorted(leg[j].items(), key=lambda kv: float(kv[0]))}
                  for j in JOINTS} for leg in legs],
    }
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def update_table():
    """calibration.json 다시 쓰기 (check_calibration.py --update 와 같음). 실패하면 이유를 돌려준다."""
    try:
        spec = importlib.util.spec_from_file_location("_check_calibration", os.path.join(HERE, "check_calibration.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        with open(mod.TABLE, "w", encoding="utf-8") as f:
            json.dump(mod.build_table(), f, ensure_ascii=False, indent=2)
            f.write("\n")
        return None
    except Exception as e:  # noqa: BLE001
        return str(e)


class FakeLink:
    def __init__(self):
        self.sent = []

    def write(self, data):
        self.sent.append(data)

    def read(self):
        time.sleep(0.05)
        return b""

    def close(self):
        pass


def _bt_devices():
    """bluetoothctl 이 아는 기기 {이름: 주소}."""
    try:
        out = subprocess.run(["bluetoothctl", "devices"], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return {}
    found = {}
    for line in out.splitlines():
        m = re.match(r"Device\s+(\S+)\s+(.*)", line.strip())
        if m:
            found[m.group(2).strip()] = m.group(1)
    return found


def find_bt(name, scan_sec=12):
    """이름으로 블루투스 주소를 찾는다. 모르는 기기면 scan_sec 동안 검색한다. 못 찾으면 None."""
    mac = _bt_devices().get(name)
    if mac:
        return mac
    print(f"블루투스 '{name}' 검색 중 ({scan_sec}초)...")
    try:
        subprocess.run(["bluetoothctl", "--timeout", str(scan_sec), "scan", "on"],
                       capture_output=True, timeout=scan_sec + 10)
    except (OSError, subprocess.TimeoutExpired):
        pass
    return _bt_devices().get(name)


def open_link(target):
    spec = importlib.util.spec_from_file_location(
        "_pose_gui", os.path.join(ROOT, "legacy", "bluetooth_pc", "hexapod_pose_gui.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Link(target)


class App:
    def __init__(self, root, link, target):
        self.root, self.link, self.target = root, link, target
        self.points = load_points()
        try:
            lm.set_joint_points(self.points)
        except ValueError as e:
            messagebox.showwarning("측정점 확인", f"저장된 측정점에 문제가 있어 무시함:\n{e}")
            lm.set_joint_points(None)
        self.raw = None                     # 지금 보낸 펌웨어 각도 6×3 (None = 아직 안 보냄)
        self.glide = None                   # (시작, 목표, 시작 시각) — 부드럽게 옮기는 중
        self.step = tk.IntVar(value=0)
        self.leg = tk.IntVar(value=0)
        self.rx = queue.Queue()
        self.build()
        threading.Thread(target=self._reader, daemon=True).start()
        self.root.after(int(1000 / SEND_HZ), self._tick)
        self.refresh()

    # ───────── 화면 ─────────
    def build(self):
        r = self.root
        r.title("관절 캘리브레이션")
        top = tk.Frame(r)
        top.pack(fill="x", padx=8, pady=6)
        self.status = tk.Label(top, text=f"{self.target}", fg="green")
        self.status.pack(side="left")
        tk.Button(top, text="전부 힘 빼기 (o)", fg="red", command=self.relax).pack(side="right")
        tk.Button(top, text="확인: 서기", command=lambda: self.check_pose(REST_TRUE)).pack(side="right", padx=4)
        tk.Button(top, text="확인: 쫙 편 자세", command=lambda: self.check_pose([180, 180, 180])).pack(side="right")

        sf = tk.LabelFrame(r, text="자세")
        sf.pack(fill="x", padx=8)
        for i, st in enumerate(STEPS):
            tk.Radiobutton(sf, text=st["title"], variable=self.step, value=i, indicatoron=False, width=16,
                           command=self.refresh).grid(row=0, column=i, padx=2, pady=4)
        self.how = tk.Label(sf, justify="left", font=("TkDefaultFont", 11))
        self.how.grid(row=1, column=0, columnspan=len(STEPS), sticky="w", padx=6, pady=4)

        lf = tk.LabelFrame(r, text="다리 (펌웨어 번호)")
        lf.pack(fill="x", padx=8, pady=4)
        self.leg_btns = []
        for l in range(lm.NUM_LEGS):
            b = tk.Radiobutton(lf, text="", variable=self.leg, value=l, indicatoron=False, width=14,
                               command=self.refresh)
            b.grid(row=0, column=l, padx=2, pady=4)
            self.leg_btns.append(b)

        jf = tk.LabelFrame(r, text="관절 (펌웨어로 보내는 각도. 보정 없이 그대로)")
        jf.pack(fill="x", padx=8, pady=4)
        self.val_labels, self.rows = [], []
        for j, name in enumerate(JOINTS):
            row = tk.Frame(jf)
            row.pack(fill="x", pady=2)
            tag = tk.Label(row, text=name, width=7, anchor="w", font=("TkDefaultFont", 11, "bold"))
            tag.pack(side="left")
            for d in STEPS_BTN:
                tk.Button(row, text=f"{d:+g}", width=4, command=lambda j=j, d=d: self.nudge(j, d)).pack(side="left")
            lab = tk.Label(row, text="--", width=36, anchor="w", font=("TkFixedFont", 12))
            lab.pack(side="left", padx=8)
            ent = tk.Entry(row, width=7)
            ent.pack(side="left")
            tk.Button(row, text="보내기", command=lambda j=j, e=ent: self.set_from_entry(j, e)).pack(side="left", padx=2)
            self.val_labels.append(lab)
            self.rows.append(tag)

        bf = tk.Frame(r)
        bf.pack(fill="x", padx=8, pady=6)
        tk.Button(bf, text="이 다리·자세 기록 (Enter)", font=("TkDefaultFont", 11, "bold"), bg="#d8f0d8",
                  command=self.record).pack(side="left")
        tk.Button(bf, text="이 다리·자세 기록 지우기", command=self.clear).pack(side="left", padx=6)
        tk.Button(bf, text="추정값으로 되돌리기", command=self.refresh).pack(side="left")
        self.msg = tk.Label(bf, text="", fg="#555")
        self.msg.pack(side="left", padx=10)

        tf = tk.LabelFrame(r, text="기록 (펌웨어 각도)")
        tf.pack(fill="both", expand=True, padx=8, pady=4)
        self.table = tk.Text(tf, height=9, width=100, font=("TkFixedFont", 10), state="disabled")
        self.table.pack(fill="both", expand=True)
        r.bind("<Return>", lambda e: self.record())

    def refresh(self):
        """고른 자세·다리로 옮긴다 (기록이 있으면 그 값, 없으면 지금 보정으로 추정한 값)."""
        st, l = STEPS[self.step.get()], self.leg.get()
        self.how.config(text=st["how"])
        for j, name in enumerate(JOINTS):
            meas = name in st["target"]
            self.rows[j].config(fg="#c00000" if meas else "#888")
        target = self._pose_for(st, l)
        self.move_to(target)
        self._redraw()

    def _pose_for(self, st, l):
        pose = [row[:] for row in (self.raw or self._rest_pose())]
        for ll in range(lm.NUM_LEGS):
            if ll != l:
                pose[ll] = [lm.to_firmware(ll, j, REST_TRUE[j]) for j in range(3)]
        for j, name in enumerate(JOINTS):
            if name in st["target"]:
                true = st["target"][name]
                rec = self.points[l][name].get(str(true))
                pose[l][j] = float(rec) if rec is not None else lm.to_firmware(l, j, true)
            else:
                pose[l][j] = lm.to_firmware(l, j, st["base"].get(name, REST_TRUE[j]))
        return pose

    def _rest_pose(self):
        return [[lm.to_firmware(l, j, REST_TRUE[j]) for j in range(3)] for l in range(lm.NUM_LEGS)]

    def _redraw(self):
        st, l = STEPS[self.step.get()], self.leg.get()
        cur = self.glide[1] if self.glide else self.raw
        for j, name in enumerate(JOINTS):
            v = cur[l][j] if cur else None
            true = st["target"].get(name, st["base"].get(name))
            tick = lm.fw_ticks(l, j, v) if v is not None else None
            txt = "--" if v is None else f"{v:7.2f}°  틱 {tick:5.1f}"
            if true is not None:
                txt += f"  → 실제 {true}°"
            self.val_labels[j].config(text=txt)
        for ll, b in enumerate(self.leg_btns):
            done = all(str(st["target"][n]) in self.points[ll][n] for n in st["target"])
            b.config(text=f"{'✓ ' if done else ''}{ll} {lm.LEG_NAMES[ll]}")
        widths = [22] + [12] * (len(STEPS) - 1)
        lines = ["다리       " + "".join(f"{s['short']:<{w}}" for s, w in zip(STEPS, widths))]
        for ll in range(lm.NUM_LEGS):
            cells = []
            for s in STEPS:
                vals = [self.points[ll][n].get(str(t)) for n, t in s["target"].items()]
                cells.append("/".join("  -- " if v is None else f"{float(v):5.1f}" for v in vals))
            lines.append(f"{ll} {lm.LEG_NAMES[ll]:<6}  " + "".join(f"{c:<{w}}" for c, w in zip(cells, widths)))
        lines.append("")
        lines.append("지금 쓰는 보정: " + "  ".join(
            f"{ll}:" + ",".join(lm.calib_source(ll, j).replace("측정 ", "") for j in range(3)) for ll in range(6)))
        self.table.config(state="normal")
        self.table.delete("1.0", "end")
        self.table.insert("end", "\n".join(lines))
        self.table.config(state="disabled")

    # ───────── 움직이기 ─────────
    def _clamp_raw(self, l, j, v):
        """펌웨어 틱 한계 안으로 (밖이면 펌웨어가 잘라 버려서 화면 값과 실제가 달라진다)."""
        lo, hi = sorted((self._raw_for_tick(l, j, lm.TICK_HARD_MIN), self._raw_for_tick(l, j, lm.TICK_HARD_MAX)))
        return min(max(v, lo), hi)

    @staticmethod
    def _raw_for_tick(l, j, tick):
        t0, t1 = lm.fw_ticks(l, j, 0.0), lm.fw_ticks(l, j, 1.0)
        return (tick - t0) / (t1 - t0)

    def move_to(self, target):
        target = [[self._clamp_raw(l, j, target[l][j]) for j in range(3)] for l in range(lm.NUM_LEGS)]
        if self.raw is None:
            self._send(target, everything=True)         # 첫 명령: 18개 전부 (펌웨어가 이때 전부 켠다)
            return
        self.glide = ([row[:] for row in self.raw], target, time.monotonic())

    def nudge(self, j, d):
        self._set_joint(j, (self.glide[1] if self.glide else self.raw)[self.leg.get()][j] + d)

    def set_from_entry(self, j, ent):
        try:
            self._set_joint(j, float(ent.get()))
        except ValueError:
            self.msg.config(text="숫자를 넣을 것")

    def _set_joint(self, j, v):
        l = self.leg.get()
        target = [row[:] for row in (self.glide[1] if self.glide else self.raw)]
        target[l][j] = self._clamp_raw(l, j, v)
        self.glide = None
        self._send(target)
        self._redraw()

    def check_pose(self, true):
        """지금 보정(기록 반영)으로 모든 다리를 실제 각도 true 로."""
        self.move_to([[lm.to_firmware(l, j, true[j]) for j in range(3)] for l in range(lm.NUM_LEGS)])
        self.msg.config(text=f"확인: 모든 다리 실제 {true} — 6개가 같은 모양이어야 함")

    def relax(self):
        self.glide = None
        self.link.write(b"o\n")
        self.raw = None
        self.msg.config(text="힘 뺌. 다음 명령에 18개가 다시 켜짐")

    def _send(self, pose, everything=False):
        out = []
        for l in range(lm.NUM_LEGS):
            for j in range(3):
                if everything or self.raw is None or abs(pose[l][j] - self.raw[l][j]) > 1e-3:
                    out.append(f"<{l},{j},{pose[l][j]:.2f}>\n")
        if out:
            try:
                self.link.write("".join(out).encode())
            except Exception as e:  # noqa: BLE001
                self.status.config(text=f"전송 실패: {e}", fg="red")
        self.raw = [row[:] for row in pose]

    def _tick(self):
        if self.glide:
            start, target, t0 = self.glide
            u = min(1.0, (time.monotonic() - t0) / GLIDE_SEC)
            k = u * u * (3 - 2 * u)
            self._send([[start[l][j] + (target[l][j] - start[l][j]) * k for j in range(3)] for l in range(6)])
            if u >= 1.0:
                self.glide = None
                self._redraw()
        while not self.rx.empty():
            line = self.rx.get()
            if not line.startswith(("#ft", "#hb", "#imu")):
                self.status.config(text=f"{self.target}  |  로봇: {line[:80]}", fg="green")
        self.root.after(int(1000 / SEND_HZ), self._tick)

    def _reader(self):
        buf = b""
        while True:
            try:
                buf += self.link.read()
            except Exception as e:  # noqa: BLE001
                self.rx.put(f"끊김: {e}")
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                t = line.decode(errors="replace").strip()
                if t:
                    self.rx.put(t)

    # ───────── 기록 ─────────
    def record(self):
        st, l = STEPS[self.step.get()], self.leg.get()
        cur = self.glide[1] if self.glide else self.raw
        if cur is None:
            self.msg.config(text="아직 보낸 자세가 없음")
            return
        new = {n: dict(v) for n, v in self.points[l].items()}
        for j, name in enumerate(JOINTS):
            if name in st["target"]:
                new[name][str(st["target"][name])] = round(cur[l][j], 2)
        trial = [dict(p) for p in self.points]
        trial[l] = new
        try:
            lm.set_joint_points(trial)
        except ValueError as e:
            lm.set_joint_points(self.points)
            messagebox.showerror("기록 안 함", f"이 값이면 측정점 순서가 뒤집힘 (방향을 반대로 맞췄는지 확인):\n{e}")
            return
        self.points = trial
        save_points(self.points, self.target)
        err = update_table()
        rel = os.path.relpath(POINTS_PATH, ROOT)
        self.msg.config(text=f"기록·저장: {rel}" + ("" if err is None else f"  (calibration.json 갱신 실패: {err})"))
        self._redraw()

    def clear(self):
        st, l = STEPS[self.step.get()], self.leg.get()
        for name, t in st["target"].items():
            self.points[l][name].pop(str(t), None)
        lm.set_joint_points(self.points)
        save_points(self.points, self.target)
        update_table()
        self.msg.config(text="지움")
        self.refresh()


def main():
    ap = argparse.ArgumentParser(description="관절 캘리브레이션 (쫙 편 자세 / tibia·femur 90° 아래 / hip 앞뒤 90°)")
    ap.add_argument("--robot", help="ESP32 포트(/dev/ttyAMA0, /dev/ttyUSB0, COM7) 또는 블루투스 주소")
    ap.add_argument("--bt", nargs="?", const=BT_NAME, metavar="이름",
                    help=f"블루투스 이름으로 ESP32 를 찾아 연결 (기본 {BT_NAME})")
    ap.add_argument("--fake", action="store_true", help="로봇 없이 화면만 (명령은 보내지 않음)")
    a = ap.parse_args()
    if a.bt:
        if not hasattr(socket, "AF_BLUETOOTH"):
            sys.exit(f"이 파이썬에는 블루투스 소켓이 없음 → {SYSTEM_PYTHON} {os.path.abspath(__file__)} --bt 로 실행")
        mac = find_bt(a.bt)
        if not mac:
            sys.exit(f"블루투스 '{a.bt}' 를 못 찾음. 확인할 것:\n"
                     "  - ESP32 전원이 켜져 있고 펌웨어(firmware/firmware)가 돌고 있나 (켜질 때 블루투스 이름을 띄운다)\n"
                     "  - 다른 기기(휴대폰·다른 PC)에 이미 블루투스로 연결돼 있지 않나 (연결 중이면 검색에 안 보임)\n"
                     "  - 가까이 있나. 다시 찾기: bluetoothctl --timeout 20 scan on && bluetoothctl devices")
        print(f"블루투스 '{a.bt}' = {mac}")
        a.robot = mac
    if not a.robot and not a.fake:
        ap.error("--bt, --robot 또는 --fake 가 필요함")
    root = tk.Tk()
    if a.fake:
        link, target = FakeLink(), "가짜 연결 (--fake)"
    else:
        if not messagebox.askokcancel("시작 전에", "몸통을 받침에 올려 다리가 아무 데도 닿지 않게 했나요?\n"
                                     "확인을 누르면 18개 서보가 서기 자세로 켜집니다."):
            return
        try:
            link = open_link(a.robot)
        except Exception as e:  # noqa: BLE001
            hint = ""
            if MAC_RE.match(a.robot):
                hint = (f"\n\n블루투스 연결이 안 되면 한 번 페어링:\n  bluetoothctl pair {a.robot}\n"
                        f"  bluetoothctl trust {a.robot}\n그리고 파이의 robot_main.py 를 끄고 다시 시도.")
            messagebox.showerror("연결 실패", f"{e}{hint}")
            return
        target = a.robot
    App(root, link, target)
    root.mainloop()
    link.close()


if __name__ == "__main__":
    main()
