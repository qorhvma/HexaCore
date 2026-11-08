"""
헥사포드 관절 조종 프로그램 (hexapod_pose_fw.ino 용)
  18개 관절을 슬라이더로 움직이고, 자세를 저장/불러오기 한다.
  로봇으로 보내는 명령:  <다리,관절,각도>\n   (다리 0~5, 관절 0=hip 1=femur 2=tibia)
                        o                  (전부 힘 빼기)

실행
  리눅스 블루투스(주소로 직접):  /usr/bin/python3 hexapod_pose_gui.py --robot AA:BB:CC:DD:EE:FF
  USB 케이블로 직접:            python3 hexapod_pose_gui.py --robot /dev/ttyUSB0
  윈도우/맥 블루투스 포트:       python hexapod_pose_gui.py --robot COM7
  (pyserial 필요: pip install pyserial / sudo apt install python3-serial)

화면
  - 다리 6개 × 관절 3개 슬라이더. 움직이면 바로 로봇으로 전송
  - "6개 다리 같이": 맨 위 공용 슬라이더 3개가 모든 다리에 같은 각도를 보냄
  - 서보 90° 자세 / 전부 힘 빼기 / 자세 저장·불러오기 / 현재 자세 복사
  - 발 접지: 펌웨어 #ft 줄로 다리별 접지 버튼 상태 표시 (초록 = 닿음, 흰색 = 뜸, 회색 = 수신 없음)
"""

import argparse
import json
import queue
import re
import socket
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox

try:
    import serial
except ImportError:
    serial = None

# ---- 펌웨어와 같은 값 -------------------------------------------------------
REF_ANG = [180, 180, 120]           # 서보 90°일 때 관절 각도 (hip, femur, tibia)
JOINT_NAMES = ["hip", "femur", "tibia"]
RANGE = 100                         # 슬라이더 범위: REF ± 이 값 [도]
                                    # (서보는 ±90°까지인데 0.8배라 관절로는 약 ±112°)
SEND_INTERVAL = 0.03                # 슬라이더를 끌 때 최대 전송 간격 [초]
FOOT_STALE_SEC = 0.3                # 이 시간 동안 #ft 가 안 오면 회색(모름) 표시 (펌웨어는 100ms마다 보냄)
MAC_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")

# ---- firmware/firmware/firmware.ino 의 각도→틱 변환 값 (펌웨어를 고치면 여기도 맞출 것) ------
ANGLE_GAIN = 0.8                    # 관절 1도 → 서보 0.8도
CAL_ANG = [40, 65, 30]
TICKS_PER_DEG = 11.111 / 4.889
TICK_90_NOMINAL = 307
#         hip  femur tibia
FW_TCAL = [[177, 234, 184], [184, 233, 170], [182, 251, 171],
           [191, 237, 183], [193, 227, 166], [195, 229, 179]]
FW_DIR = [[1, 1, 1], [1, 1, 1], [1, 1, 1], [-1, 1, 1], [-1, 1, 1], [-1, 1, 1]]
FW_OFS = [[0, 0, 0], [0, 0, 0], [0, 0, -10], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

# ---- 모터 캘리브레이션 -----------------------------------------------------------
# 틱 70 / 285 / 500 (서보 0° / 90° / 180°) 일 때 가변저항 ADC 값 (ESP32 12비트)
CALIB_TICK = (70, 285, 500)
# 모터 번호 - 1 = 다리 × 3 + 채널 (채널 0 = tibia, 1 = femur, 2 = hip)
CALIB_POT = [
    (444, 2246, 4088), (426, 2227, 4058), (466, 2161, 3789),
    (464, 2353, 4080), (365, 2187, 3937), (263, 1917, 3608),
    (663, 2362, 3935), (357, 2186, 3972), (555, 2236, 3854),
    (339, 2020, 3666), (356, 2199, 3979), (510, 2209, 3852),
    (367, 2223, 4006), (275, 2098, 3862), (332, 2008, 3680),
    (202, 1945, 3763), (956, 2879, 3919), (533, 2271, 3836),
]
# 90° 값을 믿기 어려워 가변저항 보정 없이 틱 70~500 직선으로만 쓰는 모터 (재측정하면 빼기)
CALIB_STRAIGHT = {17}

# 다리를 수평 일자로 편 자세의 관절 각도 (hip, femur, tibia)
EXTENDED_ANG = [180, 180, 180]
# 그 자세를 만드는, 펌웨어로 그대로 보낸 관절 각도. 보정 후엔 이 자세가 EXTENDED_ANG 로 표시된다
EXTENDED_RAW = [[180, 197, 197], [166, 185, 209], [187, 173, 210],
                [180, 179, 173], [195, 187, 185], [180, 187, 185.09]]


def servo_to_tick(motor, servo_deg):
    """motor(0~17)를 서보 각도 servo_deg 로 보내는 틱. 가변저항 값이 각도에 비례한다고 본다."""
    t0, t90, t180 = CALIB_TICK
    if motor + 1 in CALIB_STRAIGHT:
        return t0 + servo_deg * (t180 - t0) / 180
    p0, p90, p180 = CALIB_POT[motor]
    pot = p0 + (p180 - p0) * servo_deg / 180
    if pot <= p90:
        return t0 + (pot - p0) * (t90 - t0) / (p90 - p0)
    return t90 + (pot - p90) * (t180 - t90) / (p180 - p90)


def tick_to_servo(motor, tick):
    """servo_to_tick 의 역변환."""
    t0, t90, t180 = CALIB_TICK
    if motor + 1 in CALIB_STRAIGHT:
        return (tick - t0) * 180 / (t180 - t0)
    p0, p90, p180 = CALIB_POT[motor]
    if tick <= t90:
        pot = p0 + (tick - t0) * (p90 - p0) / (t90 - t0)
    else:
        pot = p90 + (tick - t90) * (p180 - p90) / (t180 - t90)
    return (pot - p0) * 180 / (p180 - p0)


def calibrated_joint(l, j, joint_deg):
    """관절 각도 → 펌웨어로 보낼 각도. EXTENDED_ANG 에서 EXTENDED_RAW 가 나가고, 그 차이만큼 캘리브레이션대로 움직인다."""
    motor = l * 3 + 2 - j
    d = FW_DIR[l][j]
    t_cal = FW_TCAL[l][j]
    sign = -1 if t_cal > TICK_90_NOMINAL else 1
    raw = EXTENDED_RAW[l][j]
    fw_servo = 90 + d * (raw + FW_OFS[l][j] - REF_ANG[j]) * ANGLE_GAIN
    extended_tick = t_cal + (fw_servo - CAL_ANG[j]) * TICKS_PER_DEG * sign
    servo = tick_to_servo(motor, extended_tick) + d * (joint_deg - EXTENDED_ANG[j]) * ANGLE_GAIN
    tick = servo_to_tick(motor, servo)
    fw_servo = CAL_ANG[j] + (tick - t_cal) / (TICKS_PER_DEG * sign)
    return REF_ANG[j] - FW_OFS[l][j] + d * (fw_servo - 90) / ANGLE_GAIN


# ---- 연결 (블루투스 주소 / 시리얼 포트 둘 다) ---------------------------------
class Link:
    def __init__(self, target):
        self.target = target
        self.sock = None
        self.ser = None
        if MAC_RE.match(target):
            if not hasattr(socket, "AF_BLUETOOTH"):
                raise RuntimeError(
                    "이 파이썬은 블루투스 기능이 없음.\n"
                    "/usr/bin/python3 로 실행하거나, sudo rfcomm bind 0 주소 1 후 --robot /dev/rfcomm0")
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            s.settimeout(5)
            s.connect((target, 1))
            s.settimeout(0.5)
            self.sock = s
        else:
            if serial is None:
                raise RuntimeError("pyserial 필요: pip install pyserial")
            self.ser = serial.Serial(target, 115200, timeout=0.1, write_timeout=0.5)
            # USB로 직접 연결하면 ESP32가 리셋될 수 있음 → 부팅 대기
            time.sleep(1.5)

    def write(self, data: bytes):
        if self.sock:
            self.sock.sendall(data)
        else:
            self.ser.write(data)

    def read(self) -> bytes:
        try:
            if self.sock:
                d = self.sock.recv(256)
                if d == b"":
                    raise ConnectionError("연결 끊김")
                return d
            return self.ser.read(256)
        except socket.timeout:
            return b""

    def close(self):
        try:
            if self.sock:
                self.sock.close()
            if self.ser:
                self.ser.close()
        except Exception:
            pass


# ---- 화면 -------------------------------------------------------------------
class App:
    def __init__(self, root, target, use_calib=True):
        self.root = root
        self.target = target
        self.use_calib = use_calib
        self.synced = False             # 이번 연결에서 18개 관절을 전부 보냈나
        self.link = None
        self.rx_queue = queue.Queue()
        self.pending = {}               # (다리, 관절) → 보낼 각도 (끌 때 몰아서 보냄)
        self.last_flush = 0.0
        self.suppress = False           # 프로그램이 슬라이더 값을 바꿀 땐 전송 안 함
        self.foot = None                # [bool]*6, 다리 l 발이 닿았나 (#ft 줄)
        self.foot_t = 0.0               # 마지막 #ft 수신 시각
        self.foot_shown = None          # 화면에 그려진 상태 (바뀔 때만 다시 그림)

        root.title("Hexapod 관절 조종")
        self.build_ui()
        self.connect()
        root.after(10, self.tick)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- 화면 구성 ----------
    def build_ui(self):
        top = tk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=6)
        self.status = tk.Label(top, text="연결 중…", fg="gray")
        self.status.pack(side="left")
        tk.Button(top, text="다시 연결", command=self.connect).pack(side="right")

        btns = tk.Frame(self.root)
        btns.pack(fill="x", padx=8)
        tk.Button(btns, text="서보 90° 자세", command=self.go_ref).pack(side="left")
        tk.Button(btns, text="전부 힘 빼기 (o)", fg="red", command=self.relax).pack(side="left", padx=4)
        tk.Button(btns, text="자세 저장", command=self.save_pose).pack(side="left")
        tk.Button(btns, text="자세 불러오기", command=self.load_pose).pack(side="left", padx=4)
        tk.Button(btns, text="현재 자세 복사", command=self.copy_pose).pack(side="left")

        # 발 접지
        footf = tk.LabelFrame(self.root, text="발 접지 (초록 = 닿음, 흰색 = 뜸, 회색 = 수신 없음)")
        footf.pack(fill="x", padx=8, pady=(6, 0))
        self.foot_labels = []
        for l in range(6):
            lb = tk.Label(footf, text=f"다리 {l + 1}\n?", width=10, relief="ridge", bg="gray80",
                          font=("TkDefaultFont", 11, "bold"))
            lb.pack(side="left", padx=4, pady=4, expand=True, fill="x")
            self.foot_labels.append(lb)

        # 6개 다리 같이
        allf = tk.LabelFrame(self.root, text="6개 다리 같이 (움직이면 모든 다리에 같은 각도 전송)")
        allf.pack(fill="x", padx=8, pady=6)
        self.all_vars = []
        for j in range(3):
            v = tk.IntVar(value=REF_ANG[j])
            s = tk.Scale(allf, from_=REF_ANG[j] - RANGE, to=REF_ANG[j] + RANGE, orient="horizontal",
                         length=300, label=f"전체 {JOINT_NAMES[j]}", variable=v,
                         command=lambda val, j=j: self.on_all(j, int(float(val))))
            s.grid(row=0, column=j, padx=4)
            self.all_vars.append(v)

        # 다리별
        grid = tk.Frame(self.root)
        grid.pack(padx=8, pady=4)
        self.vars = [[None] * 3 for _ in range(6)]
        self.leg_frames = []
        for l in range(6):
            f = tk.LabelFrame(grid, text=f"다리 {l + 1}  (번호 {l})")
            self.leg_frames.append(f)
            f.grid(row=l // 3, column=l % 3, padx=4, pady=4, sticky="n")
            for j in range(3):
                v = tk.IntVar(value=REF_ANG[j])
                s = tk.Scale(f, from_=REF_ANG[j] - RANGE, to=REF_ANG[j] + RANGE, orient="horizontal",
                             length=240, label=JOINT_NAMES[j], variable=v,
                             command=lambda val, l=l, j=j: self.on_one(l, j, int(float(val))))
                s.pack()
                self.vars[l][j] = v

        # 로봇 메시지
        self.log = tk.Text(self.root, height=8, width=100, state="disabled")
        self.log.pack(fill="both", expand=True, padx=8, pady=6)

    def write_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 300:
            self.log.delete("1.0", f"{lines - 300}.0")
        self.log.configure(state="disabled")

    # ---------- 연결 ----------
    def connect(self):
        if self.link:
            self.link.close()
            self.link = None
        self.synced = False
        self.status.config(text=f"{self.target} 연결 중…", fg="gray")
        self.root.update_idletasks()
        try:
            self.link = Link(self.target)
        except Exception as e:
            self.status.config(text=f"연결 실패: {e}", fg="red")
            return
        self.status.config(text=f"{self.target} 연결됨", fg="green")
        threading.Thread(target=self.reader, args=(self.link,), daemon=True).start()

    def reader(self, link):
        buf = b""
        while self.link is link:
            try:
                buf += link.read()
            except Exception as e:
                self.rx_queue.put(("disc", str(e)))
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                t = line.decode(errors="replace").strip()
                # 펌웨어의 다른 로그가 줄바꿈 없이 앞에 붙어도 #ft 는 살려서 읽음
                k = t.find("#ft,")
                if k > 0:
                    before, t = t[:k].strip(), t[k:]
                    if before and before != "#hb":
                        self.rx_queue.put(("msg", before))
                if t.startswith("#ft,"):
                    # #ft,<ms>,<다리1>...<다리6>  예) #ft,12345,110111  (1 = 닿음)
                    parts = t.split(",")
                    if len(parts) == 3 and len(parts[2]) == 6 and not set(parts[2]) - {"0", "1"}:
                        self.rx_queue.put(("foot", [c == "1" for c in parts[2]]))
                elif t and t != "#hb":
                    self.rx_queue.put(("msg", t))

    def send(self, data: bytes):
        if not self.link:
            return
        try:
            self.link.write(data)
        except Exception as e:
            self.status.config(text=f"전송 실패: {e} — 다시 연결 눌러줘", fg="red")
            self.link.close()
            self.link = None

    # ---------- 슬라이더 ----------
    def on_one(self, l, j, deg):
        if self.suppress:
            return
        self.pending[(l, j)] = deg

    def on_all(self, j, deg):
        if self.suppress:
            return
        self.set_sliders(lambda l, jj: deg if jj == j else None)
        for l in range(6):
            self.pending[(l, j)] = deg

    def set_sliders(self, fn):
        """fn(l, j) → 새 각도 또는 None. 화면만 바꾸고 전송은 따로."""
        self.suppress = True
        for l in range(6):
            for j in range(3):
                v = fn(l, j)
                if v is not None:
                    self.vars[l][j].set(v)
        self.suppress = False

    def flush(self):
        if not self.pending or time.time() - self.last_flush < SEND_INTERVAL:
            return
        if self.use_calib and not self.synced:
            # 펌웨어는 첫 명령에 18개를 전부 켜므로, 안 보낸 관절은 보정 없는 기준값으로 움직인다
            self.synced = True
            for l in range(6):
                for j in range(3):
                    self.pending.setdefault((l, j), self.vars[l][j].get())
        data = b"".join(
            f"<{l},{j},{calibrated_joint(l, j, d) if self.use_calib else d:.2f}>\n".encode()
            for (l, j), d in self.pending.items())
        self.pending.clear()
        self.last_flush = time.time()
        self.send(data)

    def send_all(self):
        for l in range(6):
            for j in range(3):
                self.pending[(l, j)] = self.vars[l][j].get()
        self.last_flush = 0
        self.flush()

    # ---------- 버튼 ----------
    def go_ref(self):
        self.set_sliders(lambda l, j: REF_ANG[j])
        for j in range(3):
            self.all_vars[j].set(REF_ANG[j])
        self.send_all()

    def relax(self):
        self.pending.clear()
        self.send(b"o\n")

    def current_pose(self):
        return [[self.vars[l][j].get() for j in range(3)] for l in range(6)]

    def save_pose(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("자세", "*.json")])
        if path:
            with open(path, "w") as f:
                json.dump({"order": "hip,femur,tibia", "pose": self.current_pose()}, f, indent=1)
            self.write_log(f"[저장] {path}")

    def load_pose(self):
        path = filedialog.askopenfilename(filetypes=[("자세", "*.json")])
        if not path:
            return
        try:
            pose = json.load(open(path))["pose"]
            assert len(pose) == 6 and all(len(p) == 3 for p in pose)
        except Exception as e:
            messagebox.showerror("불러오기 실패", str(e))
            return
        self.set_sliders(lambda l, j: int(pose[l][j]))
        self.send_all()
        self.write_log(f"[불러옴] {path}")

    def copy_pose(self):
        p = self.current_pose()
        text = "{ " + ", ".join("{%d, %d, %d}" % tuple(row) for row in p) + " }   // hip, femur, tibia"
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.write_log("[복사됨] " + text)

    # ---------- 주기 처리 ----------
    def tick(self):
        self.flush()
        while not self.rx_queue.empty():
            kind, t = self.rx_queue.get()
            if kind == "disc":
                self.status.config(text=f"끊김: {t} — 다시 연결 눌러줘", fg="red")
                if self.link:
                    self.link.close()
                self.link = None
            elif kind == "foot":
                if self.foot is not None:
                    for l in range(6):
                        if t[l] != self.foot[l]:
                            self.write_log(f"[접지] 다리 {l + 1} {'닿음' if t[l] else '뜸'}")
                self.foot = t
                self.foot_t = time.time()
            else:
                self.write_log(t)
        self.draw_foot()
        self.root.after(10, self.tick)

    def draw_foot(self):
        fresh = self.foot is not None and time.time() - self.foot_t < FOOT_STALE_SEC
        state = tuple(self.foot) if fresh else None
        if state == self.foot_shown:
            return
        self.foot_shown = state
        for l in range(6):
            if state is None:
                text, bg, fg = "?", "gray80", "black"
            elif state[l]:
                text, bg, fg = "닿음", "#3cb44b", "white"
            else:
                text, bg, fg = "뜸", "white", "black"
            self.foot_labels[l].config(text=f"다리 {l + 1}\n{text}", bg=bg, fg=fg)
            self.leg_frames[l].config(text=f"다리 {l + 1}  (번호 {l})  {'' if state is None else '● ' + text}",
                                      fg=("#2a8a36" if state and state[l] else "black"))

    def on_close(self):
        if self.link:
            self.link.close()
        self.link = None
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True,
                    help="블루투스 주소(AA:BB:..) 또는 포트(/dev/ttyUSB0, /dev/rfcomm0, COM7)")
    ap.add_argument("--no-calib", action="store_true", help="모터 캘리브레이션 없이 슬라이더 각도를 그대로 전송")
    a = ap.parse_args()
    root = tk.Tk()
    App(root, a.robot, use_calib=not a.no_calib)
    root.mainloop()


if __name__ == "__main__":
    main()
