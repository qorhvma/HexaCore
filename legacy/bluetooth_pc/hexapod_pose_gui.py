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
    def __init__(self, root, target):
        self.root = root
        self.target = target
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
        data = b"".join(f"<{l},{j},{d}>\n".encode() for (l, j), d in self.pending.items())
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
    a = ap.parse_args()
    root = tk.Tk()
    App(root, a.robot)
    root.mainloop()


if __name__ == "__main__":
    main()
