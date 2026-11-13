"""
라즈베리파이 → ESP32 펌웨어 제어 (USB 시리얼 / UART)

펌웨어(firmware/firmware/firmware.ino) 프로토콜
    보내기
        <다리,관절,각도>   예) <0,1,135.0>     다리 0~5, 관절 0=hip 1=femur 2=tibia
        o                  전부 힘 빼기
        i / k              IMU 스트림 켜기 / 끄기
        z                  지금 자세를 수평으로 기억
        e                  환경 센서 상태 요청 (펌웨어 ENV_SENSORS_ENABLED=1 일 때만)
    받기
        #hb                          1초마다 살아있음
        #ft,<ms>,<101101>            발 접지 (1 = 닿음)
        #imu,<ms>,<roll>,<pitch>,<acc>,<age>
        #imu,err
        #zero,<ux>,<uy>,<uz> / #zero,err
        #env,<ms>,<temp>,<hum>,<ppm>,<raw>,<flame>,<alarm>
        그 외                        펌웨어 로그 (한국어 텍스트)

시리얼 포트가 없거나 pyserial 이 없으면 시뮬레이션 모드로 돌아가 가짜 텔레메트리를 만든다.
"""

import math
import random
import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict

try:
    import serial
except ImportError:
    serial = None


NUM_LEGS, NUM_JOINTS = 6, 3
HIP, FEMUR, TIBIA = 0, 1, 2
STAND_POSE = [[180.0, 135.0, 45.0] for _ in range(NUM_LEGS)]

HB_TIMEOUT = 1.5        # 이 시간 동안 아무 줄도 안 오면 끊김
KEEPALIVE_IDLE_SEC = 0.3   # 이만큼 아무것도 안 보냈으면 마지막 자세를 다시 보냄 (펌웨어 FAILSAFE 2초 대비)
DEADBAND_DEG = 0.3         # 마지막으로 보낸 값과 이만큼 다를 때만 그 관절을 보냄 (서보 분해능 ≈ 0.5°)
FULL_REFRESH_SEC = 0.5     # 이 간격으로는 18관절을 전부 보냄 (ESP32 가 재부팅돼도 따라오게)
# 115200 baud ≈ 11.25 KB/s. 예전처럼 50Hz × 18관절 + keepalive 20Hz 를 다 보내면 약 14 KB/s 라 넘쳐서
# 명령이 0.3초 넘게 밀리고 제어 루프가 쓰기에서 막혔다. 바뀐 관절만 묶어 보내면 걸을 때 약 7 KB/s.
LOG_MAXLEN = 300

ENV_ALARM_BITS = {1: "high_temp", 2: "smoke", 4: "flame", 8: "sensor_fault"}


@dataclass
class Telemetry:
    connected: bool = False
    simulated: bool = False
    last_rx: float = 0.0
    powered: bool = False
    foot: list = field(default_factory=lambda: [False] * NUM_LEGS)
    foot_t: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    imu_acc: int = 0
    imu_t: float = 0.0
    imu_err: bool = False
    temp_c: float = float("nan")
    humidity: float = float("nan")
    smoke_ppm: float = 0.0
    smoke_raw: int = 0
    flame: bool = False
    env_alarm: int = 0
    env_t: float = 0.0
    joints: list = field(default_factory=lambda: [row[:] for row in STAND_POSE])

    def to_dict(self):
        d = asdict(self)
        d["env_alarms"] = [name for bit, name in ENV_ALARM_BITS.items() if self.env_alarm & bit]
        for k in ("temp_c", "humidity"):
            if isinstance(d[k], float) and math.isnan(d[k]):
                d[k] = None
        return d


class FirmwareLink:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, simulate=None):
        self.port, self.baud = port, baud
        self.ser = None
        self.simulate = simulate
        self.tel = Telemetry()
        self.logs = deque(maxlen=LOG_MAXLEN)
        self._lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []
        self._listeners = []          # 펌웨어 로그 줄 콜백 (PC 로 전달용)
        self._sent = [[None] * NUM_JOINTS for _ in range(NUM_LEGS)]   # 마지막으로 보낸 각도 (None = 모름)
        self._last_tx = 0.0           # 마지막으로 무언가 보낸 시각 (monotonic)
        self._last_full = 0.0         # 마지막으로 18관절을 다 보낸 시각

    # ───────────── 연결 ─────────────
    def connect(self):
        if self.simulate is None:
            self.simulate = serial is None
        if not self.simulate:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                time.sleep(2.0)                       # ESP32 는 포트 열면 리셋됨 → 부팅 대기
                self.ser.reset_input_buffer()
            except Exception as e:  # noqa: BLE001
                self._log(f"[pi] 시리얼 {self.port} 열기 실패 ({e}) → 시뮬레이션 모드")
                self.simulate = True
        self.tel.simulated = self.simulate
        target = self._sim_loop if self.simulate else self._rx_loop
        for fn in (target, self._keepalive_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)
        self.imu_stream(True)
        self._log(f"[pi] 펌웨어 링크 시작 ({'SIM' if self.simulate else self.port})")
        return True

    def close(self):
        try:
            self.power_off()
        finally:
            self._stop.set()
            for t in self._threads:
                t.join(timeout=1.0)
            if self.ser:
                self.ser.close()

    def on_log(self, cb):
        self._listeners.append(cb)

    @property
    def alive(self):
        return self.simulate or (time.time() - self.tel.last_rx) < HB_TIMEOUT

    # ───────────── 명령 ─────────────
    def _write(self, s: str):
        self._last_tx = time.monotonic()
        if self.simulate:
            return
        with self._tx_lock:
            self.ser.write(s.encode("ascii"))

    def send_joint(self, leg, joint, deg):
        if not (0 <= leg < NUM_LEGS and 0 <= joint < NUM_JOINTS):
            raise ValueError(f"잘못된 관절 인덱스 {leg},{joint}")
        deg = max(0.0, min(359.9, float(deg)))
        with self._lock:
            self.tel.joints[leg][joint] = deg
            self.tel.powered = True
            self._sent[leg][joint] = deg
        self._write(f"<{leg},{joint},{deg:.1f}>")

    def send_pose(self, pose, full=False):
        """18관절 자세. 마지막으로 보낸 값과 DEADBAND_DEG 이상 다른 관절만 한 번에 묶어 보낸다
        (FULL_REFRESH_SEC 마다, 또는 full=True 면 전부)."""
        now = time.monotonic()
        full = full or now - self._last_full >= FULL_REFRESH_SEC
        parts = []
        with self._lock:
            for l in range(NUM_LEGS):
                for j in range(NUM_JOINTS):
                    deg = max(0.0, min(359.9, float(pose[l][j])))
                    self.tel.joints[l][j] = deg
                    prev = self._sent[l][j]
                    if full or prev is None or abs(deg - prev) >= DEADBAND_DEG:
                        parts.append(f"<{l},{j},{deg:.1f}>")
                        self._sent[l][j] = deg
            self.tel.powered = True
        if full:
            self._last_full = now
        if parts:
            self._write("".join(parts))

    def stand(self):
        self.send_pose(STAND_POSE, full=True)

    def power_off(self):
        with self._lock:
            self.tel.powered = False
            self._sent = [[None] * NUM_JOINTS for _ in range(NUM_LEGS)]   # 다시 켤 때 전부 보내게
        self._write("o")

    def imu_stream(self, on=True):
        self._write("i" if on else "k")

    def imu_zero(self):
        self._write("z")

    def request_env(self):
        self._write("e")

    def snapshot(self):
        with self._lock:
            d = self.tel.to_dict()
        d["alive"] = self.alive
        return d

    # ───────────── 수신 ─────────────
    def _log(self, line):
        self.logs.append((time.time(), line))
        for cb in self._listeners:
            try:
                cb(line)
            except Exception:  # noqa: BLE001
                pass

    def _rx_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as e:  # noqa: BLE001
                self._log(f"[pi] 시리얼 읽기 오류: {e}")
                time.sleep(0.5)
                continue
            if not chunk:
                with self._lock:
                    self.tel.connected = self.alive
                continue
            buf += chunk
            if len(buf) > 4096 and b"\n" not in buf:
                buf = b""
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle_line(raw.decode("utf-8", "replace").strip())

    def _handle_line(self, line):
        if not line:
            return
        now = time.time()
        with self._lock:
            self.tel.last_rx = now
            self.tel.connected = True
        if not line.startswith("#"):
            self._log(f"[로봇] {line}")
            return
        parts = line[1:].split(",")
        tag = parts[0]
        try:
            with self._lock:
                t = self.tel
                if tag == "hb":
                    pass
                elif tag == "ft" and len(parts) >= 3:
                    t.foot = [c == "1" for c in parts[2][:NUM_LEGS]]
                    t.foot_t = now
                elif tag == "imu":
                    if len(parts) >= 2 and parts[1] == "err":
                        t.imu_err = True
                    elif len(parts) >= 5:
                        t.roll, t.pitch = float(parts[2]), float(parts[3])
                        t.imu_acc = int(parts[4])
                        t.imu_t, t.imu_err = now, False
                elif tag == "env" and len(parts) >= 8:
                    t.temp_c = float(parts[2])
                    t.humidity = float(parts[3])
                    t.smoke_ppm = float(parts[4])
                    t.smoke_raw = int(parts[5])
                    t.flame = parts[6] == "1"
                    t.env_alarm = int(parts[7])
                    t.env_t = now
                elif tag == "zero":
                    pass
            if tag == "zero":
                self._log(f"[로봇] 수평 기준 저장: {','.join(parts[1:])}")
        except ValueError:
            self._log(f"[pi] 파싱 실패: {line}")

    def _keepalive_loop(self):
        """제어 루프가 한동안 아무것도 안 보냈을 때만 마지막 자세를 다시 보낸다 (보내는 중에는 끼어들지 않음)."""
        while not self._stop.wait(0.1):
            with self._lock:
                powered = self.tel.powered
                pose = [row[:] for row in self.tel.joints]
            if powered and time.monotonic() - self._last_tx >= KEEPALIVE_IDLE_SEC:
                self.send_pose(pose, full=True)

    # ───────────── 시뮬레이션 ─────────────
    def _sim_loop(self):
        t0 = time.time()
        temp, smoke = 31.0, 80.0
        while not self._stop.wait(0.05):
            t = time.time() - t0
            temp += random.uniform(-0.05, 0.06)
            temp = min(max(temp, 25.0), 48.0)
            smoke = max(40.0, smoke + random.uniform(-6, 6) + (40 if random.random() < 0.002 else 0))
            gait = int(t * 2) % 2
            lines = [
                "#hb",
                f"#imu,{int(t*1000)},{2.0*math.sin(t*0.9):.2f},{1.5*math.cos(t*0.6):.2f},3,4",
                f"#ft,{int(t*1000)}," + "".join("1" if (i % 2 == gait) or not self.tel.powered else "0"
                                                 for i in range(NUM_LEGS)),
            ]
            if int(t * 20) % 20 == 0:
                alarm = (1 if temp >= 60 else 0) | (2 if smoke >= 300 else 0)
                lines.append(f"#env,{int(t*1000)},{temp:.1f},{45+5*math.sin(t/30):.1f},"
                             f"{smoke:.0f},{int(smoke*2.1)},0,{alarm}")
            for ln in lines:
                self._handle_line(ln)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="ESP32 펌웨어 링크 테스트")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()

    fw = FirmwareLink(args.port, simulate=True if args.sim else None)
    fw.on_log(print)
    fw.connect()
    try:
        fw.stand()
        while True:
            s = fw.snapshot()
            print(f"alive={s['alive']} roll={s['roll']:+.2f} pitch={s['pitch']:+.2f} "
                  f"foot={''.join('1' if f else '0' for f in s['foot'])} "
                  f"temp={s['temp_c']} smoke={s['smoke_ppm']:.0f}ppm")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        fw.close()
