"""
헥사포드 PC 쪽 SDK

사용 순서
    link = Link("AA:BB:CC:DD:EE:FF")        # 또는 "/dev/ttyUSB0"
    hexa = Hexapod()
    if link.connect():
        hexa.sync(link)                      # ★ 연결 직후 반드시 한 번 (USB 재연결이면 assume=BOOT_POSE)
    hexa.standing()                          # 목표 자세만 바꿈
    while hexa.move_update_available():
        hexa.move_update(link)               # 주기적으로 호출
    print(hexa.report(link))                 # 로그 지표 확인

각도 기준: 관절 좌표계 (모든 관절 180 = 다리를 쭉 편 상태).
관절 좌표 → 서보 펄스 변환과 femur/tibia 좌우 미러링은 펌웨어(캘리브레이션) 담당.
PC가 아는 각도는 '보낸 각도(명령각)'일 뿐, 실제 각도가 아니다 (MG996R은 각도를 되돌려주지 않음).

IMU (펌웨어 v2 + BNO086)
    link.imu_stream(True)                    # 로봇이 #imu 줄을 최대 200Hz로 보내기 시작
    if link.imu_fresh():                     # 최근 IMU_STALE_SEC 안에 받은 값이 있나
        roll, pitch = link.imu["roll"], link.imu["pitch"]
    roll  + = 왼쪽이 들림,  pitch + = 앞쪽이 들림 (deg, 펌웨어 기준)
    #imu 줄은 로그 큐에 안 넣는다 (초당 최대 200줄이라 로그가 넘침)

발 접지 버튼 (펌웨어 #ft 줄, GPIO 4,16,17,5,18,19 = 다리1~6)
    펌웨어가 바뀔 때마다 + 100ms마다 자동으로 보냄 (켜는 명령 필요 없음)
    if link.foot_fresh():
        liif link.connect(): breaknk.foot[0]                         # True = 다리1(인덱스 0) 발이 닿음
    link.foot_wait(leg, True, timeout=2.0)   # 그 발이 닿을 때까지 기다림
"""

import math
import re
import socket
import threading
import time
from collections import deque

try:
    import serial
except ImportError:
    serial = None


# ───────────────────────── 프로토콜 ─────────────────────────
MAC_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
# <다리 0~5, 관절 0~2, 각도> 또는 한 글자 명령. 관절 좌표계에선 음수 각도를 안 쓰므로 막는다.
#   o: 힘 빼기   i: IMU 스트림 켜기   k: 끄기   z: 지금 자세를 수평으로 기억
CMD_RE = re.compile(r"^<\s*[0-5]\s*,\s*[0-2]\s*,\s*\d{1,3}(\.\d)?\s*>\s*$|^[oikz]\s*$")

# 각도를 소수 첫째 자리까지 보낼지. 펌웨어 v2는 소수를 받음 (구버전 펌웨어도 정수 부분만 읽고 동작)
# 서보 분해능이 관절 기준 약 0.55°라서, 정수로 보내면 그 절반을 버리게 됨 → 느린 동작·수평 보정이 계단처럼 움직임
SEND_DECIMALS = True

FOOT_STALE_SEC = 0.3  # 이보다 오래된 접지 값은 '없는 것'으로 취급 (펌웨어는 최소 100ms마다 보냄)
IMU_STALE_SEC = 0.2  # 이보다 오래된 IMU 값은 '없는 것'으로 취급 (펌웨어 최대 200Hz = 5ms 간격)

HB_TIMEOUT = 1.5     # 초. 이 시간 동안 아무것도 안 들어오면 끊긴 걸로 처리.
                     # 펌웨어 하트비트 주기의 3배 정도로 맞출 것 (report의 '최대 수신 간격' 보고 조정)
LOG_MAXLEN = 500     # 로그 큐 최대 줄 수 (넘으면 오래된 것부터 버림)
RX_BUF_MAX = 4096    # 줄바꿈 없이 이만큼 쌓이면 버퍼 버림 (쓰레기 데이터 대비)


# ───────────────────────── 로봇 설정 ─────────────────────────
NUM_LEGS, NUM_JOINTS = 6, 3
COXA, FEMUR, TIBIA = 0, 1, 2
COXA_LEN, FEMUR_LEN, TIBIA_LEN = 42.4, 100.02, 137.5

# coxa 좌우 미러링 부호. 다리 0~2는 -1, 3~5는 +1.
# 접기 자세의 coxa 90 / 270 이 여기서 나온다 (180 + SIDE*90).
# femur/tibia는 좌우 같은 값을 보내고, 방향 반전은 펌웨어 캘리브레이션이 처리한다.
SIDE = [-1, -1, -1, +1, +1, +1]

# 관절 가동범위 (최소, 최대) deg, 관절 좌표계.
#   다리마다 다르게 하고 싶으면 JOINT_LIMITS[다리][관절] 만 고치면 된다.
#   근거: 펌웨어 캘리브레이션 표 + ticks 한계(600~2400us)에서 6다리 모두 갈 수 있는 범위를 계산 (2026-10-14)
#   - coxa 90~270: 서보로는 88~280 가능
#   - femur 62~225: 서보로는 62~251 가능 (펌웨어 REF_ANG femur를 150으로 고친 기준).
#       ★ 방향: 작을수록 위로 듦 (hexapod_body 해석). 90 = 수직 위, 180 = 수평, 225 = 수평보다 45° 아래.
#       하한 62 = 수직을 28° 넘어 몸 쪽으로 젖힌 각도 = 서보 한계이자 접기 자세가 실제로 가던 자리
#              (접기가 문제없이 됐다면 기구적으로도 안전하다는 뜻).
#       상한 270 = 수직 아래 (2026-10-17 실측: tibia 180 으로 편 채 femur 270 까지 내려도 몸통·coxa 에 안 닿음).
#              지금 펌웨어(REF 180) 기준 서보로는 다리별 281~294 까지 가능
#       ★ 지금 펌웨어(REF 180) 기준 서보 하한은 다리별 79~92 → 62~92 명령은 펌웨어에서 잘림 (하한은 아직 안 고침)
#       (예전 90~180은 '임시값'이었고, 펌웨어 기준이 30° 틀어져 있어서 실제로는 60~150°만 쓰고 있었음)
#       hexapod_body.py도 이 표를 그대로 씀 (예전엔 거기 80~280이 따로 있어서 두 군데서 막혔음)
#   - tibia 23~180: 23 = 무릎 접힘 한계 (실측), 180 = 쭉 편 상태. 서보로는 21~214 가능
DEFAULT_LIMITS = [(90, 270), (62, 270), (23, 180)]
JOINT_LIMITS = [list(DEFAULT_LIMITS) for _ in range(NUM_LEGS)]

# femur 105 = 예전 펌웨어 기준(REF 180)에서 135를 보냈을 때의 실제 각도 → 서 있는 모양은 예전과 똑같음
STAND_POSE = [[180, 135, 45] for _ in range(NUM_LEGS)]
BOOT_POSE = STAND_POSE   # ★ 펌웨어가 전원 켤 때 들어가는 자세와 반드시 같아야 함

UPDATE_PERIOD = 0.1      # 초. 갱신 주기 (10Hz)
MAX_SPEED = 5.0          # deg/s. 관절 최대 속도 (기존값 유지 — 근거 정해지면 수정)
EPS = 1e-6


def _round_half_up(x: float) -> int:
    # 파이썬 round()는 x.5를 짝수 쪽으로 보내서(134.5→134, 135.5→136) 0.5°씩 움직일 때 걸음이 들쭉날쭉해짐
    return int(math.floor(x + 0.5))


def _quant(x: float) -> int:
    """전송 단위로 양자화한 값 (정수). SEND_DECIMALS면 0.1° 단위(×10), 아니면 1° 단위."""
    return _round_half_up(x * 10) if SEND_DECIMALS else _round_half_up(x)


def _fmt(q: int) -> str:
    return f"{q / 10:.1f}" if SEND_DECIMALS else str(q)


def _copy_pose(pose):
    return [[float(a) for a in leg] for leg in pose]


# ───────────────────────── 통신 ─────────────────────────
class Link:
    def __init__(self, target, log_path=None, verbose_tx=False):
        self.target = target
        self.sock = None
        self.ser = None
        self.lock = threading.Lock()            # sock/ser/_stop 을 바꾸거나 쓸 때 잡는다
        self.log_queue = deque(maxlen=LOG_MAXLEN)
        self.verbose_tx = verbose_tx            # True면 전송할 때마다 로그 (기본은 끔 — 초당 최대 180줄)

        self._stop = None                       # 현재 연결 전용 정지 신호. 연결마다 새로 만든다
        self.last_rx = None                     # 마지막 수신 시각 (monotonic)
        self.max_rx_gap = 0.0                   # 지표: 수신 사이 최대 간격(초)

        self._log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self._log_lock = threading.Lock()

        # IMU (펌웨어 #imu 줄). 읽기 스레드가 dict를 통째로 바꿔 끼우므로 읽는 쪽은 락 없이 써도 됨
        self.imu = None                         # {"roll","pitch","acc","age_ms","t"} t = PC 수신 시각(monotonic)
        self.imu_err = False                    # 펌웨어가 #imu,err 를 보냄 (IMU 없음/고장)
        self.imu_n = 0                          # 지표: 받은 #imu 줄 수
        self.imu_max_gap = 0.0                  # 지표: #imu 수신 사이 최대 간격(초)
        self._imu_streaming = False             # 재연결 때 스트림을 다시 켜기 위해 기억
        self.imu_hist = deque(maxlen=200)       # 최근 샘플 (t, roll, pitch) — 200Hz면 약 1초 분량

        # 발 접지 (펌웨어 #ft 줄). foot[l] = 다리 l(0~5) 발이 닿았나. 읽기 스레드가 리스트를 통째로 바꿔 끼움
        self.foot = None                        # [bool]*6, 아직 못 받았으면 None
        self.foot_t = None                      # 마지막 #ft 수신 시각 (monotonic)
        self.foot_ms = None                     # 펌웨어 millis()
        self.foot_events = deque(maxlen=200)    # 바뀐 순간 (t, leg, 닿음 여부)

    # ── 로그 ──
    def log(self, msg: str):
        self.log_queue.append(msg)
        if self._log_file:
            with self._log_lock:
                self._log_file.write(f"{time.monotonic():.3f}\t{msg}\n")
                self._log_file.flush()

    @property
    def is_open(self):
        return self.sock is not None or self.ser is not None

    # ── 연결 ──
    def _connect(self):
        self.close()                                          # 항상 깨끗하게 시작
        stop = threading.Event()

        if MAC_RE.match(self.target):
            if not hasattr(socket, "AF_BLUETOOTH"):
                raise RuntimeError("이 파이썬은 블루투스 기능이 없음 → /usr/bin/python3 로 실행")
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            try:
                s.settimeout(5)
                s.connect((self.target, 1))
                s.settimeout(0.5)
            except Exception:
                s.close()
                raise
            conn = s
            with self.lock:
                self.sock, self._stop = s, stop
        else:
            if serial is None:
                raise RuntimeError("pyserial 필요: pip install pyserial")
            ser = serial.Serial(self.target, 115200, timeout=0.1, write_timeout=0.5)
            time.sleep(1.5)            # 포트 열 때 ESP32가 리셋되는 경우 부팅 대기 → 이 경우 sync(assume=BOOT_POSE)
            conn = ser
            with self.lock:
                self.ser, self._stop = ser, stop

        self.last_rx = None
        self.max_rx_gap = 0.0
        self.imu = None
        self.foot = None
        self.foot_t = None
        # 시리얼도 읽기 스레드를 띄운다 (기존엔 블루투스만 띄웠음)
        # 스레드에 '이 연결 객체'와 '이 연결의 정지 신호'를 직접 넘김 → 재연결 후 옛 스레드가 새 소켓을 못 건드림
        threading.Thread(target=self._reader, args=(conn, stop), daemon=True).start()

    def connect(self) -> bool:
        self.log(f"{self.target} connecting…")
        try:
            self._connect()
        except Exception as e:
            self.log(f"connect failed: {e}")
            return False
        self.log(f"{self.target} connected")
        if self._imu_streaming:                 # 재연결이면 펌웨어는 스트림 설정을 잊었으므로 다시 켬
            self.send(b"i\n")
        return True

    # ── IMU ──
    def imu_stream(self, on: bool = True) -> bool:
        self._imu_streaming = on
        return self.send(b"i\n" if on else b"k\n")

    def imu_zero(self) -> bool:
        """로봇의 '지금 자세 = 수평'으로 기억시킴. 평평한 바닥에서만 쓸 것. 결과는 로그에 #zero,.. 로 옴."""
        return self.send(b"z\n")

    def imu_fresh(self, max_age: float = IMU_STALE_SEC) -> bool:
        m = self.imu
        return m is not None and time.monotonic() - m["t"] < max_age

    def imu_recent(self, n: int):
        """가장 최근 샘플 n개의 평균 (roll, pitch). 값이 오래됐거나 모자라면 None."""
        if not self.imu_fresh():
            return None
        pts = list(self.imu_hist)[-n:]
        if len(pts) < n:
            return None
        return sum(p[1] for p in pts) / n, sum(p[2] for p in pts) / n

    def imu_window(self, sec: float):
        """최근 sec초 안에 받은 샘플 [(t, roll, pitch), ...]."""
        t0 = time.monotonic() - sec
        return [p for p in list(self.imu_hist) if p[0] >= t0]

    def _on_imu(self, t: str, now: float):
        # #imu,<ms>,<roll>,<pitch>,<acc>,<age>  또는  #imu,err
        parts = t.split(",")
        if len(parts) == 2 and parts[1] == "err":
            if not self.imu_err:
                self.log("IMU 오류 — 로봇 IMU 없음/초기화 실패 (배선 32/33, 주소 0x4B 확인)")
            self.imu_err = True
            return
        try:
            _, _, roll, pitch, acc, age = parts
            m = {"roll": float(roll), "pitch": float(pitch), "acc": int(acc), "age_ms": int(age), "t": now}
        except ValueError:
            return
        old = self.imu
        if old is not None:
            self.imu_max_gap = max(self.imu_max_gap, now - old["t"])
        self.imu = m
        self.imu_hist.append((now, m["roll"], m["pitch"]))
        self.imu_err = False
        self.imu_n += 1

    # ── 발 접지 ──
    def foot_fresh(self, max_age: float = FOOT_STALE_SEC) -> bool:
        t = self.foot_t
        return self.foot is not None and t is not None and time.monotonic() - t < max_age

    def foot_wait(self, leg: int, contact: bool = True, timeout: float = 2.0) -> bool:
        """다리 leg(0~5)의 접지가 contact가 될 때까지 기다림. 시간 안에 되면 True."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            f = self.foot
            if f is not None and self.foot_fresh() and f[leg] == contact:
                return True
            time.sleep(0.002)
        return False

    def _on_foot(self, t: str, now: float):
        # #ft,<ms>,<다리1>...<다리6>  예) #ft,12345,110111
        parts = t.split(",")
        if len(parts) != 3 or len(parts[2]) != NUM_LEGS or set(parts[2]) - {"0", "1"}:
            return
        try:
            ms = int(parts[1])
        except ValueError:
            return
        new = [c == "1" for c in parts[2]]
        old = self.foot
        if old is not None:
            for l in range(NUM_LEGS):
                if new[l] != old[l]:
                    self.foot_events.append((now, l, new[l]))
        self.foot = new
        self.foot_ms = ms
        self.foot_t = now

    # ── 전송 ──
    def send(self, data: bytes) -> bool:
        text = data.decode("utf-8", errors="ignore")
        if not CMD_RE.match(text):
            self.log(f"형식 오류로 안 보냄: {text.strip()}")
            return False
        try:
            with self.lock:
                if self.sock is not None:
                    self.sock.sendall(data)
                elif self.ser is not None:
                    self.ser.write(data)
                else:
                    self.log("연결 안 됨 — 다시 연결 눌러줘")
                    return False
        except Exception as e:
            self.log(f"전송 실패: {e} — 다시 연결 눌러줘")
            self.close()
            return False
        if self.verbose_tx:
            self.log(f"전송: {text.strip()}")
        return True

    # ── 수신 (연결마다 스레드 1개) ──
    def _reader(self, conn, stop):
        buf = b""
        is_sock = isinstance(conn, socket.socket)
        while not stop.is_set():
            try:
                if is_sock:
                    try:
                        d = conn.recv(256)
                    except socket.timeout:
                        d = b""
                    else:
                        if d == b"":
                            raise ConnectionError("상대가 연결을 끊음")
                else:
                    d = conn.read(256)
            except Exception as e:
                if not stop.is_set():                 # 내가 닫은 게 아니면 진짜 끊김
                    self.log(f"수신 끊김: {e}")
                    self.close(stop)
                return

            now = time.monotonic()
            if d:
                if self.last_rx is not None:
                    self.max_rx_gap = max(self.max_rx_gap, now - self.last_rx)
                self.last_rx = now
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    t = line.decode(errors="replace").strip()
                    # 펌웨어의 다른 로그가 줄바꿈 없이 앞에 붙어도 #ft 는 살려서 읽음
                    k = t.find("#ft,")
                    if k > 0:
                        before, t = t[:k].strip(), t[k:]
                        if before and before != "#hb":
                            self.log(before)
                    if t.startswith("#imu,"):
                        self._on_imu(t, now)
                    elif t.startswith("#ft,"):
                        self._on_foot(t, now)
                    elif t and t != "#hb":
                        self.log(t)
                if len(buf) > RX_BUF_MAX:
                    buf = b""
            elif self.last_rx is not None and now - self.last_rx > HB_TIMEOUT:
                # 한 번이라도 받은 뒤에만 판정 (펌웨어가 하트비트를 안 보내는 경우 오판 방지)
                self.log(f"{HB_TIMEOUT}s 동안 수신 없음 → 끊긴 걸로 처리")
                self.close(stop)
                return

    # ── 닫기 ──
    def close(self, stop=None):
        with self.lock:
            if stop is not None and stop is not self._stop:
                return                                 # 이미 새 연결로 바뀜 → 옛 스레드가 새 연결을 닫지 않게
            if self._stop is not None:
                self._stop.set()
            objs = (self.sock, self.ser)
            self.sock = self.ser = self._stop = None   # 닫으면 반드시 None
        for obj in objs:
            if obj is not None:
                try:
                    obj.close()
                except Exception:
                    pass


# ───────────────────────── 로봇 ─────────────────────────
class Hexapod:
    def __init__(self):
        # cmd    : 보간 중인 명령각(float). '실제각'이 아님
        # target : 목표각
        # last_sent : 마지막으로 '전송 성공한' 각도(_quant 단위 정수) = 로봇이 실제로 받은 값
        self.cmd = _copy_pose(BOOT_POSE)
        self.target = _copy_pose(BOOT_POSE)
        self.last_sent = [[None] * NUM_JOINTS for _ in range(NUM_LEGS)]
        self.max_speed = [[MAX_SPEED] * NUM_JOINTS for _ in range(NUM_LEGS)]   # deg/s, 관절별 상한

        self.state_known = False       # PC가 od.cur_speed = [[20, 200, 250] for i in range(6)]  # 이로봇 자세를 안다고 믿어도 되나. sync() 전엔 False
        self.timedelay = UPDATE_PERIOD

        self._speed = [[0.0] * NUM_JOINTS for _ in range(NUM_LEGS)]   # 이번 동작에서 관절별 실제 속도
        self._plan_target = None       # 속도 배분을 계산했던 목표 (목표가 바뀌면 다시 계산)
        self._last_tick = None
        self._was_moving = False
        self._warned_unsync = False

        self.stats = {
            "tick_n": 0, "tick_sum_ms": 0.0, "tick_max_ms": 0.0,   # 움직이는 동안 갱신 간격
            "frame_max_ms": 0.0,                                  # 한 프레임 전송 시간
            "sent": 0,                                            # 실제 보낸 관절 명령 수
            "skipped_same_deg": 0,                                # 각도는 변했지만 정수가 같아 안 보낸 수
            "clamped": 0,                                         # 가동범위 밖이라 잘린 횟수
        }

    # 구버전 이름 호환 (pos = 현재, cur = 목표)
    @property
    def pos(self):
        return self.cmd

    @property
    def cur(self):
        return self.target

    @cur.setter
    def cur(self, pose):
        self.set_pose(pose)

    # ── 목표 설정 ──
    def set_pose(self, pose):
        for l in range(NUM_LEGS):
            for i in range(NUM_JOINTS):
                self.target[l][i] = float(pose[l][i])

    def set_target(self, leg, joint, angle):
        self.target[leg][joint] = float(angle)

    def folding(self, link=None):
        # femur 62 = 서보 한계. 예전(90 명령 = 실제 60)과 거의 같은 모양
        self.set_pose([[180 + SIDE[l] * 90, 62, 23] for l in range(NUM_LEGS)])

    def standing(self, link=None):
        self.set_pose(STAND_POSE)

    def expanding(self, link=None):
        # 다리를 쭉 편 자세. 자기 간섭은 없지만, 바닥에 선 채로 하면 모멘트 암이 최대(≈22cm)가 됨
        # → 몸통을 들어 올린 상태에서만 쓸 것
        self.set_pose([[180, 180, 180] for _ in range(NUM_LEGS)])

    def poweroff(self, link: Link) -> bool:
        ok = link.send(b"o\n")
        # 서보 힘이 빠지면 자세가 무너져서 PC의 cmd는 더 이상 사실이 아님
        # 다시 움직이기 전에 sync() 필요. 첫 명령에서 서보가 순간 점프하는 건 피드백 없이 못 막음
        self.state_known = False
        self._warned_unsync = False
        return ok

    # ── 동기화 ──
    def sync(self, link: Link, assume=None) -> bool:
        """연결 직후 한 번 호출. 명령각 18개를 한 번에 보내서 PC와 로봇을 맞춘다.
        assume: '로봇이 지금 이 자세다'라고 선언할 때. 예) USB 연결로 ESP32가 리셋됐으면 BOOT_POSE.
        ※ 로봇의 실제 자세가 cmd와 다르면 이 순간 서보가 점프한다."""
        if assume is not None:
            self.cmd = _copy_pose(assume)
            self.target = _copy_pose(assume)          # 대기 중이던 동작 취소
        self.last_sent = [[None] * NUM_JOINTS for _ in range(NUM_LEGS)]
        for l in range(NUM_LEGS):
            for i in range(NUM_JOINTS):
                self.cmd[l][i] = self._clamp(l, i, self.cmd[l][i])
                deg = _quant(self.cmd[l][i])
                if not link.send(f"<{l}, {i}, {_fmt(deg)}>\n".encode()):
                    self.state_known = False
                    link.log("sync 실패 — 재연결 후 다시")
                    return False
                self.last_sent[l][i] = deg
        self.state_known = True
        self._plan_target = None
        self._warned_unsync = False
        link.log("자세 동기화 완료")
        return True

    # ── 내부 계산 ──
    @staticmethod
    def _clamp(leg, joint, angle):
        lo, hi = JOINT_LIMITS[leg][joint]
        return min(max(angle, lo), hi)

    def _replan(self, link: Link):
        # 1) 범위 밖 목표는 잘라내고 알림 (cur[l][i] = x 처럼 직접 바꾼 값도 여기서 걸림)
        for l in range(NUM_LEGS):
            for i in range(NUM_JOINTS):
                a = self.target[l][i]
                c = self._clamp(l, i, a)
                if c != a:
                    link.log(f"다리{l} 관절{i}: {a:g} → {c:g} (가동범위 밖이라 자름)")
                    self.target[l][i] = c
                    self.stats["clamped"] += 1

        # 2) 모든 관절이 '같은 시각에' 도착하도록 속도 배분
        #    가장 오래 걸리는 관절의 시간을 기준으로 나머지를 느리게
        duration = 0.0
        for l in range(NUM_LEGS):
            for i in range(NUM_JOINTS):
                diff = abs(self.target[l][i] - self.cmd[l][i])
                if diff > EPS:
                    duration = max(duration, diff / self.max_speed[l][i])
        for l in range(NUM_LEGS):
            for i in range(NUM_JOINTS):
                diff = abs(self.target[l][i] - self.cmd[l][i])
                self._speed[l][i] = diff / duration if duration > 0 else 0.0

        self._plan_target = [leg[:] for leg in self.target]

    # ── 주기 갱신 ──
    def move_update(self, link: Link):
        now = time.monotonic()
        if self._last_tick is not None and now - self._last_tick < self.timedelay:
            return
        dt = UPDATE_PERIOD if self._last_tick is None else min(now - self._last_tick, UPDATE_PERIOD)
        if self._was_moving and self._last_tick is not None:
            gap_ms = (now - self._last_tick) * 1000
            self.stats["tick_n"] += 1
            self.stats["tick_sum_ms"] += gap_ms
            self.stats["tick_max_ms"] = max(self.stats["tick_max_ms"], gap_ms)
        self._last_tick = now

        if not link.is_open:
            self.state_known = False
        if not self.state_known:
            if not self._warned_unsync:
                link.log("자세 동기화 안 됨 — 연결 후 sync() 먼저")
                self._warned_unsync = True
            self._was_moving = False
            return

        if self.target != self._plan_target:
            self._replan(link)

        # 1) 보간
        moved = [[False] * NUM_JOINTS for _ in range(NUM_LEGS)]
        for l in range(NUM_LEGS):
            for i in range(NUM_JOINTS):
                diff = self.target[l][i] - self.cmd[l][i]
                if diff == 0:
                    continue
                step = self._speed[l][i] * dt
                if abs(diff) <= step or abs(diff) < EPS:
                    self.cmd[l][i] = self.target[l][i]    # 마지막 걸음은 목표에 딱 맞춤
                else:
                    self.cmd[l][i] += step if diff > 0 else -step
                    if abs(self.target[l][i] - self.cmd[l][i]) < EPS:
                        self.cmd[l][i] = self.target[l][i]   # 소수점 찌꺼기(1e-13 등) 정리
                moved[l][i] = True

        # 2) 전송 단위(0.1° 또는 1°)로 바뀐 각도만 전송. 전송 성공한 것만 last_sent에 기록
        t0 = time.monotonic()
        for l in range(NUM_LEGS):
            for i in range(NUM_JOINTS):
                deg = _quant(self.cmd[l][i])
                if deg == self.last_sent[l][i]:
                    if moved[l][i]:
                        self.stats["skipped_same_deg"] += 1
                    continue
                if not link.send(f"<{l}, {i}, {_fmt(deg)}>\n".encode()):
                    self.state_known = False
                    self._warned_unsync = False
                    link.log("전송 실패 → 자세 모름 상태. 재연결 후 sync() 필요")
                    self._was_moving = False
                    return
                self.last_sent[l][i] = deg
                self.stats["sent"] += 1
        self.stats["frame_max_ms"] = max(self.stats["frame_max_ms"], (time.monotonic() - t0) * 1000)
        self._was_moving = self.move_update_available()

    def move_update_available(self) -> bool:
        return any(abs(self.target[l][i] - self.cmd[l][i]) > EPS
                   for l in range(NUM_LEGS) for i in range(NUM_JOINTS))

    # ── 지표 ──
    def report(self, link: Link = None) -> str:
        s = self.stats
        avg = s["tick_sum_ms"] / s["tick_n"] if s["tick_n"] else 0.0
        total = s["sent"] + s["skipped_same_deg"]
        dup = 100 * s["skipped_same_deg"] / total if total else 0.0
        lines = [
            f"갱신 간격(움직이는 동안): 평균 {avg:.1f} ms / 최대 {s['tick_max_ms']:.1f} ms (목표 {UPDATE_PERIOD*1000:.0f} ms)",
            f"프레임 전송 시간 최대: {s['frame_max_ms']:.1f} ms",
            f"전송 {s['sent']}건 / 중복이라 생략 {s['skipped_same_deg']}건 ({dup:.0f}%)",
            f"가동범위 클램프: {s['clamped']}회",
        ]
        if link is not None:
            lines.append(f"최대 수신 간격: {link.max_rx_gap*1000:.0f} ms (끊김 판정 {HB_TIMEOUT*1000:.0f} ms)")
            if link.imu_n:
                lines.append(f"IMU 수신 {link.imu_n}줄, 최대 간격 {link.imu_max_gap*1000:.0f} ms "
                             f"(펌웨어 200Hz = 5 ms, 오래됨 판정 {IMU_STALE_SEC*1000:.0f} ms)")
        return "\n".join(lines)