"""
라즈베리파이 ← Unitree 4D LiDAR L2 제어

연결 (Unitree unilidar_sdk2 예제 main.cpp 와 같은 설정)
    LiDAR  192.168.1.62 : 6101
    Pi     192.168.1.2  : 6201    (eth0 고정 IP 로 설정해 둘 것)

동작
    1. UDP 소켓을 열고 SDK 와 같은 순서로 회전 시작 → 작업 모드 설정 → 리셋
    2. 수신 스레드가 패킷을 모아 한 바퀴(scan) 단위 포인트 클라우드로 묶음
    3. get_scan() 으로 최신 스캔 (N x 4: x, y, z, intensity) 을 가져감

★ 실제 패킷 디코딩은 C++ SDK(unitree_lidar_sdk/lib)가 담당한다.
   여기서는 SDK 바이너리가 없거나 LiDAR 가 응답하지 않으면 시뮬레이션(가상 방)으로 대체한다.
"""

import math
import random
import socket
import struct
import threading
import time

import numpy as np


LIDAR_IP, LIDAR_PORT = "192.168.1.62", 6101
LOCAL_IP, LOCAL_PORT = "192.168.1.2", 6201

# SDK 패킷 타입 (unitree_lidar_sdk.h)
LIDAR_POINT_DATA_PACKET_TYPE = 102
LIDAR_2D_POINT_DATA_PACKET_TYPE = 103
LIDAR_IMU_DATA_PACKET_TYPE = 104
LIDAR_VERSION_PACKET_TYPE = 105

# 사용자 제어 명령 (LidarUserCtrlCmd)
CMD_STANDBY, CMD_START_ROTATION, CMD_RESET = 0, 1, 2
WORK_MODE_3D_STANDARD_FOV = 0          # bit0=0: 표준 FOV, bit1=0: 3D, bit2=0: IMU 켬

PACKET_HEADER = struct.Struct("<4sII")  # magic, packet_type, packet_size (SDK FrameHeader 구조 근사)
PACKET_MAGIC = b"\x55\xaa\x05\x0a"

SCAN_HZ = 10                    # L2 기본 회전 주기 (한 바퀴 = 0.1s)
MOUNT_HEIGHT = 0.22             # 몸통 위 장착 높이 [m]
RANGE_MIN, RANGE_MAX = 0.05, 30.0


class UnitreeL2:
    def __init__(self, lidar_ip=LIDAR_IP, lidar_port=LIDAR_PORT,
                 local_ip=LOCAL_IP, local_port=LOCAL_PORT,
                 simulate=None, pose_provider=None):
        self.lidar_addr = (lidar_ip, lidar_port)
        self.local_addr = (local_ip, local_port)
        self.simulate = simulate
        self.pose_provider = pose_provider or (lambda: (0.0, 0.0, 0.0))
        self.sock = None
        self.rotating = False
        self.work_mode = WORK_MODE_3D_STANDARD_FOV
        self.dirty_percentage = 0.0
        self.version = {"sdk": "2.0.x", "firmware": "-", "hardware": "-"}
        self.imu = {"quat": [1, 0, 0, 0], "gyro": [0, 0, 0], "acc": [0, 0, 9.81], "t": 0.0}
        self.stats = {"scans": 0, "points": 0, "hz": 0.0}

        self._scan = np.zeros((0, 4), np.float32)
        self._scan_t = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sim_world = SimRoom()

    # ───────────── SDK 와 같은 이름의 제어 함수 ─────────────
    def initialize_udp(self, timeout=1.5):
        """0 = 성공 (SDK 와 같은 규칙)"""
        if self.simulate:
            return 0
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            self.sock.bind(self.local_addr)
            self.sock.settimeout(timeout)
            self._send_ctrl(CMD_STANDBY)
            self.sock.recvfrom(2048)                # 아무 패킷이나 오면 살아있는 것
            return 0
        except OSError as e:
            print(f"[lidar] UDP 초기화 실패 ({e}) → 시뮬레이션 모드")
            if self.sock:
                self.sock.close()
            self.sock = None
            self.simulate = True
            return 0

    def start_lidar_rotation(self):
        self._send_ctrl(CMD_START_ROTATION)
        self.rotating = True

    def stop_lidar_rotation(self):
        self._send_ctrl(CMD_STANDBY)
        self.rotating = False

    def set_lidar_work_mode(self, mode: int):
        self.work_mode = mode
        self._send_raw(struct.pack("<4sIII", PACKET_MAGIC, 0x0A, 12, mode))

    def reset_lidar(self):
        self._send_ctrl(CMD_RESET)

    def sync_lidar_timestamp(self):
        ns = time.time_ns()
        self._send_raw(struct.pack("<4sIIII", PACKET_MAGIC, 0x0B, 16, ns // 1_000_000_000, ns % 1_000_000_000))

    def get_dirty_percentage(self):
        return self.dirty_percentage

    # ───────────── 시작 / 정지 ─────────────
    def start(self):
        if self.simulate is None:
            self.simulate = False
        if self.initialize_udp() != 0:
            raise RuntimeError("Unilidar initialization failed")
        self.start_lidar_rotation()
        time.sleep(0.2 if self.simulate else 1.0)
        self.set_lidar_work_mode(WORK_MODE_3D_STANDARD_FOV)
        if not self.simulate:
            self.reset_lidar()
            time.sleep(1.0)
        self.sync_lidar_timestamp()
        self._thread = threading.Thread(target=self._sim_loop if self.simulate else self._rx_loop, daemon=True)
        self._thread.start()
        print(f"[lidar] L2 시작 ({'SIM' if self.simulate else self.lidar_addr[0]})")

    def stop(self):
        self._stop.set()
        try:
            self.stop_lidar_rotation()
        finally:
            if self._thread:
                self._thread.join(timeout=1.0)
            if self.sock:
                self.sock.close()

    def get_scan(self):
        """(시각, N x 4 float32 [x, y, z, intensity]) — 라이다 좌표계"""
        with self._lock:
            return self._scan_t, self._scan

    def status(self):
        return {
            "simulated": bool(self.simulate),
            "rotating": self.rotating,
            "work_mode": self.work_mode,
            "dirty_pct": round(self.dirty_percentage, 1),
            "scan_hz": round(self.stats["hz"], 1),
            "points": self.stats["points"],
            "version": self.version,
        }

    # ───────────── 내부 ─────────────
    def _send_ctrl(self, cmd):
        self._send_raw(struct.pack("<4sIII", PACKET_MAGIC, 0x09, 12, cmd))

    def _send_raw(self, data: bytes):
        if self.sock and not self.simulate:
            try:
                self.sock.sendto(data, self.lidar_addr)
            except OSError as e:
                print(f"[lidar] 송신 실패: {e}")

    def _publish(self, pts):
        now = time.time()
        with self._lock:
            dt = now - self._scan_t if self._scan_t else 0
            self._scan, self._scan_t = pts, now
            self.stats["scans"] += 1
            self.stats["points"] = len(pts)
            if dt > 0:
                self.stats["hz"] = 0.8 * self.stats["hz"] + 0.2 / dt

    def _rx_loop(self):
        acc = []
        last_angle = None
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            if len(data) < PACKET_HEADER.size:
                continue
            magic, ptype, size = PACKET_HEADER.unpack_from(data)
            if magic != PACKET_MAGIC:
                continue
            if ptype == LIDAR_POINT_DATA_PACKET_TYPE:
                pts, angle, dirty = decode_point_packet(data)
                self.dirty_percentage = dirty
                if last_angle is not None and angle < last_angle and acc:
                    self._publish(np.concatenate(acc))        # 한 바퀴 완성
                    acc = []
                acc.append(pts)
                last_angle = angle
            elif ptype == LIDAR_IMU_DATA_PACKET_TYPE:
                self.imu.update(decode_imu_packet(data))
            elif ptype == LIDAR_VERSION_PACKET_TYPE:
                self.version.update(decode_version_packet(data))

    def _sim_loop(self):
        self.version = {"sdk": "2.0.x", "firmware": "SIM", "hardware": "L2-SIM"}
        while not self._stop.wait(1.0 / SCAN_HZ):
            if not self.rotating:
                continue
            x, y, yaw = self.pose_provider()
            pts = self._sim_world.scan(x, y, yaw)
            self.dirty_percentage = 0.8 + 0.3 * random.random()
            self.imu["t"] = time.time()
            self._publish(pts)


# ───────────── 패킷 디코더 (SDK 구조체 레이아웃에 맞춰 채울 것) ─────────────
def decode_point_packet(data: bytes):
    """LidarPointDataPacket → (N x 4 점, 시작 각도, 오염도 %).  TODO: SDK 헤더의 정확한 오프셋 반영"""
    off = PACKET_HEADER.size
    com_horizontal_angle_start, com_horizontal_angle_step, dirty_index = struct.unpack_from("<ffI", data, off)
    off += 12
    n = (len(data) - off) // 4
    ranges = np.frombuffer(data, dtype="<u2", count=n, offset=off).astype(np.float32) * 0.001
    inten = np.frombuffer(data, dtype="u1", count=n, offset=off + 2 * n).astype(np.float32)
    az = com_horizontal_angle_start + np.arange(n) * com_horizontal_angle_step
    pts = np.stack([ranges * np.cos(az), ranges * np.sin(az), np.zeros_like(ranges), inten], axis=1)
    keep = (ranges > RANGE_MIN) & (ranges < RANGE_MAX)
    return pts[keep], com_horizontal_angle_start, float(dirty_index)


def decode_imu_packet(data: bytes):
    q = struct.unpack_from("<4f", data, PACKET_HEADER.size)
    g = struct.unpack_from("<3f", data, PACKET_HEADER.size + 16)
    a = struct.unpack_from("<3f", data, PACKET_HEADER.size + 28)
    return {"quat": list(q), "gyro": list(g), "acc": list(a), "t": time.time()}


def decode_version_packet(data: bytes):
    raw = data[PACKET_HEADER.size:PACKET_HEADER.size + 24]
    hw, fw, sdk = raw[0:8], raw[8:16], raw[16:24]
    fmt = lambda b: ".".join(str(x) for x in b[:4])  # noqa: E731
    return {"hardware": fmt(hw), "firmware": fmt(fw), "sdk": fmt(sdk)}


# ───────────── 시뮬레이션용 가상 방 ─────────────
class SimRoom:
    """8 x 6 m 방 + 상자 몇 개. 2D 벽을 레이캐스트해서 3D 고리 모양 스캔을 만든다."""

    def __init__(self):
        W, H = 8.0, 6.0
        box = lambda cx, cy, w, h: [((cx-w/2, cy-h/2), (cx+w/2, cy-h/2)), ((cx+w/2, cy-h/2), (cx+w/2, cy+h/2)),  # noqa: E731
                                    ((cx+w/2, cy+h/2), (cx-w/2, cy+h/2)), ((cx-w/2, cy+h/2), (cx-w/2, cy-h/2))]
        self.walls = np.array(
            box(0, 0, W, H) + box(1.8, 1.2, 0.8, 0.6) + box(-2.2, -1.5, 1.2, 0.5)
            + box(-1.0, 1.9, 0.4, 0.4) + [((2.5, -3.0), (2.5, -1.2)), ((-4.0, 0.4), (-2.6, 0.4))],
            dtype=np.float32,
        )
        self.az = np.radians(np.arange(0, 360, 1.0, dtype=np.float32))
        self.elev = np.radians(np.array([-6, -3, 0, 3, 6, 10], dtype=np.float32))

    def _raycast(self, ox, oy, angles):
        dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
        p, q = self.walls[:, 0], self.walls[:, 1]
        ex, ey = (q - p)[:, 0][None], (q - p)[:, 1][None]
        wx, wy = (p[:, 0] - ox)[None], (p[:, 1] - oy)[None]
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (wx * ey - wy * ex) / den
            u = (wx * dy - wy * dx) / den
        t = np.where((np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
        return t.min(axis=1)

    def scan(self, x, y, yaw):
        world_az = self.az + yaw
        r2d = self._raycast(x, y, world_az)
        out = []
        for e in self.elev:
            r = r2d / math.cos(e) + np.random.normal(0, 0.01, r2d.shape)
            z = MOUNT_HEIGHT + r * math.sin(e)
            ok = np.isfinite(r) & (r < RANGE_MAX) & (z > 0.0)
            ok &= np.random.random(r.shape) > 0.03                    # 드롭아웃
            rh = r * math.cos(e)
            out.append(np.stack([rh * np.cos(self.az), rh * np.sin(self.az), z - MOUNT_HEIGHT,
                                 np.random.uniform(20, 200, r.shape)], axis=1)[ok])
        return np.concatenate(out).astype(np.float32)


if __name__ == "__main__":
    lidar = UnitreeL2(simulate=None)
    lidar.start()
    try:
        while True:
            t, pts = lidar.get_scan()
            print(f"{lidar.status()}  최신 스캔 {len(pts)}점")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
