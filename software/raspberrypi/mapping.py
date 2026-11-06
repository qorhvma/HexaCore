"""
2D 점유 격자 지도 (Occupancy Grid) — L2 포인트 클라우드에서 몸통 높이 띠만 잘라 쓴다

    로봇 위치는 '명령 속도 적분(오도메트리) + IMU yaw' 로 추정 (dead reckoning).
    ★ 스캔 매칭(ICP) 보정은 아직 없음 → 오래 걸으면 지도가 밀린다. 나중에 slam_toolbox / Cartographer 로 교체 예정.

    셀 값: log-odds (float32).  내보낼 땐 0~255 (0 = 빈 곳, 127 = 모름, 255 = 장애물)
"""

import base64
import math
import threading

import numpy as np


RESOLUTION = 0.05          # 셀 한 칸 [m]
SIZE_M = 12.0              # 지도 한 변 [m]  → 240 x 240 셀
Z_BAND = (-0.15, 0.35)     # 라이다 기준 이 높이 띠에 있는 점만 장애물로 씀 [m]
L_OCC, L_FREE = 0.85, -0.4
L_MIN, L_MAX = -4.0, 4.0
MAX_RAY = 8.0


class Pose2D:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def integrate(self, vx, vy, wz, dt):
        """몸 기준 속도 (vx 앞, vy 왼쪽 [m/s], wz [rad/s]) 를 적분"""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

    def as_tuple(self):
        return self.x, self.y, self.yaw


class OccupancyGrid:
    def __init__(self, resolution=RESOLUTION, size_m=SIZE_M):
        self.res = resolution
        self.n = int(round(size_m / resolution))
        self.origin = -size_m / 2                      # 지도 (0,0) 셀의 월드 좌표
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.trail = []                                # 지나온 경로
        self._lock = threading.Lock()

    def world_to_cell(self, x, y):
        return (np.floor((x - self.origin) / self.res).astype(np.int32),
                np.floor((y - self.origin) / self.res).astype(np.int32))

    def integrate_scan(self, pts, pose: Pose2D, max_points=2500):
        """pts: N x 4 (라이다 좌표계 x, y, z, i)"""
        if pts is None or len(pts) == 0:
            return
        band = pts[(pts[:, 2] > Z_BAND[0]) & (pts[:, 2] < Z_BAND[1])]
        if len(band) > max_points:
            band = band[np.random.choice(len(band), max_points, replace=False)]
        r = np.hypot(band[:, 0], band[:, 1])
        band = band[(r > 0.1) & (r < MAX_RAY)]
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        wx = pose.x + band[:, 0] * c - band[:, 1] * s
        wy = pose.y + band[:, 0] * s + band[:, 1] * c

        ox, oy = self.world_to_cell(np.array([pose.x]), np.array([pose.y]))
        ex, ey = self.world_to_cell(wx, wy)
        ox, oy = int(ox[0]), int(oy[0])

        with self._lock:
            # 빈 공간: 로봇 → 끝점 사이 셀을 등간격 샘플링 (브레젠험 근사, 벡터화)
            steps = np.maximum(np.abs(ex - ox), np.abs(ey - oy)).max(initial=1)
            t = np.linspace(0, 1, int(steps), endpoint=False)[None, :]
            fx = (ox + (ex[:, None] - ox) * t).astype(np.int32).ravel()
            fy = (oy + (ey[:, None] - oy) * t).astype(np.int32).ravel()
            ok = (fx >= 0) & (fx < self.n) & (fy >= 0) & (fy < self.n)
            free = np.unique(fy[ok] * self.n + fx[ok])
            self.logodds.ravel()[free] += L_FREE
            # 장애물
            ok = (ex >= 0) & (ex < self.n) & (ey >= 0) & (ey < self.n)
            occ = np.unique(ey[ok] * self.n + ex[ok])
            self.logodds.ravel()[occ] += L_OCC - L_FREE
            np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)

            if not self.trail or math.dist(self.trail[-1], (pose.x, pose.y)) > 0.05:
                self.trail.append((round(pose.x, 3), round(pose.y, 3)))
                self.trail = self.trail[-2000:]

    def to_uint8(self):
        with self._lock:
            p = 1.0 / (1.0 + np.exp(-self.logodds))
        return (p * 255).astype(np.uint8)

    def export(self):
        """대시보드 전송용 dict (격자는 base64 로 압축 없이 — 240x240 = 57.6KB)"""
        return {
            "resolution": self.res,
            "width": self.n,
            "height": self.n,
            "origin": [self.origin, self.origin],
            "data_b64": base64.b64encode(self.to_uint8().tobytes()).decode("ascii"),
            "trail": self.trail[-400:],
        }

    def save_pgm(self, path):
        """ROS map_server 호환 PGM 으로 저장 (yaml 은 따로)"""
        img = 255 - self.to_uint8()[::-1]
        with open(path, "wb") as f:
            f.write(f"P5\n{self.n} {self.n}\n255\n".encode())
            f.write(img.tobytes())

    def reset(self):
        with self._lock:
            self.logodds[:] = 0
            self.trail.clear()
