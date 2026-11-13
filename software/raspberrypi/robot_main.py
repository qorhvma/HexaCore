"""
라즈베리파이 메인 노드

    [PC / 대시보드] ⇄ WebSocket ⇄ [Pi: robot_main] ⇄ USB 시리얼 ⇄ [ESP32 firmware.ino]
                                         ⇅ UDP
                                   [Unitree L2 LiDAR]

실행
    python3 robot_main.py                       # 시리얼·라이다 자동, 없으면 시뮬레이션
    python3 robot_main.py --sim                 # 전부 시뮬레이션 (PC 에서도 돌아감)
    python3 robot_main.py --port /dev/serial0   # GPIO UART 로 ESP32 연결 시

스레드
    control  50Hz : cmd_vel → 삼각 보행(tripod) 관절각 → 펌웨어
    mapping   5Hz : 라이다 스캔 + 추정 자세 → 점유 격자
    pc_link       : asyncio WebSocket 서버 (메인 스레드)
"""

import argparse
import math
import os
import threading
import time

import numpy as np

from firmware_link import FirmwareLink, STAND_POSE, NUM_LEGS
from lidar_l2 import UnitreeL2
from mapping import OccupancyGrid, Pose2D
from pc_link import PcServer
from ik_gait import IkTripodGait


VERSION = "0.4.0-pi"   # 0.4 = IK 보행 + 캘리브레이션 표(data/calibration) 관절 보정
CONTROL_HZ = 50
MAPPING_HZ = 5
DEADMAN_SEC = 0.5

MAX_VX = float(os.environ.get("HEXAPOD_MAX_VX") or 0.25)   # m/s (보행이 보폭 한계로 다시 줄임)
MAX_VY = float(os.environ.get("HEXAPOD_MAX_VY") or 0.15)
MAX_WZ = 0.6                                             # rad/s
GAIT_HZ = 1.2                                  # 한 걸음 주기
STEP_LIFT_DEG = 18
STRIDE_DEG_PER_MPS = 140                       # 속도 → coxa 스윙 진폭
TRIPOD = ((0, 2, 4), (1, 3, 5))
SIDE = [-1, -1, -1, +1, +1, +1]
# 다리 장착 각도 (몸 중심 기준, 앞 = 0) — 회전 보행에 씀
MOUNT_ANGLE = [math.radians(a) for a in (60, 90, 120, -120, -90, -60)]

SAFE_TEMP_C = 60.0
SAFE_TILT_DEG = 25.0


class TripodGait:
    def __init__(self):
        self.phase = 0.0

    def update(self, vx, vy, wz, dt):
        moving = abs(vx) + abs(vy) + abs(wz) * 0.1 > 1e-3
        if not moving:
            self.phase = 0.0
            return [row[:] for row in STAND_POSE]
        self.phase = (self.phase + GAIT_HZ * dt) % 1.0
        pose = []
        for leg in range(NUM_LEGS):
            group = 0 if leg in TRIPOD[0] else 1
            ph = (self.phase + 0.5 * group) % 1.0
            swing = ph < 0.5                                  # 앞 절반 = 공중
            s = math.sin(ph * 2 * math.pi) if swing else -math.sin((ph - 0.5) * 2 * math.pi)
            # 다리 방향으로 투영한 속도 → coxa 스윙
            a = MOUNT_ANGLE[leg]
            v_tan = vx * math.sin(a + math.pi / 2) + vy * math.cos(a + math.pi / 2) + wz * 0.11
            coxa = STAND_POSE[leg][0] + SIDE[leg] * STRIDE_DEG_PER_MPS * v_tan * (abs(s) if swing else -abs(s))
            lift = STEP_LIFT_DEG * math.sin(ph * 2 * math.pi) if swing else 0.0
            femur = STAND_POSE[leg][1] - lift
            tibia = STAND_POSE[leg][2] - lift * 0.6
            pose.append([coxa, femur, tibia])
        return pose


class RobotNode:
    def __init__(self, args):
        self.args = args
        self.pose = Pose2D()
        self.cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "t": 0.0}
        self.mode = "idle"                   # idle | stand | walk | estop
        self.safety = []
        # 기본은 IK 보행 (모터 캘리브레이션·최종 CAD 치수 기반). --gait sine 이면 예전 관절 사인파 보행
        self.gait = TripodGait() if getattr(args, "gait", "ik") == "sine" else IkTripodGait()
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.fw = FirmwareLink(args.port, simulate=True if args.sim else None)
        self.lidar = UnitreeL2(simulate=True if args.sim else None, pose_provider=self.pose.as_tuple)
        self.grid = OccupancyGrid()
        self.server = PcServer(self, port=args.ws_port)
        self.fw.on_log(lambda line: (print(line), self.server.push_log(line)))

    # ───────────── 생명주기 ─────────────
    def start(self):
        print(f"[pi] 노드 v{VERSION}, 보행 {self.gait_name()}, 코드 {os.path.dirname(os.path.abspath(__file__))}", flush=True)
        if hasattr(self.gait, "stand_command"):
            import leg_model
            print(f"[pi] 캘리브레이션 표 {leg_model.TABLE_PATH}", flush=True)
            if not leg_model.USE_JOINT_POINTS:
                print("[pi] HEXAPOD_JOINT_POINTS=0 → 측정점 무시, pose_gui 보정만 씀", flush=True)
            sp = self.gait.stand_pose[0]
            print(f"[pi] 서기 자세 (실제 각도) hip {sp[0]:.0f} femur {sp[1]:.0f} tibia {sp[2]:.0f} — {self.gait.stand_note}, "
                  f"몸 높이 {-self.gait.ground:.0f}mm", flush=True)
            print(f"[pi] 몸 높이: 서기 {self.gait.stand_height:.0f}mm, 앉은 {self.gait.sit_height:.0f}mm, "
                  f"조절 범위 {self.gait.height_range[0]:.0f}~{self.gait.height_range[1]:.0f}mm", flush=True)
            print(f"[pi] 보폭 {self.gait.stride:.1f}mm, 직진 최고 {self.gait.max_speed():.3f} m/s", flush=True)
            warn = self.gait.walk_warning()
            if warn:
                print("[pi] ★ " + warn, flush=True)
            print("[pi] 서기 자세 (펌웨어로 보내는 각도): "
                  + " ".join(f"{l}:[{', '.join(f'{v:.1f}' for v in row)}]" for l, row in enumerate(self.gait.stand_command())),
                  flush=True)
        self.fw.connect()
        if not self.args.no_lidar:
            self.lidar.start()
        for fn in (self._control_loop, self._mapping_loop):
            threading.Thread(target=fn, daemon=True).start()
        try:
            self.server.run_forever()
        except KeyboardInterrupt:
            print("\n[pi] 종료")
        finally:
            self._stop.set()
            self.lidar.stop()
            self.fw.close()

    # ───────────── 제어 ─────────────
    def set_cmd_vel(self, vx, vy, wz):
        with self._lock:
            self.cmd = {
                "vx": max(-MAX_VX, min(MAX_VX, float(vx))),
                "vy": max(-MAX_VY, min(MAX_VY, float(vy))),
                "wz": max(-MAX_WZ, min(MAX_WZ, float(wz))),
                "t": time.time(),
            }

    def _check_safety(self, tel):
        issues = []
        if tel["temp_c"] is not None and tel["temp_c"] >= SAFE_TEMP_C:
            issues.append("over_temp")
        if "smoke" in tel["env_alarms"] or "flame" in tel["env_alarms"]:
            issues.append("fire_risk")
        if max(abs(tel["roll"]), abs(tel["pitch"])) > SAFE_TILT_DEG:
            issues.append("tilt")
        if not tel["alive"]:
            issues.append("firmware_lost")
        return issues

    def _control_loop(self):
        dt = 1.0 / CONTROL_HZ
        last = time.monotonic()
        while not self._stop.wait(max(0.0, dt - (time.monotonic() - last))):
            now = time.monotonic()
            step, last = now - last, now
            tel = self.fw.snapshot()
            self.safety = self._check_safety(tel)
            with self._lock:
                cmd = dict(self.cmd)
                if time.time() - cmd["t"] > DEADMAN_SEC:
                    cmd.update(vx=0.0, vy=0.0, wz=0.0)
                if "fire_risk" in self.safety or "tilt" in self.safety:
                    if self.mode == "walk":
                        self.fw._log(f"[pi] 안전 정지: {', '.join(self.safety)}")
                    cmd.update(vx=0.0, vy=0.0, wz=0.0)
                mode = self.mode
            if mode in ("idle", "estop"):
                continue
            moving = abs(cmd["vx"]) + abs(cmd["vy"]) + abs(cmd["wz"]) > 1e-3
            self.mode = "walk" if moving else "stand"
            self.fw.send_pose(self.gait.update(cmd["vx"], cmd["vy"], cmd["wz"], step))
            # 오도메트리: 실제 보폭 효율 70% 가정
            self.pose.integrate(cmd["vx"] * 0.7, cmd["vy"] * 0.7, cmd["wz"] * 0.8, step)
            if self.args.sim:                                  # 가상 방 밖으로 못 나가게
                self.pose.x = max(-3.6, min(3.6, self.pose.x))
                self.pose.y = max(-2.6, min(2.6, self.pose.y))

    def _mapping_loop(self):
        last_t = 0.0
        while not self._stop.wait(1.0 / MAPPING_HZ):
            t, pts = self.lidar.get_scan()
            if t and t != last_t:
                last_t = t
                self.grid.integrate_scan(pts, self.pose)

    # ───────────── PC 링크에서 호출 ─────────────
    def gait_name(self):
        return "ik+calib" if hasattr(self.gait, "stand_command") else "sine"

    def info(self):
        return {"robot": "Hyperism-HEXAPODv1", "version": VERSION, "simulated": bool(self.args.sim),
                "gait": self.gait_name(), "code": os.path.dirname(os.path.abspath(__file__))}

    def telemetry(self):
        with self._lock:
            cmd = {k: round(v, 3) for k, v in self.cmd.items() if k != "t"}
        return {
            "mode": self.mode,
            "safety": self.safety,
            "robot": self.fw.snapshot(),
            "pose": {"x": round(self.pose.x, 3), "y": round(self.pose.y, 3),
                     "yaw": round(math.degrees(self.pose.yaw), 1)},
            "cmd_vel": cmd,
            **({"gait": self.gait.status()} if hasattr(self.gait, "status") else {}),
            "lidar": self.lidar.status(),
            "pi": pi_health(),
        }

    def scan_world_xy(self, max_points):
        _, pts = self.lidar.get_scan()
        if pts is None or len(pts) == 0:
            return None
        band = pts[np.abs(pts[:, 2]) < 0.1]
        if len(band) > max_points:
            band = band[:: math.ceil(len(band) / max_points)]
        x, y, yaw = self.pose.as_tuple()
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + band[:, 0] * c - band[:, 1] * s
        wy = y + band[:, 0] * s + band[:, 1] * c
        return np.round(np.stack([wx, wy], axis=1), 3).tolist()

    def _stand_up(self):
        """일어서기: IK 보행이면 앉은 높이에서 발을 제자리로 옮긴 뒤 몸을 들어 올린다 (제어 루프가 진행)."""
        if not hasattr(self.gait, "stand_up"):
            self._gait_reset()
            self.fw.stand()
            return
        snap = self.fw.snapshot()
        cur = None
        if snap.get("powered"):                       # 서보가 켜져 있으면 마지막으로 보낸 자세에서 천천히 출발
            import leg_model
            cur = [[leg_model.from_firmware(l, j, snap["joints"][l][j]) for j in range(3)] for l in range(NUM_LEGS)]
        self.gait.stand_up(cur)

    def _gait_reset(self):
        """서기 자세에서 다시 시작 (IK 보행은 발 위치를 기억하므로 자세가 바뀌면 지워야 한다)."""
        if hasattr(self.gait, "reset"):
            self.gait.reset()

    def handle_command(self, msg):
        cmd = msg.get("cmd")
        if cmd == "walk":
            if self.mode in ("idle", "estop"):
                self._stand_up()
                self.mode = "stand"
            self.set_cmd_vel(msg.get("vx", 0), msg.get("vy", 0), msg.get("wz", 0))
        elif cmd == "stop":
            self.set_cmd_vel(0, 0, 0)
        elif cmd == "stand":
            self.mode = "stand"
            self._stand_up()
        elif cmd == "height":
            # {"cmd": "height", "mm": 90} (절대) 또는 {"cmd": "height", "delta_mm": 10} (상대). 몸 높이 = 바닥 → 몸통 원점
            if not hasattr(self.gait, "set_height"):
                raise ValueError("이 보행(--gait sine)은 몸 높이 조절이 없음")
            if "mm" in msg:
                target = self.gait.set_height(float(msg["mm"]))
            else:
                target = self.gait.change_height(float(msg.get("delta_mm", 0)))
            return {"height_target": round(target, 1)}
        elif cmd == "power_off":
            self.mode = "idle"
            self.set_cmd_vel(0, 0, 0)
            self._gait_reset()
            self.fw.power_off()
        elif cmd == "estop":
            self.mode = "estop"
            self.set_cmd_vel(0, 0, 0)
            self._gait_reset()
            self.fw.power_off()
        elif cmd == "joint":
            self.fw.send_joint(int(msg["leg"]), int(msg["joint"]), float(msg["deg"]))
        elif cmd == "imu_zero":
            self.fw.imu_zero()
        elif cmd == "lidar":
            (self.lidar.start_lidar_rotation if msg.get("on", True) else self.lidar.stop_lidar_rotation)()
        elif cmd == "map_reset":
            self.grid.reset()
            self.pose.x = self.pose.y = self.pose.yaw = 0.0
        elif cmd == "map_save":
            os.makedirs("maps", exist_ok=True)
            path = time.strftime("maps/map_%Y%m%d_%H%M%S.pgm")
            self.grid.save_pgm(path)
            return {"path": path}
        else:
            raise ValueError(f"모르는 명령: {cmd}")
        return None


def pi_health():
    """CPU 온도·부하 (라즈베리파이가 아니면 None)"""
    out = {"cpu_temp_c": None, "load": None}
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            out["cpu_temp_c"] = round(int(f.read()) / 1000, 1)
        out["load"] = round(os.getloadavg()[0], 2)
    except OSError:
        pass
    return out


def main():
    ap = argparse.ArgumentParser(description="헥사포드 라즈베리파이 노드")
    ap.add_argument("--port", default=os.environ.get("HEXAPOD_SERIAL", "/dev/ttyUSB0"), help="ESP32 시리얼 포트")
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--sim", action="store_true", help="ESP32·라이다 없이 시뮬레이션")
    ap.add_argument("--gait", choices=("ik", "sine"), default="ik",
                    help="ik = 역기구학 보행 (기본, ik_gait.py), sine = 예전 관절 사인파 보행")
    ap.add_argument("--no-lidar", action="store_true")
    RobotNode(ap.parse_args()).start()


if __name__ == "__main__":
    main()
