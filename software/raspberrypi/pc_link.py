"""
라즈베리파이 ↔ 컴퓨터 통신 (WebSocket, JSON)

    ws://<라즈베리파이 IP>:8765
    PC 조종 프로그램(software/pc/pc_client.py)과 대시보드(software/dashboard/index.html)가 같은 주소로 붙는다.

Pi → PC
    {"type": "hello",     "robot": ..., "version": ..., "rates": {...}}
    {"type": "telemetry", "t": ..., "robot": {...}, "pose": {...}, "cmd_vel": {...}, "lidar": {...}}   10Hz
    {"type": "scan",      "t": ..., "points": [[x, y], ...]}      월드 좌표, 5Hz, 최대 720점
    {"type": "map",       ...OccupancyGrid.export()}             1Hz
    {"type": "log",       "t": ..., "line": "..."}
    {"type": "ack",       "cmd": ..., "ok": true/false, "error": ...}

PC → Pi
    {"cmd": "auth", "token": "..."}                    HEXAPOD_TOKEN 환경변수를 설정했을 때만 필요
    {"cmd": "walk", "vx": 0.1, "vy": 0, "wz": 0}       m/s, rad/s — 0.5초 안에 다시 안 오면 자동 정지 (데드맨)
    {"cmd": "stop"} | {"cmd": "stand"} | {"cmd": "power_off"}     stand = 일어서기 (앉은 높이 → 서기 높이로 몸을 듦)
    {"cmd": "height", "mm": 90} | {"cmd": "height", "delta_mm": 10}  몸 높이 [mm] (바닥 → 몸통 원점), IK 보행만
    {"cmd": "joint", "leg": 0, "joint": 1, "deg": 135}
    {"cmd": "imu_zero"}
    {"cmd": "lidar", "on": true}
    {"cmd": "map_reset"} | {"cmd": "map_save"}
    {"cmd": "ping", "t": ...}                          → {"type": "pong", "t": ...}
"""

import asyncio
import json
import os
import time

try:
    import websockets
except ImportError:
    websockets = None


TELEMETRY_HZ = 10
SCAN_HZ = 5
MAP_HZ = 1
MAX_SCAN_POINTS = 720


class PcServer:
    def __init__(self, node, host="0.0.0.0", port=8765, token=None):
        self.node = node
        self.host, self.port = host, port
        self.token = token if token is not None else os.environ.get("HEXAPOD_TOKEN")
        self.clients = set()
        self._authed = set()
        self._loop = None

    # ───────────── 외부(다른 스레드)에서 호출 ─────────────
    def push_log(self, line):
        if self._loop:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"type": "log", "t": time.time(), "line": line}), self._loop)

    # ───────────── 실행 ─────────────
    def run_forever(self):
        if websockets is None:
            raise RuntimeError("pip install websockets 필요")
        asyncio.run(self._main())

    async def _main(self):
        self._loop = asyncio.get_running_loop()
        async with websockets.serve(self._handler, self.host, self.port, max_size=2**20,
                                    ping_interval=5, ping_timeout=5):
            print(f"[pc] WebSocket 서버 ws://{self.host}:{self.port}")
            await asyncio.gather(
                self._periodic(TELEMETRY_HZ, self._telemetry_msg),
                self._periodic(SCAN_HZ, self._scan_msg),
                self._periodic(MAP_HZ, self._map_msg),
            )

    async def _periodic(self, hz, make_msg):
        period = 1.0 / hz
        while True:
            start = time.monotonic()
            if self.clients:
                msg = await asyncio.to_thread(make_msg)
                if msg:
                    await self._broadcast(msg)
            await asyncio.sleep(max(0.0, period - (time.monotonic() - start)))

    async def _broadcast(self, msg):
        if not self.clients:
            return
        data = json.dumps(msg, separators=(",", ":"), allow_nan=False, default=_json_default)
        targets = [c for c in self.clients if not self.token or c in self._authed]
        await asyncio.gather(*(self._safe_send(c, data) for c in targets))

    @staticmethod
    async def _safe_send(ws, data):
        try:
            await ws.send(data)
        except Exception:  # noqa: BLE001 — 끊긴 클라이언트는 handler 에서 정리됨
            pass

    async def _handler(self, ws, path=None):
        peer = getattr(ws, "remote_address", ("?", 0))
        self.clients.add(ws)
        print(f"[pc] 접속: {peer[0]}  (총 {len(self.clients)})")
        try:
            await ws.send(json.dumps({"type": "hello", **self.node.info(),
                                      "auth_required": bool(self.token),
                                      "rates": {"telemetry": TELEMETRY_HZ, "scan": SCAN_HZ, "map": MAP_HZ}}))
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    await ws.send(json.dumps({"type": "ack", "ok": False, "error": "bad json"}))
                    continue
                await self._dispatch(ws, msg)
        except Exception as e:  # noqa: BLE001
            print(f"[pc] 연결 오류 {peer[0]}: {e}")
        finally:
            self.clients.discard(ws)
            self._authed.discard(ws)
            print(f"[pc] 끊김: {peer[0]}  (총 {len(self.clients)})")
            if not self.clients:
                self.node.set_cmd_vel(0, 0, 0)          # 마지막 조종자가 나가면 정지

    async def _dispatch(self, ws, msg):
        cmd = msg.get("cmd")
        if cmd == "ping":
            await ws.send(json.dumps({"type": "pong", "t": msg.get("t")}))
            return
        if cmd == "auth":
            ok = not self.token or msg.get("token") == self.token
            if ok:
                self._authed.add(ws)
            await ws.send(json.dumps({"type": "ack", "cmd": "auth", "ok": ok}))
            return
        if self.token and ws not in self._authed:
            await ws.send(json.dumps({"type": "ack", "cmd": cmd, "ok": False, "error": "unauthorized"}))
            return
        try:
            result = await asyncio.to_thread(self.node.handle_command, msg)
            if cmd != "walk":                             # walk 는 초당 20번이라 ack 생략
                await ws.send(json.dumps({"type": "ack", "cmd": cmd, "ok": True, "result": result},
                                         default=_json_default))
        except Exception as e:  # noqa: BLE001
            await ws.send(json.dumps({"type": "ack", "cmd": cmd, "ok": False, "error": str(e)}))

    # ───────────── 메시지 만들기 (스레드에서 실행) ─────────────
    def _telemetry_msg(self):
        return {"type": "telemetry", "t": time.time(), **self.node.telemetry()}

    def _scan_msg(self):
        pts = self.node.scan_world_xy(MAX_SCAN_POINTS)
        return {"type": "scan", "t": time.time(), "points": pts} if pts is not None else None

    def _map_msg(self):
        return {"type": "map", "t": time.time(), **self.node.grid.export()}


def _json_default(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    raise TypeError(f"JSON 변환 불가: {type(o)}")
