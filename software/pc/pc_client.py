"""
컴퓨터 → 라즈베리파이 조종 클라이언트 (터미널)

    python3 pc_client.py --host 192.168.0.42
    python3 pc_client.py --host localhost        # Pi 노드를 --sim 으로 PC 에서 돌릴 때

키 (엔터 없이 바로)
    w/s : 앞/뒤      a/d : 왼쪽/오른쪽 옆걸음      q/e : 제자리 회전
    space : 정지     t : 서기      o : 힘 빼기      x : 비상정지
    z : IMU 수평 기억              l : 라이다 켜기/끄기
    r : 지도 초기화                m : 지도 저장 (Pi 의 maps/)
    Ctrl+C : 종료
"""

import argparse
import asyncio
import json
import os
import sys
import time

try:
    import websockets
except ImportError:
    sys.exit("pip install websockets")

WALK_HZ = 20
SPEED = {"vx": 0.10, "vy": 0.06, "wz": 0.5}
KEYMAP = {
    "w": (1, 0, 0), "s": (-1, 0, 0),
    "a": (0, 1, 0), "d": (0, -1, 0),
    "q": (0, 0, 1), "e": (0, 0, -1),
}
HOLD_SEC = 0.35          # 키를 이만큼 안 누르면 멈춤 (터미널 키 반복 간격보다 길게)


class KeyReader:
    """터미널 raw 모드 한 글자 입력 (리눅스/맥). 윈도우는 msvcrt."""

    def __enter__(self):
        if os.name == "nt":
            return self
        import termios
        import tty
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        if os.name != "nt":
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read(self):
        if os.name == "nt":
            import msvcrt
            return msvcrt.getwch() if msvcrt.kbhit() else None
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None


class Client:
    def __init__(self, url, token=None):
        self.url, self.token = url, token
        self.vel = (0.0, 0.0, 0.0)
        self.vel_t = 0.0
        self.lidar_on = True
        self.last_tel = {}
        self.rtt_ms = None

    async def run(self):
        async for ws in websockets.connect(self.url, ping_interval=5, max_size=2**22):
            try:
                print(f"\r[pc] 연결됨 {self.url}")
                if self.token:
                    await ws.send(json.dumps({"cmd": "auth", "token": self.token}))
                await asyncio.gather(self._rx(ws), self._tx(ws), self._status())
            except websockets.ConnectionClosed:
                print("\r[pc] 연결 끊김 — 재접속 중...")
                await asyncio.sleep(1.0)

    async def _rx(self, ws):
        async for raw in ws:
            msg = json.loads(raw)
            typ = msg.get("type")
            if typ == "telemetry":
                self.last_tel = msg
            elif typ == "log":
                print(f"\r{msg['line']}")
            elif typ == "hello":
                print(f"\r[pi] {msg['robot']} v{msg['version']}  sim={msg.get('simulated')}")
            elif typ == "pong":
                self.rtt_ms = (time.time() - msg["t"]) * 1000
            elif typ == "ack" and not msg.get("ok"):
                print(f"\r[pi] 명령 실패 {msg.get('cmd')}: {msg.get('error')}")
            elif typ == "ack" and msg.get("result"):
                print(f"\r[pi] {msg['cmd']}: {msg['result']}")

    async def _tx(self, ws):
        with KeyReader() as kb:
            ping_t = 0.0
            while True:
                k = kb.read()
                while k:
                    await self._key(ws, k.lower())
                    k = kb.read()
                if time.time() - self.vel_t > HOLD_SEC:
                    self.vel = (0.0, 0.0, 0.0)
                vx, vy, wz = self.vel
                if any(self.vel):
                    await ws.send(json.dumps({"cmd": "walk", "vx": vx, "vy": vy, "wz": wz}))
                if time.time() - ping_t > 2.0:
                    ping_t = time.time()
                    await ws.send(json.dumps({"cmd": "ping", "t": ping_t}))
                await asyncio.sleep(1.0 / WALK_HZ)

    async def _key(self, ws, k):
        send = lambda **m: ws.send(json.dumps(m))  # noqa: E731
        if k in KEYMAP:
            fx, fy, fw = KEYMAP[k]
            self.vel = (fx * SPEED["vx"], fy * SPEED["vy"], fw * SPEED["wz"])
            self.vel_t = time.time()
        elif k == " ":
            self.vel = (0.0, 0.0, 0.0)
            await send(cmd="stop")
        elif k == "t":
            await send(cmd="stand")
        elif k == "o":
            await send(cmd="power_off")
        elif k == "x":
            await send(cmd="estop")
            print("\r[pc] ★ 비상정지")
        elif k == "z":
            await send(cmd="imu_zero")
        elif k == "l":
            self.lidar_on = not self.lidar_on
            await send(cmd="lidar", on=self.lidar_on)
        elif k == "r":
            await send(cmd="map_reset")
        elif k == "m":
            await send(cmd="map_save")

    async def _status(self):
        while True:
            await asyncio.sleep(0.5)
            t = self.last_tel
            if not t:
                continue
            r, p = t["robot"], t["pose"]
            temp = "--" if r["temp_c"] is None else f"{r['temp_c']:.1f}"
            feet = "".join("●" if f else "○" for f in r["foot"])
            rtt = f"{self.rtt_ms:.0f}ms" if self.rtt_ms is not None else "-"
            warn = f"  ⚠ {','.join(t['safety'])}" if t["safety"] else ""
            sys.stdout.write(
                f"\r[{t['mode']:>5}] x={p['x']:+.2f} y={p['y']:+.2f} yaw={p['yaw']:+6.1f}° "
                f"R={r['roll']:+5.1f} P={r['pitch']:+5.1f} 발={feet} "
                f"온도={temp}°C 연기={r['smoke_ppm']:.0f}ppm 라이다={t['lidar']['scan_hz']}Hz rtt={rtt}{warn}   ")
            sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="헥사포드 PC 조종 클라이언트")
    ap.add_argument("--host", default="raspberrypi.local")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--token", default=os.environ.get("HEXAPOD_TOKEN"))
    args = ap.parse_args()
    print(__doc__)
    try:
        asyncio.run(Client(f"ws://{args.host}:{args.port}", args.token).run())
    except KeyboardInterrupt:
        print("\n[pc] 종료")


if __name__ == "__main__":
    main()
