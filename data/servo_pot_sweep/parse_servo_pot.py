#!/usr/bin/env python3
"""
서보 스윕 시리얼 출력 파서 (Deg / Pot 쌍)

Arduino 출력 형식:
    Deg: 123
    Pot: 2048

보드에는 ServoPotSweep/ServoPotSweep.ino 를 올린다.
키를 누르면 시리얼로 시작 신호를 보내고, 보드가 "END"를 보내면 자동 종료한다.

사용법:
    # 시리얼 포트에서 실시간 수집
    python parse_servo_pot.py --port /dev/ttyUSB0 --out sweep.csv --plot

    # 저장해 둔 로그 파일 파싱
    python parse_servo_pot.py --file serial.log --out sweep.csv --plot
"""
import argparse
import csv
import re
import sys

DEG_RE = re.compile(r"^\s*Deg:\s*(-?\d+)")
POT_RE = re.compile(r"^\s*Pot:\s*(-?\d+)")
START_CMD = b"s\n"   # 보드는 어떤 바이트든 시작 신호로 받음
END_MARK = "END"     # 보드가 스윕을 끝내면 보내는 줄

# Arduino 코드의 상수와 맞춤 (펄스 값 역산용)
PULSE_MIN = 205 // 2   # 102
PULSE_MAX = 410 * 2    # 820


def deg_to_pulse(deg):
    """Arduino map(i, PULSE_MIN, PULSE_MAX, 0, 360)의 근사 역변환"""
    return PULSE_MIN + deg * (PULSE_MAX - PULSE_MIN) / 360.0


class Parser:
    """줄 단위로 입력받아 Deg/Pot 쌍이 완성되면 반환"""

    def __init__(self):
        self.pending_deg = None

    def feed(self, line):
        m = DEG_RE.match(line)
        if m:
            self.pending_deg = int(m.group(1))
            return None
        m = POT_RE.match(line)
        if m and self.pending_deg is not None:
            pair = (self.pending_deg, int(m.group(1)))
            self.pending_deg = None
            return pair
        return None


def wait_any_key(prompt):
    """Enter 없이 아무 키나 한 번 누를 때까지 대기"""
    print(prompt, file=sys.stderr, end="", flush=True)
    if not sys.stdin.isatty():
        sys.stdin.readline()
    elif sys.platform == "win32":
        import msvcrt
        msvcrt.getch()
    else:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print(file=sys.stderr)


def read_lines_serial(port, baud):
    import serial  # pip install pyserial

    # 포트를 열 때 보드가 리셋되지 않도록 DTR/RTS를 내린 상태로 연다
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 1
    ser.dtr = False
    ser.rts = False
    with ser:
        wait_any_key(f"[{port} @ {baud}] 아무 키나 누르면 측정을 시작합니다...")
        # 아무 바이트나 보내면 보드(ServoPotSweep.ino)가 스윕을 1회 실행
        ser.reset_input_buffer()
        ser.write(START_CMD)
        print("측정 시작 (END 수신 시 자동 종료, Ctrl+C로 중단)", file=sys.stderr)
        while True:
            raw = ser.readline()
            if raw:
                yield raw.decode("utf-8", errors="ignore").strip()


def read_lines_file(path):
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            yield line.strip()


def plot(rows):
    import matplotlib.pyplot as plt

    degs = [r[1] for r in rows]
    pots = [r[2] for r in rows]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(degs, pots, marker=".", linewidth=1)
    ax.set_xlabel("Deg (map 0~360)")
    ax.set_ylabel("Pot (ADC)")
    ax.set_title("Servo command vs. potentiometer")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Deg/Pot 시리얼 출력 파서")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="시리얼 포트 (예: /dev/ttyUSB0, COM3)")
    src.add_argument("--file", help="저장된 시리얼 로그 파일")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="sweep.csv", help="CSV 출력 경로")
    ap.add_argument("--plot", action="store_true", help="종료 후 그래프 표시")
    args = ap.parse_args()

    lines = read_lines_serial(args.port, args.baud) if args.port else read_lines_file(args.file)
    parser = Parser()
    rows = []

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "deg", "pot", "pulse_approx"])
        try:
            for line in lines:
                if line == END_MARK:
                    break
                pair = parser.feed(line)
                if pair is None:
                    continue
                deg, pot = pair
                row = (len(rows), deg, pot, round(deg_to_pulse(deg), 1))
                rows.append(row)
                writer.writerow(row)
                f.flush()
                print(f"{row[0]:4d}  deg={deg:4d}  pot={pot:5d}")
        except KeyboardInterrupt:
            pass

    print(f"\n{len(rows)}개 샘플 -> {args.out}", file=sys.stderr)
    if args.plot and rows:
        plot(rows)


if __name__ == "__main__":
    main()
