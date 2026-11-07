"""시뮬용 메시(미터 단위)를 3D 프린팅용 STL(밀리미터 단위)로 바꾼다.

onshape-to-robot이 내보낸 learning/.../assets/assets/*.stl 은 미터 단위라
슬라이서에 그대로 넣으면 1000배 작게 보인다. 프린팅 부품만 골라 1000배 키워서
hardware/cad/stl/ 에 저장하고, 부품별 크기·부피·수량 표를 출력한다.

    python3 tools/cad/export_print_stl.py
"""
from __future__ import annotations

import re
import struct
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "learning/src/RLforHexapod/assets/assets"
URDF = ROOT / "learning/src/RLforHexapod/assets/urdf/robot.urdf"
DST = ROOT / "hardware/cad/stl"

# 직접 프린팅하는 부품. 서보·배터리·서보혼·컨버터는 구매 부품이라 뺀다.
PRINTED = [
    "mainbody", "bodytop", "batterycover",
    "coxajoint", "femur", "tibia",
    "tibiatip1", "tibiatip2", "tibiatip3", "jointcover",
]

DTYPE = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    return np.frombuffer(data[84:84 + n * 50], dtype=DTYPE).copy()


def write_stl(path: Path, tris: np.ndarray, name: str) -> None:
    header = f"HexaCore {name} (mm)".encode()[:80].ljust(80, b" ")
    path.write_bytes(header + struct.pack("<I", len(tris)) + tris.tobytes())


def volume_mm3(v: np.ndarray) -> float:
    a, b, c = v[:, 0], v[:, 1], v[:, 2]
    return abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0


def open_edges(v: np.ndarray) -> int:
    """삼각형 하나에만 붙은 모서리(구멍) 수. 0이면 물이 새지 않는 메시."""
    pts = np.round(v.reshape(-1, 3), 4)
    _, idx = np.unique(pts, axis=0, return_inverse=True)
    f = idx.reshape(-1, 3)
    e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _, counts = np.unique(e, axis=0, return_counts=True)
    return int((counts == 1).sum())


def main() -> None:
    # URDF는 visual·collision에 같은 메시를 한 번씩 쓰므로 2로 나눈다.
    uses = Counter(re.findall(r'filename="\.\./assets/(\w+)\.stl"', URDF.read_text()))
    DST.mkdir(parents=True, exist_ok=True)
    print("| 부품 | 수량 | 크기 X×Y×Z (mm) | 부피 1개 (cm³) | 열린 모서리 |")
    print("|---|---|---|---|---|")
    for name in PRINTED:
        tris = read_stl(SRC / f"{name}.stl")
        tris["v"] *= 1000.0
        v = tris["v"].reshape(-1, 3, 3)
        v = v - v.reshape(-1, 3).min(0)          # 바닥 모서리를 원점으로
        tris["v"] = v.reshape(-1, 9)
        write_stl(DST / f"{name}.stl", tris, name)
        ext = v.reshape(-1, 3).max(0)
        qty = uses[name] // 2
        print(f"| `{name}.stl` | {qty} | {ext[0]:.1f} × {ext[1]:.1f} × {ext[2]:.1f} "
              f"| {volume_mm3(v) / 1000:.1f} | {open_edges(v)} |")


if __name__ == "__main__":
    main()
