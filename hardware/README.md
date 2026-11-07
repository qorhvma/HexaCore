# hardware

HexaCore를 다시 만드는 데 필요한 설계 자료입니다. 라이선스는 [CERN-OHL-P-2.0](LICENSE)입니다.

| 경로 | 내용 |
|---|---|
| [`bom/BOM.csv`](bom/BOM.csv) | 부품 목록 (수량·사양·가격·확인 상태) |
| [`cad/stl/`](cad/stl/) | 3D 프린팅용 STL 10종 (mm 단위) |
| [`cad/onshape-to-robot.json`](cad/onshape-to-robot.json) | Onshape 문서 → URDF 변환 설정, 관절별 최대 토크·속도·범위 |
| [`pcb/`](pcb/) | 제어 PCB 거버, 핀 배치 |

URDF는 [`learning/src/RLforHexapod/assets/urdf/robot.urdf`](../learning/src/RLforHexapod/assets/urdf/robot.urdf)에 있습니다.

## CAD 원본 (Onshape)

[HEXAPOD — Onshape](https://cad.onshape.com/documents/f646c0fde8bd91cfa43cbae7/w/279609403375cff4404c6eae/e/76a6985095c8e8f19d67aec6)

- 링크가 있으면 로그인 없이 보기·내보내기(STEP, STL)가 됩니다. Onshape 공개 검색에는 나오지 않습니다.
- 위 링크는 작업 중인 workspace라 내용이 바뀔 수 있습니다. 바뀌지 않는 판이 필요하면 Onshape의 *Versions*에서 판을 골라 쓰세요.

## 3D 프린팅 부품

| 부품 | 수량 | 크기 X×Y×Z (mm) | 꽉 찬 부피 1개 (cm³) |
|---|---|---|---|
| [`mainbody.stl`](cad/stl/mainbody.stl) | 1 | 234.0 × 228.5 × 40.0 | 183.0 |
| [`bodytop.stl`](cad/stl/bodytop.stl) | 1 | 133.6 × 99.9 × 16.0 | 10.6 |
| [`batterycover.stl`](cad/stl/batterycover.stl) | 1 | 45.0 × 97.4 × 20.3 | 15.4 |
| [`coxajoint.stl`](cad/stl/coxajoint.stl) | 6 | 61.9 × 58.0 × 31.3 | 16.0 |
| [`femur.stl`](cad/stl/femur.stl) | 6 | 113.0 × 59.4 × 13.0 | 22.5 |
| [`tibia.stl`](cad/stl/tibia.stl) | 6 | 31.0 × 154.9 × 31.3 | 25.7 |
| [`tibiatip1.stl`](cad/stl/tibiatip1.stl) | 6 | 18.0 × 18.0 × 20.0 | 3.1 |
| [`tibiatip2.stl`](cad/stl/tibiatip2.stl) | 6 | 18.0 × 18.0 × 10.1 | 1.6 |
| [`tibiatip3.stl`](cad/stl/tibiatip3.stl) | 6 | 18.0 × 15.0 × 18.0 | 2.0 |
| [`jointcover.stl`](cad/stl/jointcover.stl) | 12 | 13.0 × 13.0 × 12.4 | 0.5 |

- 모두 51개, 꽉 찬 부피 합 약 640 cm³ (PLA 100% 채움이면 약 0.8 kg, 실제 필라멘트는 채움률만큼 적게 듭니다).
- `mainbody`는 234 × 229 mm라 **출력 면적이 235 × 230 mm 이상인 프린터**가 필요합니다.
- STL은 시뮬용 메시를 mm로 바꾼 것이라 출력 방향은 정해져 있지 않습니다. 슬라이서에서 넓은 면이 바닥에 오게 돌려 주세요.
- 모든 메시는 열린 모서리가 0개입니다. `mainbody`·`bodytop`에는 면이 맞닿는 모서리가 몇 개 있는데, 슬라이서가 그대로 처리합니다.
- 다시 만들기: `python3 tools/cad/export_print_stl.py` ([스크립트](../tools/cad/export_print_stl.py))

## 아직 없는 것

- 볼트·너트·배선 목록, 일부 부품 가격 ([BOM](bom/BOM.csv)의 *상태* 열)
- PCB 원본(EasyEDA Pro)과 회로도 PDF
- 조립 순서·사진 (빌드 가이드에서 다룰 예정)
- 서보·배터리·서보 혼·컨버터 메시(`learning/.../assets/assets/`의 `mg996r`, `rc9mg`, `motorhorn`, `hw674`, `battery`)는 시뮬레이션용이고 출처를 확인하고 있습니다.
