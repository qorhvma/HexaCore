# hardware

| 경로 | 내용 | 상태 |
|---|---|---|
| `cad/onshape-to-robot.json` | Onshape 문서 → URDF 변환 설정 ([onshape-to-robot](https://github.com/Rhoban/onshape-to-robot)), 관절별 최대 토크 | 있음 |
| `pcb/gerber/Gerber_PCB1.zip` | 자체 설계 원형 PCB 거버 (2층, 드릴 파일 포함). PCB 업체에 이 zip을 그대로 올리면 됨 | 있음 |
| `pcb/` 원본·회로도 PDF | EasyEDA 원본, 회로도 | 예정 |
| `bom/` | 부품 목록 (가격·구매처) | 예정 |
| `cad/stl/` | 3D 프린팅용 STL | 예정 (지금은 [`learning/.../assets/assets/`](../learning/src/RLforHexapod/assets/assets)의 시뮬용 메시) |

URDF는 [`learning/src/RLforHexapod/assets/urdf/robot.urdf`](../learning/src/RLforHexapod/assets/urdf/robot.urdf)에 있습니다.
