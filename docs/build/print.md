# 2. 3D 프린팅

STL 파일: [`hardware/cad/stl/`](repo:hardware/cad/stl/) (mm 단위). CAD 원본은 [Onshape](https://cad.onshape.com/documents/f646c0fde8bd91cfa43cbae7/w/279609403375cff4404c6eae/e/76a6985095c8e8f19d67aec6)에서 로그인 없이 보고 STEP·STL로 내보낼 수 있습니다.

| 부품 | 수량 | 크기 X×Y×Z (mm) | 꽉 찬 부피 1개 (cm³) |
|---|---|---|---|
| `mainbody.stl` (몸통 판) | 1 | 234.0 × 228.5 × 40.0 | 183.0 |
| `bodytop.stl` (몸통 덮개) | 1 | 133.6 × 99.9 × 16.0 | 10.6 |
| `batterycover.stl` (배터리 덮개) | 1 | 45.0 × 97.4 × 20.3 | 15.4 |
| `coxajoint.stl` | 6 | 61.9 × 58.0 × 31.3 | 16.0 |
| `femur.stl` | 6 | 113.0 × 59.4 × 13.0 | 22.5 |
| `tibia.stl` | 6 | 31.0 × 154.9 × 31.3 | 25.7 |
| `tibiatip1.stl` | 6 | 18.0 × 18.0 × 20.0 | 3.1 |
| `tibiatip2.stl` | 6 | 18.0 × 18.0 × 10.1 | 1.6 |
| `tibiatip3.stl` | 6 | 18.0 × 15.0 × 18.0 | 2.0 |
| `jointcover.stl` | 12 | 13.0 × 13.0 × 12.4 | 0.5 |

- 모두 51개입니다. 꽉 찬 부피 합은 약 640 cm³로, PLA를 100% 채우면 약 0.8 kg입니다. 실제 필라멘트는 채움률만큼 적게 듭니다.
- 다리 부품(coxajoint·femur·tibia·tibiatip 1~3)은 다리 하나에 하나씩, 6벌입니다. `jointcover`는 다리마다 2개입니다.
- STL에는 출력 방향이 정해져 있지 않습니다. 슬라이서에서 넓은 면이 바닥에 오게 돌려 주세요.

!!! note "아직 정리하지 않은 것"
    재질·채움률·레이어 높이 같은 출력 설정은 팀이 쓴 값을 정리하고 있습니다.
