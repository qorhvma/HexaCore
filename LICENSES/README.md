# 라이선스 적용 범위

이 저장소는 파일 종류에 따라 라이선스가 다릅니다. 저작권자: 오연우, 박수용, 양서진 (HexaCore, 2026).

| 대상 | 라이선스 | 원문 |
|---|---|---|
| 코드 — `firmware/`, `software/`, `learning/`(아래 하드웨어 파일 제외), `tools/`, `legacy/` | MIT | [`../LICENSE`](../LICENSE) |
| 하드웨어 설계 — `hardware/` 전체, `learning/src/RLforHexapod/assets/`의 `urdf/`, `robot/`, `assets/`(자체 설계 부품) | CERN-OHL-P-2.0 | [`CERN-OHL-P-2.0.txt`](CERN-OHL-P-2.0.txt) |
| 데이터·문서·사진 — `data/`, `docs/`, README 등 문서의 이미지 | CC BY 4.0 | [`CC-BY-4.0.txt`](CC-BY-4.0.txt) |

## 예외

- **Isaac Lab 템플릿에서 만들어진 파일**은 원래 머리말(BSD-3-Clause, The Isaac Lab Project Developers)을 그대로 유지합니다.
  `learning/pyproject.toml`, `learning/.pre-commit-config.yaml`, `learning/src/RLforHexapod/__init__.py`,
  `learning/src/RLforHexapod/assets/__init__.py`, `learning/src/RLforHexapod/tasks/__init__.py`,
  `learning/scripts/list_envs.py`, `learning/tests/test_registration.py`
- **구매 부품 3D 모델**은 위 라이선스 대상이 아닙니다(출처 확인 중). 시뮬레이션 질량·형상 표현용으로만 들어 있습니다.
  `learning/src/RLforHexapod/assets/assets/`의 `mg996r.*`, `rc9mg.*`, `hw674.*`, `battery.*`, `motorhorn.*`
- 외부 라이브러리는 각자의 라이선스를 따릅니다. [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)를 보세요.

## 다시 쓸 때

- **코드(MIT)**: 저작권 표시와 라이선스 문구를 함께 남기면 상업적 사용·수정·재배포 모두 됩니다.
- **하드웨어(CERN-OHL-P)**: 수정·제작·판매 모두 됩니다. 설계 파일을 배포할 때 저작권·라이선스 표시를 유지해야 합니다. 수정본을 공개할 의무는 없습니다.
- **데이터·문서(CC BY 4.0)**: 출처(HexaCore, 저장소 주소)를 밝히면 자유롭게 쓸 수 있습니다.
