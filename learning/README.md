# learning — Isaac Lab 3.0 강화학습 환경

Isaac Lab 3.0 + Newton 백엔드(Isaac Sim 불필요), RSL-RL. teacher(특권 관측) 학습 후 student(실물 센서로 얻을 수 있는 관측)로 증류합니다.

## 등록된 task

| task | 내용 |
|---|---|
| `Hexapod-Walk` | 평지 보행 |
| `Hexapod-Stair` | 계단 teacher |
| `Hexapod-Stair-Student-A` / `-B` / `-A-Phase` | student 증류 (관측 구성별, [docs/student_observation_test.md](docs/student_observation_test.md)) |

계단 형상은 `src/RLforHexapod/tasks/walk/mdp/observations.py`의 `StairsGeometryCfg`와 `assets/obstacles/stairs_*mm.usda`가 함께 정합니다.

## 주요 스크립트

| 스크립트 | 내용 |
|---|---|
| `scripts/eval_stairs.py` | 저장된 정책을 계단 앞에서 출발시켜 꼭대기 도착 비율 측정 (teacher·student 같은 조건) |
| `scripts/make_stairs.py` | 계단 USD 생성 |
| `scripts/convert_urdf.py` | URDF → USD 변환 |
| `scripts/set_joint_limits.py` | USD 관절 한계만 수정 |

```bash
uv run python scripts/eval_stairs.py --task Hexapod-Stair --checkpoint <teacher.pt>
```

설치·학습 명령과 결과는 정리 중입니다.
