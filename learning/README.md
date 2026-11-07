# learning — Isaac Lab 3.0 강화학습 환경

Isaac Lab 3.0 + Newton 백엔드(Isaac Sim 불필요), RSL-RL. teacher(특권 관측) 학습 후 student(실물 센서로 얻을 수 있는 관측)로 증류합니다.

## 등록된 task

| task | 내용 |
|---|---|
| `Hexapod-Walk` | 평지 보행 |
| `Hexapod-Stair` | 계단 teacher |
| `Hexapod-Stair-Student-A` / `-B` / `-A-Phase` | student 증류 (관측 구성별, [docs/student_observation_test.md](docs/student_observation_test.md)) |
| `Hexapod-Stair-Student-A-TopEnd` / `-A-Mem128` / `-A-Mem256` | student 기억 길이 시험 |

계단 형상은 `src/RLforHexapod/tasks/walk/mdp/observations.py`의 `StairsGeometryCfg`와 `assets/obstacles/stairs_*mm.usda`가 함께 정합니다.
지금 계단: 단높이 160 mm × 3단 + 꼭대기, 단너비 300 mm (경사 28.1°).

## 다시 해 보기

NVIDIA GPU가 있는 Linux 또는 Windows, [uv](https://docs.astral.sh/uv/)가 필요합니다. 명령은 모두 이 `learning/` 폴더에서 실행합니다.
`--viz`를 주지 않으면 화면 없이 돕니다.

### 1. 설치

```bash
uv sync
```

### 2. teacher 학습

```bash
uv run isaaclab train --rl_library rsl_rl --task Hexapod-Stair --num_envs 4096
```

체크포인트는 `logs/rsl_rl/hexapod_walk/<날짜_시각>/model_*.pt`에 쌓입니다 (기본 800 iteration).

### 3. student 증류

teacher 체크포인트를 `--checkpoint`로 넘깁니다.

```bash
uv run isaaclab train --rl_library rsl_rl --task Hexapod-Stair-Student-A --num_envs 4096 \
    --checkpoint logs/rsl_rl/hexapod_walk/<teacher 폴더>/model_799.pt
```

결과는 `logs/rsl_rl/hexapod_stair_student/<날짜_시각>_A/`에 쌓입니다 (기본 1500 iteration).

### 4. 평가 (고정 시드, k/N)

[`scripts/eval_stairs.py`](scripts/eval_stairs.py)는 학습하지 않고 정책만 돌려서 꼭대기에 도달한 로봇 수를 셉니다.

- 환경 하나가 시행 하나입니다. `--num_envs`가 곧 N입니다.
- 환경마다 첫 에피소드만 셉니다. 시작 위치는 0번(계단 앞)과 1번(앞발이 첫 수직면 앞) 둘뿐입니다.
- 행동 노이즈는 끄고 평균 행동을 씁니다. student의 높이맵 노이즈는 그대로 둡니다.

```bash
uv run python scripts/eval_stairs.py --task Hexapod-Stair \
    --checkpoint <teacher.pt> --num_envs 1000 --seed 42 --out results/teacher.json
uv run python scripts/eval_stairs.py --task Hexapod-Stair-Student-A \
    --checkpoint <student.pt> --num_envs 1000 --seed 42 --out results/student_A.json
```

출력 형식:

```text
[전체] N대
  꼭대기 도달: k/N = 비율 (95% 신뢰구간 ±반폭)
  정지 성공:   k/N = 비율
```

전체 외에 시작 위치별(0번·1번) 결과도 따로 나옵니다.

`--out` JSON에는 task, 체크포인트 파일의 sha256, 시드, N이 결과와 함께 남습니다.

### 5. ONNX로 내보내기

`play`가 정책을 불러오면서 체크포인트 폴더 아래 `exported/policy.onnx`와 `exported/policy.pt`를 만듭니다.

```bash
uv run isaaclab play --rl_library rsl_rl --task Hexapod-Stair-Student-A --num_envs 1 --checkpoint <student.pt>
```

## 주요 스크립트

| 스크립트 | 내용 |
|---|---|
| `scripts/eval_stairs.py` | 고정 시드 계단 평가 (k/N, JSON 저장) |
| `scripts/make_stairs.py` | 계단 USD 생성 |
| `scripts/convert_urdf.py` | URDF → USD 변환 (Isaac Sim 필요) |
| `scripts/set_joint_limits.py` | USD 관절 한계만 수정 |
| `scripts/list_envs.py` | 등록된 task 목록 |
