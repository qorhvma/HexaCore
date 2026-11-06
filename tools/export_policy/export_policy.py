# /// script
# requires-python = ">=3.10,<3.13"
# dependencies = ["torch", "onnx", "onnxruntime", "numpy"]
#
# [tool.uv.sources]
# torch = { index = "pytorch-cpu" }
#
# [[tool.uv.index]]
# name = "pytorch-cpu"
# url = "https://download.pytorch.org/whl/cpu"
# explicit = true
# ///
"""RSL-RL 체크포인트(model_N.pt)에서 정책(actor)만 꺼내 ONNX 파일로 내보낸다.

사용법:
    uv run export_policy.py models/flat/model_750.pt

결과:
    같은 폴더에 policy.onnx 가 생기고, PyTorch와 ONNX의 출력이 같은지 검사한다.
GPU나 Isaac Sim 없이 CPU만으로 동작한다.
"""

import argparse
import json
import math
import warnings
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn

# rsl_rl EmpiricalNormalization 의 기본값: (x - mean) / (std + 0.01)
NORMALIZER_EPS = 1e-2
# 단순 MLP 라서 기존(TorchScript) ONNX 내보내기로 충분하다. 관련 안내 경고는 숨긴다.
warnings.filterwarnings("ignore", message=".*legacy TorchScript-based ONNX export.*")
ACTIVATIONS = {"elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh, "selu": nn.SELU}


def read_activation(run_dir: Path) -> str:
    """params/agent.yaml 에서 활성화 함수 이름을 읽는다. 없으면 elu."""
    agent_yaml = run_dir / "params" / "agent.yaml"
    if agent_yaml.exists():
        for line in agent_yaml.read_text().splitlines():
            if line.strip().startswith("activation:"):
                return line.split(":", 1)[1].strip()
    return "elu"


def build_actor(state_dict: dict, activation: str) -> nn.Sequential:
    """가중치 모양을 보고 actor MLP를 다시 만든다 (층 크기를 하드코딩하지 않음)."""
    weights = sorted(
        (int(k.split(".")[1]), v) for k, v in state_dict.items()
        if k.startswith("actor.") and k.endswith(".weight")
    )
    layers = []
    for i, (_, w) in enumerate(weights):
        layers.append(nn.Linear(w.shape[1], w.shape[0]))
        if i < len(weights) - 1:
            layers.append(ACTIVATIONS[activation]())
    actor = nn.Sequential(*layers)
    actor.load_state_dict({k[len("actor."):]: v for k, v in state_dict.items() if k.startswith("actor.")})
    return actor


class Policy(nn.Module):
    """관측 정규화 + actor 를 하나로 묶은 추론 전용 모듈."""

    def __init__(self, actor: nn.Module, mean: torch.Tensor, std: torch.Tensor):
        super().__init__()
        self.actor = actor
        self.register_buffer("obs_mean", mean.reshape(1, -1))
        self.register_buffer("obs_std", std.reshape(1, -1))

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.actor((obs - self.obs_mean) / (self.obs_std + NORMALIZER_EPS))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("checkpoint", type=Path, help="model_N.pt 경로")
    parser.add_argument("--output", type=Path, help="저장할 .onnx 경로 (기본: 체크포인트 폴더/policy.onnx)")
    args = parser.parse_args()

    ckpt_path = args.checkpoint.resolve()
    run_dir = ckpt_path.parent
    output = (args.output or run_dir / "policy.onnx").resolve()

    # 팀이 직접 학습한 파일이므로 weights_only=False 로 연다 (출처를 모르는 .pt 에는 쓰지 말 것).
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    state_dict = ckpt["model_state_dict"]
    norm = ckpt.get("obs_norm_state_dict")
    activation = read_activation(run_dir)

    actor = build_actor(state_dict, activation)
    obs_dim = actor[0].in_features
    act_dim = actor[-1].out_features
    if norm is None:
        print("경고: 관측 정규화 값이 없어 정규화 없이 내보냅니다.")
        mean, std = torch.zeros(obs_dim), torch.ones(obs_dim) - NORMALIZER_EPS
    else:
        mean, std = norm["_mean"].float(), norm["_std"].float()

    policy = Policy(actor, mean, std).eval()
    print(f"체크포인트 : {ckpt_path.name} (학습 반복 {ckpt.get('iter')})")
    print(f"신경망     : {' -> '.join(str(m.in_features) for m in actor if isinstance(m, nn.Linear))} -> {act_dim}, {activation}")

    torch.onnx.export(
        policy, torch.zeros(1, obs_dim), output,
        input_names=["obs"], output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=17, dynamo=False,
    )
    print(f"저장 완료  : {output}")

    # 검사: 같은 입력에 PyTorch 와 ONNX 가 같은 출력을 내는지 확인
    rng = np.random.default_rng(0)
    test_obs = (mean.numpy() + rng.standard_normal((256, obs_dim)) * std.numpy()).astype(np.float32)
    with torch.no_grad():
        expected = policy(torch.from_numpy(test_obs)).numpy()
    session = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
    got = session.run(None, {"obs": test_obs})[0]
    max_diff = float(np.abs(expected - got).max())
    print(f"검사       : PyTorch vs ONNX 최대 차이 {max_diff:.2e} -> {'통과' if max_diff < 1e-4 else '실패'}")

    # 예시: 평균 관측을 넣었을 때 정책이 고르는 관절 목표각 (필터·변화율 제한 적용 전)
    metadata_path = run_dir / "run_metadata.json"
    if metadata_path.exists():
        meta = json.loads(metadata_path.read_text())
        names, limits = meta["joint_names"], meta["angle_limits_rad"]
        action = session.run(None, {"obs": mean.numpy().reshape(1, -1)})[0][0]
        print("\n예시 출력 (평균 관측 입력, 필터·변화율 제한 전 목표각)")
        for name, a, (lo, hi) in zip(names, action, limits):
            target = lo + (min(max(float(a), -1.0), 1.0) + 1.0) / 2.0 * (hi - lo)
            print(f"  {name:20s} action={a:+.3f}  목표각={math.degrees(target):+7.2f} deg")


if __name__ == "__main__":
    main()
