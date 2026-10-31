"""Non-destructive TwinLiteNet+ Small three-class lane-head adapter and losses."""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import torch
import torch.nn.functional as F

PROJECT = Path(__file__).resolve().parents[2]
UPSTREAM = PROJECT / "third_party/TwinLiteNetPlus"
sys.dont_write_bytecode = True  # Avoid untracked pycache inside upstream checkout.
sys.path.insert(0, str(UPSTREAM))
from model.model import TwinLiteNetPlus, UpConvBlock  # noqa: E402


def build_small_three_class(weight_path: Path | None = None) -> tuple[torch.nn.Module, dict]:
    """Reuse matching official weights; initialize only newly shaped lane-head tensors."""
    model = TwinLiteNetPlus(SimpleNamespace(config="small"))
    model.out_ll = UpConvBlock(8, 3, last=True)
    source = weight_path or PROJECT / "checkpoints/upstream_small.pth"
    upstream = torch.load(source, map_location="cpu", weights_only=True)
    current = model.state_dict()
    compatible = {key: value for key, value in upstream.items()
                  if key in current and value.shape == current[key].shape}
    skipped = [key for key in upstream if key not in compatible]
    status = model.load_state_dict(compatible, strict=False)
    if any(k.startswith('out_da.') for k in skipped):
        raise RuntimeError("Drivable head unexpectedly failed to load")
    info = {"loaded_tensors": len(compatible), "source_tensors": len(upstream),
            "skipped_shape_or_name": skipped, "new_missing_keys": list(status.missing_keys),
            "unexpected_keys": list(status.unexpected_keys),
            "params": sum(p.numel() for p in model.parameters())}
    return model, info


def masked_multitask_loss(da_logits: torch.Tensor, lane_logits: torch.Tensor,
                          da_target: torch.Tensor, lane_target: torch.Tensor,
                          da_valid: torch.Tensor, lane_valid: torch.Tensor,
                          lane_weights: torch.Tensor | None = None) -> tuple[torch.Tensor, dict]:
    """CE ignores pixel value 255 and whole tasks with no valid annotation."""
    if da_logits.shape[1] != 2 or lane_logits.shape[1] != 3:
        raise ValueError("Expected 2-channel DA and 3-channel lane logits")
    terms = []
    log: dict[str, float] = {}
    for name, logits, target, valid, weights in [
        ("da", da_logits, da_target, da_valid, None),
        ("lane", lane_logits, lane_target, lane_valid, lane_weights),
    ]:
        idx = torch.where(valid.to(dtype=torch.bool, device=logits.device))[0]
        if len(idx) == 0:
            log[name + "_count"] = 0
            continue
        picked_target = target[idx].to(logits.device).long()
        if not torch.any(picked_target != 255):
            log[name + "_count"] = 0
            continue
        term = F.cross_entropy(logits[idx], picked_target, weight=weights, ignore_index=255)
        terms.append(term)
        log[name + "_loss"] = float(term.detach().cpu())
        log[name + "_count"] = len(idx)
    if not terms:
        raise ValueError("Batch has no valid supervision")
    return sum(terms), log
