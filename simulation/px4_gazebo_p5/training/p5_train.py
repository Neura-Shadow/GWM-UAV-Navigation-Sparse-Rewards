"""Listwise training of `p5-ranker-mlp-v1` in NumPy float64 (plan §4.4).

The training-side forward pass performs the runtime's float64 operations. Training may run under a
different NumPy/BLAS than the flight runtime (the runtime's reference BLAS is slow), so each round
saves recorded tensors with their training-side scores and the pipeline checks the runtime's
`FrozenPolicy.scores` against them (plan §3: within 1e-9). Loss: cross-entropy between softmax(scores) and the oracle
target softmax(-c / tau), both over the allowed slots. AdamW with decoupled weight decay, a cosine
learning-rate schedule, fixed seeds and a seeded data order; the epoch with the lowest validation
regret is kept. Validation states come from validation layouts only (never held-out or acceptance).
"""
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys

import numpy as np

import p5_paths  # noqa: F401  (import paths)
import p5_dataset
import p5_oracle
from gwm_policy_control.contracts import LAYER_SHAPES

HYPER = {"seed": 5, "epochs": 40, "batch": 1024, "lr": 2e-3, "lr_final": 1e-4, "weight_decay": 1e-4,
         "beta1": 0.9, "beta2": 0.999, "eps": 1e-8, "tau_m": p5_oracle.TAU_M}
LAYERS = (("W1", "b1"), ("W2", "b2"), ("W3", "b3"))


def weights_sha256(arrays):
    """Content hash of the weights alone (name, shape and little-endian float64 bytes, sorted)."""
    digest = hashlib.sha256()
    for name in sorted(LAYER_SHAPES):
        value = np.ascontiguousarray(arrays[name], dtype="<f8")
        digest.update(name.encode() + repr(value.shape).encode() + value.tobytes())
    return digest.hexdigest()


def init_params(seed):
    rng = np.random.default_rng(seed)
    params = {}
    for w, b in LAYERS:
        fan_in, fan_out = LAYER_SHAPES[w]
        params[w] = rng.normal(0.0, math.sqrt(2.0 / fan_in), size=(fan_in, fan_out))
        params[b] = np.zeros(fan_out)
    return params


def assemble(candidate, context):
    """(B, 26, 11) candidate block + (B, 35) shared context -> (B, 26, 46) adapter tensor."""
    b, slots, _ = candidate.shape
    return np.concatenate([candidate, np.broadcast_to(context[:, None, :], (b, slots, context.shape[1]))], axis=2)


def forward(params, x):
    """x (B, 26, 46) -> scores (B, 26); the runtime's operations in the runtime's order, on the
    (B * 26, 46) matrix so the BLAS sees one large product."""
    flat = x.reshape(-1, x.shape[-1])
    h1 = np.maximum(flat @ params["W1"] + params["b1"], 0.0)
    h2 = np.maximum(h1 @ params["W2"] + params["b2"], 0.0)
    return (h2 @ params["W3"] + params["b3"])[:, 0].reshape(x.shape[:-1]), (flat, h1, h2)


def targets(costs, allowed, tau):
    out = np.zeros(costs.shape)
    for i in range(costs.shape[0]):
        out[i] = p5_oracle.target_distribution(costs[i], allowed[i], tau)
    return out


def loss_and_grads(params, x, allowed, target):
    scores, (x2, h1, h2) = forward(params, x)
    masked = np.where(allowed, scores, -np.inf)
    top = masked.max(axis=1, keepdims=True)
    e = np.where(allowed, np.exp(masked - top), 0.0)
    z = e.sum(axis=1, keepdims=True)
    log_p = np.where(allowed, scores - top - np.log(z), 0.0)
    n = x.shape[0]
    loss = float(-(target * log_p).sum() / n)
    d_scores = (e / z - target) / n  # zero outside the mask
    d3 = d_scores.reshape(-1, 1)
    grads = {"W3": h2.T @ d3, "b3": d3.sum(axis=0)}
    d_h2 = (d3 @ params["W3"].T) * (h2 > 0)
    grads["W2"], grads["b2"] = h1.T @ d_h2, d_h2.sum(axis=0)
    d_h1 = (d_h2 @ params["W2"].T) * (h1 > 0)
    grads["W1"], grads["b1"] = x2.T @ d_h1, d_h1.sum(axis=0)
    return loss, grads


def informative(data):
    """States where the choice matters (at least two allowed slots)."""
    return np.flatnonzero(data["allowed"].sum(axis=1) >= 2)


def evaluate(params, data, target_all, batch=4096):
    """Validation loss, oracle top-1 agreement and mean regret (metres) on informative states;
    `target_all` holds the targets of `informative(data)` in order."""
    idx = informative(data)
    if idx.size == 0:
        return {"states": 0}
    loss_sum, agree, regret = 0.0, 0, 0.0
    for start in range(0, idx.size, batch):
        sel = idx[start:start + batch]
        x = assemble(data["candidate"][sel], data["context"][sel])
        allowed, costs = data["allowed"][sel], data["costs"][sel]
        loss, _ = loss_and_grads(params, x, allowed, target_all[start:start + batch])
        loss_sum += loss * sel.size
        scores, _ = forward(params, x)
        choice = np.where(allowed, scores, -np.inf).argmax(axis=1)
        best = np.where(allowed, costs, np.inf)
        chosen = best[np.arange(sel.size), choice]
        agree += int((chosen <= best.min(axis=1) + 1e-12).sum())
        regret += float((chosen - best.min(axis=1)).sum())
    return {"states": int(idx.size), "loss": loss_sum / idx.size, "oracle_top1": agree / idx.size,
            "mean_regret_m": regret / idx.size}


def train(train_data, val_data, hyper=None, log=print):
    """Returns (best params, history)."""
    hyper = HYPER if hyper is None else hyper
    params = init_params(hyper["seed"])
    m = {k: np.zeros_like(v) for k, v in params.items()}
    v = {k: np.zeros_like(v) for k, v in params.items()}
    idx = informative(train_data)
    target_all = targets(train_data["costs"][idx], train_data["allowed"][idx], hyper["tau_m"])
    val_idx = informative(val_data)
    val_target = targets(val_data["costs"][val_idx], val_data["allowed"][val_idx], hyper["tau_m"])
    steps_per_epoch = int(math.ceil(idx.size / hyper["batch"]))
    total, step = steps_per_epoch * hyper["epochs"], 0
    best, best_regret, best_epoch, history = None, math.inf, None, []
    for epoch in range(hyper["epochs"]):
        order = np.random.default_rng([hyper["seed"], epoch]).permutation(idx.size)
        epoch_loss = 0.0
        for start in range(0, idx.size, hyper["batch"]):
            pick = order[start:start + hyper["batch"]]
            sel = idx[pick]
            x = assemble(train_data["candidate"][sel], train_data["context"][sel])
            loss, grads = loss_and_grads(params, x, train_data["allowed"][sel], target_all[pick])
            epoch_loss += loss * sel.size
            step += 1
            lr = hyper["lr_final"] + 0.5 * (hyper["lr"] - hyper["lr_final"]) * (1 + math.cos(math.pi * step / total))
            for k in params:
                m[k] = hyper["beta1"] * m[k] + (1 - hyper["beta1"]) * grads[k]
                v[k] = hyper["beta2"] * v[k] + (1 - hyper["beta2"]) * grads[k] ** 2
                m_hat = m[k] / (1 - hyper["beta1"] ** step)
                v_hat = v[k] / (1 - hyper["beta2"] ** step)
                decay = hyper["weight_decay"] * params[k] if k.startswith("W") else 0.0
                params[k] = params[k] - lr * (m_hat / (np.sqrt(v_hat) + hyper["eps"]) + decay)
        val = evaluate(params, val_data, val_target)
        entry = {"epoch": epoch, "train_loss": epoch_loss / idx.size, **{"val_" + k: x for k, x in val.items()}}
        history.append(entry)
        log("epoch %2d train %.4f val loss %.4f top1 %.3f regret %.4f" % (
            epoch, entry["train_loss"], val["loss"], val["oracle_top1"], val["mean_regret_m"]))
        if val["mean_regret_m"] < best_regret:
            best_regret, best_epoch = val["mean_regret_m"], epoch
            best = {k: p.copy() for k, p in params.items()}
    history[best_epoch]["selected"] = True
    return best, history


PARITY_STATES = 256


def aggregate(work, round_index, name):
    return p5_dataset.concatenate([
        {k: np.load(Path(work) / "data" / ("round%d" % r) / name / (k + ".npy"), allow_pickle=False)
         for k in p5_dataset.ARRAYS} for r in range(round_index + 1)])


def environment():
    blas = "unknown"
    try:
        blas = np.show_config(mode="dicts")["Build Dependencies"]["blas"].get("name", "unknown")
    except Exception:  # older NumPy without mode="dicts"
        pass
    return {"python": platform.python_version(), "numpy": np.__version__, "blas": blas,
            "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS"), "executable": sys.executable}


def main(argv):
    """python p5_train.py WORKDIR ROUND: train on rounds 0..ROUND and save weights, history and
    recorded parity tensors under WORKDIR/models/roundROUND."""
    work, round_index = Path(argv[1]), int(argv[2])
    out = work / "models" / ("round%d" % round_index)
    out.mkdir(parents=True, exist_ok=True)
    train_data, val_data = aggregate(work, round_index, "train"), aggregate(work, round_index, "validation")
    print("round %d training on %d states (validation %d)" % (round_index, len(train_data["state"]), len(val_data["state"])),
          flush=True)
    params, history = train(train_data, val_data, log=lambda m: print("  " + m, flush=True))
    for name, value in params.items():
        np.save(out / (name + ".npy"), value, allow_pickle=False)
    pick = informative(val_data)[:PARITY_STATES]
    x = assemble(val_data["candidate"][pick], val_data["context"][pick])
    np.save(out / "parity_x.npy", x, allow_pickle=False)
    np.save(out / "parity_scores.npy", forward(params, x)[0], allow_pickle=False)
    (out / "environment.json").write_bytes((json.dumps(environment(), sort_keys=True, indent=1) + "\n").encode())
    (out / "history.json").write_bytes((json.dumps(history, sort_keys=True, indent=1, allow_nan=False) + "\n").encode())


if __name__ == "__main__":
    main(sys.argv)
