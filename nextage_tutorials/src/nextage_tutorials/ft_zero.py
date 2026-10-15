"""Six-axis static tare estimation; no gravity or frame transformation."""

import numpy as np


def estimate_bias(samples, force_std_limit, torque_std_limit):
    data = np.asarray(samples, dtype=float)
    if data.ndim != 2 or data.shape[1] != 6 or len(data) < 2:
        raise ValueError("At least two six-axis samples are required")
    if not np.isfinite(data).all():
        raise ValueError("Non-finite wrench sample")
    limits = np.array([force_std_limit] * 3 + [torque_std_limit] * 3)
    if not np.isfinite(limits).all() or np.any(limits <= 0):
        raise ValueError("Standard deviation limits must be finite and positive")
    std = data.std(axis=0, ddof=1)
    if np.any(std > limits):
        raise ValueError(f"Wrench is not stable; standard deviations (N, Nm): {std.tolist()}")
    return data.mean(axis=0).tolist(), std.tolist()
