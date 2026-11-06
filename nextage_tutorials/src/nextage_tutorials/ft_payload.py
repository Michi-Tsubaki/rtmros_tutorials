"""Static payload identification: F=m*g+b_f, T=(m*r) cross g+b_t."""

import numpy as np


def load_calibration(path, sensor_frame, base_frame):
    from pathlib import Path
    import yaml
    result = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(result, dict):
        raise ValueError('Invalid calibration file')
    if result.get('sensor_frame') != sensor_frame or result.get('base_frame') != base_frame:
        raise ValueError('Calibration frames do not match this sensor')
    if result.get('wrench_sign') not in (-1.0, 1.0):
        raise ValueError('Invalid calibration polarity')
    mass = result.get('mass', float('nan'))
    if not np.isfinite(mass) or mass <= 0:
        raise ValueError('Invalid payload mass')
    for name in ('center_of_mass', 'force_bias', 'torque_bias'):
        value = np.asarray(result.get(name), dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError(f'Invalid calibration {name}')
    return result


def gravity_from_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return -9.80665 * np.array([2*(x*z + w*y), 2*(y*z - w*x), 1-2*(x*x+y*y)])


def fit_payload(gravity, wrench, max_condition=200.0,
                max_force_rms=0.3, max_torque_rms=0.03):
    gravity, wrench = np.asarray(gravity, float), np.asarray(wrench, float)
    if gravity.ndim != 2 or gravity.shape[1] != 3 or len(gravity) < 6:
        raise ValueError('At least six distinct static poses are required')
    if wrench.shape != (len(gravity), 6) or not np.isfinite(gravity).all() or not np.isfinite(wrench).all():
        raise ValueError('Invalid gravity/wrench samples')
    matrix = []
    for x, y, z in gravity:
        block = np.zeros((6, 10))
        block[:3, 0] = [x, y, z]
        block[:3, 1:4] = np.eye(3)
        block[3:, 4:7] = [[0, z, -y], [-z, 0, x], [y, -x, 0]]
        block[3:, 7:] = np.eye(3)
        matrix.append(block)
    matrix = np.vstack(matrix)
    # Normalize columns so the condition check is not dominated by units.
    scale = np.linalg.norm(matrix, axis=0)
    if np.any(scale < 1e-9):
        raise ValueError('Insufficient orientation diversity')
    normalized = matrix / scale
    solution, _, rank, singular = np.linalg.lstsq(normalized, wrench.ravel(), rcond=None)
    condition = float(singular[0] / singular[-1]) if singular[-1] else float('inf')
    if rank != 10 or condition > max_condition:
        raise ValueError(f'Insufficient orientation diversity: rank={rank}, condition={condition:.1f}')
    solution /= scale
    residual = (matrix @ solution).reshape(-1, 6) - wrench
    rms = np.sqrt(np.mean(residual**2, axis=0))
    if max(rms[:3]) > max_force_rms or max(rms[3:]) > max_torque_rms:
        raise ValueError(f'Fit residual too large (N, Nm): {rms.tolist()}')
    mass = float(solution[0])
    if mass <= 1e-5:
        raise ValueError(f'Payload mass {mass:.6f} kg is non-positive or too small to identify CoG; '
                         'check sensor polarity, orientation diversity and unloaded tare')
    return {'mass': mass, 'center_of_mass': (solution[4:7] / mass).tolist(),
            'force_bias': solution[1:4].tolist(), 'torque_bias': solution[7:].tolist(),
            'residual_rms': rms.tolist(), 'condition_number': condition,
            'pose_count': len(gravity)}


def compensate(wrench, gravity, calibration):
    force = calibration['mass'] * np.asarray(gravity)
    torque = np.cross(calibration['center_of_mass'], force)
    return np.asarray(wrench) - np.r_[force, torque] - np.r_[
        calibration['force_bias'], calibration['torque_bias']]
