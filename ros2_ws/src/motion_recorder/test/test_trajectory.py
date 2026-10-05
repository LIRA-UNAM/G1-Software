import math

import numpy as np
import pytest

from motion_recorder.trajectory import (interp_frames, reorder, safe_move_duration,
                                        smooth_interp, validate_recording)

NAMES = ["a", "b", "c"]


def rec(t, q, names=NAMES):
    return {"joint_names": list(names), "t": np.asarray(t, float), "q": np.asarray(q, float)}


def test_safe_move_duration_respects_peak_velocity():
    T = safe_move_duration([0, 0, 0], [0.6, -0.3, 0], max_vel=0.3, min_duration=0.5)
    assert T == pytest.approx(0.5 * math.pi * 0.6 / 0.3)
    # Peak speed of the cosine profile equals max_vel.
    dt = 1e-4
    peak = np.max(np.abs(smooth_interp(0, 0.6, 0.5 + dt / T) - smooth_interp(0, 0.6, 0.5))) / dt
    assert peak == pytest.approx(0.3, rel=1e-3)


def test_safe_move_duration_minimum():
    assert safe_move_duration([0, 0], [0.01, 0], 0.3, 2.0) == 2.0


def test_smooth_interp_endpoints_and_clamp():
    assert np.allclose(smooth_interp([0, 1], [1, 3], 0.0), [0, 1])
    assert np.allclose(smooth_interp([0, 1], [1, 3], 1.0), [1, 3])
    assert np.allclose(smooth_interp([0, 1], [1, 3], 2.0), [1, 3])
    assert np.allclose(smooth_interp([0, 1], [1, 3], 0.5), [0.5, 2])


def test_interp_frames():
    t = [0.0, 0.05, 0.10]
    q = [[0, 0], [1, 2], [2, 4]]
    assert np.allclose(interp_frames(t, q, -1), [0, 0])
    assert np.allclose(interp_frames(t, q, 0.025), [0.5, 1])
    assert np.allclose(interp_frames(t, q, 0.075), [1.5, 3])
    assert np.allclose(interp_frames(t, q, 9), [2, 4])


def test_validate_ok_and_reorder():
    r = rec([0, 0.05, 0.1], [[0, 1, 2], [0.01, 1, 2], [0.02, 1, 2]])
    validate_recording(r, ["c", "a"], max_vel=4.0)
    assert np.allclose(reorder(r, ["c", "a"])[0], [2, 0])


@pytest.mark.parametrize("r, match", [
    (rec([0, 0.05], [[0, 0, 0], [0, 0, 0]], names=["a", "b"]), "lacks joints"),
    (rec([0], [[0, 0, 0]]), "at least 2"),
    (rec([0, 0], [[0, 0, 0], [0, 0, 0]]), "increasing"),
    (rec([0, 0.05], [[0, 0, 0], [1.0, 0, 0]]), "rad/s"),
    (rec([0, 0.05], [[0, 0, 0], [np.nan, 0, 0]]), "NaN"),
])
def test_validate_rejects(r, match):
    with pytest.raises(ValueError, match=match):
        validate_recording(r, NAMES, max_vel=4.0)
