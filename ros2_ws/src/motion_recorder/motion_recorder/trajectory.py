"""Pure trajectory helpers for recording playback (no ROS, unit-tested)."""

import math

import numpy as np


def safe_move_duration(q_from, q_to, max_vel, min_duration):
    """Duration of a cosine-smoothed move whose peak joint speed is <= max_vel.

    The cosine profile peaks at pi/2 times the average speed, so
    T = pi/2 * max|dq| / max_vel, but never shorter than min_duration.
    """
    if max_vel <= 0.0:
        raise ValueError("max_vel must be > 0")
    delta = np.max(np.abs(np.asarray(q_to, float) - np.asarray(q_from, float)), initial=0.0)
    return max(float(min_duration), 0.5 * math.pi * float(delta) / float(max_vel))


def smooth_interp(q_from, q_to, s):
    """Cosine (zero start/end velocity) interpolation for s in [0, 1]."""
    s = min(max(float(s), 0.0), 1.0)
    alpha = 0.5 - 0.5 * math.cos(math.pi * s)
    q_from = np.asarray(q_from, float)
    return q_from + alpha * (np.asarray(q_to, float) - q_from)


def interp_frames(t_frames, q_frames, t):
    """Joint positions at time t, linearly interpolated between recorded frames."""
    t_frames = np.asarray(t_frames, float)
    q_frames = np.asarray(q_frames, float)
    if t <= t_frames[0]:
        return q_frames[0].copy()
    if t >= t_frames[-1]:
        return q_frames[-1].copy()
    k = int(np.searchsorted(t_frames, t, side="right"))
    t0, t1 = t_frames[k - 1], t_frames[k]
    w = (t - t0) / (t1 - t0)
    return q_frames[k - 1] + w * (q_frames[k] - q_frames[k - 1])


def validate_recording(rec, joint_names, max_vel):
    """Raises ValueError if the recording cannot drive these (played) joints.

    Only the given joints are checked, so a glitching limb that is not played
    does not block the others.
    """
    if not joint_names:
        raise ValueError("no joints to play")
    missing = [n for n in joint_names if n not in rec["joint_names"]]
    if missing:
        raise ValueError("recording lacks joints: " + ", ".join(missing))
    t = np.asarray(rec["t"], float)
    q_all = np.asarray(rec["q"], float)
    if t.ndim != 1 or len(t) < 2:
        raise ValueError("recording needs at least 2 frames")
    if q_all.shape != (len(t), len(rec["joint_names"])):
        raise ValueError("q has shape %s, expected %s"
                         % (q_all.shape, (len(t), len(rec["joint_names"]))))
    index = [rec["joint_names"].index(n) for n in joint_names]
    q = q_all[:, index]
    online = rec.get("online")
    if online is not None:
        offline = [n for n, ok in zip(joint_names, np.asarray(online, bool)[:, index].all(axis=0))
                   if not ok]
        if offline:
            raise ValueError("motors were offline during the take: " + ", ".join(offline))
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(t)):
        raise ValueError("recording contains NaN/inf")
    dt = np.diff(t)
    if np.any(dt <= 0.0):
        raise ValueError("recording timestamps are not strictly increasing")
    vel = np.abs(np.diff(q, axis=0)) / dt[:, None]
    if vel.max() > max_vel:
        worst = joint_names[int(np.argmax(vel.max(axis=0)))]
        raise ValueError("recording moves at %.2f rad/s on %s (> max_recorded_vel %.2f)"
                         % (vel.max(), worst, max_vel))


def reorder(rec, joint_names):
    """Recorded q columns in the order of joint_names."""
    index = [rec["joint_names"].index(n) for n in joint_names]
    return np.asarray(rec["q"], float)[:, index]
