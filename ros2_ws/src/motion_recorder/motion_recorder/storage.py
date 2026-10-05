"""Saving / loading recordings as .pkl files."""

import os
import pickle
import re
import tempfile

import numpy as np

FORMAT_VERSION = 1
# Protocol 4: readable by Python 3.8 (robot) and newer (laptop).
PICKLE_PROTOCOL = 4
_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def valid_name(name):
    return bool(_NAME_RE.match(name))


def unique_path(directory, name):
    """<directory>/<name>.pkl, with _1, _2... appended instead of overwriting."""
    path = os.path.join(directory, name + ".pkl")
    k = 1
    while os.path.exists(path):
        path = os.path.join(directory, "%s_%d.pkl" % (name, k))
        k += 1
    return path


def save_recording(directory, name, rec):
    """Writes rec atomically (temp file + rename) and returns the final path."""
    os.makedirs(directory, exist_ok=True)
    path = unique_path(directory, name)
    fd, tmp = tempfile.mkstemp(dir=directory, suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as f:
            pickle.dump(rec, f, protocol=PICKLE_PROTOCOL)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path


def load_recording(path):
    # Only load recordings you created: unpickling runs arbitrary code.
    with open(path, "rb") as f:
        rec = pickle.load(f)
    if not isinstance(rec, dict) or rec.get("format_version") != FORMAT_VERSION:
        raise ValueError("%s is not a motion_recorder v%d file" % (path, FORMAT_VERSION))
    return rec


def make_recording(joint_names, rate_hz, t, q, dq, tau, created):
    q = np.asarray(q, float)
    return {
        "format_version": FORMAT_VERSION,
        "robot": "g1_23dof",
        "joint_names": list(joint_names),
        "rate_hz": float(rate_hz),
        "created": created,
        "duration_s": float(t[-1] - t[0]) if len(t) else 0.0,
        "t": np.asarray(t, float),
        "q": q,
        "dq": np.asarray(dq, float),
        "tau_est": np.asarray(tau, float),
        "start_q": q[0].copy() if len(q) else q,
    }


class RecordingIndex:
    """Cached listing of the recordings directory (re-reads only changed files)."""

    def __init__(self, directory):
        self.directory = directory
        self._cache = {}  # name -> (mtime, summary)

    def list(self):
        try:
            names = sorted(f for f in os.listdir(self.directory) if f.endswith(".pkl"))
        except FileNotFoundError:
            return []
        out = []
        for name in names:
            path = os.path.join(self.directory, name)
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            cached = self._cache.get(name)
            if cached is None or cached[0] != mtime:
                try:
                    rec = load_recording(path)
                    summary = {"name": name, "duration_s": round(rec["duration_s"], 2),
                               "frames": int(len(rec["t"])), "created": rec.get("created", "")}
                except Exception as exc:  # noqa: B902 - any unreadable file is just flagged
                    summary = {"name": name, "error": str(exc)[:80]}
                cached = (mtime, summary)
                self._cache[name] = cached
            out.append(cached[1])
        for gone in set(self._cache) - set(names):
            del self._cache[gone]
        return out
