import os
import pickle

import numpy as np
import pytest

from motion_recorder import storage


def make(n=5):
    t = np.arange(n) * 0.05
    q = np.tile(np.arange(3.0), (n, 1))
    return storage.make_recording(["a", "b", "c"], 20.0, t, q, q * 0, q * 0, "2026-10-05T10:00:00")


def test_round_trip(tmp_path):
    path = storage.save_recording(str(tmp_path), "wave", make())
    assert os.path.basename(path) == "wave.pkl"
    rec = storage.load_recording(path)
    assert rec["joint_names"] == ["a", "b", "c"]
    assert rec["rate_hz"] == 20.0
    assert rec["q"].shape == (5, 3) and rec["duration_s"] == pytest.approx(0.2)
    assert np.allclose(rec["start_q"], [0, 1, 2])
    assert not [f for f in os.listdir(tmp_path) if f.endswith(".tmp")]


def test_no_overwrite(tmp_path):
    a = storage.save_recording(str(tmp_path), "wave", make())
    b = storage.save_recording(str(tmp_path), "wave", make())
    assert a != b and os.path.basename(b) == "wave_1.pkl"


def test_rejects_foreign_pickle(tmp_path):
    path = tmp_path / "x.pkl"
    path.write_bytes(pickle.dumps({"hello": 1}))
    with pytest.raises(ValueError):
        storage.load_recording(str(path))


def test_index_lists_and_flags_bad_files(tmp_path):
    storage.save_recording(str(tmp_path), "good", make(7))
    (tmp_path / "bad.pkl").write_bytes(b"not a pickle")
    listing = {r["name"]: r for r in storage.RecordingIndex(str(tmp_path)).list()}
    assert listing["good.pkl"]["frames"] == 7
    assert "error" in listing["bad.pkl"]


def test_valid_name():
    assert storage.valid_name("wave_01-a")
    assert not storage.valid_name("../etc")
    assert not storage.valid_name("")


def test_v2_groups_and_online(tmp_path):
    t = np.arange(3) * 0.05
    q = np.zeros((3, 2))
    online = [[True, True], [True, False], [True, True]]
    r = storage.make_recording(["a", "b"], 20.0, t, q, q, q, "now", online=online, groups=["left_arm"])
    rec = storage.load_recording(storage.save_recording(str(tmp_path), "sub", r))
    assert rec["format_version"] == 2 and rec["groups"] == ["left_arm"]
    assert rec["online"].dtype == bool and not rec["online"][1, 1]
    listing = storage.RecordingIndex(str(tmp_path)).list()
    assert listing[0]["groups"] == ["left_arm"]


def test_v1_files_still_load(tmp_path):
    v1 = make(4)
    v1["format_version"] = 1
    del v1["online"], v1["groups"]
    v1["q"][:, 0] = 0.0  # joint "a" recorded while its motor was offline
    v1["q"][:, 1:] += 0.1
    path = tmp_path / "old.pkl"
    path.write_bytes(pickle.dumps(v1, protocol=4))
    rec = storage.load_recording(str(path))
    assert rec["groups"] is None and rec["online"].shape == (4, 3)
    assert not rec["online"][:, 0].any() and rec["online"][:, 1:].all()


def test_home_keyframe_round_trip(tmp_path):
    assert storage.load_home(str(tmp_path)) is None
    home = storage.save_home(str(tmp_path), ["a", "b", "c"], [0.1, 0.2, 0.0],
                             [True, True, False], "2026-10-07T09:00:00")
    assert home["joints"] == {"a": 0.1, "b": 0.2}  # offline joint c not stored
    loaded = storage.load_home(str(tmp_path))
    assert loaded == home
    # The home file is never listed as a recording.
    assert storage.RecordingIndex(str(tmp_path)).list() == []
    (tmp_path / storage.HOME_FILE).write_text("{broken")
    assert storage.load_home(str(tmp_path)) is None
