#!/usr/bin/env python3
"""Minimal self-test for keyframe json speeds contract.

This is intentionally not using pytest to keep dependencies minimal.
Run:
  python3 test_speed_json.py
"""

import json
import os
import tempfile


def _write_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


def _read_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def test_legacy_file_is_upgraded(tmpdir: str) -> None:
    legacy = [[0.0] * 14, [0.1] * 14]
    p = os.path.join(tmpdir, "legacy.json")
    _write_json(p, legacy)

    from bw_sim2real.vel_config import load_keyframe_file, save_keyframe_file

    kf = load_keyframe_file(p)
    assert len(kf.keyframes) == 2
    assert len(kf.speeds) == 2
    assert all(v > 0.0 for v in kf.speeds)

    save_keyframe_file(kf, p)
    data = _read_json(p)
    assert isinstance(data, dict)
    assert "keyframes" in data and "speeds" in data
    assert len(data["speeds"]) == len(data["keyframes"])


def test_new_file_keeps_speeds_len(tmpdir: str) -> None:
    obj = {
        "meta": {"format": "bw_sim2real_keyframes_v1"},
        "keyframes": [[0.0] * 14, [0.2] * 14, [0.4] * 14],
        "speeds": [1.0, 2.0, 3.0],
    }
    p = os.path.join(tmpdir, "new.json")
    _write_json(p, obj)

    from bw_sim2real.vel_config import load_keyframe_file

    kf = load_keyframe_file(p)
    assert len(kf.keyframes) == 3
    assert len(kf.speeds) == 3
    assert kf.speeds[1] == 2.0


def main() -> None:
    with tempfile.TemporaryDirectory() as td:
        test_legacy_file_is_upgraded(td)
        test_new_file_keeps_speeds_len(td)
    print("OK")


if __name__ == "__main__":
    main()
