"""Tests fuer ur5e_bullet.visualize_scan (Kamera-Sichtachsen-Visualisierung).

Laeuft komplett headless: pyplot wird vor Import des Moduls aufs Agg-Backend
gestellt, gespeichert wird in einen temporaeren Ordner.
"""

import json
import os
import tempfile

import numpy as np
import matplotlib

matplotlib.use("Agg")

import ur5e_bullet.visualize_scan as vs

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _write_scan(root, n_poses=3, with_cameras=True):
    with open(os.path.join(root, "render_settings.json"), "w") as f:
        json.dump({
            "start_position": "o1l",
            "baseline_m": 0.003,
            "camera_fov_deg": 87.0,
        }, f)
    for i in range(n_poses):
        pose = {
            "waypoint": i,
            "label": f"W{i}",
        }
        if with_cameras:
            pose["camera_left"] = {
                "position": [-0.02, -0.01, 0.01],
                "quaternion": [0.0, 0.0, -0.99999999, 6.15e-06],
            }
            pose["camera_right"] = {
                "position": [0.02, -0.01, 0.01],
                "quaternion": [0.0, 0.0, -0.99999999, 6.15e-06],
            }
        with open(os.path.join(root, f"{i + 1}_pose.json"), "w") as f:
            json.dump(pose, f)


def test_rotate():
    assert np.allclose(vs._rotate([0, 0, 0, 1], [0, 0, -1]), [0, 0, -1])
    q90 = [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
    assert np.allclose(vs._rotate(q90, [1, 0, 0]), [0, 1, 0], atol=1e-9)


def test_load_poses_all_and_legacy_skip():
    with tempfile.TemporaryDirectory() as root:
        _write_scan(root, n_poses=3)
        poses, skipped = vs._load_poses(root)
        assert len(poses) == 3 and skipped == 0
    with tempfile.TemporaryDirectory() as root:
        _write_scan(root, n_poses=3, with_cameras=False)
        poses, skipped = vs._load_poses(root)
        assert len(poses) == 0 and skipped == 3


def test_scan_dirs_discovery():
    with tempfile.TemporaryDirectory() as root:
        os.makedirs(os.path.join(root, "scan_a"))
        os.makedirs(os.path.join(root, "scan_b"))
        os.makedirs(os.path.join(root, "leer"))
        _write_scan(os.path.join(root, "scan_a"), n_poses=1)
        _write_scan(os.path.join(root, "scan_b"), n_poses=2)
        dirs = vs._scan_dirs(root)
        assert [os.path.basename(d) for d in dirs] == ["scan_a", "scan_b"]
        assert vs._scan_dirs(os.path.join(root, "scan_a")) == [os.path.join(root, "scan_a")]


def test_plot_scan_dir_writes_png():
    with tempfile.TemporaryDirectory() as root:
        _write_scan(root, n_poses=3)
        out = os.path.join(root, "out.png")
        fig = vs.plot_scan_dir(root, out=out, ray_len=0.25)
        assert os.path.isfile(out) and os.path.getsize(out) > 0
        assert fig is not None


def test_main_skips_legacy_scan():
    with tempfile.TemporaryDirectory() as root:
        os.makedirs(os.path.join(root, "legacy_scan"))
        os.makedirs(os.path.join(root, "new_scan"))
        _write_scan(os.path.join(root, "legacy_scan"), n_poses=2, with_cameras=False)
        _write_scan(os.path.join(root, "new_scan"), n_poses=2)
        outdir = os.path.join(root, "plots")
        rc = vs.main([root, "--out", outdir])
        assert rc == 0
        assert os.path.isfile(os.path.join(outdir, "new_scan.png"))
        assert not os.path.isfile(os.path.join(outdir, "legacy_scan.png"))


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK {name}")
    print("ALLE VISUALIZE-SCAN-TESTS OK")