"""Tests fuer ur5e_bullet.visualize_scan (Kamera-Sichtachsen-Visualisierung).

Laeuft komplett headless: pyplot wird vor Import des Moduls aufs Agg-Backend
gestellt, gespeichert wird in einen temporaeren Ordner.
"""

import json
import os
import struct
import tempfile

import numpy as np
import matplotlib

matplotlib.use("Agg")

import ur5e_bullet.visualize_scan as vs

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _write_stl(path, x, y, z):
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", 2))
        for tri in (((0, 0, 0), (x, y, 0), (0, z, 0)), ((x, y, z), (x, 0, 0), (0, 0, z))):
            fh.write(b"\0" * 12)
            for v in tri:
                fh.write(struct.pack("<3f", *v))
            fh.write(b"\0" * 2)


def _write_scan(root, n_poses=3, with_cameras=True):
    with open(os.path.join(root, "render_settings.json"), "w") as f:
        json.dump({
            "info": {
                "start_position": "o1l",
                "jaw_folder": 1,
                "jaw_type": "lower",
            },
            "reconstruction": {
                "baseline_m": 0.003,
                "camera_fov_deg": 87.0,
                "camera_far_m": 0.03,
            },
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


def test_stl_spans_m_and_default_ray_len():
    with tempfile.TemporaryDirectory() as root:
        p = os.path.join(root, "jaw.stl")
        _write_stl(p, 60.0, 40.0, 20.0)
        spans = vs._stl_spans_m(p)
        assert np.allclose(spans, [0.06, 0.04, 0.02], atol=1e-6)
        real = os.path.join(vs.JAWS_DIR, "1", "lower.stl")
        if os.path.isfile(real):
            assert 0.05 <= vs._jaw_diameter_m(1, "lower") <= 0.08
        assert vs._jaw_diameter_m(folder=999, jaw_type="lower") is None
        assert vs._default_ray_len({"reconstruction": {"camera_far_m": 0.03}}) == 0.03
        assert vs._default_ray_len({}) == round(vs._cfg_mod.CAMERA_FAR_M, 4)
        assert vs._settings_get({"baseline_m": 0.003}, "baseline_m") == 0.003  # flat Fallback
        assert vs._settings_get({"info": {"start_position": "o1l"}}, "start_position") == "o1l"
        assert vs._settings_get({}, "nicht_da", 7) == 7


def test_plot_scan_dir_green_baseline_and_auto_ray():
    with tempfile.TemporaryDirectory() as root:
        _write_scan(root, n_poses=3)
        fig = vs.plot_scan_dir(root, out=os.path.join(root, "green.png"), ray_len=None)
        assert os.path.isfile(os.path.join(root, "green.png"))
        ax = fig.axes[0]
        green = [l for l in ax.lines if l.get_color() == "#2ca02c"]
        assert len(green) == 6
        assert any(l.get_label() == "Stereo-Paar (L\u2013R)" for l in green)
        poses, _ = vs._load_poses(root)
        left = np.array([p[2][0]["position"] for p in poses])
        right = np.array([p[2][1]["position"] for p in poses])
        ql = np.array([p[2][0]["quaternion"] for p in poses])
        qr = np.array([p[2][1]["quaternion"] for p in poses])
        ray = vs._default_ray_len(vs._load_settings(root))
        tips_l = left + np.array([vs._rotate(q, (0, 0, -1)) * ray for q in ql])
        tips_r = right + np.array([vs._rotate(q, (0, 0, -1)) * ray for q in qr])
        segs = np.array([np.asarray(l.get_data_3d()).T for l in green])
        expected_tips = np.vstack([tips_l[0], tips_r[0]])
        expected_pts = np.vstack([left[0], right[0]])
        assert (np.abs(segs - expected_tips[None]) <= 1e-9).all(axis=(1, 2)).any()
        assert (np.abs(segs - expected_pts[None]) <= 1e-9).all(axis=(1, 2)).any()
        mid = (left + right) / 2
        dashed = [l for l in ax.lines if l.get_linestyle() == "--"]
        assert len(dashed) == 1
        assert np.allclose(np.vstack(dashed[0].get_data_3d()), mid.T, atol=1e-9)
        labels = [t for t in ax.texts if t.get_text().strip().startswith("W")]
        for t, m in zip(sorted(labels, key=lambda t: t.get_text()), mid):
            assert np.allclose(t.get_position_3d(), m, atol=1e-9)
        assert np.allclose(ax.get_proj()[3, :3], 0)  # orthografische Projektion
        assert any(l.get_color() == "0.85" for l in ax.lines)
        xl = ax.get_xlim()
        assert np.allclose(ax.get_ylim(), xl, atol=1e-9)
        assert np.allclose(ax.get_zlim(), xl, atol=1e-9)  # isometrische Achs-Skalierung
        infos = [t.get_text() for t in ax.texts if "|L-R|" in t.get_text()]
        assert infos and "konstant" in infos[0] and "n=3" in infos[0]


def test_jaw_bbox_m():
    with tempfile.TemporaryDirectory() as root:
        p = os.path.join(root, "jaw.stl")
        _write_stl(p, 60.0, 40.0, 20.0)
        lo, hi = vs._stl_bbox_m(p)
        assert np.allclose(lo, [0, 0, 0], atol=1e-6)
        assert np.allclose(hi, [0.06, 0.04, 0.02], atol=1e-6)
        real = os.path.join(vs.JAWS_DIR, "1", "lower.stl")
        if os.path.isfile(real):
            blo, bhi = vs._jaw_bbox_m(1, "lower")
            assert np.allclose(bhi - blo, vs._stl_spans_m(real), atol=1e-9)


def test_plot_scan_dir_explicit_ray_len():
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