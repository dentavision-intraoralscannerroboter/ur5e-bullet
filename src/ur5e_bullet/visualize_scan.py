"""Visualisiert die Kamera-Sichtachsen aller pose.json eines gerenderten Scan-Ordners.

Nimmt einen Scan-Ordner (enthält <n>_pose.json, optional render_settings.json)
oder das render/-Root (alle Scan-Unterordner) und zeichnet im Gebiss-Frame (in m):

  - Kamera-Zentren: links = rot, rechts = blau
  - Sichtachse je Kamera: Linie entlang R(q) @ (0,0,-1)  (Blender-Kamera-Blickrichtung),
    Laenge = camera_far_m (Far-Clip-Ebene aus render_settings), keine Pfeilspitze
  - Stereo-Baseline: gruene Linie zwischen den Kamerapunkten und am Ende der
    beiden Sichtlinien (Pfeilspitzen)
  - Waypoint-Labels: im Mittelpunkt des jeweiligen L/R-Stereo-Paares
  - Sweep-Reihenfolge: gestrichelte Linie durch die L/R-Mittelpunkte der Waypoints
  - Gebiss-Ursprung: kurze Achsen + Marker; hellgrauer Rahmen = Gebiss-Bounding-Box aus der jaw-STL
  - Info-Zeile: gemessenes |L-R| (sollte konstant = baseline_m sein)
  - Orthografische Projektion + isometrische Achs-Skalierung (x/y/z gleiche Daten-Spanne):
  gleiche 3D-Laengen erscheinen in jeder Richtung und Blickrichtung gleich lang

Beispiele:

    poetry run visualize-scan render/lower_1_o1l_1920x1080_lat1.5mm
    poetry run visualize-scan render/ --out /tmp/cameras
    poetry run visualize-scan <scan_dir> --ray-len 0.3 --out plot.png
"""

import argparse
import json
import os
import re
import struct
import sys

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib fehlt:  poetry add 'matplotlib>=3.9,<4.0'", file=sys.stderr)
    sys.exit(1)

from .config import JAWS_DIR, CAMERA_FAR_M

_POSE_RE = re.compile(r"(\d+)_pose\.json$")


def _stl_bbox_m(path):
    try:
        with open(path, "rb") as fh:
            data = fh.read()
        ntri = struct.unpack("<I", data[80:84])[0]
        u8 = np.frombuffer(data[84 : 84 + ntri * 50], dtype="<u1").reshape(ntri, 50)
        verts = np.frombuffer(u8[:, 12:48].tobytes(), dtype="<f4").reshape(-1, 3)
        return verts.min(0) / 1000.0, verts.max(0) / 1000.0
    except Exception:
        return None


def _stl_spans_m(path):
    bb = _stl_bbox_m(path)
    return (bb[1] - bb[0]) if bb else None


def _jaw_bbox_m(folder=1, jaw_type="lower"):
    return _stl_bbox_m(os.path.join(JAWS_DIR, str(folder), f"{jaw_type}.stl"))


def _jaw_diameter_m(folder=1, jaw_type="lower"):
    path = os.path.join(JAWS_DIR, str(folder), f"{jaw_type}.stl")
    spans = _stl_spans_m(path)
    if spans is None:
        return None
    return float(max(spans[0], spans[1]))


def _settings_get(settings, key, default=None):
    if key in settings:
        return settings[key]
    for block in ("info", "reconstruction"):
        if isinstance(settings.get(block), dict) and key in settings[block]:
            return settings[block][key]
    return default


def _default_ray_len(settings):
    return round(_settings_get(settings, "camera_far_m", CAMERA_FAR_M), 4)


def _rotate(q, v):
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return R @ np.asarray(v, dtype=float)


def _load_poses(scan_dir):
    matches = []
    skipped = 0
    for name in os.listdir(scan_dir):
        m = _POSE_RE.search(name)
        if not m:
            continue
        with open(os.path.join(scan_dir, name)) as f:
            pose = json.load(f)
        lcam = pose.get("camera_left")
        rcam = pose.get("camera_right")
        if not lcam or not rcam:
            skipped += 1
            continue
        matches.append((int(m.group(1)), pose.get("label", m.group(1)) or m.group(1), [lcam, rcam]))
    matches.sort(key=lambda t: t[0])
    return matches, skipped


def _scan_dirs(path):
    if any(_POSE_RE.search(f) for f in os.listdir(path)):
        return [path]
    sub = sorted(
        os.path.join(path, d) for d in os.listdir(path)
        if os.path.isdir(os.path.join(path, d))
        and any(_POSE_RE.search(f) for f in os.listdir(os.path.join(path, d)))
    )
    return sub


def _load_settings(scan_dir):
    p = os.path.join(scan_dir, "render_settings.json")
    if os.path.isfile(p):
        with open(p) as f:
            return json.load(f)
    return {}


def plot_scan_dir(scan_dir, out=None, ray_len=None):
    poses, skipped = _load_poses(scan_dir)
    if not poses:
        raise ValueError(f"Keine Kamera-pose.json (camera_left/right) in {scan_dir}")
    settings = _load_settings(scan_dir)
    if ray_len is None:
        ray_len = _default_ray_len(settings)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d", proj_type="ortho")

    left = np.array([p[2][0]["position"] for p in poses])
    right = np.array([p[2][1]["position"] for p in poses])
    ql = np.array([p[2][0]["quaternion"] for p in poses])
    qr = np.array([p[2][1]["quaternion"] for p in poses])

    dl = np.array([_rotate(q, (0, 0, -1)) * ray_len for q in ql])
    dr = np.array([_rotate(q, (0, 0, -1)) * ray_len for q in qr])

    all_pts = np.vstack([left, right, np.zeros((1, 3))])
    L = float(np.abs(all_pts).max())
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlim(-L, L)
    ax.set_ylim(-L, L)
    ax.set_zlim(-L, L)
    ax.view_init(elev=35, azim=-60)

    for lb, pts, d, color in (("links", left, dl, "#d62728"), ("rechts", right, dr, "#1f77b4")):
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=color, s=30, label=lb, depthshade=False)
        ax.quiver(
            pts[:, 0], pts[:, 1], pts[:, 2],
            d[:, 0], d[:, 1], d[:, 2],
            color=color, arrow_length_ratio=0.0, linewidth=0.8,
        )

    mid = (left + right) / 2
    tips_l = left + dl
    tips_r = right + dr
    for i in range(len(poses)):
        ax.plot(
            [tips_l[i, 0], tips_r[i, 0]],
            [tips_l[i, 1], tips_r[i, 1]],
            [tips_l[i, 2], tips_r[i, 2]],
            color="#2ca02c", linewidth=1.2, zorder=2,
            label="Stereo-Paar (L\u2013R)" if i == 0 else None,
        )
        ax.plot(
            [left[i, 0], right[i, 0]],
            [left[i, 1], right[i, 1]],
            [left[i, 2], right[i, 2]],
            color="#2ca02c", linewidth=1.2, zorder=2,
        )

    if len(left) > 1:
        ax.plot(mid[:, 0], mid[:, 1], mid[:, 2], color="0.1", linestyle="--", linewidth=0.8)

    jaw_bbox = _jaw_bbox_m(_settings_get(settings, "jaw_folder", 1), _settings_get(settings, "jaw_type", "lower"))
    if jaw_bbox:
        lo, hi = jaw_bbox
        z_plane = hi[2]
        xs = [lo[0], hi[0], hi[0], lo[0], lo[0]]
        ys = [lo[1], lo[1], hi[1], hi[1], lo[1]]
        ax.plot(xs, ys, [z_plane] * 5, color="0.85", linewidth=1.0, zorder=1)

    for row, (idx, label) in enumerate((p[0], p[1]) for p in poses):
        ax.text(mid[row, 0], mid[row, 1], mid[row, 2], f" {label}",
                color="0.1", fontsize=8, zorder=5)

    axis_len = max(0.01, min(0.02, L * 0.5))
    for axis, color in (((1, 0, 0), "#d62728"), ((0, 1, 0), "#2ca02c"), ((0, 0, 1), "#1f77b4")):
        ax.quiver(0, 0, 0, *[c * axis_len for c in axis], color=color, arrow_length_ratio=0.15, linewidth=1.5)
    ax.scatter([0], [0], [0], s=14, c="0.2", marker="+", zorder=6)
    ax.text(axis_len * 0.04, axis_len * 0.04, 0, "Gebiss-Ursprung",
            color="0.25", fontsize=8, zorder=6)

    ax.set_xlabel("x (m)"), ax.set_ylabel("y (m)"), ax.set_zlabel("z (m)")
    ax.set_title(os.path.basename(scan_dir))
    info = _settings_get(settings, "start_position")
    if info:
        dists = np.linalg.norm(right - left, axis=1)
        dmin, dmax = float(dists.min()), float(dists.max())
        if dmax - dmin < 1e-4:
            base_line = f"|L-R|={dmin:.4f} m konstant (n={len(dists)})"
        else:
            base_line = f"|L-R| variiert {dmin:.4f}..{dmax:.4f} m (!)"
        ax.text2D(0.02, 0.02,
                  f"Gebiss-Frame  start={info}  baseline_m={_settings_get(settings, 'baseline_m', '?')}  "
                  f"fov={_settings_get(settings, 'camera_fov_deg', '?')}°\n{base_line}",
                  transform=ax.transAxes, fontsize=8)
    ax.legend(loc="upper right", fontsize=8)

    if out is None:
        plt.show()
    else:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        fig.savefig(out, dpi=150)
        plt.close(fig)
        print(f"  → {out}")
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description="Kamerasichtachsen aus scan pose.json visualisieren")
    ap.add_argument("path", help="Scan-Ordner oder render/-Root")
    ap.add_argument("--out", help="PNG-Datei (eine Figur) oder Verzeichnis (mehrere Scans)")
    ap.add_argument("--ray-len", type=float, default=None, help="Sichtachsen-Laenge in m (Default: camera_far_m aus render_settings, sonst config CAMERA_FAR_M)")
    args = ap.parse_args(argv)

    if not os.path.isdir(args.path):
        print(f"  Pfad existiert nicht: {args.path}", file=sys.stderr)
        return 1
    dirs = _scan_dirs(args.path)
    if not dirs:
        print(f"  Keine Scan-Ordner (ohne <n>_pose.json) unter {args.path}", file=sys.stderr)
        return 1

    for i, scan_dir in enumerate(dirs):
        base = os.path.basename(scan_dir)
        poses, skipped = _load_poses(scan_dir)
        if not poses:
            if skipped:
                print(f"  {base}: {skipped} legacy pose.json ohne Kamera-Daten – übersprungen")
            else:
                print(f"  {base}: keine <n>_pose.json – übersprungen")
            continue
        note = f" ({skipped} ohne Kamera-Daten übersprungen)" if skipped else ""
        print(f"  {base}: {len(poses)} Kamera-poses{note}")
        if args.out is None:
            plot_scan_dir(scan_dir, ray_len=args.ray_len)
        elif len(dirs) == 1:
            plot_scan_dir(scan_dir, out=args.out, ray_len=args.ray_len)
        else:
            os.makedirs(args.out, exist_ok=True)
            plot_scan_dir(
                scan_dir,
                out=os.path.join(args.out, base + ".png"),
                ray_len=args.ray_len,
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())