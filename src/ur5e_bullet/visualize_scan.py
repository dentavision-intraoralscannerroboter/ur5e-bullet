"""Visualisiert die Kamera-Sichtachsen aller pose.json eines gerenderten Scan-Ordners.

Nimmt einen Scan-Ordner (enthält <n>_pose.json, optional render_settings.json)
oder das render/-Root (alle Scan-Unterordner) und zeichnet im Gebiss-Frame (in m):

  - Kamera-Zentren: links = rot, rechts = blau
  - Sichtachse je Kamera: Pfeil entlang R(q) @ (0,0,-1)  (Blender-Kamera-Blickrichtung)
  - Stereo-Baseline: graue Linie L <-> R je Waypoint
  - Sweep-Reihenfolge: gestrichelte Linie durch die linken Kameras
  - Gebiss-Achsen im Ursprung (X rot, Y grün, Z blau)

Beispiele:

    poetry run visualize-scan render/lower_1_o1l_1920x1080_lat1.5mm
    poetry run visualize-scan render/ --out /tmp/cameras
    poetry run visualize-scan <scan_dir> --ray-len 0.3 --out plot.png
"""

import argparse
import json
import os
import re
import sys

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib fehlt:  poetry add 'matplotlib>=3.9,<4.0'", file=sys.stderr)
    sys.exit(1)

_POSE_RE = re.compile(r"(\d+)_pose\.json$")


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


def plot_scan_dir(scan_dir, out=None, ray_len=0.25):
    poses, skipped = _load_poses(scan_dir)
    if not poses:
        raise ValueError(f"Keine Kamera-pose.json (camera_left/right) in {scan_dir}")
    settings = _load_settings(scan_dir)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    left = np.array([p[2][0]["position"] for p in poses])
    right = np.array([p[2][1]["position"] for p in poses])
    ql = np.array([p[2][0]["quaternion"] for p in poses])

    dl = np.array([_rotate(q, (0, 0, -1)) * ray_len for q in ql])

    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=35, azim=-60)

    for lb, pts, color in (("links", left, "#d62728"), ("rechts", right, "#1f77b4")):
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=color, s=30, label=lb, depthshade=False)
        ax.quiver(
            pts[:, 0], pts[:, 1], pts[:, 2],
            dl[:, 0], dl[:, 1], dl[:, 2],
            color=color, arrow_length_ratio=0.25, linewidth=0.8,
        )

    for i in range(len(poses)):
        ax.plot(
            [left[i, 0], right[i, 0]],
            [left[i, 1], right[i, 1]],
            [left[i, 2], right[i, 2]],
            color="0.6", linewidth=0.6, zorder=0,
        )

    if len(left) > 1:
        ax.plot(left[:, 0], left[:, 1], left[:, 2], color="0.1", linestyle="--", linewidth=0.8)

    for row, (idx, label) in enumerate((p[0], p[1]) for p in poses):
        ax.text(left[row, 0], left[row, 1], left[row, 2], f" {label}",
                color="0.1", fontsize=8, zorder=5)

    all_pts = np.vstack([left, right, np.zeros((1, 3))])
    span = float(np.max(np.abs(all_pts)))
    axis_len = max(0.02, span)
    for axis, color in (((1, 0, 0), "#d62728"), ((0, 1, 0), "#2ca02c"), ((0, 0, 1), "#1f77b4")):
        ax.quiver(0, 0, 0, *[c * axis_len for c in axis], color=color, arrow_length_ratio=0.15, linewidth=1.5)

    ax.set_xlabel("x (m)"), ax.set_ylabel("y (m)"), ax.set_zlabel("z (m)")
    ax.set_title(os.path.basename(scan_dir))
    info = settings.get("start_position")
    if info:
        ax.text2D(0.02, 0.02,
                  f"Gebiss-Frame  start={info}  baseline_m={settings.get('baseline_m', '?')}  "
                  f"fov={settings.get('camera_fov_deg', '?')}°",
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
    ap.add_argument("--ray-len", type=float, default=0.25, help="Sichtachsen-Laenge in m (Default 0.25)")
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