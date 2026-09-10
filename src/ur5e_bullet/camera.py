import math
import os

import pybullet

from .math_utils import _quat_mul, _quat_conj, _to_jaw_frame

import importlib.util as _ilu
_cfg = _ilu.spec_from_file_location(
    "config",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config.py"),
)
_cfg_mod = _ilu.module_from_spec(_cfg)
_cfg.loader.exec_module(_cfg_mod)
CAMERA_ROLL_DEG = _cfg_mod.CAMERA_ROLL_DEG
CAMERA_LATERAL_OFFSET = _cfg_mod.CAMERA_LATERAL_OFFSET
CAMERA_FAR_M = _cfg_mod.CAMERA_FAR_M
CAMERA_NEAR_M = _cfg_mod.CAMERA_NEAR_M
CAMERA_FOV_DEG = _cfg_mod.CAMERA_FOV_DEG
CAMERA_SENSOR_W_MM = _cfg_mod.CAMERA_SENSOR_W_MM
CAMERA_SENSOR_H_MM = _cfg_mod.CAMERA_SENSOR_H_MM
CAMERA_LENS_MM = _cfg_mod.CAMERA_LENS_MM
RENDER_W = _cfg_mod.RENDER_W
RENDER_H = _cfg_mod.RENDER_H


def _camera_poses_in_jaw(sim, r_jaw, jaw_pos, q_jaw):
    """L/R-Kamera-Posen im Gebiss-Frame, identisch zur Render-Platzierung.

    Zur Renderzeit positioniert der Blender-Mirror die Kameras als
    TCP-in-Scanner-Frame +/- CAMERA_LATERAL_OFFSET (mirror.py _apply_tcp),
    mit fester Scanner->Kamera-Rotation (rig.py base_q @ cam_roll).
    Scanner-Frame->Welt ueber den echten pybullet scanner_link."""
    tcp_in_sc = sim.get_tcp_in_scanner_frame()
    sc_id = sim.joints["scanner_joint"].id
    sc_ls = pybullet.getLinkState(sim.ur5, sc_id, computeForwardKinematics=True)
    sc_pos, sc_orn = list(sc_ls[4]), list(sc_ls[5])
    q_base = pybullet.getQuaternionFromEuler([0.0, math.radians(-90.0), 0.0])
    q_roll = pybullet.getQuaternionFromEuler([0.0, 0.0, math.radians(CAMERA_ROLL_DEG)])
    q_cam_sc = _quat_mul(q_base, q_roll)
    out = []
    for sign in (-1.0, 1.0):
        cam_sc = [tcp_in_sc[0], tcp_in_sc[1] + sign * CAMERA_LATERAL_OFFSET, tcp_in_sc[2]]
        wpos, wori = pybullet.multiplyTransforms(sc_pos, sc_orn, cam_sc, q_cam_sc)
        out.append((
            _to_jaw_frame(r_jaw, jaw_pos, wpos),
            _quat_mul(_quat_conj(q_jaw), wori),
        ))
    return out


def _camera_intrinsic():
    return {
        "fx_px": round(RENDER_W / CAMERA_SENSOR_W_MM * CAMERA_LENS_MM, 2),
        "fy_px": round(RENDER_H / CAMERA_SENSOR_H_MM * CAMERA_LENS_MM, 2),
        "cx_px": round(RENDER_W / 2.0, 1),
        "cy_px": round(RENDER_H / 2.0, 1),
        "sensor_w_mm": CAMERA_SENSOR_W_MM,
        "sensor_h_mm": CAMERA_SENSOR_H_MM,
        "lens_mm": round(CAMERA_LENS_MM, 3),
    }


def _frustum_dims(far=CAMERA_FAR_M, near=CAMERA_NEAR_M, fov_deg=CAMERA_FOV_DEG,
                  sensor_w_mm=CAMERA_SENSOR_W_MM, sensor_h_mm=CAMERA_SENSOR_H_MM):
    """Halbe Weit-Ebenen-Maße + Nah/Far-Verhältnis des Kamera-Frustums (rein numerisch)."""
    hfov = math.radians(fov_deg) / 2
    vfov = math.atan(math.tan(hfov) * sensor_h_mm / sensor_w_mm)
    return {
        "half_w": far * math.tan(hfov) + CAMERA_LATERAL_OFFSET,
        "half_h": far * math.tan(vfov),
        "far": far,
        "near": near,
        "n_ratio": near / far,
    }


def _build_frustum_obj(path, n_ratio=0.1):
    """Schreibt ein kanonisches Frustum-OBJ (Far-Ebene ±1, Nah-Ebene ±n_ratio).
    Dreiecke werden programmatisch auf Aussen-Winding korrigiert; gibt True
    zurueck, wenn die Datei neu geschrieben wurde."""
    if os.path.isfile(path) and os.path.getsize(path) > 0:
        return False
    n = n_ratio
    near = [(n, n, -n), (-n, n, -n), (-n, -n, -n), (n, -n, -n)]
    far = [(1, 1, -1), (-1, 1, -1), (-1, -1, -1), (1, -1, -1)]
    verts = near + far
    quads = [
        (0, 1, 5, 4),   # Top (+Y)
        (1, 2, 6, 5),   # Links (-X)
        (2, 3, 7, 6),   # Unten (-Y)
        (3, 0, 4, 7),   # Rechts (+X)
    ]
    faces = [0, 1, 2, 0, 2, 3,        # Nah-Kappe
             4, 6, 5, 4, 6, 7]        # Weit-Kappe
    for a, b, c, d in quads:
        faces += [a, c, b, a, d, c]
    center = [sum(v[k] for v in verts) / len(verts) for k in range(3)]
    ordered = []
    for i in range(0, len(faces), 3):
        tri = faces[i:i + 3]
        p = [verts[j] for j in tri]
        e1 = [p[1][k] - p[0][k] for k in range(3)]
        e2 = [p[2][k] - p[0][k] for k in range(3)]
        normal = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ]
        ctri = [sum(v[k] for v in p) / 3 for k in range(3)]
        to_out = [ctri[k] - center[k] for k in range(3)]
        nlen = math.sqrt(sum(x * x for x in normal)) or 1.0
        tnlen = math.sqrt(sum(x * x for x in to_out)) or 1.0
        dot = sum(normal[k] * to_out[k] for k in range(3)) / (nlen * tnlen)
        ordered.append(tri if dot >= 0 else list(reversed(tri)))
    lines = ["o frustum"]
    for x, y, z in verts:
        lines.append(f"v {x} {y} {z}")
    for tri in ordered:
        lines.append(f"f {tri[0]+1} {tri[1]+1} {tri[2]+1}")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return True
