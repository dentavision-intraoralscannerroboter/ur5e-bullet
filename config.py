import math
import os

import importlib.util as _ilu

# ── Bootstrap ───────────────────────────────────────────────────────────────
PKG_DIR = os.path.dirname(os.path.abspath(__file__))

# Waypoint-Generator: ausgelagert in das Schwestermodul waypoints.py
# (sys.path-unabhaengig geladen, wie der config-Import in __init__/sim).
_wp_spec = _ilu.spec_from_file_location("waypoints", os.path.join(PKG_DIR, "waypoints.py"))
_wp_mod = _ilu.module_from_spec(_wp_spec)
_wp_spec.loader.exec_module(_wp_mod)
parabola_waypoints = _wp_mod.parabola_waypoints


# ── Pfade ───────────────────────────────────────────────────────────────────
PROJECT_ROOT = PKG_DIR
ROBOT_URDF_PATH = os.path.join(PKG_DIR, "data", "robot_description", "urdf", "ur5e.urdf")
JAWS_DIR = os.path.join(PKG_DIR, "data", "meshes_jaws")
ARM_MESH_DIR = os.path.join(PKG_DIR, "data", "meshes_arm")
BLENDER_URDF_DATA_JSON = os.path.join(PKG_DIR, "data", "urdf_data.json")
SCANNER_STAB_STL = os.path.join(PKG_DIR, "data", "robot_description", "meshes", "scanner-stab.stl")
RENDER_DIR = os.path.join(PKG_DIR, "render")


# ── Roboter / Kinematik ────────────────────────────────────────────────────
# Tool-Offset (Scanner → TCP)
TOOL_OFFSET_POS = [0.213, 0, -0.006]
TOOL_OFFSET_ORN = [0, 0, 0, 1]

# Kollisions-Abstand des Gebisses
GEBISS_COLL_CELL = 1.5

# IK / RRT
IK_LAMBDA = 0.05
IK_TOLERANCE = 0.08
RRT_RESTARTS = 30
RRT_SMOOTH = 30
RRT_SEED = 0

# Joint Limits (Winkel in Grad)
# Ein Eintrag pro steuerbarem Gelenk, Reihenfolge = Joint-Reihenfolge
# (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3).
# 'rest': Neutralpose (fuer IK-Seed).
# wrist_3: Bereich auf -180..+540 (720°) vergroessert, um 180°-Orientierungsdrehungen
#          der Blickrichtung ohne Umklappen/Ans-Limit-Stossen zu ermoeglichen.
JOINTS = [
    {"name": "shoulder_pan",  "lower_deg": -180, "upper_deg": 180, "rest_deg": 0},
    {"name": "shoulder_lift", "lower_deg": -180, "upper_deg": 180, "rest_deg": -90},
    {"name": "elbow",         "lower_deg": 0,    "upper_deg": 180, "rest_deg": 90},
    {"name": "wrist_1",       "lower_deg": 0,    "upper_deg": 360, "rest_deg": 180},
    {"name": "wrist_2",       "lower_deg": -180, "upper_deg": 180, "rest_deg": -90},
    {"name": "wrist_3",       "lower_deg": -360, "upper_deg": 360, "rest_deg": 0},
]


# ── Kamera / Sensor (RealSense D455) ───────────────────────────────────────
CAMERA_ROLL_DEG = -90
CAMERA_SENSOR_W_MM = 36.0
CAMERA_SENSOR_H_MM = CAMERA_SENSOR_W_MM * 9 / 16
CAMERA_FOV_DEG = 87
CAMERA_LENS_MM = CAMERA_SENSOR_W_MM / (2 * math.tan(math.radians(CAMERA_FOV_DEG) / 2))
# Seitlicher Versatz der beiden Kameras (Stereo-Baseline) relativ zum TCP,
# in Scanner-lokalen Koordinaten (Y-Achse), in Meter (S=1 -> 1 BU).
# Jede Kamera wird um +/- CAMERA_LATERAL_OFFSET quer zur Blickrichtung versetzt.
CAMERA_LATERAL_OFFSET = 0.0015
# Kamera-Clipping
CAMERA_NEAR_M = 0.001
CAMERA_FAR_M = 0.04
CAMERA_DISPLAY_M = 0.2


# ── Rendering ───────────────────────────────────────────────────────────────
RENDER_W = 1920
RENDER_H = 1080
RENDER_ENGINE = "CYCLES"
RENDER_DEVICE = "GPU"
# Gesamttimeout pro Render-Paar (L+R, Worst-Case ~6 min/Bild), danach Scan-Abbruch.
RENDER_TIMEOUT = 900
RENDER_TRANSPARENT = False


# ── PyBullet GUI ────────────────────────────────────────────────────────────
# Orbit-Kamera beim Start: "Startposition" = cameraTargetPosition + cameraDistance;
# "Startwinkel" = yaw + pitch.
PB_CAMERA_DISTANCE   = 1.0
PB_CAMERA_YAW        = 70.0
PB_CAMERA_PITCH      = -25.0
PB_CAMERA_TARGET_POS = [0.6, 0.0, 0.4]
PREVIEW_PAUSE = 0.6
WAYPOINT_MARKER_RADIUS = 0.001

# Boot-Startposition (nur Arm, kein Gebiss).
# Auf None setzen, um das Verhalten zu deaktivieren.
BOOT_START = {
    "tcp_pos":     [0.85, 0, 0.38],
    "tcp_ori_deg": [0, 0, 0],
}


# ── Visualisierung / Debug ──────────────────────────────────────────────────
# Kamera-Frustum (Sichtvolumen bis CAMERA_FAR_M, nur bei Startposition sichtbar)
DRAW_CAMERA_FRUSTUM = True
CAMERA_FRUSTUM_COLOR = [0.0, 0.55, 1.0]
CAMERA_FRUSTUM_ALPHA = 0.2

# Look-Target (Blickachse-Ziel, Default: Gebiss-Mittelpunkt)
LOOK_TARGET_RADIUS = 0.008
LOOK_TARGET_COLOR = [0.1, 1.0, 0.3, 0.95]


# ── Gebiss / Material ──────────────────────────────────────────────────────
# Position/Orientierung kommen NICHT aus globalen Konstanten, sondern aus den
# Startpositionen (START_POSITIONS[<name>]["jaw_pos"] / ["jaw_euler_deg"]).
GEBISS_SCALE = [0.001, 0.001, 0.001]
GEBISS_ROUGHNESS = 0.8
GEBISS_SPECULAR = 0.2


# ── Blender ─────────────────────────────────────────────────────────────────
S = 1
LIGHT_POWER = 0.0009
LIGHT_OFFSET = [0.008, 0.025, 0.213]

# Blender-Sync (Robot-Zustand per TCP-Socket an Blender-GUI)
ENABLE_BLENDER_SYNC = True
MIRROR_SCRIPT = os.path.join(PKG_DIR, "blender", "mirror.py")
SOCKET_BUFFER = 4096
SOCKET_POLL_INTERVAL = 0.05

# Viewport an scene.camera (ScannerCamera_R) heften
ATTACH_VIEWPORT_TO_CAMERA = True


# ── Scan-Positionen ─────────────────────────────────────────────────────────
# tcp_ori_deg / jaw_euler_deg: Orientierung in Grad
# approach (optional): Liste von Zwischen-TCP-Posen, die beim 'start <name>'
#   nacheinander angefahren werden (jeweils mit eigenem IK-Seed), bevor die
#   endgueltige tcp_pos angefahren wird – zur besseren Beeinflussung des IK.
#   Achtung: approach != waypoints (letztere sind die '+/'-navigierbaren Punkte).
START_POSITIONS = {
    "a1l": {
        "tcp_pos":  [0.616, 0, 0.296],
        "tcp_ori_deg": [180, 90, 0],
        "approach": [
            {"tcp_pos": [0.85, 0, 0.38], "tcp_ori_deg": [0, 0, 0], "label": "1", "use_current_seed": False},
            {"tcp_pos": [0.616, 0, 0.296], "tcp_ori_deg": [0, 90, 0], "label": "2", "use_current_seed": True},
        ],
        "jaw_pos":  [0.65, 0, 0.3],
        "jaw_euler_deg": [0, 0, 90],
        "jaw_folder": 1,
        "jaw_type":  "lower",
        "generator": parabola_waypoints,
        "look_at_jaw": True,
        "look_target": [0.662, 0.0],
        "parabola":  {"x0": 0.616, "a": 0.047, "z": 0.296, "n": 21, "y_max": 0.037, "power": 4},
        "ori_anchors": {"start": [90, 0, 0], "mid": [180, 90, 0], "end": [-90, 0, 0]},
        "view": {"apply": True, "distance": 0.1, "yaw": 90.0, "pitch": -89.0, "target": [0.65, 0.0, 0.3]},
    },
    "a2l": {
        "tcp_pos":  [0.619, 0, 0.297],
        "tcp_ori_deg": [180, 90, 0],
        "approach": [
            {"tcp_pos": [0.85, 0, 0.38], "tcp_ori_deg": [0, 0, 0], "label": "1", "use_current_seed": False},
            {"tcp_pos": [0.619, 0, 0.297], "tcp_ori_deg": [0, 90, 0], "label": "2", "use_current_seed": True},
        ],
        "jaw_pos":  [0.65, 0, 0.3],
        "jaw_euler_deg": [0, 0, 90],
        "jaw_folder": 2,
        "jaw_type":  "lower",
        "generator": parabola_waypoints,
        "look_at_jaw": True,
        "look_target": [0.662, 0.0],
        "parabola":  {"x0": 0.619, "a": 0.05, "z": 0.297, "n": 21, "y_max": 0.039, "power": 4}, #37
        "ori_anchors": {"start": [90, 0, 0], "mid": [180, 90, 0], "end": [-90, 0, 0]},
        "view": {"apply": True, "distance": 1.0, "yaw": 90.0, "pitch": -25.0, "target": [0.66, 0.0, 0.35]},
    },
    "aussen2": {
        "tcp_pos":  [0.615, 0, 0.295],
        "tcp_ori_deg": [0, -90, 0],
        "approach": [
            {"tcp_pos": [0.85, 0, 0.38], "tcp_ori_deg": [0, 0, 0], "label": "1", "use_current_seed": False},
        ],
        "jaw_pos":  [0.65, 0, 0.3],
        "jaw_euler_deg": [0, 0, 90],
        "jaw_folder": 1,
        "jaw_type":  "lower",
        "generator": parabola_waypoints,
        "look_at_jaw": True,
        "look_target": [0.65, 0.0],
        "parabola":  {"x0": 0.615, "a": 0.0463, "z": 0.295, "n": 21, "y_max": 0.05, "power": 4},
        "ori_anchors": {"start": [90, 0, 0], "mid": [180, 90, 0], "end": [-90, 0, 0]},
        "view": {"apply": True, "distance": 1.0, "yaw": 90.0, "pitch": -25.0, "target": [0.65, 0.0, 0.35]},
    },
    "o1l": {
        "tcp_pos":  [0.8265, 0.0, 0.31],
        "tcp_ori_deg": [0, 0, 0],
        "approach": [],
        "jaw_pos":  [0.85, 0, 0.3],
        "jaw_euler_deg": [0, 0, 90],
        "jaw_folder": 1,
        "jaw_type":  "lower",
        "generator": parabola_waypoints,
        "look_at_jaw": False,
        "look_target": [0.862, 0.0],
        "parabola":  {"x0": 0.8265, "a": 0.04, "z": 0.31, "n": 21, "y_max": 0.023, "power": 2},
        "ori_anchors": {"start": [0 , 0, 0], "mid": [0, 0, 0], "end": [0, 0, 0]},
        "view": {"apply": True, "distance": 0.1, "yaw": 90.0, "pitch": -89.0, "target": [0.85, 0.0, 0.3]},},
    "innen": {
        "tcp_pos":  [0.78, -0.05, 0.29],
        "tcp_ori_deg": [0, 0, -90],
        "approach": [],
        "jaw_pos":  [0.85, 0, 0.3],
        "jaw_euler_deg": [0, 0, 90],
        "jaw_folder": 1,
        "jaw_type":  "lower",
        "waypoints": [],
    },
}
