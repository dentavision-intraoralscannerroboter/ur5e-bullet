# ur5e-bullet

UR5e dental-scan simulation. A UR5e robot with a stereo camera rig (RealSense D455 modeling) mounted at the TCP is simulated in pybullet. The robot state is mirrored live to a Blender GUI instance over a TCP socket, which renders the scanner cameras (Cycles). Several preconfigured scan start positions generate waypoints the arm moves along while looking at the jaw.

## Requirements

- Python >= 3.14
- Poetry >= 2.x
- Blender (GUI-capable build) available as `blender` on `PATH`

## Installation

### Linux

```bash
git clone <repo>
cd ur5e-bullet
poetry install
```

### macOS

pybullet ships no macOS wheels, so it always builds from source. The bundled
legacy zlib breaks on modern clang (the `TARGET_OS_MAC` macro activates a
`fdopen` stub), so the build fails without this workaround:

```bash
CFLAGS="-fno-define-target-os-macros" poetry install
```

Xcode Command Line Tools are required. The `CFLAGS` are needed for every fresh
pybullet build. Alternatives: the `pybullet-arm64` fork (wheels up to Python
3.13) or conda-forge binaries.

## Blender prerequisite

No `.blend` file is needed — the Blender scene is built procedurally from
`data/urdf_data.json` and `data/meshes_arm` (`blender/rig.py`) and the robot and
scanner are then synchronized over a TCP socket:

- `blender` must be on `PATH`; the simulation launches it automatically
  (`blender --python blender/mirror.py -- --port=...`) when the mirror is enabled.
- The 3D viewport attaches to `ScannerCamera_R`
  (`ATTACH_VIEWPORT_TO_CAMERA=True`) and follows the camera view axis.
- Scans render left/right stereo frames via Cycles on the GPU
  (`RENDER_DEVICE="GPU"`), stored under `render/{...}` folders.
- If Blender is missing, the mirror is disabled and the simulation keeps
  running; set `ENABLE_BLENDER_SYNC=False` to disable the sync entirely.

## Running

```bash
poetry run ur5e
```

(equivalent to `poetry run python -m ur5e_bullet`).

On boot the arm moves to `BOOT_START` (no jaw loaded), the pybullet GUI opens
and the Blender mirror viewport appears. Use `start <name>` to move to a scan
position and load the jaw.

## Console commands

| Command                | Effect                                        |
|------------------------|-----------------------------------------------|
| `m x y z [rx ry rz]`   | Move via RRT (asks for confirmation)          |
| `o x y z`              | Set tool offset                               |
| `s <speed>`            | Global speed (default 0.5)                    |
| `@`                    | Reset to last joint config (after manual drag)|
| `start <name>`         | Move to start position, load jaw              |
| `jaw <nr> [upper/lower]` | Load jaw from `data/meshes_jaws/<nr>` (only after `start`) |
| `+ [n]` / `- [n]`      | Next / previous waypoint (only after `start`) |
| `scan`                 | Drive all waypoints (0..max), save TCP poses and render L/R at each (only after `start`) |
| `scan-all <start1> [start2 ...]` | `start` + full `scan` for each given position in sequence (unreachable ones are skipped, render failure aborts) |
| `q`                    | Quit                                          |

Available start positions: `a1l`, `a2l`, `aussen2`, `o1l`, `innen`.

## Configuration (`config.py`)

All simulation, rendering and scan parameters live in `config.py`. The file is
sorted by consumer: values used by only one module first, shared values below.

| Consumer | Keys |
|----------|------|
| all modules (shared paths) | `PROJECT_ROOT`, `ROBOT_URDF_PATH`, `JAWS_DIR`, `BLENDER_URDF_DATA_JSON`, `ARM_MESH_DIR`, `SCANNER_STAB_STL`, `MIRROR_SCRIPT`, `RENDER_DIR` |
| `sim.py` only | `TOOL_OFFSET_POS`, `TOOL_OFFSET_ORN` · `GEBISS_COLL_CELL` · `IK_LAMBDA`, `IK_TOLERANCE` · `RRT_RESTARTS`, `RRT_SMOOTH`, `RRT_SEED` · `JOINTS` (`lower_deg`, `upper_deg`, `rest_deg` per joint, order = joint order) |
| `sim.py` + `__init__.py` | `PB_CAMERA_DISTANCE`, `PB_CAMERA_YAW`, `PB_CAMERA_PITCH`, `PB_CAMERA_TARGET_POS` |
| `sim.py` + `blender/rig.py` | `GEBISS_SCALE` |
| `__init__.py` only | `BOOT_START` (`tcp_pos`, `tcp_ori_deg` in deg) — `None` disables · `RENDER_TIMEOUT` · `PREVIEW_PAUSE`, `WAYPOINT_MARKER_RADIUS` · `ENABLE_BLENDER_SYNC` · `DRAW_CAMERA_FRUSTUM` (blau-transparentes Kamerafrustum bis `CAMERA_FAR_M`, nur bei geladener Startposition), `CAMERA_FRUSTUM_COLOR`, `CAMERA_FRUSTUM_ALPHA` · `LOOK_TARGET_RADIUS`, `LOOK_TARGET_COLOR` · `START_POSITIONS` (see below) |
| `__init__.py` + `blender/rig.py` | `CAMERA_ROLL_DEG`, `CAMERA_SENSOR_W_MM`, `CAMERA_SENSOR_H_MM`, `CAMERA_FOV_DEG`, `CAMERA_LENS_MM`, `CAMERA_LATERAL_OFFSET`, `CAMERA_FAR_M` · `RENDER_W`, `RENDER_H`, `RENDER_ENGINE`, `RENDER_DEVICE` |
| `blender/rig.py` only | `S` (Blender units per meter) · `CAMERA_NEAR_M`, `CAMERA_DISPLAY_M` · `LIGHT_POWER`, `LIGHT_OFFSET` · `GEBISS_ROUGHNESS`, `GEBISS_SPECULAR` · `RENDER_TRANSPARENT` |
| `blender/mirror.py` only | `SOCKET_BUFFER`, `SOCKET_POLL_INTERVAL` · `ATTACH_VIEWPORT_TO_CAMERA` |

Only edit `config.py`; the values are forwarded into the live session.

### Start positions (`START_POSITIONS`)

Each entry `"<name>"` supports:

| Key | Meaning |
|-----|---------|
| `tcp_pos` | Target TCP position `[x, y, z]` |
| `tcp_ori_deg` | Target TCP orientation in degrees `[r, p, y]` |
| `approach` | Optional list of intermediate poses (own IK seed each) before the final pose |
| `jaw_pos`, `jaw_euler_deg` | Jaw placement (position + orientation in deg) |
| `jaw_folder`, `jaw_type` | Jaw mesh folder in `data/meshes_jaws/` and type (`lower`/`upper`) |
| `generator` | Callable computing waypoints from this config (e.g. `parabola_waypoints`) |
| `waypoints` | Static waypoint list (fallback when no `generator`) |
| `parabola` | Generator parameters (`x0`, `a`, `z`, `n`, `y_max`, `power`) |
| `ori_anchors` | Orientation anchors for the generator (`start`, `mid`, `end`) |
| `look_at_jaw` | Aim the camera view axis at the jaw / look target, projected to plane height |
| `look_target` | Optional custom view-axis aim point `[x, y]` (default: jaw center, plane height) |
| `view` | Pybullet GUI camera for this position: `{"apply": bool, "distance": float, "yaw": float, "pitch": float, "target": [x, y, z]}` — `apply: false` (or absent) leaves the camera untouched |

## Layout

```
config.py                 all simulation/render/scan parameters
waypoints.py              waypoint generators (parabola_waypoints)
src/ur5e_bullet/          pybullet simulation package (__init__.py = CLI + demo_simulation)
  sim.py                  UR5Sim (physics, IK, RRT, mirror)
  blender_link.py         TCP-socket mirror → Blender
  visualize_scan.py       3D-Plot der Kamerasichtachsen aus scan pose.json
                          (Sichtachsenlaenge = camera_far_m aus render_settings,
                          Stereo-Paare L-R als gruene Linie an Kamerapunkten und
                          Pfeilspitzen, gestrichelte Linie durch die Waypoint-Mittelpunkte;
                          CLI `visualize-scan <scan_dir|render/> [--out png|dir] [--ray-len m]`)
blender/                  Blender-side scripts (mirror.py, rig.py)
scripts/                  standalone tools (regenerate_urdf_data.py)
data/                     robot description (URDF + meshes), arm/mesh assets, jaw meshes
archive/                  Archived, unused scripts and assets (git-ignored)
tests/                    FK / IK checks
```