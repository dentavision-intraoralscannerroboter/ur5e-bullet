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
`data/urdf_data.json` and `data/meshes` (`blender/rig.py`) and the robot and
scanner are then synchronized over a TCP socket:

- `blender` must be on `PATH`; the simulation launches it automatically
  (`blender --python blender/mirror.py -- --port=...`) when the mirror is enabled.
- The 3D viewport attaches to `ScannerCamera_R`
  (`ATTACH_VIEWPORT_TO_CAMERA=True`) and follows the camera view axis.
- The `render` command renders left/right stereo frames via Cycles on the GPU
  (`RENDER_DEVICE="GPU"`, output `render_L.png` / `render_R.png`).
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
| `render`               | Render scene in Blender (Cycles, stereo)      |
| `q`                    | Quit                                          |

Available start positions: `aussen1low`, `aussen2low`, `aussen2`, `oben`, `innen`.

## Configuration (`config.py`)

All simulation, rendering and scan parameters live in `config.py`.

| Section | Keys |
|---------|------|
| Paths | `PKG_DIR`, `JAWS_DIR`, `ROBOT_URDF_PATH` |
| Jaw (dental model) | `GEBISS_SCALE`, `GEBISS_COLL_CELL`, `GEBISS_ROUGHNESS`, `GEBISS_SPECULAR` |
| Boot start | `BOOT_START` (`tcp_pos`, `tcp_ori_deg` in deg) — `None` disables |
| Camera (RealSense D455) | `CAMERA_ROLL_DEG`, `CAMERA_SENSOR_W_MM`, `CAMERA_SENSOR_H_MM`, `CAMERA_FOV_DEG`, `CAMERA_LENS_MM`, `CAMERA_NEAR_M`, `CAMERA_FAR_M`, `CAMERA_DISPLAY_M`, `CAMERA_LATERAL_OFFSET` |
| Light | `LIGHT_POWER`, `LIGHT_OFFSET` |
| Render | `RENDER_W`, `RENDER_H`, `RENDER_ENGINE`, `RENDER_DEVICE`, `RENDER_TRANSPARENT` |
| Tool offset | `TOOL_OFFSET_POS`, `TOOL_OFFSET_ORN`, `CAMERA_OFFSET` |
| Blender sync | `ENABLE_BLENDER_SYNC` |
| Socket | `SOCKET_HOST`, `SOCKET_BUFFER`, `SOCKET_POLL_INTERVAL` |
| RRT planner | `RRT_RESTARTS`, `RRT_SMOOTH`, `RRT_SEED` |
| Scale factor | `S` (Blender units per meter) |
| Pybullet misc | `IK_LAMBDA`, `IK_TOLERANCE`, `GHOST_COLOR`, `PREVIEW_PAUSE`, `WAYPOINT_MARKER_RADIUS` |
| Viewport | `ATTACH_VIEWPORT_TO_CAMERA` |
| Pybullet GUI camera | `PB_CAMERA_DISTANCE`, `PB_CAMERA_YAW`, `PB_CAMERA_PITCH`, `PB_CAMERA_TARGET_POS` |
| Debug view axis | `DRAW_VIEW_STICK`, `VIEW_STICK_LENGTH`, `VIEW_STICK_RADIUS`, `VIEW_STICK_COLOR` |
| Look target | `LOOK_TARGET_RADIUS`, `LOOK_TARGET_COLOR` |
| Joint limits | `JOINTS` (`lower_deg`, `upper_deg`, `rest_deg` per joint, order = joint order) |
| Scan positions | `START_POSITIONS` (see below) |

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
blender/                  Blender-side scripts (mirror.py, rig.py, animate.py, decimate_stl.py)
data/                     URDF data, robot meshes, jaw meshes (meshes_jaws/)
tests/                    FK / IK checks
```