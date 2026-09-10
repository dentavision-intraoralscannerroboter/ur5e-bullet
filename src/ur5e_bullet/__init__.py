"""UR5e Bullet Simulation – Package Hub.

Re-exports aus den Untermodule fuer Rueckwaertskompatilitaet
mit Tests und externen Aufrufern (z.B. `ur5e_bullet._parse_command`).
"""

from .math_utils import _quat_mul, _quat_conj, _to_jaw_frame
from .camera import (_camera_poses_in_jaw, _camera_intrinsic,
                     _frustum_dims, _build_frustum_obj)
from .commands import Command, _parse_command
from .viz import (_draw_crosshair, _draw_waypoint_bodies,
                  _draw_look_target_body, _resolve_waypoints)
from .demo import demo_simulation
