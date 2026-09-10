import pybullet

from .config import (
    WAYPOINT_MARKER_RADIUS,
    LOOK_TARGET_RADIUS,
    LOOK_TARGET_COLOR,
)


def _draw_crosshair(pos, color, items, label=None):
    h = 0.03
    items.append(pybullet.addUserDebugLine(
        [pos[0]-h, pos[1], pos[2]], [pos[0]+h, pos[1], pos[2]], color, 2,
    ))
    items.append(pybullet.addUserDebugLine(
        [pos[0], pos[1]-h, pos[2]], [pos[0], pos[1]+h, pos[2]], color, 2,
    ))
    items.append(pybullet.addUserDebugLine(
        [pos[0], pos[1], pos[2]-h], [pos[0], pos[1], pos[2]+h], color, 2,
    ))
    if label:
        items.append(pybullet.addUserDebugText(
            label, [pos[0], pos[1], pos[2]+0.04], color, 0.8,
        ))
    return items


def _draw_waypoint_bodies(wps):
    """Erzeugt fuer jeden Waypoint einen kleinen, persistenten Kugel-Body.

    Waypoints werden als echte pybullet-Bodies statt als Debug-Items dargestellt:
    Damit sind sie von removeAllUserDebugItems() und der Debug-Handle-Invalidierung
    (die zu Geister-Resten fuehrt) vollstaendig unberuehrt. Sie bleiben stehen, bis
    die Bodies explizit per removeBody() entfernt werden -> kein staendiges
    Neuzeichnen. Kugeln erhalten KEINE Kollisionsgeometrie, daher kollidieren sie
    nie mit dem Roboter und zaehlen nicht als Hindernisse."""
    bodies = []
    for wp in wps:
        pos = wp["tcp_pos"]
        vis = pybullet.createVisualShape(
            pybullet.GEOM_SPHERE, radius=WAYPOINT_MARKER_RADIUS, rgbaColor=[0.2, 0.8, 1.0, 0.9],
        )
        body = pybullet.createMultiBody(
            baseVisualShapeIndex=vis, basePosition=pos,
        )
        bodies.append(body)
    return bodies


def _draw_look_target_body(pos):
    """Persistenter, kollisionsfreier Marker fuer das Blickachse-Ziel (Look-Target).
    Wie die Waypoints: nur visuelle Geometrie, kollidiert nie mit dem Roboter."""
    vis = pybullet.createVisualShape(
        pybullet.GEOM_SPHERE, radius=LOOK_TARGET_RADIUS, rgbaColor=LOOK_TARGET_COLOR,
    )
    return pybullet.createMultiBody(baseVisualShapeIndex=vis, basePosition=pos)


def _resolve_waypoints(cfg):
    """Erzeugt die Liste der Waypoint-Dicts einer Startposition.

    - 'generator' : echte Python-Funktion (in config.py definiert), die aus
                    der Startposition-CFG Position UND Orientierung berechnet.
    - 'waypoints' : fallback auf eine feste Liste (dicts mit tcp_pos/
                    tcp_ori_deg), wie bisher.
    """
    gen = cfg.get("generator")
    if callable(gen):
        return gen(cfg)
    return cfg.get("waypoints", [])
