import os
import json
import math
import time
import tempfile

import pybullet

from .sim import UR5Sim
from .math_utils import _quat_mul
from .camera import (_camera_poses_in_jaw, _camera_intrinsic,
                     _frustum_dims, _build_frustum_obj)
from .commands import _parse_command
from .viz import (_draw_crosshair, _draw_waypoint_bodies,
                  _draw_look_target_body, _resolve_waypoints)
from .config import (
    START_POSITIONS,
    BOOT_START,
    PREVIEW_PAUSE,
    ENABLE_BLENDER_SYNC,
    DRAW_CAMERA_FRUSTUM,
    CAMERA_FRUSTUM_COLOR,
    CAMERA_FRUSTUM_ALPHA,
    PB_CAMERA_DISTANCE,
    PB_CAMERA_YAW,
    PB_CAMERA_PITCH,
    PB_CAMERA_TARGET_POS,
    RENDER_W,
    RENDER_H,
    RENDER_ENGINE,
    RENDER_DEVICE,
    RENDER_TIMEOUT,
    CAMERA_LATERAL_OFFSET,
    CAMERA_FOV_DEG,
    CAMERA_LENS_MM,
    CAMERA_ROLL_DEG,
    CAMERA_FAR_M,
    RENDER_DIR,
    SCANNER_BASE_ORN_DEG,
)


def demo_simulation():
    sim = UR5Sim(mirror=ENABLE_BLENDER_SYNC)

    def draw_tcp():
        pos, _ = sim.get_tcp_pose()
        _draw_crosshair(pos, [0, 1, 0], [])

    def _remove_camera_frustum():
        nonlocal camera_frustum_id
        if camera_frustum_id is not None:
            try:
                pybullet.removeBody(camera_frustum_id)
            except Exception:
                pass
            camera_frustum_id = None

    def draw_camera_frustum():
        """Zeichnet das blau-transparente Kamera-Sichtvolumen (Frustum) bis
        CAMERA_FAR_M exakt im Blender-Kamera-Frame. Persistenter Multibody,
        nur bei geladener Startposition sichtbar."""
        nonlocal camera_frustum_id
        if not DRAW_CAMERA_FRUSTUM or current_start is None:
            _remove_camera_frustum()
            return
        pos, _ = sim.get_tcp_pose()
        sc_id = sim.joints["scanner_joint"].id
        sc_ls = pybullet.getLinkState(sim.ur5, sc_id, computeForwardKinematics=True)
        sc_orn = list(sc_ls[5])
        q_cam_sc = _quat_mul(
            pybullet.getQuaternionFromEuler([math.radians(v) for v in SCANNER_BASE_ORN_DEG]),
            pybullet.getQuaternionFromEuler([0.0, 0.0, math.radians(CAMERA_ROLL_DEG)]),
        )
        cam_quat = _quat_mul(sc_orn, q_cam_sc)
        dims = _frustum_dims()
        obj = os.path.join(tempfile.gettempdir(), "ur5e_scan_frustum.obj")
        _build_frustum_obj(obj, dims["n_ratio"])
        if camera_frustum_id is None:
            vis = pybullet.createVisualShape(
                pybullet.GEOM_MESH,
                fileName=obj,
                meshScale=[dims["half_w"], dims["half_h"], dims["far"]],
                rgbaColor=[*CAMERA_FRUSTUM_COLOR, CAMERA_FRUSTUM_ALPHA],
            )
            camera_frustum_id = pybullet.createMultiBody(
                baseVisualShapeIndex=vis,
                basePosition=pos,
                baseOrientation=cam_quat,
            )
        else:
            pybullet.resetBasePositionAndOrientation(camera_frustum_id, pos, cam_quat)

    def draw_waypoints():
        nonlocal look_target_body_id
        for b in waypoint_bodies:
            try:
                pybullet.removeBody(b)
            except Exception:
                pass
        waypoint_bodies.clear()
        if look_target_body_id is not None:
            try:
                pybullet.removeBody(look_target_body_id)
            except Exception:
                pass
            look_target_body_id = None
        if current_start is None:
            return
        cfg = START_POSITIONS[current_start]
        wps = _resolve_waypoints(cfg)
        if not wps:
            return
        waypoint_bodies.extend(_draw_waypoint_bodies(wps))
        plane_z = wps[0]["tcp_pos"][2]
        lt = cfg.get("look_target")
        jaw = cfg.get("jaw_pos")
        if lt is not None:
            target = [lt[0], lt[1], plane_z]
        elif jaw is not None:
            target = [jaw[0], jaw[1], plane_z]
        else:
            target = None
        if target is not None:
            look_target_body_id = _draw_look_target_body(target)

    def _set_view(cfg):
        """Setzt die pybullet-Orbit-Kamera auf die Startposition/den Startwinkel
        der Startposition (cfg['view']). Fehlt 'view' oder 'apply' ist False,
        wird die Kamera nicht beruehrt (Global aus sim.py gilt dann)."""
        v = cfg.get("view")
        if not v or not v.get("apply", True):
            return
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
        pybullet.resetDebugVisualizerCamera(
            v.get("distance", PB_CAMERA_DISTANCE),
            v.get("yaw", PB_CAMERA_YAW),
            v.get("pitch", PB_CAMERA_PITCH),
            list(v.get("target", PB_CAMERA_TARGET_POS)),
        )
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    def reset_overlay():
        """Entfernt ALLE Debug-Items (auch verwaiste "Geister") via
        removeAllUserDebugItems und zeichnet das TCP-Crosshair neu.
        Die persistenten Waypoint-Bodies sind davon unberuehrt und bleiben stehen."""
        pybullet.removeAllUserDebugItems()
        items.clear()
        draw_tcp()
        draw_camera_frustum()

    def draw_probe_preview(rrt_waypoints, target_position):
        if rrt_waypoints:
            step = max(1, len(rrt_waypoints) // 20)
            for i in range(0, len(rrt_waypoints)-step, step):
                items.append(pybullet.addUserDebugLine(
                    rrt_waypoints[i], rrt_waypoints[i+step], [0, 1, 0], 1,
                ))
        else:
            print(f"  ⚠ Kein RRT-Pfad zu ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})")

    def preview_and_move(pos, ori, speed, seed=None, confirm=False):
        """Gemeinsamer Ablauf fuer den m-Befehl (confirm=True, mit Rueckfrage)
        und alle automatischen Bewegungen (confirm=False, kurze Pause):
        Zielmarker + RRT-Pfad zeichnen, warten, den Pfad (1x) fahren, aufraeumen.
        Gibt True bei Erfolg, False wenn kein Pfad/unerreichbar, None bei Abbruch."""
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        saved_fds = (os.dup(1), os.dup(2))
        os.dup2(devnull_fd, 1), os.dup2(devnull_fd, 2)

        probe_result = sim._probe_path(pos, ori, seed=seed)

        os.dup2(saved_fds[0], 1), os.dup2(saved_fds[1], 2)
        os.close(devnull_fd)

        rrt_waypoints, plan_path = probe_result
        _draw_crosshair(pos, [1, 1, 0], items,
                        f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        draw_probe_preview(rrt_waypoints, pos)
        ok = False
        if plan_path is not None:
            if confirm:
                try:
                    c = input("  Ausführen? [Y/n] ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    reset_overlay()
                    return None
                if c in ("", "y", "yes"):
                    ok = sim.move_to(pos, ori, speed=speed, path=plan_path)
            else:
                time.sleep(PREVIEW_PAUSE)
                ok = sim.move_to(pos, ori, speed=speed, path=plan_path)
        reset_overlay()
        return ok

    def _render_pair(rel_left, rel_right, timeout=RENDER_TIMEOUT):
        m = sim._mirror
        if m is None or not m._connected:
            print("  ⚠ Blender-Mirror nicht verbunden – Render übersprungen")
            return False
        m._render_done.clear()
        m.send_message({"render": {"left": rel_left, "right": rel_right}})
        if not m._render_done.wait(timeout):
            print(f"  ⚠ Render-Timeout nach {timeout}s")
            return False
        return True

    items = []
    waypoint_bodies = []
    look_target_body_id = None
    camera_frustum_id = None
    current_start = None
    waypoint_idx = 0
    current_speed = 0.5

    if BOOT_START is not None:
        cfg = BOOT_START
        ori = [math.radians(v) for v in cfg["tcp_ori_deg"]]
        print("  Boot: fahre zur Startposition...")
        ok = preview_and_move(list(cfg["tcp_pos"]), ori, 0.5, seed=sim._null_space[3])
        if ok:
            print(f"  Boot: Startposition erreicht (Gebiss nicht geladen)")
        else:
            print("  Boot: Startposition nicht erreichbar – Arm bleibt an Neutralposition")
        _set_view(cfg)

    print("── UR5e Demo ──────────────────────────────")
    print("Move:        'm x y z [rx ry rz]' (RRT, Default-Orientierung 0 0 0)")
    print("  Tool-Offset: 'o x y z' (z. B. o 0 0 0.15)")
    print("  Geschw.:     's 0.5' (global, Default 0.5)")
    print("  Reset:       '@'  (nach manuellem Ziehen)")
    print("  Gebiss:      'jaw <nr> [upper|lower]' (z. B. jaw 3 upper)")
    print("  Start:       'start <Aussen|Oben|Innen>' (Startposition anfahren)")
    print("  Waypoints:   '+'/'-' naechster/vorheriger Waypoint")
    print("  Scan:        'scan [n]' (alle bzw. nur die ersten n Waypoints + L/R rendern)")
    print("  Scan-All:    'scan-all <start1> [start2 ...]' (Startpositionen der Reihe nach scannen)")
    print("────────────────────────────────────────────")
    reset_overlay()

    def _do_start(name):
        nonlocal current_start, waypoint_idx
        cfg = START_POSITIONS[name]
        tcp_ori = [math.radians(v) for v in cfg["tcp_ori_deg"]]
        print("  → jaw entfernt...")
        sim.unload_jaw()
        if sim._mirror is not None:
            sim._mirror.send_message({"jaw_unload": True})

        def _dump_pose(tag, pos, ori_deg):
            q = pybullet.getQuaternionFromEuler(ori_deg)
            ee = sim._tcp_to_ee(list(pos), list(ori_deg))
            joints = sim.get_joint_angles()
            jstr = ", ".join(f"{math.degrees(j):.0f}" for j in joints)
            print(f"    [dbg {tag}] tcp=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) euler_deg=({math.degrees(ori_deg[0]):.0f},{math.degrees(ori_deg[1]):.0f},{math.degrees(ori_deg[2]):.0f})")
            print(f"    [dbg {tag}] quat=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
            print(f"    [dbg {tag}] ee_pos=({ee[0][0]:.3f},{ee[0][1]:.3f},{ee[0][2]:.3f})")
            print(f"    [dbg {tag}] joints_deg={jstr}")

        approach = cfg.get("approach", [])
        start_seed = sim._null_space[3]
        for i, a in enumerate(approach):
            a_ori = [math.radians(v) for v in a["tcp_ori_deg"]]
            lbl = a.get("label", str(i + 1))
            print(f"  → approach {lbl}...")
            _dump_pose(f"approach{lbl}", a["tcp_pos"], a_ori)
            a_seed = None if a.get("use_current_seed") else start_seed
            ok = preview_and_move(a["tcp_pos"], a_ori, current_speed, seed=a_seed)
            if not ok:
                print(f"  ⛔ Approach {lbl} nicht erreichbar – start abgebrochen")
                continue
        print(f"  → fahre zu {name}-Start...")
        _dump_pose("final", cfg["tcp_pos"], tcp_ori)
        ok = preview_and_move(cfg["tcp_pos"], tcp_ori, current_speed, seed=start_seed)
        if not ok:
            print(f"  ⛔ {name}-Start nicht erreichbar – start abgebrochen")
            return False
        jpos = cfg["jaw_pos"]
        jeuler = [math.radians(v) for v in cfg["jaw_euler_deg"]]
        sim.load_jaw_at(cfg["jaw_folder"], cfg["jaw_type"], jpos, jeuler)
        if sim._mirror is not None:
            sim._mirror._jaw_done.clear()
            sim._mirror.send_message({"replace_jaw": {"folder": cfg["jaw_folder"], "type": cfg["jaw_type"], "pos": jpos, "euler": cfg["jaw_euler_deg"]}})
            sim._mirror._jaw_done.wait(timeout=10)
            sim._mirror.send_current()
        print(f"  → jaw eingefuegt ({cfg['jaw_type']}, pos=({jpos[0]:.3f}, {jpos[1]:.3f}, {jpos[2]:.3f}))")
        current_start = name
        waypoint_idx = 10
        wps = _resolve_waypoints(cfg)
        print(f"  Waypoints: {len(wps)}")
        for i, wp in enumerate(wps):
            print(f"    {i+1}: {wp.get('name') or wp.get('label', str(i+1))} ({wp['tcp_pos'][0]:.3f}, {wp['tcp_pos'][1]:.3f}, {wp['tcp_pos'][2]:.3f})")
        _set_view(cfg)
        reset_overlay()
        draw_waypoints()
        return True

    def _do_scan(max_waypoints=None):
        nonlocal waypoint_idx
        render_ok = True
        if current_start is None:
            print("  ? Keine Startposition aktiv – zuerst 'start <name>'")
            return render_ok, 0, 0, 0
        cfg = START_POSITIONS[current_start]
        all_wps = _resolve_waypoints(cfg)
        wps = all_wps[:max_waypoints] if max_waypoints else all_wps
        if not wps:
            print(f"  ? Keine Waypoints definiert fuer {current_start}")
            return render_ok, 0, 0, 0
        jaw_folder = cfg["jaw_folder"]
        jaw_type = cfg["jaw_type"]
        jaw_pos = cfg["jaw_pos"]
        jaw_euler_deg = cfg["jaw_euler_deg"]
        q_jaw = pybullet.getQuaternionFromEuler([math.radians(v) for v in jaw_euler_deg])
        r_jaw = pybullet.getMatrixFromQuaternion(q_jaw)
        ordner = f"{jaw_type}_{jaw_folder}_{current_start}_{RENDER_W}x{RENDER_H}_lat{CAMERA_LATERAL_OFFSET*1000:.1f}mm"
        scan_dir = os.path.join(RENDER_DIR, ordner)
        os.makedirs(scan_dir, exist_ok=True)
        settings = {
            "info": {
                "start_position": current_start,
                "jaw_folder": jaw_folder,
                "jaw_type": jaw_type,
                "jaw_pos": list(jaw_pos),
                "jaw_euler_deg": list(jaw_euler_deg),
                "jaw_quat": list(q_jaw),
                "render_w": RENDER_W,
                "render_h": RENDER_H,
                "render_engine": RENDER_ENGINE,
                "render_device": RENDER_DEVICE,
                "world_unit": "m",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            },
            "reconstruction": {
                "camera_lateral_offset": CAMERA_LATERAL_OFFSET,
                "camera_fov_deg": CAMERA_FOV_DEG,
                "camera_lens_mm": CAMERA_LENS_MM,
                "camera_far_m": CAMERA_FAR_M,
                "rectified": True,
                "lens_distortion": False,
                "baseline_m": round(2 * CAMERA_LATERAL_OFFSET, 6),
                "camera_intrinsic": _camera_intrinsic(),
            },
        }
        with open(os.path.join(scan_dir, "render_settings.json"), "w") as f:
            json.dump(settings, f, indent=2)
        print(f"  → Scan-Ordner: {scan_dir}")
        scan_t0 = time.time()
        # ── Phase 1: Rückweg zum Waypoint 0 entlang der Kurve (kein Render) ──
        print("  → Rückweg zum Waypoint 0...")
        while waypoint_idx > 0:
            wp = all_wps[waypoint_idx - 1]
            lbl = wp.get("name") or wp.get("label", str(waypoint_idx))
            print(f"  → {current_start} {lbl} ({waypoint_idx}/{len(wps)})...")
            wp_ori = [math.radians(v) for v in wp["tcp_ori_deg"]]
            ok = preview_and_move(wp["tcp_pos"], wp_ori, current_speed)
            if not ok:
                print(f"  ⛔ Rückweg abgebrochen ({lbl} nicht erreichbar)")
                break
            waypoint_idx -= 1
        # ── Phase 2: Sweep W0→max mit Render an jedem Waypoint ──
        done = rendered = skipped = 0
        for i, wp in enumerate(wps):
            lbl = wp.get("name") or wp.get("label", str(i))
            print(f"  → scan {current_start} {lbl} ({i+1}/{len(wps)})...")
            wp_ori = [math.radians(v) for v in wp["tcp_ori_deg"]]
            if i != waypoint_idx:
                ok = preview_and_move(wp["tcp_pos"], wp_ori, current_speed)
                if not ok:
                    skipped += 1
                    print(f"  ⛔ {current_start} {lbl} nicht erreichbar – übersprungen")
                    continue
            cam_l, cam_r = _camera_poses_in_jaw(sim, r_jaw, jaw_pos, q_jaw)
            pose = {
                "waypoint": i,
                "label": lbl,
                "camera_left": {
                    "position": cam_l[0],
                    "quaternion": cam_l[1],
                },
                "camera_right": {
                    "position": cam_r[0],
                    "quaternion": cam_r[1],
                },
            }
            with open(os.path.join(scan_dir, f"{i}_pose.json"), "w") as f:
                json.dump(pose, f, indent=2)
            rel_left = os.path.join("render", ordner, f"{i}L_render.png")
            rel_right = os.path.join("render", ordner, f"{i}R_render.png")
            if _render_pair(rel_left, rel_right):
                rendered += 1
            else:
                print("  ⛔ Render fehlgeschlagen – Scan abgebrochen")
                render_ok = False
                break
            done += 1
            waypoint_idx = i
        settings["info"]["render_duration_s"] = round(time.time() - scan_t0, 1)
        with open(os.path.join(scan_dir, "render_settings.json"), "w") as f:
            json.dump(settings, f, indent=2)
        print(f"  ✔ Scan fertig: {scan_dir} ({done} Waypoints, {rendered} gerendert, {skipped} übersprungen)")
        return render_ok, done, rendered, skipped

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        tokens = line.split()
        cmd = _parse_command(tokens)
        if cmd.action == "quit":
            break
        if cmd.action == "offset":
            sim.set_tool_offset(cmd.params["pos"])
            reset_overlay()
            continue
        if cmd.action == "reset" and sim._last_conf is not None:
            sim._execute([sim._last_conf])
            reset_overlay()
            continue
        if cmd.action == "speed":
            current_speed = cmd.params["speed"]
            print(f"  Geschwindigkeit: {current_speed:.2f}")
            reset_overlay()
            continue
        if cmd.action == "jaw":
            if current_start is None:
                print(f"  ? Keine Startposition aktiv – zuerst 'start <name>'")
                continue
            folder = cmd.params["folder"]
            jaw_type = cmd.params["type"]
            jcfg = START_POSITIONS[current_start]
            jpos = jcfg["jaw_pos"]
            jeuler = [math.radians(v) for v in jcfg["jaw_euler_deg"]]
            try:
                ok = sim.load_jaw(folder, jaw_type, jpos, jeuler)
            except FileNotFoundError as e:
                print(f"  ! {e}")
                continue
            if not ok:
                continue
            print(f"  PyBullet: gebiss_{jaw_type} aus Ordner {folder}")
            if sim._mirror is not None:
                sim._mirror._jaw_done.clear()
                sim._mirror.send_message({"replace_jaw": {"folder": folder, "type": jaw_type, "pos": jpos, "euler": jcfg["jaw_euler_deg"]}})
                sim._mirror._jaw_done.wait(timeout=10)
            reset_overlay()
            continue
        if cmd.action == "start_pos":
            _do_start(cmd.params["name"])
            continue
        if cmd.action == "waypoint_next":
            if current_start is None:
                print(f"  ? Keine Startposition aktiv (zuerst 'start <name>')")
                continue
            cfg = START_POSITIONS[current_start]
            wps = _resolve_waypoints(cfg)
            if not wps:
                print(f"  ? Keine Waypoints definiert fuer {current_start}")
                continue
            steps = cmd.params["steps"]
            for s in range(steps):
                if waypoint_idx + 1 >= len(wps):
                    print(f"  → Pfad-Ende ({len(wps)} Waypoints)")
                    break
                target = waypoint_idx + 1
                wp = wps[target]
                lbl = wp.get("name") or wp.get("label", str(target + 1))
                print(f"  → {current_start} {lbl} ({target+1}/{len(wps)})...")
                wp_ori = [math.radians(v) for v in wp["tcp_ori_deg"]]
                ok = preview_and_move(wp["tcp_pos"], wp_ori, current_speed)
                if not ok:
                    print(f"  ⛔ {current_start} {lbl} nicht erreichbar – Navigation abgebrochen")
                    break
                waypoint_idx = target
            reset_overlay()
            continue
        if cmd.action == "waypoint_prev":
            if current_start is None:
                print(f"  ? Keine Startposition aktiv (zuerst 'start <name>')")
                continue
            cfg = START_POSITIONS[current_start]
            wps = _resolve_waypoints(cfg)
            if not wps:
                print(f"  ? Keine Waypoints definiert fuer {current_start}")
                continue
            steps = cmd.params["steps"]
            for s in range(steps):
                if waypoint_idx <= 0:
                    print(f"  → Startposition erreicht")
                    break
                target = waypoint_idx - 1
                wp = wps[target]
                lbl = wp.get("name") or wp.get("label", str(target + 1))
                print(f"  → {current_start} {lbl} ({target+1}/{len(wps)}) zurueck...")
                wp_ori = [math.radians(v) for v in wp["tcp_ori_deg"]]
                ok = preview_and_move(wp["tcp_pos"], wp_ori, current_speed)
                if not ok:
                    print(f"  ⛔ {current_start} {lbl} nicht erreichbar – Navigation abgebrochen")
                    break
                waypoint_idx = target
            reset_overlay()
            continue

        if cmd.action == "scan":
            _do_scan(cmd.params.get("max_waypoints"))
            continue
        if cmd.action == "scan_all":
            for name in cmd.params["names"]:
                print(f"── scan-all: {name} ──")
                if not _do_start(name):
                    print(f"  ⛔ {name}-Start nicht erreichbar – übersprungen")
                    continue
                render_ok, done, rendered, skipped = _do_scan()
                if not render_ok:
                    print(f"  ⛔ Batch abgebrochen (Render-Fehler bei {name})")
                    break
            continue

        if cmd.action == "error":
            continue
        target_position = cmd.params["target_position"]
        target_orientation = cmd.params["target_orientation"]

        result = preview_and_move(target_position, target_orientation, current_speed, confirm=True)
        if result is None:
            break

    if sim._mirror is not None:
        sim._mirror.close()
