import math
from collections import namedtuple

import importlib.util as _ilu
import os as _os

_cfg = _ilu.spec_from_file_location(
    "config",
    _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "..", "config.py"),
)
_cfg_mod = _ilu.module_from_spec(_cfg)
_cfg.loader.exec_module(_cfg_mod)
START_POSITIONS = _cfg_mod.START_POSITIONS


Command = namedtuple("Command", [
    "action", "params",
])


def _parse_command(tokens):
    if tokens[0] == "q":
        return Command("quit", {})
    if tokens[0] == "o" and len(tokens) == 4:
        try:
            pos = [float(tokens[1]), float(tokens[2]), float(tokens[3])]
        except ValueError:
            print(f"  ? '{' '.join(tokens)}' verstanden?")
            return Command("error", {})
        return Command("offset", {"pos": pos})
    if tokens[0] == "s" and len(tokens) == 2:
        try:
            speed = float(tokens[1])
        except ValueError:
            print(f"  ? '{' '.join(tokens)}' verstanden?")
            return Command("error", {})
        return Command("speed", {"speed": max(speed, 0.01)})
    if tokens[0] == "@":
        return Command("reset", {})
    if tokens[0] == "scan":
        if len(tokens) == 1:
            return Command("scan", {})
        try:
            n = int(tokens[1])
        except ValueError:
            print(f"  ? '{tokens[1]}' ist keine gueltige Zahl")
            return Command("error", {})
        if n < 1:
            print("  ? Anzahl Waypoints muss >= 1 sein")
            return Command("error", {})
        return Command("scan", {"max_waypoints": n})
    if tokens[0] == "scan-all":
        if len(tokens) < 2:
            print(f"  ? Verwende: scan-all <start1> [start2 ...]")
            return Command("error", {})
        names = tokens[1:]
        unknown = [n for n in names if n not in START_POSITIONS]
        if unknown:
            print(f"  ? Unbekannte Startposition(en): {', '.join(unknown)} – verfügbar: {', '.join(START_POSITIONS.keys())}")
            return Command("error", {})
        return Command("scan_all", {"names": names})
    if tokens[0] == "jaw":
        if len(tokens) < 2:
            print("  ? 'jaw <nr> [upper|lower]' erwartet")
            return Command("error", {})
        try:
            folder = int(tokens[1])
        except ValueError:
            print(f"  ? '{tokens[1]}' ist keine gueltige Nr.")
            return Command("error", {})
        jaw_type = tokens[2] if len(tokens) >= 3 else "lower"
        if jaw_type not in ("upper", "lower"):
            print(f"  ? '{jaw_type}' – nur 'upper' oder 'lower'")
            return Command("error", {})
        return Command("jaw", {"folder": folder, "type": jaw_type})
    if tokens[0] == "start":
        if len(tokens) < 2:
            names = ", ".join(START_POSITIONS.keys())
            print(f"  ? Verwende: start <{names}>")
            return Command("error", {})
        name = tokens[1]
        if name not in START_POSITIONS:
            print(f"  ? '{name}' – verfuegbar: {', '.join(START_POSITIONS.keys())}")
            return Command("error", {})
        return Command("start_pos", {"name": name})
    if tokens[0] == "+":
        n = 1
        if len(tokens) >= 2:
            try:
                n = int(tokens[1])
            except ValueError:
                print(f"  ? '{tokens[1]}' ist keine Zahl")
                return Command("error", {})
        return Command("waypoint_next", {"steps": n})
    if tokens[0] == "-":
        n = 1
        if len(tokens) >= 2:
            try:
                n = int(tokens[1])
            except ValueError:
                print(f"  ? '{tokens[1]}' ist keine Zahl")
                return Command("error", {})
        return Command("waypoint_prev", {"steps": n})

    if tokens[0] != "m":
        print(f"  ? '{' '.join(tokens)}' verstanden? (Move: 'm x y z [rx ry rz]')")
        return Command("error", {})
    try:
        parsed_values = [float(v) for v in tokens[1:]]
    except ValueError:
        print(f"  ? '{' '.join(tokens)}' verstanden? (Move: 'm x y z [rx ry rz]')")
        return Command("error", {})
    if len(parsed_values) < 3:
        print(f"  ? '{' '.join(tokens)}' – Position 'x y z' fehlt")
        return Command("error", {})
    target_position = parsed_values[:3]
    target_orientation = [math.radians(v) for v in parsed_values[3:6]] if len(parsed_values) >= 6 else [0, 0, 0]
    return Command("move", {
        "target_position": target_position,
        "target_orientation": target_orientation,
    })
