"""Parsing-Tests fuer die REPL-Kommandos (ur5e_bullet._parse_command)."""

import ur5e_bullet as u


def test_scan_all_ok():
    c = u._parse_command(["scan-all", "a1l", "o1l", "aussen2"])
    assert c.action == "scan_all"
    assert c.params["names"] == ["a1l", "o1l", "aussen2"]


def test_scan_all_without_names():
    assert u._parse_command(["scan-all"]).action == "error"


def test_scan_all_unknown_name():
    assert u._parse_command(["scan-all", "a1l", "xyz"]).action == "error"


def test_scan_unchanged():
    assert u._parse_command(["scan"]).action == "scan"
    c = u._parse_command(["scan", "3"])
    assert c.action == "scan" and c.params["max_waypoints"] == 3
    assert u._parse_command(["scan", "0"]).action == "error"
    assert u._parse_command(["scan", "abc"]).action == "error"


def test_start_unchanged():
    assert u._parse_command(["start", "o1l"]).params["name"] == "o1l"
    assert u._parse_command(["start", "nope"]).action == "error"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"OK {name}")
    print("ALLE COMMAND-TESTS OK")