import collections
import json
import os
import socket
import subprocess
import sys
import threading
import time

import importlib.util as _ilu
_cfg = _ilu.spec_from_file_location(
    "config",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config.py"),
)
_cfg_mod = _ilu.module_from_spec(_cfg)
_cfg.loader.exec_module(_cfg_mod)
MIRROR_SCRIPT = _cfg_mod.MIRROR_SCRIPT

MIRROR_CONNECT_WARN_S = 30
_ERROR_MARKERS = (
    "Traceback", "Error:", "Exception:", "AttributeError", "NameError",
    "TypeError", "ValueError", "ImportError", "ModuleNotFoundError",
    "SyntaxError", "OSError", "KeyError", "IndexError", "RuntimeError",
    "AssertionError",
)


def _is_error_line(text):
    return any(m in text for m in _ERROR_MARKERS)


def _print_tail(lines, tag):
    if not lines:
        print(f"  ({tag}: keine Ausgabe vorhanden)")
        return
    for line in list(lines):
        print(f"  {tag}: {line}")


class BlenderMirror:
    """Pusht den Roboterzustand per TCP-Socket an eine Blender-GUI-Instanz."""

    def __init__(self, sim, port=0):
        self.sim = sim
        self._alive = True
        self._connected = False
        self._conn = None
        self._lock = threading.Lock()
        self._render_done = threading.Event()
        self._jaw_done = threading.Event()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", port))
        self._sock.listen(1)
        self._sock.settimeout(1.0)
        port = self._sock.getsockname()[1]

        self._proc = self._launch_blender(port)
        if self._proc is None:
            self.close()
            return

        self._stdout_tail = collections.deque(maxlen=50)
        self._stderr_tail = collections.deque(maxlen=50)
        self._connect_warned = False
        self._connect_start = time.monotonic()

        if self._proc.stdout is not None:
            threading.Thread(target=self._drain_stdout, daemon=True).start()
        if self._proc.stderr is not None:
            threading.Thread(target=self._drain_stderr, daemon=True).start()
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _launch_blender(self, port):
        if not os.path.exists(MIRROR_SCRIPT):
            print("[mirror] blender/mirror.py nicht gefunden – Mirror deaktiviert")
            return None
        try:
            proc = subprocess.Popen(
                ["blender", "--python", MIRROR_SCRIPT, "--", f"--port={port}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True,
            )
        except (OSError, FileNotFoundError):
            print("[mirror] Blender konnte nicht gestartet werden – Mirror deaktiviert")
            return None
        return proc

    def _drain_stdout(self):
        try:
            for line in self._proc.stdout:
                text = line.decode("utf-8", "ignore").rstrip("\n")
                if text:
                    self._stdout_tail.append(text)
                    if text.startswith("[mirror]"):
                        print(text)
        except (OSError, ValueError):
            pass

    def _drain_stderr(self):
        try:
            for line in self._proc.stderr:
                text = line.decode("utf-8", "ignore").rstrip("\n")
                if text:
                    self._stderr_tail.append(text)
                    if _is_error_line(text):
                        print(text)
        except (OSError, ValueError):
            pass

    def _accept_loop(self):
        buf = b""
        conn = None
        try:
            while self._alive:
                try:
                    conn, _ = self._sock.accept()
                    break
                except socket.timeout:
                    if self._proc.poll() is not None:
                        print("[mirror] Blender-Mirror hat sich OHNE Verbindung beendet – Log:")
                        _print_tail(self._stdout_tail, "stdout")
                        _print_tail(self._stderr_tail, "stderr")
                        return
                    if not self._connect_warned and (
                        time.monotonic() - self._connect_start > MIRROR_CONNECT_WARN_S
                    ):
                        self._connect_warned = True
                        print(
                            f"[mirror] Warnung: Nach {MIRROR_CONNECT_WARN_S}s noch keine "
                            "Blender-Verbindung – warte weiter…"
                        )
                    continue
            if conn is None:
                return
            conn.settimeout(1.0)
            self._conn = conn
            ready = False
            while self._alive:
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    if not ready:
                        if b"READY" in line:
                            ready = True
                            self._connected = True
                            self.send_current()
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8", "ignore"))
                    except ValueError:
                        continue
                    self._handle_host_msg(msg)
        except OSError:
            pass
        finally:
            self._connected = False
            if conn is not None:
                try:
                    conn.close()
                except OSError:
                    pass

    def _handle_host_msg(self, msg):
        if "render_progress" in msg:
            n = msg['render_progress']
            print(f"\r  Rendering... [{n}]", end="", flush=True)
        if "render_complete" in msg:
            paths = msg["render_complete"]
            if isinstance(paths, str):
                paths = [paths]
            print("\n  Render gespeichert: " + ", ".join(paths))
            self._render_done.set()
        if "jaw_complete" in msg:
            self._jaw_done.set()
            if "jaw_log" in msg and msg.get("jaw_log"):
                print(msg["jaw_log"])

    def send_current(self):
        try:
            joints = self.sim.get_joint_angles()
        except Exception:
            return
        msg = {"joints": joints}
        try:
            msg["tcp"] = self.sim.get_tcp_in_scanner_frame()
        except Exception:
            pass
        self.send_message(msg)

    def send_message(self, payload):
        if not self._connected or self._conn is None:
            return
        try:
            with self._lock:
                self._conn.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        except OSError:
            pass

    def close(self):
        self._alive = False
        if self._proc is not None:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    self._proc.kill()
                except OSError:
                    pass
        try:
            self._sock.close()
        except OSError:
            pass
        try:
            if self._conn is not None:
                self._conn.close()
        except OSError:
            pass
