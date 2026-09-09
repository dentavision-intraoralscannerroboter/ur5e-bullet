import sys
import threading

_lock = threading.Lock()
_status = None
_buffer = []
_MAX_BUFFER = 100


def _flush_locked():
    while _buffer:
        print(_buffer.pop(0))


def status(text):
    global _status
    with _lock:
        _status = text
        sys.stdout.write("\r" + text)
        sys.stdout.flush()


def status_end():
    global _status
    with _lock:
        if _status is not None:
            sys.stdout.write("\n")
            _status = None
            _flush_locked()
            sys.stdout.flush()


def log(text):
    global _status
    with _lock:
        if _status is not None:
            _buffer.append(text)
            if len(_buffer) >= _MAX_BUFFER:
                _flush_locked()
            return
        _flush_locked()
        print(text)


def progress(text):
    with _lock:
        sys.stdout.write("\r" + text)
        sys.stdout.flush()


def progress_done(text):
    with _lock:
        sys.stdout.write("\n")
        print(text)
        sys.stdout.flush()