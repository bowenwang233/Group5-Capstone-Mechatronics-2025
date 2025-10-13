# state_store.py
"""
Thread-safe data store for the Digital Twin Dashboard.

READ API used by the dashboard:
--------------------------------
get_timeseries(window_sec) -> dict
get_lidar() -> (x, y, z)
get_camera_b64() -> str | None
get_status() -> dict

WRITE API for your data producers (ZeroMQ, simulators, etc.):
-------------------------------------------------------------
append_sample(v, a, steer, t=None)
set_lidar(xs, ys, zs)
set_camera_b64(data_uri)
set_status(battery=None, conn=None, mode=None)
"""

from __future__ import annotations
from collections import deque
from threading import RLock
from time import monotonic
from typing import Optional, Dict, List, Tuple

# ------------------ configuration ------------------
MAX_SAMPLES = 10_000
MAX_LIDAR_POINTS = 30_000

# ------------------ storage -------------------------
_lock = RLock()
_t, _v, _a, _steer = (deque(maxlen=MAX_SAMPLES) for _ in range(4))
_lidar_xyz: Tuple[List[float], List[float], List[float]] = ([], [], [])
_camera_b64: Optional[str] = None
_status: Dict[str, str] = {"battery": "—", "conn": "—", "mode": "—"}

# ------------------ write API -----------------------
def append_sample(v: Optional[float], a: Optional[float], steer: Optional[float], t: Optional[float] = None) -> None:
    """Append one telemetry sample (v, a, steer)."""
    with _lock:
        now = t if t is not None else monotonic()
        v_val = _v[-1] if v is None and _v else (v or 0.0)
        a_val = _a[-1] if a is None and _a else (a or 0.0)
        s_val = _steer[-1] if steer is None and _steer else (steer or 0.0)

        _t.append(now)
        _v.append(float(v_val))
        _a.append(float(a_val))
        _steer.append(float(s_val))

def set_lidar(xs: List[float], ys: List[float], zs: List[float]) -> None:
    """Replace current LiDAR frame, downsampling if too large."""
    with _lock:
        n = min(len(xs), len(ys), len(zs))
        if n > MAX_LIDAR_POINTS:
            stride = max(1, n // MAX_LIDAR_POINTS)
            xs, ys, zs = xs[::stride][:MAX_LIDAR_POINTS], ys[::stride][:MAX_LIDAR_POINTS], zs[::stride][:MAX_LIDAR_POINTS]
        global _lidar_xyz
        _lidar_xyz = (xs[:], ys[:], zs[:])

def set_camera_b64(data_uri: Optional[str]) -> None:
    """Set the latest camera frame as a base64 data URI string."""
    if data_uri is None:
        return
    with _lock:
        global _camera_b64
        _camera_b64 = data_uri

def set_status(battery: Optional[str] = None, conn: Optional[str] = None, mode: Optional[str] = None) -> None:
    """Update vehicle status values."""
    with _lock:
        if battery is not None:
            _status["battery"] = battery
        if conn is not None:
            _status["conn"] = conn
        if mode is not None:
            _status["mode"] = mode

# ------------------ read API ------------------------
def get_timeseries(window_sec: float) -> Dict[str, List[float]]:
    """Return the most recent samples within window_sec seconds."""
    with _lock:
        if not _t:
            return {"t": [], "v": [], "a": [], "steer": []}
        now = monotonic()
        t_min = now - max(0.0, float(window_sec))
        # find first index within window
        start = next((i for i, ti in enumerate(_t) if ti >= t_min), len(_t))
        t_abs = list(_t)[start:]
        v, a, s = list(_v)[start:], list(_a)[start:], list(_steer)[start:]
        t_rel = [ti - now for ti in t_abs]
        return {"t": t_rel, "v": v, "a": a, "steer": s}

def get_lidar() -> Tuple[List[float], List[float], List[float]]:
    with _lock:
        xs, ys, zs = _lidar_xyz
        return xs[:], ys[:], zs[:]

def get_camera_b64() -> Optional[str]:
    with _lock:
        return _camera_b64

def get_status() -> Dict[str, str]:
    with _lock:
        return dict(_status)

