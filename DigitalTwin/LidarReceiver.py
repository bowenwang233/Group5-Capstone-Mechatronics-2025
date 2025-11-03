# modified_LidarReceiver.py
"""
Drop-in lighter LiDAR receiver with:
- ZMQ CONFLATE (keep latest only) + low RCVHWM
- Non-blocking drain loop (coalesce bursts to newest frame)
- Input rate cap (e.g., 12 Hz) to reduce UI churn
- Safer defaults for downsampling in latest_xy (e.g., 3)
- Optional max_points clamp to protect Plotly from huge arrays

Usage from Dashboard.py (unchanged API):
    import modified_LidarReceiver as LidarReceiver
    LidarReceiver.start()  # or LidarReceiver.start(f"tcp://{ip}:5560")
    x, y, r = LidarReceiver.latest_xy(downsample=3, clamp_max_range=8.0)
"""
from __future__ import annotations
import os, time, threading
from collections import deque
from typing import Tuple, Optional, Dict, Any

import numpy as np
import zmq

# Defaults (env overridable)
CAR_IP = os.environ.get("PI_IP", os.environ.get("CAR_IP", "192.168.149.1"))
DEFAULT_ENDPOINT = f"tcp://{CAR_IP}:5560"

# Tunables
INPUT_MAX_HZ = float(os.environ.get("LIDAR_INPUT_MAX_HZ", "12.0"))   # cap accepted scans
RCV_TIMEOUT_MS = int(os.environ.get("LIDAR_RCV_TIMEOUT_MS", "200"))  # poll timeout
RCV_HWM = int(os.environ.get("LIDAR_RCV_HWM", "10"))                 # small queue

class LidarReceiver:
    def __init__(self, endpoint: Optional[str] = None, queue_size: int = 1):
        self._endpoint = endpoint or DEFAULT_ENDPOINT
        self._latest = deque(maxlen=max(1, queue_size))
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._started = False

        self._ctx: Optional[zmq.Context] = None
        self._sock: Optional[zmq.Socket] = None

        self._last_accept_wall = 0.0
        self._accept_interval = (1.0 / INPUT_MAX_HZ) if INPUT_MAX_HZ > 0 else 0.0

    @property
    def endpoint(self) -> str:
        return self._endpoint

    def start(self) -> None:
        if self._started:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="LidarReceiverLite", daemon=True)
        self._thread.start()
        self._started = True

    def stop(self, timeout: float = 1.0) -> None:
        if not self._started: return
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout)
        self._thread = None
        self._started = False
        try:
            if self._sock is not None:
                self._sock.close(linger=0)
        finally:
            self._sock = None
            if self._ctx is not None:
                self._ctx.term()
            self._ctx = None

    def _setup_sock(self):
        if self._ctx is None:
            self._ctx = zmq.Context.instance()
        if self._sock is None:
            s = self._ctx.socket(zmq.SUB)
            s.setsockopt(zmq.RCVHWM, RCV_HWM)
            s.setsockopt(zmq.CONFLATE, 1)                 # newest only
            s.setsockopt_string(zmq.SUBSCRIBE, "")         # accept any topic
            s.rcvtimeo = RCV_TIMEOUT_MS
            s.connect(self._endpoint)
            self._sock = s

    def _loop(self) -> None:
        backoff = 0.02
        while not self._stop.is_set():
            try:
                self._setup_sock()

                # Drain any backlog and keep only the newest payload
                newest = None
                while True:
                    try:
                        newest = self._sock.recv_json(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                if newest is None:
                    # No immediate data; do a blocking-ish wait with timeout
                    newest = self._sock.recv_json()
                if newest is None:
                    continue

                now = time.time()
                # rate limit accepted frames
                if self._accept_interval > 0 and (now - self._last_accept_wall) < self._accept_interval:
                    # Skip storing to reduce churn
                    continue

                newest["_ts"] = now
                with self._lock:
                    self._latest.append(newest)

                self._last_accept_wall = now
                backoff = 0.02
            except zmq.Again:
                # timeout, loop to check stop
                pass
            except Exception:
                # transient errors: reset socket and backoff
                time.sleep(backoff)
                backoff = min(backoff * 2, 1.0)
                try:
                    if self._sock is not None:
                        self._sock.close(linger=0)
                finally:
                    self._sock = None

    # -------- Retrieval helpers --------
    def latest_scan(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return None if not self._latest else self._latest[-1]

    def latest_xy(
        self,
        default_radius: float = 8.0,
        downsample: int = 3,                      # lighter default than 1
        clamp_max_range: Optional[float] = None,
        max_points: Optional[int] = 5000,         # safety clamp for Plotly
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        scan = self.latest_scan()
        if scan is None:
            return np.array([0.0], dtype=np.float32), np.array([0.0], dtype=np.float32), default_radius

        try:
            angle_min = float(scan["angle_min"])
            angle_inc = float(scan["angle_increment"])
            ranges = np.asarray(scan["ranges"], dtype=np.float32)
            rmin = max(0.01, float(scan.get("range_min", 0.01)))
            rmax = float(scan.get("range_max", np.inf))
        except Exception:
            return np.array([0.0], dtype=np.float32), np.array([0.0], dtype=np.float32), default_radius

        if clamp_max_range is not None:
            rmax = min(rmax, float(clamp_max_range)) if np.isfinite(rmax) else float(clamp_max_range)

        if downsample > 1:
            ranges = ranges[::downsample]
            idx = np.arange(ranges.size, dtype=np.float32) * downsample
        else:
            idx = np.arange(ranges.size, dtype=np.float32)

        ang = angle_min + idx * angle_inc

        valid = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= (rmax if np.isfinite(rmax) else np.inf))
        if not np.any(valid):
            return np.array([0.0], dtype=np.float32), np.array([0.0], dtype=np.float32), default_radius

        x = (ranges[valid] * np.cos(ang[valid])).astype(np.float32)
        y = (ranges[valid] * np.sin(ang[valid])).astype(np.float32)

        # Safety clamp to protect UI if a weird message explodes points
        if max_points is not None and x.size > max_points:
            step = max(1, int(np.ceil(x.size / max_points)))
            x = x[::step]
            y = y[::step]

        rad = float(max(default_radius, (np.hypot(x, y).max() + 0.5))) if x.size else default_radius
        return x, y, rad


# -------- Convenience singleton --------
_receiver_singleton: Optional[LidarReceiver] = None

def start(endpoint: Optional[str] = None) -> LidarReceiver:
    global _receiver_singleton
    if _receiver_singleton is None:
        _receiver_singleton = LidarReceiver(endpoint=endpoint)
        _receiver_singleton.start()
    return _receiver_singleton

def latest_scan() -> Optional[Dict[str, Any]]:
    r = start()
    return r.latest_scan()

def latest_xy(default_radius: float = 8.0, downsample: int = 3, clamp_max_range: Optional[float] = None, max_points: Optional[int] = 5000):
    r = start()
    return r.latest_xy(default_radius=default_radius, downsample=downsample, clamp_max_range=clamp_max_range, max_points=max_points)
