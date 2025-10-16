#LidarReciever.py
"""
LiDAR ZMQ receiver module (Digital Twin side)

- Connects to Pi's ZMQ PUB (tcp://<PI_IP>:5560)
- Runs a background thread to keep the latest scan
- Provides helpers to get XY points (for Plotly) or raw scan dict
- Designed to be imported by Dashboard.py (or any other consumer)

Config:
- Set PI_IP via environment variable, or pass the endpoint explicitly to start()
- Default endpoint: tcp://192.168.0.103:5560
"""

from __future__ import annotations
import os
import time
import threading
from collections import deque
from typing import Tuple, Optional, Dict, Any

import numpy as np
import zmq


class LidarReceiver:
    def __init__(self, endpoint: Optional[str] = None, queue_size: int = 1):
        # Endpoint for the Pi (publisher)
        if endpoint is None:
            pi_ip = os.environ.get("PI_IP", "192.168.68.103")
            endpoint = f"tcp://{pi_ip}:5560"
        self._endpoint = endpoint

        # Latest message buffer (keep last only by default)
        self._latest = deque(maxlen=max(1, queue_size))
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._started = False

        # ZMQ objects
        self._ctx: Optional[zmq.Context] = None
        self._sock: Optional[zmq.Socket] = None

    @property
    def endpoint(self) -> str:
        return self._endpoint

    def start(self) -> None:
        """Start background receiver thread (idempotent)."""
        if self._started:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="LidarReceiver", daemon=True)
        self._thread.start()
        self._started = True

    def stop(self, timeout: float = 1.0) -> None:
        """Stop background thread and close sockets."""
        if not self._started:
            return
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout)
        self._thread = None
        self._started = False
        # Close ZMQ
        try:
            if self._sock is not None:
                self._sock.close(linger=0)
        finally:
            self._sock = None
            if self._ctx is not None:
                self._ctx.term()
            self._ctx = None

    def _loop(self) -> None:
        """ZMQ SUB loop with simple auto-reconnect."""
        backoff = 0.2
        while not self._stop.is_set():
            try:
                if self._ctx is None:
                    self._ctx = zmq.Context.instance()
                if self._sock is None:
                    self._sock = self._ctx.socket(zmq.SUB)
                    self._sock.connect(self._endpoint)
                    self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
                    self._sock.setsockopt(zmq.RCVTIMEO, 1000)  # 1s poll

                msg = self._sock.recv_json()
                msg["_ts"] = time.time()
                with self._lock:
                    self._latest.append(msg)
                backoff = 0.02  # fast again after success
            except zmq.Again:
                # timeout; just loop to check stop flag
                pass
            except Exception:
                # transient network or publisher restart: back off a bit then retry
                time.sleep(backoff)
                backoff = min(backoff * 2, 1.0)
                # reset socket to force reconnect
                try:
                    if self._sock is not None:
                        self._sock.close(linger=0)
                finally:
                    self._sock = None

    # -------- Retrieval helpers --------

    def latest_scan(self) -> Optional[Dict[str, Any]]:
        """Return the most recent JSON scan dict or None if nothing yet."""
        with self._lock:
            return None if not self._latest else self._latest[-1]

    def latest_xy(
        self,
        default_radius: float = 8.0,
        downsample: int = 1,
        clamp_max_range: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Convert the latest polar scan to XY.

        Returns:
            x (np.ndarray), y (np.ndarray), r_view (float)
        """
        scan = self.latest_scan()
        if scan is None:
            return np.array([0.0], dtype=np.float32), np.array([0.0], dtype=np.float32), default_radius

        angle_min = float(scan["angle_min"])
        angle_inc = float(scan["angle_increment"])
        ranges = np.asarray(scan["ranges"], dtype=np.float32)
        rmin = max(0.01, float(scan["range_min"]))
        rmax = float(scan["range_max"])
        if clamp_max_range is not None:
            rmax = min(rmax, float(clamp_max_range)) if np.isfinite(rmax) else float(clamp_max_range)

        if downsample > 1:
            ranges = ranges[::downsample]
            # adjust angles accordingly
            idx = np.arange(ranges.size, dtype=np.float32) * downsample
        else:
            idx = np.arange(ranges.size, dtype=np.float32)

        ang = angle_min + idx * angle_inc

        valid = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= (rmax if np.isfinite(rmax) else np.inf))
        if not np.any(valid):
            return np.array([0.0], dtype=np.float32), np.array([0.0], dtype=np.float32), default_radius

        x = (ranges[valid] * np.cos(ang[valid])).astype(np.float32)
        y = (ranges[valid] * np.sin(ang[valid])).astype(np.float32)

        rad = float(max(default_radius, (np.hypot(x, y).max() + 0.5))) if x.size else default_radius
        return x, y, rad


# -------- Convenience singleton (easy import in Dashboard) --------
_receiver_singleton: Optional[LidarReceiver] = None

def start(endpoint: Optional[str] = None) -> LidarReceiver:
    """Start (or return) a module-level singleton receiver."""
    global _receiver_singleton
    if _receiver_singleton is None:
        _receiver_singleton = LidarReceiver(endpoint=endpoint)
        _receiver_singleton.start()
    return _receiver_singleton

def latest_scan() -> Optional[Dict[str, Any]]:
    """Get latest scan dict using the singleton."""
    r = start()
    return r.latest_scan()

def latest_xy(default_radius: float = 8.0, downsample: int = 1, clamp_max_range: Optional[float] = None):
    """Get latest XY using the singleton (ready for Plotly)."""
    r = start()
    return r.latest_xy(default_radius=default_radius, downsample=downsample, clamp_max_range=clamp_max_range)