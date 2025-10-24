# KinematicsReceiver.py
from __future__ import annotations
import json
import os
import threading
import time
from math import degrees
from typing import Optional

import zmq   # pip install pyzmq
import state_store  # your existing store with append_sample()/set_status()
CAR_IP = "192.168.149.1"
CAR_PORT = "5557"
# ---- Config (env overrides) ----
PI_IP = os.environ.get("PI_IP", os.environ.get("CAR_IP", CAR_IP))
TELEMETRY_PORT = int(os.environ.get("TELEMETRY_PORT", CAR_PORT))  # avoid 5557 (camera) / 5560 (LiDAR)
TOPIC = os.environ.get("TELEMETRY_TOPIC", "telemetry").encode("utf-8")

# Conn health thresholds
STALE_S = 1.0
LOST_S  = 5.0

class _Receiver:
    def __init__(self, ip: str, port: int, topic: bytes):
        self.endpoint = f"tcp://{ip}:{port}"
        self.topic = topic
        self.ctx = zmq.Context.instance()
        self.sock: Optional[zmq.Socket] = None
        self.stop_ev = threading.Event()
        self.th: Optional[threading.Thread] = None
        self.last_rx_wall: float = 0.0

    def start(self):
        if self.th and self.th.is_alive():
            return
        self.stop_ev.clear()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.RCVHWM, 2000)
        self.sock.setsockopt(zmq.SUBSCRIBE, self.topic)
        self.sock.connect(self.endpoint)
        self.th = threading.Thread(target=self._loop, name="KinematicsReceiver", daemon=True)
        self.th.start()
        # background status updater
        threading.Thread(target=self._status_loop, name="KinematicsStatus", daemon=True).start()

    def stop(self):
        self.stop_ev.set()
        if self.th:
            self.th.join(timeout=0.5)
        if self.sock is not None:
            try: self.sock.close(0)
            except Exception: pass
            self.sock = None

    def _loop(self):
        poller = zmq.Poller()
        poller.register(self.sock, zmq.POLLIN)
        while not self.stop_ev.is_set():
            try:
                ev = dict(poller.poll(100))  # 100 ms
                if self.sock in ev and ev[self.sock] & zmq.POLLIN:
                    topic, payload = self.sock.recv_multipart()
                    if topic != self.topic:
                        continue
                    msg = json.loads(payload.decode("utf-8"))
                    v = msg.get("v")      # m/s
                    a = msg.get("a")      # m/s^2
                    steer_rad = msg.get("steer")  # radians
                    steer_deg = degrees(steer_rad) if steer_rad is not None else None

                    # Use dashboard-side monotonic wall time (let state_store stamp it)
                    state_store.append_sample(v, a, steer_deg, t=None)

                    # mark connection fresh
                    self.last_rx_wall = time.time()
                    state_store.set_status(conn="OK")
            except Exception:
                time.sleep(0.05)

    def _status_loop(self):
        while not self.stop_ev.is_set():
            now = time.time()
            age = now - self.last_rx_wall if self.last_rx_wall else 1e9
            if age >= LOST_S:
                state_store.set_status(conn="Lost")
            elif age >= STALE_S:
                state_store.set_status(conn="Stale")
            time.sleep(0.2)

_receiver_singleton = _Receiver(PI_IP, TELEMETRY_PORT, TOPIC)

def start_receiver():
    """Call once at app startup."""
    _receiver_singleton.start()

def stop_receiver():
    _receiver_singleton.stop()

if __name__ == "__main__":
    # Handy for a quick manual run while developing (still needs the dashboard
    # process to import the same state_store instance; typically you won't run this standalone).
    start_receiver()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_receiver()