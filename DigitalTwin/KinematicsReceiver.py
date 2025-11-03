# KinematicsReceiver_compat.py
from __future__ import annotations
import json, os, threading, time, math
from typing import Optional, Any, Dict, List
import zmq
import state_store

CAR_IP = os.environ.get("CAR_IP", os.environ.get("PI_IP", "192.168.68.103"))
PORT   = int(os.environ.get("TELEMETRY_PORT", "5558"))
TOPIC_BYTES  = os.environ.get("TELEMETRY_TOPIC", "telemetry").encode("utf-8")

STALE_S = 1.0
LOST_S  = 5.0

def _pick(d: Dict[str, Any], names: List[str]) -> Optional[float]:
    for n in names:
        if n in d and d[n] is not None:
            try:
                return float(d[n])
            except Exception:
                continue
    return None

def _rad2deg_if_needed(x: Optional[float]) -> Optional[float]:
    if x is None:
        return None
    # If the magnitude suggests radians (<= ~6 rad), convert; if already degrees, leave as-is.
    if abs(x) <= math.pi * 2.2:
        return math.degrees(x)
    return x

class _Receiver:
    def __init__(self, ip: str, port: int):
        self.endpoint = f"tcp://{ip}:{port}"
        self.ctx = zmq.Context.instance()
        self.sock: Optional[zmq.Socket] = None
        self.stop_ev = threading.Event()
        self.th: Optional[threading.Thread] = None
        self.last_rx_wall: float = 0.0
        self._last_push_wall: float = 0.0
        self._last_status_wall: float = 0.0
        self._rx_count: int = 0
        self._rx_hz: float = 0.0

    def start(self):
        if self.th and self.th.is_alive():
            return
        self.stop_ev.clear()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.RCVHWM, 2000)
        self.sock.setsockopt(zmq.CONFLATE, 1)
        # Broad subscribe to accept any topic framing; we'll not require a topic match.
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.sock.connect(self.endpoint)
        self.th = threading.Thread(target=self._loop, name="KinematicsReceiverCompat", daemon=True)
        self.th.start()
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
                ev = dict(poller.poll(100))
                if self.sock in ev and ev[self.sock] & zmq.POLLIN:
                    frames = self.sock.recv_multipart()
                    # Accept 1-frame (payload) or 2-frame (topic, payload)
                    payload = frames[-1] if len(frames) >= 1 else None
                    if not payload:
                        continue
                    try:
                        msg = json.loads(payload.decode("utf-8"))
                    except Exception:
                        # not JSON, ignore
                        continue

                    # Accept various field names for robustness
                    v = _pick(msg, ["v","vel","velocity","speed","linear_v","vx"])
                    a = _pick(msg, ["a","acc","accel","acceleration","ax"])
                    steer = _pick(msg, ["steer","steer_deg","steering","steering_deg","delta"])
                    steer_deg = _rad2deg_if_needed(steer)

                    now_wall = time.time()

                    # Only push ~20 Hz
                    if now_wall - self._last_push_wall >= 0.05:
                        state_store.append_sample(v, a, steer_deg, t=None)
                        self._last_push_wall = now_wall

                    # Metrics
                    self._rx_count += 1
                    if self.last_rx_wall == 0.0:
                        self.last_rx_wall = now_wall
                    # update hz once per second
                    if now_wall - self.last_rx_wall >= 1.0:
                        self._rx_hz = self._rx_count / (now_wall - self.last_rx_wall)
                        self._rx_count = 0
                        self.last_rx_wall = now_wall

                    if now_wall - self._last_status_wall >= 0.5:
                        state_store.set_status(conn="OK", rx_hz=round(self._rx_hz, 1))
                        self._last_status_wall = now_wall
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

_receiver_singleton = _Receiver(CAR_IP, PORT)

def start_receiver():
    _receiver_singleton.start()

def stop_receiver():
    _receiver_singleton.stop()

if __name__ == "__main__":
    start_receiver()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_receiver()
