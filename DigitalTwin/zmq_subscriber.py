# zmq_subscriber.py
"""
Background ZeroMQ SUB client that feeds state_store.
Usage from Dashboard.py:
    import zmq_subscriber
    zmq_subscriber.start()   # starts a daemon thread

Expects publisher at tcp://127.0.0.1:5556 sending topics:
'telemetry', 'lidar', 'camera', 'status'
"""

import json
import threading
import time
from typing import Optional
import zmq
import state_store as store

PUB_ADDR = "tcp://127.0.0.1:5556"

def _run(stop_event: threading.Event, addr: str):
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    # High-water mark so we don't fall too far behind
    sock.setsockopt(zmq.RCVHWM, 2000)
    sock.setsockopt(zmq.LINGER, 0)
    # subscribe to the four topics
    for topic in (b"telemetry", b"lidar", b"camera", b"status"):
        sock.setsockopt(zmq.SUBSCRIBE, topic)
    sock.connect(addr)
    print(f"[sub] Connected to {addr}")

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    while not stop_event.is_set():
        try:
            events = dict(poller.poll(timeout=200))  # ms
            if sock in events and events[sock] == zmq.POLLIN:
                topic, payload = sock.recv_multipart()
                msg = json.loads(payload.decode("utf-8"))

                if topic == b"telemetry":
                    # v, a, steer
                    store.append_sample(
                        v=msg.get("v"),
                        a=msg.get("a"),
                        steer=msg.get("steer")
                    )
                elif topic == b"lidar":
                    # x, y, z arrays (downsampling handled by state_store)
                    store.set_lidar(msg.get("x", []), msg.get("y", []), msg.get("z", []))
                elif topic == b"camera":
                    uri = msg.get("data_uri")
                    if uri:
                        store.set_camera_b64(uri)
                elif topic == b"status":
                    store.set_status(
                        battery=msg.get("battery"),
                        conn=msg.get("conn"),
                        mode=msg.get("mode")
                    )
        except zmq.ZMQError as e:
            print(f"[sub] ZMQ error: {e}; reconnecting in 1s...")
            time.sleep(1)
        except Exception as e:
            # Never crash the UI due to bad payloads
            print(f"[sub] Error handling message: {e}")

    try:
        sock.close(0)
    except Exception:
        pass
    print("[sub] Stopped.")

_thread: Optional[threading.Thread] = None
_stop: Optional[threading.Event] = None

def start(addr: str = PUB_ADDR):
    global _thread, _stop
    if _thread and _thread.is_alive():
        return
    _stop = threading.Event()
    _thread = threading.Thread(target=_run, args=(_stop, addr), daemon=True)
    _thread.start()

def stop():
    global _thread, _stop
    if _stop:
        _stop.set()
    _thread = None