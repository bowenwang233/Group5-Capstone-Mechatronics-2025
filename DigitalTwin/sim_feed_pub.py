# sim_feed_pub.py
"""
ZeroMQ Publisher that broadcasts simulated Digital Twin data.
Run this in a separate terminal:
    python sim_feed_pub.py
Subscribers connect to tcp://127.0.0.1:5556 and subscribe to topics:
  'telemetry', 'lidar', 'camera', 'status'
Message format: multipart [topic, json-utf8]
"""

import time, math, random, json, base64, io
from typing import Tuple, List
from PIL import Image, ImageDraw
import zmq

PUB_ADDR = "tcp://127.0.0.1:5556"
UPDATE_HZ = 10         # telemetry rate
LIDAR_HZ = 2
CAMERA_HZ = 1

def make_fake_lidar(n=800) -> Tuple[List[float], List[float], List[float]]:
    xs, ys, zs = [], [], []
    for _ in range(n):
        ang = random.random() * 2 * math.pi
        r = 5 + random.random() * 2
        xs.append(r * math.cos(ang))
        ys.append(r * math.sin(ang))
        zs.append(random.random() * 0.5)
    return xs, ys, zs

def make_fake_camera_datauri() -> str:
    img = Image.new("RGB", (160, 120), "black")
    draw = ImageDraw.Draw(img)
    for _ in range(5):
        x0, y0 = random.randint(0, 120), random.randint(0, 80)
        x1, y1 = x0 + random.randint(10, 40), y0 + random.randint(10, 40)
        draw.rectangle([x0, y0, x1, y1], outline="white",
                       fill=random.choice(["red", "green", "blue"]))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    b64 = base64.b64encode(buf.getvalue()).decode()
    return f"data:image/jpeg;base64,{b64}"

def main():
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    # Avoid publisher blocking on slow sub
    sock.setsockopt(zmq.SNDHWM, 1000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(PUB_ADDR)

    print(f"[pub] Publishing on {PUB_ADDR}")
    t0 = time.monotonic()
    last_lidar = last_cam = t0
    try:
        while True:
            t = time.monotonic() - t0

            # Telemetry @ UPDATE_HZ
            v = 2.0 + 0.5 * math.sin(2 * math.pi * 0.3 * t)
            a = 0.5 * math.cos(2 * math.pi * 0.3 * t)
            steer = 10 * math.sin(2 * math.pi * 0.1 * t)
            sock.send_multipart([b"telemetry", json.dumps({"v": v, "a": a, "steer": steer}).encode()])

            # Status at a low rate
            if int(t) % 2 == 0:
                sock.send_multipart([b"status", json.dumps({
                    "battery": f"{random.randint(70,95)}%",
                    "conn": "OK",
                    "mode": "AUTO"
                }).encode()])

            # LiDAR @ LIDAR_HZ
            if time.monotonic() - last_lidar >= 1 / max(1, LIDAR_HZ):
                xs, ys, zs = make_fake_lidar()
                sock.send_multipart([b"lidar", json.dumps({"x": xs, "y": ys, "z": zs}).encode()])
                last_lidar = time.monotonic()

            # Camera @ CAMERA_HZ
            if time.monotonic() - last_cam >= 1 / max(1, CAMERA_HZ):
                data_uri = make_fake_camera_datauri()
                sock.send_multipart([b"camera", json.dumps({"data_uri": data_uri}).encode()])
                last_cam = time.monotonic()

            time.sleep(1 / UPDATE_HZ)
    except KeyboardInterrupt:
        print("\n[pub] Stopped.")
    finally:
        sock.close(0)

if __name__ == "__main__":
    main()