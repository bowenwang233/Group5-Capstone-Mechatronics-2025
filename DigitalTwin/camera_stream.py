# camera_stream.py
import threading
import base64
import imagezmq

class CameraStream:
    """
    Pulls JPEG frames from the JetAcker (ImagePublisher) via ImageZMQ in a
    background thread and exposes the latest frame as base64 for Dash <img>.
    """
    def __init__(self, car_ip: str, port: int = 5557):
        # Jetson side binds tcp://*:5557 (PUB). We connect as SUB here.
        self._endpoint = f"tcp://{car_ip}:{port}"
        self._hub = imagezmq.ImageHub(open_port=self._endpoint, REQ_REP=False)

        self._latest_jpg: bytes | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        if not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def latest_base64(self) -> str | None:
        """Return the latest JPEG as base64 (ready for <img src='data:image/jpeg;base64,...'>)."""
        with self._lock:
            if self._latest_jpg is None:
                return None
            return base64.b64encode(self._latest_jpg).decode("ascii")

    # --- internal loop ---
    def _loop(self):
        while not self._stop.is_set():
            try:
                # name is unused; jpg_buf is already JPEG bytes-like
                _, jpg_buf = self._hub.recv_jpg()
                with self._lock:
                    self._latest_jpg = bytes(jpg_buf)
            except Exception:
                # transient network hiccup: keep trying
                continue