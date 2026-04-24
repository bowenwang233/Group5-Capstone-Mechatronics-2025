import sys
import socket
import traceback
import threading
import numpy as np
import time as time_module
import imagezmq
import cv2
from ultralytics import YOLO
import struct
import zmq
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# =========================
# Shared HTTP State
# =========================
latest_jpg = None
state_lock = threading.Lock()

# =========================
# HTTP SERVER FOR QT UI
# =========================
class HttpHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/frame.jpg":
            with state_lock:
                jpg = latest_jpg
            if jpg is None:
                self.send_response(204)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            self.wfile.write(jpg)
            return

        if self.path == "/detections":
            payload = json.dumps({"boxes": []}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        return

def start_http_server(host="127.0.0.1", port=8000):
    server = ThreadingHTTPServer((host, port), HttpHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"[HTTP] Serving on http://{host}:{port}")
    return server

def update_http_frame(vis_frame):
    global latest_jpg
    ok, jpg = cv2.imencode(".jpg", vis_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        return
    with state_lock:
        latest_jpg = jpg.tobytes()

# =========================
# MULTI-CLIENT NON-BLOCKING TCP SERVER
# =========================
class BoundingBoxTCPServer:
    def __init__(self, host="0.0.0.0", port=5005):
        self.host = host
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen()
        print(f"[TCP] Server started on {self.host}:{self.port}")
        self.thread = threading.Thread(target=self._accept_clients, daemon=True)
        self.thread.start()

    def _accept_clients(self):
        while True:
            try:
                conn, addr = self.sock.accept()
                print(f"[TCP] Client connected from {addr}")
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.lock:
                    self.clients.append(conn)
            except:
                break

    def send_packet(self, packet: bytes):
        with self.lock:
            alive_clients = []
            for c in self.clients:
                try:
                    c.sendall(packet)
                    alive_clients.append(c)
                except:
                    print("[TCP] Client disconnected")
            self.clients = alive_clients

    def close(self):
        self.sock.close()

# =========================
# VIDEO SUBSCRIBER
# =========================
class VideoStreamSubscriber:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self._data_ready = threading.Event()
        self._data = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def receive(self):
        if self._data_ready.is_set():
            data = self._data
            self._data_ready.clear()
            return data
        return None

    def _run(self):
        receiver = imagezmq.ImageHub(f"tcp://{self.hostname}:{self.port}", REQ_REP=False)
        receiver.zmq_socket.setsockopt(zmq.CONFLATE, 1)
        while not self._stop:
            self._data = receiver.recv_jpg()
            self._data_ready.set()

    def close(self): self._stop = True

# =========================
# MAIN
# =========================
def main():
    hostname = "10.42.0.1"
    port = 5555

    print("[INFO] Initializing YOLO models")
    gesture_model = YOLO("YOLOv10n_gestures.pt")
    person_model = YOLO("yolov8s")

    # Start HTTP Server for UI
    http_server = start_http_server(host="127.0.0.1", port=8000)
    subscriber = VideoStreamSubscriber(hostname, port)
    bbox_tcp = BoundingBoxTCPServer(port=5005)

    target_person_box = None
    trigger_class = 9 
    IOU_THRESHOLD = 0.3

    try:
        while True:
            received_data = subscriber.receive()
            if received_data is None:
                time_module.sleep(0.01)
                continue

            rpi_name, jpg_buffer = received_data
            frame = cv2.imdecode(np.frombuffer(jpg_buffer, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None: continue

            # --- YOLO Inference ---
            gesture_results = gesture_model.predict(frame, verbose=False)
            person_results = person_model.predict(frame, verbose=False)

            gesture_boxes = gesture_results[0].boxes
            person_boxes = person_results[0].boxes

            # --- Trigger logic ---
            trigger_gesture_box = None
            for g in gesture_boxes:
                if int(g.cls[0].cpu()) == trigger_class:
                    trigger_gesture_box = g.xyxy[0].cpu().numpy()
                    break

            # --- Initialization ---
            if target_person_box is None and trigger_gesture_box is not None and len(person_boxes) > 0:
                gx = (trigger_gesture_box[0] + trigger_gesture_box[2]) / 2
                gy = (trigger_gesture_box[1] + trigger_gesture_box[3]) / 2
                dists = []
                for p in person_boxes:
                    pb = p.xyxy[0].cpu().numpy()
                    px = (pb[0] + pb[2]) / 2
                    py = (pb[1] + pb[3]) / 2
                    dists.append(np.hypot(gx - px, gy - py))
                target_person_box = person_boxes[np.argmin(dists)].xyxy[0].cpu().numpy()
                print("[INFO] Target initialized")

            # --- Tracking ---
            if target_person_box is not None:
                ious = []
                for p in person_boxes:
                    pb = p.xyxy[0].cpu().numpy()
                    xA, yA = max(target_person_box[0], pb[0]), max(target_person_box[1], pb[1])
                    xB, yB = min(target_person_box[2], pb[2]), min(target_person_box[3], pb[3])
                    inter = max(0, xB - xA) * max(0, yB - yA)
                    areaA = (target_person_box[2]-target_person_box[0])*(target_person_box[3]-target_person_box[1])
                    areaB = (pb[2]-pb[0])*(pb[3]-pb[1])
                    ious.append(inter / float(areaA + areaB - inter + 1e-6))

                if not ious or max(ious) < IOU_THRESHOLD:
                    target_person_box = None
                    print("[INFO] Target lost")
                else:
                    best_idx = int(np.argmax(ious))
                    target_person_box = person_boxes[best_idx].xyxy[0].cpu().numpy()

            # --- Gesture belonging to target ---
            gesture_to_send = None
            if target_person_box is not None:
                tx1, ty1, tx2, ty2 = target_person_box
                for g in gesture_boxes:
                    gx1, gy1, gx2, gy2 = g.xyxy[0].cpu().numpy()
                    gcx, gcy = (gx1 + gx2) / 2, (gy1 + gy2) / 2
                    if tx1 <= gcx <= tx2 and ty1 <= gcy <= ty2:
                        gesture_to_send = g
                        break

            # --- Build 52-byte TCP Packet ---
            packet = struct.pack("<I", 1) 
            if target_person_box is not None:
                px1, py1, px2, py2 = target_person_box
                packet += struct.pack("<i5f", 1, 1.0, float(px1), float(py1), float(px2-px1), float(py2-py1))
            else:
                packet += struct.pack("<i5f", 0, 0.0, 0, 0, 0, 0)

            if gesture_to_send is not None:
                g = gesture_to_send
                gx1, gy1, gx2, gy2 = g.xyxy[0].cpu().numpy()
                packet += struct.pack("<i5f", int(g.cls[0].cpu()), float(g.conf[0].cpu()), float(gx1), float(gy1), float(gx2-gx1), float(gy2-gy1))
            else:
                packet += struct.pack("<i5f", 0, 0.0, 0, 0, 0, 0)

            bbox_tcp.send_packet(packet)

            # --- Visualization and UI Update ---
            vis_frame = gesture_results[0].plot()
            if target_person_box is not None:
                cv2.rectangle(vis_frame, (int(target_person_box[0]), int(target_person_box[1])), 
                              (int(target_person_box[2]), int(target_person_box[3])), (0, 255, 0), 2)
            
            update_http_frame(vis_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'): break

    except (KeyboardInterrupt, SystemExit):
        print("[INFO] Exiting")
    except Exception:
        traceback.print_exc()
    finally:
        subscriber.close()
        bbox_tcp.close()
        try:
            http_server.shutdown()
            http_server.server_close()
        except: pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

