import sys
import socket
import traceback
import threading
import numpy as np
from time import time
from imutils.video import VideoStream
import imagezmq
import simplejpeg
import cv2
from ultralytics import YOLO
import struct
import zmq

# =========================
# Video Subscriber
# =========================
class VideoStreamSubscriber:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self._stop = False
        self._data_ready = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()

    def receive(self, timeout=60.0):
        flag = self._data_ready.wait(timeout=timeout)
        if not flag:
            raise TimeoutError(
                f"Timeout while reading from subscriber tcp://{self.hostname}:{self.port}")
        self._data_ready.clear()
        return self._data

    def _run(self):
        receiver = imagezmq.ImageHub(f"tcp://{self.hostname}:{self.port}", REQ_REP=False)
        while not self._stop:
            self._data = receiver.recv_jpg()
            self._data_ready.set()
        receiver.close()

    def close(self):
        self._stop = True

# =========================
# TCP SERVER FOR SIMULINK
# =========================
class BoundingBoxTCPServer:
    def __init__(self, host="127.0.0.1", port=5005):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(1)
        print(f"[TCP] Waiting for Simulink on {host}:{port}")
        self.conn, addr = self.sock.accept()
        print(f"[TCP] Simulink connected from {addr}")

    def send_packet(self, packet: bytes):
        try:
            self.conn.sendall(packet)
        except BrokenPipeError:
            print("[TCP] Simulink disconnected")

    def close(self):
        self.conn.close()
        self.sock.close()

# =========================
# Gesture stability tracker
# =========================
class StableGestureTracker:
    def __init__(self, hold_time=0.5):
        self.hold_time = hold_time
        self.current_class = None
        self.start_time = None

    def update(self, detected_class):
        if detected_class is None:
            self.current_class = None
            self.start_time = None
            return False

        now = time()
        if detected_class != self.current_class:
            self.current_class = detected_class
            self.start_time = now
            return False

        if self.start_time is not None and (now - self.start_time) >= self.hold_time:
            return True

        return False

# =========================
# MAIN
# =========================
def main():
    hostname = "127.0.0.1" #10.42.0.1 192.168.149.1
    PORT = 5555

    print("[INFO] Initializing YOLO models")
    gesture_model = YOLO("YOLOv10x_gestures.pt")
    person_model = YOLO("yolov8s")  # Placeholder for person detection

    subscriber = VideoStreamSubscriber(hostname, PORT)
    bbox_tcp = BoundingBoxTCPServer(port=5005)
    gesture_tracker = StableGestureTracker(hold_time=0.5)

    target_person_box = None  # No target yet
    trigger_class = 9         # Heart-hand gesture class placeholder
    IOU_THRESHOLD = 0.3       # Threshold to detect loss of target

    try:
        while True:
            rpi_name, jpg_buffer = subscriber.receive()
            frame = cv2.imdecode(np.frombuffer(jpg_buffer, dtype=np.uint8), cv2.IMREAD_COLOR)

            # --- Run YOLO models ---
            gesture_results = gesture_model.predict(frame, stream=False, verbose=False)
            person_results = person_model.predict(frame, stream=False, verbose=False)

            # --- Extract boxes ---
            gesture_boxes = gesture_results[0].boxes
            person_boxes = person_results[0].boxes

            # --- Find trigger heart-hand gesture ---
            trigger_gesture_box = None
            for g in gesture_boxes:
                if int(g.cls[0].cpu()) == trigger_class:  # ensure tensor on CPU
                    trigger_gesture_box = g.xyxy[0].cpu().numpy()
                    break

            # --- Initialization ---
            if target_person_box is None and trigger_gesture_box is not None and len(person_boxes) > 0:
                def box_center(b):
                    return ((b[0]+b[2])/2, (b[1]+b[3])/2)

                gx, gy = box_center(trigger_gesture_box)
                distances = []
                for p in person_boxes:
                    px1, py1, px2, py2 = p.xyxy[0].cpu().numpy()
                    distances.append(np.hypot(gx - (px1+px2)/2, gy - (py1+py2)/2))
                closest_idx = int(np.argmin(distances))
                target_person_box = person_boxes[closest_idx].xyxy[0].cpu().numpy()
                print("[INFO] Target person initialized.")

            # --- Tracking phase ---
            if target_person_box is not None:
                def iou(boxA, boxB):
                    xA = max(boxA[0], boxB[0])
                    yA = max(boxA[1], boxB[1])
                    xB = min(boxA[2], boxB[2])
                    yB = min(boxA[3], boxB[3])
                    interArea = max(0, xB - xA) * max(0, yB - yA)
                    boxAArea = (boxA[2]-boxA[0])*(boxA[3]-boxA[1])
                    boxBArea = (boxB[2]-boxB[0])*(boxB[3]-boxB[1])
                    return interArea / float(boxAArea + boxBArea - interArea + 1e-6)

                ious = []
                for p in person_boxes:
                    pb = p.xyxy[0].cpu().numpy()
                    ious.append(iou(target_person_box, pb))

                if len(ious) == 0 or max(ious) < IOU_THRESHOLD:
                    target_person_box = None
                    print("[INFO] Target person lost. Waiting for re-initialization.")
                else:
                    best_idx = int(np.argmax(ious))
                    target_person_box = person_boxes[best_idx].xyxy[0].cpu().numpy()

            # --- Determine gesture belonging to target ---
            gesture_to_send = None
            if target_person_box is not None:
                tx1, ty1, tx2, ty2 = target_person_box
                for g in gesture_boxes:
                    gx1, gy1, gx2, gy2 = g.xyxy[0].cpu().numpy()
                    gcx = (gx1+gx2)/2
                    gcy = (gy1+gy2)/2
                    if tx1 <= gcx <= tx2 and ty1 <= gcy <= ty2:
                        gesture_to_send = g
                        break

            # --- Build TCP packet ---
            packet = struct.pack("<I", 1)  # Always send 1 bounding box

            if target_person_box is not None:
                px1, py1, px2, py2 = target_person_box
                packet += struct.pack("<i5f", 1, 1.0, float(px1), float(py1), float(px2-px1), float(py2-py1))
            else:
                packet += struct.pack("<i5f", 0, 0.0, 0.0, 0.0, 0.0, 0.0)

            if gesture_to_send is not None:
                g = gesture_to_send
                gx1, gy1, gx2, gy2 = g.xyxy[0].cpu().numpy()
                packet += struct.pack("<i5f", int(g.cls[0].cpu()), float(g.conf[0].cpu()),
                                      float(gx1), float(gy1), float(gx2-gx1), float(gy2-gy1))
            else:
                packet += struct.pack("<i5f", 0, 0.0, 0.0, 0.0, 0.0, 0.0)

            bbox_tcp.send_packet(packet)

            # --- Visualization ---
            vis_frame = gesture_results[0].plot()
            if target_person_box is not None:
                cv2.rectangle(vis_frame,
                              (int(target_person_box[0]), int(target_person_box[1])),
                              (int(target_person_box[2]), int(target_person_box[3])),
                              (0,255,0), 2)
            cv2.imshow("Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except (KeyboardInterrupt, SystemExit):
        print("[INFO] Exiting")
    except Exception as ex:
        print("Python error:")
        traceback.print_exc()
    finally:
        subscriber.close()
        bbox_tcp.close()
        cv2.destroyAllWindows()
        sys.exit()

if __name__ == "__main__":
    main()
