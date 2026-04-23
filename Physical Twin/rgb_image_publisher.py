import subprocess
import socket
import imagezmq
import zmq


# ... (keep your constants DEVICE, WIDTH, etc.) ...
DEVICE = "/dev/video0"  # Orbbec RGB device
WIDTH = 640
HEIGHT = 480
FPS = 10
PORT = 5555



# OPTIMIZATION 1: Set High Water Mark to 1 to eliminate "old video" lag
sender = imagezmq.ImageSender(f"tcp://*:{PORT}", REQ_REP=False)
sender.zmq_socket.setsockopt(zmq.SNDHWM, 1) 

rpi_name = socket.gethostname()

# OPTIMIZATION 2: Use -f mpjpeg for a cleaner stream delimiter
ffmpeg = subprocess.Popen(
    [
        "ffmpeg",
        "-f", "v4l2",
        "-input_format", "mjpeg",
        "-video_size", f"{WIDTH}x{HEIGHT}",
        "-framerate", str(FPS),
        "-i", DEVICE,
        "-c:v", "copy",
        "-f", "mpjpeg",      # Multi-part JPEG stream
        "pipe:1"
    ],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
)

# A more efficient way to grab JPEG chunks
def get_frames(stream):
    buffer = b""
    while True:
        chunk = stream.read(4096) # Larger chunks are more efficient
        if not chunk: break
        buffer += chunk
        a = buffer.find(b'\xff\xd8')
        b = buffer.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            yield jpg

try:
    for frame in get_frames(ffmpeg.stdout):
        # OPTIMIZATION 3: Send directly
        sender.send_jpg(rpi_name, frame)
except KeyboardInterrupt:
    ffmpeg.terminate()
