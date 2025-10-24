
"""
Keyboard Control for Digital Twin
==================================
Controls the physical car using keyboard arrow keys via ZeroMQ.

Usage:
    python keyboard_control.py

Controls:
    ↑ (Up Arrow)     - Move Forward
    ↓ (Down Arrow)  - Move Backward
    ← (Left Arrow)  - Turn Left 
    → (Right Arrow) - Turn Right
    SPACE           - Stop
    ESC / Q         - Quit

The program sends velocity commands over ZeroMQ to the car at 192.168.68.103:5558
"""

import zmq
import json 
import time
import sys
from pynput import keyboard

# ==================== Configuration  ====================
#CAR_IP = "192.168.68.103"
CAR_IP = "192.168.149.1"
CONTROL_PORT = 5558
ZMQ_ADDR = f"tcp://{CAR_IP}:{CONTROL_PORT}"

# Velocity settings (adjust based on your car's capabilities)
MAX_LINEAR_SPEED = 0.5      # m/s
MAX_ANGULAR_SPEED = 1.0     # rad/s
SPEED_STEP = 0.1            # increment per key press

# ==================== Global State ====================
current_linear = 0.0
current_angular = 0.0
running = True

# ==================== ZeroMQ Setup ====================
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.connect(ZMQ_ADDR)
print(f"[Control] Connected to {ZMQ_ADDR}")
print("Waiting 1s for ZMQ connection to establish...")
time.sleep(1)  # Give ZMQ time to connect


# ==================== Control Functions ====================
def send_command(linear: float, angular: float):
    """Send velocity command via ZeroMQ."""
    cmd = {
        "linear": float(linear),
         "angular": float(angular),
        "timestamp": time.time()
    }
    try:
        sock.send_json(cmd)
        print(f"\r[CMD] Linear: {linear:+.2f} m/s | Angular: {angular:+.2f} rad/s", end="", flush=True)
    except Exception as e:
         print(f"\n[ERROR] Failed to send command: {e}")


def stop():
    """Send stop command."""
    global current_linear, current_angular
    current_linear = 0.0
    current_angular = 0.0
    send_command(0.0, 0.0)
    print("\n[STOP] Emergency stop")


def forward():
    """Increase forward speed."""
    global current_linear
    current_linear = min(current_linear + SPEED_STEP, MAX_LINEAR_SPEED)
    send_command(current_linear, current_angular)


def backward():
    """Increase backward speed."""
    global current_linear
    current_linear = max(current_linear - SPEED_STEP, -MAX_LINEAR_SPEED)
    send_command(current_linear, current_angular)


def turn_left():
    """Turn left."""
    global current_angular
    current_angular = min(current_angular + SPEED_STEP, MAX_ANGULAR_SPEED)
    send_command(current_linear, current_angular)


def turn_right():
    """Turn right."""
    global current_angular
    current_angular = max(current_angular - SPEED_STEP, -MAX_ANGULAR_SPEED)
    send_command(current_linear, current_angular)


# ==================== Keyboard Handling ====================
def on_press(key):
    """Handle key press events."""
    global running
    
    try:
        if key == keyboard.Key.up:
            forward()
        elif key == keyboard.Key.down:
            backward()
        elif key == keyboard.Key.left:
            turn_left()
        elif key == keyboard.Key.right:
            turn_right()
        elif key == keyboard.Key.space:
            stop()
        elif key == keyboard.Key.esc:
            print("\n[EXIT] ESC pressed - Stopping car and exiting...")
            stop()
            running = False
            return False  # Stop listener
        elif hasattr(key, 'char') and key.char == 'q':
            print("\n[EXIT] Q pressed - Stopping car and exiting...")
            stop()
            running = False
            return False  # Stop listener
    except AttributeError:
        pass


def on_release(key):
    """Handle key release - optional: could implement gradual slowdown here."""
    pass


# ==================== Main ====================
def main():
    global running
    
    print("\n" + "="*60)
    print("🚗 KEYBOARD CONTROL FOR DIGITAL TWIN 🚗")
    print("="*60)
    print("\nControls:")
    print("  ↑ (Up Arrow)    - Move Forward")
    print("  ↓ (Down Arrow)  - Move Backward")
    print("  ← (Left Arrow)  - Turn Left")
    print("  → (Right Arrow) - Turn Right")
    print("  SPACE           - Emergency Stop")
    print("  ESC / Q         - Quit")
    print("\nSettings:")
    print(f"  Max Linear Speed:  {MAX_LINEAR_SPEED} m/s")
    print(f"  Max Angular Speed: {MAX_ANGULAR_SPEED} rad/s")
    print(f"  Speed Step:        {SPEED_STEP}")
    print(f"  Target: {ZMQ_ADDR}")
    print("\n" + "="*60)
    print("\n🟢 Ready! Press arrow keys to control the car.\n")
    
    # Start keyboard listener
    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            # Keep program running
            while running:
                time.sleep(0.1)
            listener.stop()
    except KeyboardInterrupt:
        print("\n[EXIT] Keyboard interrupt")
    finally:
        print("\n[Cleanup] Sending final stop command...")
        stop()
        time.sleep(0.5)
        sock.close()
        ctx.term()
        print("[Cleanup] ZMQ connection closed. Goodbye! 👋")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        sys.exit(1)

