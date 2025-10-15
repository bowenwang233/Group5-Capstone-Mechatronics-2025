# ImagePublisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, imagezmq

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")

        # Declare configurable parameters (for launch or YAML)
        self.declare_parameter("camera_topic", "/depth_cam/rgb/image_raw")
        self.declare_parameter("bind_addr", "tcp://*:5557")
        self.declare_parameter("jpeg_quality", 75)

        # Read parameters
        cam_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        bind_addr = self.get_parameter("bind_addr").get_parameter_value().string_value
        jpeg_q = self.get_parameter("jpeg_quality").get_parameter_value().integer_value

        # Initialize converters and ZMQ publisher
        self.bridge = CvBridge()
        self.sender = imagezmq.ImageSender(connect_to=bind_addr, REQ_REP=False)
        self.jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_q)]

        # Subscribe to the camera feed
        self.sub = self.create_subscription(Image, cam_topic, self._on_image, 10)

        self.get_logger().info(
            f"[ImagePublisher] Streaming {cam_topic} "
            f"→ {bind_addr} (quality={jpeg_q})"
        )

    def _on_image(self, msg: Image):
        """Callback that runs every time a new image arrives on the topic."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, jpg = cv2.imencode(".jpg", frame, self.jpeg_params)
            if ok:
                self.sender.send_jpg("", jpg.tobytes())
        except Exception as e:
            self.get_logger().error(f"Error encoding/sending image: {e}")

def main():
    rclpy.init()
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()