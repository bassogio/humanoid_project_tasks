# -----------------------------------
# Import Statements
# -----------------------------------
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# -----------------------------------
# ROS2 Node Definition
# -----------------------------------
class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # -------------------------------------------
        # Declare ROS2 Parameters with defaults
        # -------------------------------------------
        self.declare_parameter('publisher_topic', '/camera/image_raw')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_rate', 10.0)

        # -------------------------------------------
        # Get Parameter Values
        # -------------------------------------------
        self.publisher_topic = self.get_parameter('publisher_topic').value
        self.camera_index    = self.get_parameter('camera_index').value
        self.publish_rate    = self.get_parameter('publish_rate').value

        # -------------------------------------------
        # Create Publisher
        # -------------------------------------------
        self.publisher_ = self.create_publisher(Image, self.publisher_topic, 10)
        self.get_logger().info(f"Publisher created on topic: {self.publisher_topic}")

        # -------------------------------------------
        # Video Capture
        # -------------------------------------------
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            raise RuntimeError("Camera not available.")

        # CvBridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Timer for periodic publishing
        timer_period = 1.0 / self.publish_rate


        # Timer for periodic publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_image)

    # -------------------------------------------
    # Timer Callback - capture frame and publish
    # -------------------------------------------
    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture image from camera.")
            return

        # Convert BGR (OpenCV default) to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Convert to ROS2 Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
        img_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish
        self.publisher_.publish(img_msg)
        self.get_logger().debug("Published image frame.")

# -----------------------------------
# Main Entry Point
# -----------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# -----------------------------------
# Run the node when executed directly
# -----------------------------------
if __name__ == '__main__':
    main()
