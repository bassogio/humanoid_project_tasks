# -----------------------------------
# Import Statements
# -----------------------------------
import rclpy
import psutil
from rclpy.node import Node
from std_msgs.msg import String  

# -----------------------------------
# ROS2 Node Definition
# -----------------------------------
class HardwareDataPublisherNode(Node):
    def __init__(self):
        super().__init__('hardware_data_pub_node')

        # -------------------------------------------
        # Declare ROS2 Parameters with defaults
        # -------------------------------------------
        self.declare_parameter('publisher_topic', 'myHWtopic')
        self.declare_parameter('sampling_rate'  , 1.0)         # R1 (Hz)
        self.declare_parameter('publish_rate'   , 0.5)         # R2 (Hz)

        # -------------------------------------------
        # Get Parameter Values
        # -------------------------------------------
        self.publisher_topic = self.get_parameter('publisher_topic').value
        self.sampling_rate   = self.get_parameter('sampling_rate').value
        self.publish_rate    = self.get_parameter('publish_rate').value

        # -------------------------------------------
        # Create Publisher
        # -------------------------------------------
        self.publisher_ = self.create_publisher(String, self.publisher_topic, 10)
        self.get_logger().info(f"Publisher created on topic: {self.publisher_topic}")

        # -------------------------------------------
        # Initialize data buffer
        # -------------------------------------------
        self.latest_cpu_load = None

        # -------------------------------------------
        # Timers for sampling and publishing
        # -------------------------------------------
        self.sampling_timer = self.create_timer(1.0 / self.sampling_rate, self.sample_cpu_load)
        self.publish_timer  = self.create_timer(1.0 / self.publish_rate, self.publish_message)

    # -------------------------------------------
    # Sampling Callback (R1 rate)
    # -------------------------------------------
    def sample_cpu_load(self):
        self.latest_cpu_load = psutil.cpu_percent(interval=None)
        self.get_logger().debug(f"Sampled CPU load: {self.latest_cpu_load}%")

    # -------------------------------------------
    # Publish Callback (R2 rate)
    # -------------------------------------------
    def publish_message(self):
        if self.latest_cpu_load is None:
            self.get_logger().warn("No CPU load data yet. Skipping publish.")
            return

        msg = String()
        msg.data = f"CPU Load: {self.latest_cpu_load}%"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")

# -----------------------------------
# Main Entry Point
# -----------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HardwareDataPublisherNode()
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
