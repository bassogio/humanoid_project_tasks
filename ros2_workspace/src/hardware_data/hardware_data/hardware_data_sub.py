# -----------------------------------
# Import Statements
# -----------------------------------
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# -----------------------------------
# ROS2 Node Definition
# -----------------------------------
class HardwareDataNode(Node):
    def __init__(self):
        super().__init__('hardware_data_node')

        # -------------------------------------------
        # Declare ROS2 Parameters with defaults
        # -------------------------------------------
        self.declare_parameter('publisher_topic',   'HardwareDataPub')
        self.declare_parameter('subscriber_topic',  'HardwareDataSub')

        # -------------------------------------------
        # Get Parameter Values
        # -------------------------------------------
        self.publisher_topic   = self.get_parameter('publisher_topic').value
        self.subscriber_topic  = self.get_parameter('subscriber_topic').value

        # -------------------------------------------
        # Initialize Publishers
        # -------------------------------------------
        self.publisher_ = self.create_publisher(PoseStamped, self.publisher_topic, 10)

        # -------------------------------------------
        # Initialize Subscribers
        # -------------------------------------------
        self.subscription = self.create_subscription(
            PoseStamped,
            self.subscriber_topic,
            self.listener_callback,
            10
        )

        # -------------------------------------------
        # Flags to track received messages
        # -------------------------------------------
        self.received_sub1 = False

        # Timer to check subscription readiness
        self.subscription_check_timer = self.create_timer(2.0, self.check_initial_subscriptions)

    # -------------------------------------------
    # Timer Callback
    # -------------------------------------------
    def check_initial_subscriptions(self):
        waiting_topics = []
        if not self.received_sub1:
            waiting_topics.append(f"'{self.subscriber_topic}'")

        if waiting_topics:
            self.get_logger().info(f"Waiting for messages on topics: {', '.join(waiting_topics)}")
        else:
            self.get_logger().info(
                f"All subscribed topics have received at least one message. "
                f"Publisher on '{self.publisher_topic}'"
                f"subscriber on '{self.subscriber_topic}'"
            )
            self.subscription_check_timer.cancel()

    # -------------------------------------------
    # Subscriber Callbacks
    # -------------------------------------------
    def listener_callback(self, msg: PoseStamped):
        if not self.received_sub1:
            self.received_sub1 = True

        self.get_logger().info(
            f"(Subscriber 1) Received message with frame_id: '{msg.header.frame_id}', timestamp: {msg.header.stamp}"
        )

        processed_msg = PoseStamped()
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.header.frame_id = self.frame_id
        processed_msg.pose = msg.pose

        self.publisher_.publish(processed_msg)

        self.get_logger().info(
            f"(Publisher 1) Published processed message with frame_id: '{processed_msg.header.frame_id}', "
            f"timestamp: {processed_msg.header.stamp}"
        )

# -----------------------------------
# Main Entry Point
# -----------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HardwareDataNode()
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
