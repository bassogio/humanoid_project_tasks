# -----------------------------------
# Import Statements
# -----------------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  

# -----------------------------------
# ROS2 Node Definition
# -----------------------------------
class HardwareDataSubscriberNode(Node):
    def __init__(self):
        super().__init__('hardware_data_node')

        # -------------------------------------------
        # Declare ROS2 Parameters with defaults
        # -------------------------------------------
        self.declare_parameter('subscriber_topic', 'myHWtopic')

        # -------------------------------------------
        # Get Parameter Values
        # -------------------------------------------
        self.subscriber_topic  = self.get_parameter('subscriber_topic').value

        # -------------------------------------------
        # Initialize Subscribers
        # -------------------------------------------
        self.subscription = self.create_subscription(
            String,
            self.subscriber_topic,
            self.listener_callback,
            10
        )

        # -------------------------------------------
        # Flags to track received messages
        # -------------------------------------------
        self.received_sub1 = False

        # Timer to check subscription readiness
        self.subscription_check_timer = self.create_timer(1.0, self.check_initial_subscriptions)

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
            self.subscription_check_timer.cancel()

    # -------------------------------------------
    # Subscriber Callbacks
    # -------------------------------------------
    def listener_callback(self, msg: String):
        if not self.received_sub1:
            self.received_sub1 = True
        self.get_logger().info(f"Received: '{msg.data}'")
        
# -----------------------------------
# Main Entry Point
# -----------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HardwareDataSubscriberNode()
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
