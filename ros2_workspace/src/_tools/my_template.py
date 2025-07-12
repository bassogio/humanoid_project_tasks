# -----------------------------------
# Import Statements
# -----------------------------------
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# -----------------------------------
# ROS2 Node Definition
# -----------------------------------
class GeneralTaskNode(Node):
    """
    A generic ROS2 node with multiple publishers/subscribers,
    optional service functionality, and ROS2 parameters.
    """

    def __init__(self):
        super().__init__('general_task_node')

        # -------------------------------------------
        # Declare ROS2 Parameters with defaults
        # -------------------------------------------
        self.declare_parameter('publisher_topic',   'output_topic')
        self.declare_parameter('publisher_topic2',  'output_topic2')
        self.declare_parameter('subscriber_topic',  'input_topic')
        self.declare_parameter('subscriber_topic2', 'input_topic2')
        self.declare_parameter('frame_id',          'base_link')
        self.declare_parameter('use_service',       False)
        self.declare_parameter('service_name',      'trigger_service')

        # -------------------------------------------
        # Get Parameter Values
        # -------------------------------------------
        self.publisher_topic   = self.get_parameter('publisher_topic').value
        self.publisher_topic2  = self.get_parameter('publisher_topic2').value
        self.subscriber_topic  = self.get_parameter('subscriber_topic').value
        self.subscriber_topic2 = self.get_parameter('subscriber_topic2').value
        self.frame_id          = self.get_parameter('frame_id').value
        self.use_service       = self.get_parameter('use_service').value
        self.service_name      = self.get_parameter('service_name').value

        # -------------------------------------------
        # Initialize Publishers
        # -------------------------------------------
        self.publisher_ = self.create_publisher(PoseStamped, self.publisher_topic, 10)
        self.publisher2_ = self.create_publisher(PoseStamped, self.publisher_topic2, 10)

        # -------------------------------------------
        # Initialize Subscribers
        # -------------------------------------------
        self.subscription = self.create_subscription(
            PoseStamped,
            self.subscriber_topic,
            self.listener_callback,
            10
        )

        self.subscription2 = self.create_subscription(
            PoseStamped,
            self.subscriber_topic2,
            self.listener_callback2,
            10
        )

        # -------------------------------------------
        # Flags to track received messages
        # -------------------------------------------
        self.received_sub1 = False
        self.received_sub2 = False

        # Timer to check subscription readiness
        self.subscription_check_timer = self.create_timer(2.0, self.check_initial_subscriptions)

        # -------------------------------------------
        # Optional Service
        # -------------------------------------------
        if self.use_service:
            self.service_server = self.create_service(Trigger, self.service_name, self.service_callback)
            self.get_logger().info(f"Service server started on '{self.service_name}'")

        # -------------------------------------------
        # Pose Defaults
        # -------------------------------------------
        self.Qw = 1.0
        self.Qx = 0.0
        self.Qy = 0.0
        self.Qz = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0

    # -------------------------------------------
    # Timer Callback
    # -------------------------------------------
    def check_initial_subscriptions(self):
        waiting_topics = []
        if not self.received_sub1:
            waiting_topics.append(f"'{self.subscriber_topic}'")
        if not self.received_sub2:
            waiting_topics.append(f"'{self.subscriber_topic2}'")

        if waiting_topics:
            self.get_logger().info(f"Waiting for messages on topics: {', '.join(waiting_topics)}")
        else:
            self.get_logger().info(
                f"All subscribed topics have received at least one message. "
                f"Publishers on '{self.publisher_topic}' and '{self.publisher_topic2}', "
                f"subscribers on '{self.subscriber_topic}' and '{self.subscriber_topic2}', "
                f"frame_id '{self.frame_id}'."
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

    def listener_callback2(self, msg: PoseStamped):
        if not self.received_sub2:
            self.received_sub2 = True

        self.get_logger().info(
            f"(Subscriber 2) Received message with frame_id: '{msg.header.frame_id}', timestamp: {msg.header.stamp}"
        )

        processed_msg = PoseStamped()
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.header.frame_id = self.frame_id
        processed_msg.pose = msg.pose

        self.publisher2_.publish(processed_msg)

        self.get_logger().info(
            f"(Publisher 2) Published processed message with frame_id: '{processed_msg.header.frame_id}', "
            f"timestamp: {processed_msg.header.stamp}"
        )

    # -------------------------------------------
    # Optional Service Callback
    # -------------------------------------------
    def service_callback(self, request, response):
        self.get_logger().info("Service call received.")
        response.success = True
        response.message = "Service call processed successfully."
        return response

# -----------------------------------
# Main Entry Point
# -----------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GeneralTaskNode()
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
