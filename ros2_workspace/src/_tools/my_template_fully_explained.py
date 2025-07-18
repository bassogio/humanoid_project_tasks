# -----------------------------------
# Import Statements
# -----------------------------------
import rclpy  # Core ROS 2 Python client library
from rclpy.node import Node  # Base class for all ROS2 nodes
from std_srvs.srv import Trigger  # A simple built-in service type with empty request
from geometry_msgs.msg import PoseStamped  # Standard message for pose with header

# -----------------------------------
# ROS2 Node Definition
# -----------------------------------
class GeneralTaskNode(Node):
    """
    A generic ROS2 node with multiple publishers/subscribers,
    optional service functionality, and ROS2 parameters.
    """

    def __init__(self):
        # Initialize the Node with the given name.
        super().__init__('general_task_node')

        # -------------------------------------------
        # Declare ROS2 Parameters with default values
        # Format: self.declare_parameter('param_name', default_value)
        # - Left string: parameter name used in ROS2 (for CLI / launch)
        # - Right string/value: default value if user does not override it
        # -------------------------------------------
        self.declare_parameter('publisher_topic',   'output_topic')
        self.declare_parameter('publisher_topic2',  'output_topic2')
        self.declare_parameter('subscriber_topic',  'input_topic')
        self.declare_parameter('subscriber_topic2', 'input_topic2')
        self.declare_parameter('frame_id',          'base_link')
        self.declare_parameter('use_service',       False)
        self.declare_parameter('service_name',      'trigger_service')

        # -------------------------------------------
        # Retrieve Parameter Values from the Parameter Server
        # These reflect the final values used by the node (including CLI/launch overrides)
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
        # self.create_publisher(MessageType, topic_name, queue_size)
        # - MessageType: type of message to publish (here PoseStamped)
        # - topic_name: topic to publish to
        # - queue_size: number of messages to buffer if subscribers are slow
        # -------------------------------------------
        self.publisher_ = self.create_publisher(PoseStamped, self.publisher_topic, 10)
        self.publisher2_ = self.create_publisher(PoseStamped, self.publisher_topic2, 10)

        # -------------------------------------------
        # Initialize Subscribers
        # self.create_subscription(MessageType, topic_name, callback, queue_size)
        # - MessageType: type of message to receive (here PoseStamped)
        # - topic_name: topic to subscribe to
        # - callback: function to call when a message arrives
        # - queue_size: buffer size if messages come too fast
        # -------------------------------------------
        self.subscription = self.create_subscription(
            PoseStamped,              # MessageType: expecting PoseStamped messages
            self.subscriber_topic,    # Topic name: which topic to listen on
            self.listener_callback,   # Callback: function to handle the message
            10                        # Queue size: how many messages to buffer
        )

        self.subscription2 = self.create_subscription(
            PoseStamped,              # MessageType: PoseStamped
            self.subscriber_topic2,   # Topic: second input topic
            self.listener_callback2,  # Callback: second subscriber handler
            10                        # Queue size
        )

        # -------------------------------------------
        # Flags to track if we've received a message yet from each subscriber
        # Used to know when the node is "ready"
        # -------------------------------------------
        self.received_sub1 = False
        self.received_sub2 = False

        # -------------------------------------------
        # Timer to periodically check if both subscribers have received at least one message
        # self.create_timer(period_sec, callback)
        # - Fires every 2.0 seconds
        # - Calls check_initial_subscriptions
        # -------------------------------------------
        self.subscription_check_timer = self.create_timer(2.0, self.check_initial_subscriptions)

        # -------------------------------------------
        # OPTIONAL: Service server
        # Only created if 'use_service' parameter is True
        # self.create_service(ServiceType, service_name, callback)
        # - ServiceType: Trigger (built-in empty request/response)
        # - service_name: topic for the service
        # - callback: function to handle service requests
        # -------------------------------------------
        if self.use_service:
            self.service_server = self.create_service(
                Trigger,               # ServiceType
                self.service_name,     # Service name (string)
                self.service_callback  # Callback function
            )
            self.get_logger().info(f"Service server started on '{self.service_name}'")

        # -------------------------------------------
        # Pose Defaults (not used in this version)
        # Could be used to store orientation and position values internally
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
    # This function runs every 2 seconds (set by the timer)
    # Checks if both subscribers have received at least one message
    # Logs which topics are still waiting
    # Cancels the timer once all data has arrived
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
            # Stop the timer since we've confirmed both have received data
            self.subscription_check_timer.cancel()

    # -------------------------------------------
    # Subscriber Callback for the first topic
    # Called automatically whenever a PoseStamped message arrives on subscriber_topic
    # Steps:
    # 1. Log receipt of the message
    # 2. Mark that we've received data on this topic
    # 3. Create a new PoseStamped with updated header
    # 4. Publish it on publisher_topic
    # -------------------------------------------
    def listener_callback(self, msg: PoseStamped):
        if not self.received_sub1:
            self.received_sub1 = True

        # Log the incoming message header
        self.get_logger().info(
            f"(Subscriber 1) Received message with frame_id: '{msg.header.frame_id}', timestamp: {msg.header.stamp}"
        )

        # Create a new message with updated timestamp and frame_id
        processed_msg = PoseStamped()
        processed_msg.header.stamp = self.get_clock().now().to_msg()  # Set to current time
        processed_msg.header.frame_id = self.frame_id                 # Use configured frame ID
        processed_msg.pose = msg.pose                                 # Copy pose data as-is

        # Publish the updated message
        self.publisher_.publish(processed_msg)

        self.get_logger().info(
            f"(Publisher 1) Published processed message with frame_id: '{processed_msg.header.frame_id}', "
            f"timestamp: {processed_msg.header.stamp}"
        )

    # -------------------------------------------
    # Subscriber Callback for the second topic
    # Same as listener_callback, but for subscriber_topic2 and publisher_topic2
    # -------------------------------------------
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
    # Service Callback
    # Called when someone calls the service
    # - Logs the service call
    # - Returns success=True with a message
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
    rclpy.init(args=args)  # Initialize ROS2 system
    node = GeneralTaskNode()  # Create an instance of your node
    try:
        rclpy.spin(node)  # Keep the node alive processing callbacks
    except KeyboardInterrupt:
        pass  # Graceful shutdown on Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()  # Cleanup ROS2 resources

# -----------------------------------
# Run the node when executed directly
# -----------------------------------
if __name__ == '__main__':
    main()
