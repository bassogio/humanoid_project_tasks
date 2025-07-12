#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import torch
from ultralytics import YOLO

class DynamicYOLONode(Node):
    def __init__(self):
        super().__init__('dynamic_yolo_node')

        # ----------------------------
        # ROS2 parameters (with defaults)
        # ----------------------------
        self.declare_parameter('publisher_raw_topic', '/camera/image_raw')
        self.declare_parameter('publisher_detected_topic', '/camera/image_detected')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('target_class', 'person')

        self.raw_topic      = self.get_parameter('publisher_raw_topic').value
        self.detected_topic = self.get_parameter('publisher_detected_topic').value
        self.camera_index   = self.get_parameter('camera_index').value
        self.publish_rate   = self.get_parameter('publish_rate').value
        self.target_class   = self.get_parameter('target_class').value.strip().lower()

        self.get_logger().info(f"Starting with target_class='{self.target_class}'")
        self.get_logger().info(f"Raw images → {self.raw_topic}")
        self.get_logger().info(f"Detected images → {self.detected_topic}")

        # ----------------------------
        # Publishers & subscriber
        # ----------------------------
        self.bridge = CvBridge()
        self.raw_pub      = self.create_publisher(Image, self.raw_topic,      10)
        self.detected_pub = self.create_publisher(Image, self.detected_topic, 10)
        self.meta_pub     = self.create_publisher(String, '/camera/detections', 10)

        # listen for dynamic target updates
        self.create_subscription(String, '/target_class', self.target_callback, 10)
        self.get_logger().info("Subscribed to /target_class for updates")

        # ----------------------------
        # Prepare YOLO on GPU if possible
        # ----------------------------
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")
        self.model = YOLO('yolov8n.pt')
        self.model.to(self.device)

        # ----------------------------
        # OpenCV video capture
        # ----------------------------
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            raise RuntimeError("Camera not available.")

        # ----------------------------
        # Timer
        # ----------------------------
        period = 1.0 / float(self.publish_rate)
        self.create_timer(period, self.publish_image)

    def target_callback(self, msg: String):
        new = msg.data.strip().lower()
        if new:
            self.target_class = new
            self.get_logger().info(f"Updated target_class → '{self.target_class}'")

    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame.")
            return

        # Convert BGR→RGB for YOLO
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 1) Publish raw frame
        raw_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        self.raw_pub.publish(raw_msg)

        # 2) Run inference
        results = self.model(
            rgb,
            device=self.device,
            half=self.device.startswith('cuda')
        )

        # 3) Draw boxes & collect metadata
        annotated = frame.copy()
        detections = []
        for box in results[0].boxes:
            cls_id     = int(box.cls[0])
            name       = self.model.names[cls_id]
            conf       = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            if name.lower() == self.target_class:
                # draw
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,255,0), 2)
                label = f"{name} {conf:.2f}"
                cv2.putText(annotated, label, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                detections.append({
                    "class": name,
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2]
                })

        # 4) Publish annotated frame
        ann_rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        det_img_msg = self.bridge.cv2_to_imgmsg(ann_rgb, encoding='rgb8')
        det_img_msg.header.stamp = self.get_clock().now().to_msg()
        self.detected_pub.publish(det_img_msg)

        # 5) Publish JSON metadata
        meta = String()
        meta.data = json.dumps(detections)
        self.meta_pub.publish(meta)
        if detections:
            self.get_logger().info(f"Detected {len(detections)} '{self.target_class}'")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicYOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
