#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Try importing ultralytics, handle gracefully if missing
try:
    from ultralytics import YOLO
    import torch
except ImportError:
    YOLO = None
    torch = None

# Custom messages
from lidar_perception.msg import CameraDetection, CameraDetectionArray

class CameraDetectionNode(Node):
    def __init__(self):
        super().__init__('camera_detection_node')
        
        if YOLO is None or torch is None:
            self.get_logger().error("ultralytics or torch package is not installed.")
            raise RuntimeError("Missing dependencies")

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8m.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('inference_size', [640, 640])
        self.declare_parameter('enable_class_filtering', False)
        self.declare_parameter('whitelist_classes', ['person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck'])

        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.conf_thres = self.get_parameter('confidence_threshold').value
        self.iou_thres = self.get_parameter('iou_threshold').value
        self.image_topic = self.get_parameter('image_topic').value
        self.inference_size = tuple(self.get_parameter('inference_size').value)
        self.enable_filtering = self.get_parameter('enable_class_filtering').value
        self.whitelist_classes = self.get_parameter('whitelist_classes').value

        # Initialize YOLO
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Loading YOLO model from: {self.model_path} on device: {self.device}")
        self.model = YOLO(self.model_path)
        
        # Warmup the model
        dummy_input = np.zeros((self.inference_size[1], self.inference_size[0], 3), dtype=np.uint8)
        self.model(dummy_input, imgsz=self.inference_size, device=self.device, half=(self.device == 'cuda'), verbose=False)
        self.get_logger().info("YOLO model loaded and warmed up successfully.")

        # If filtering is enabled, build a set of allowed class IDs
        self.allowed_class_ids = set()
        if self.enable_filtering:
            for cls_id, cls_name in self.model.names.items():
                if cls_name in self.whitelist_classes:
                    self.allowed_class_ids.add(cls_id)
            self.get_logger().info(f"Class filtering enabled. Whitelist: {self.whitelist_classes}")
            self.get_logger().info(f"Allowed class IDs: {self.allowed_class_ids}")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscriptions
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(CameraDetectionArray, '/camera/detections', 10)
        self.image_pub = self.create_publisher(Image, '/camera/detection_image', 10)

        self.get_logger().info("Camera detection node initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Run inference (optimized)
        results = self.model(
            cv_image, 
            imgsz=self.inference_size, 
            conf=self.conf_thres, 
            iou=self.iou_thres, 
            device=self.device,
            half=(self.device == 'cuda'), # Use FP16 if on GPU
            verbose=False
        )
        
        if len(results) == 0:
            return

        result = results[0] # Single image inference
        
        # Prepare detection array message
        det_array_msg = CameraDetectionArray()
        det_array_msg.header = msg.header

        # Draw annotations if image_pub has subscribers
        annotate_image = self.image_pub.get_subscription_count() > 0
        annotated_frame = cv_image.copy() if annotate_image else None

        for box in result.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            
            # Apply class filtering if enabled
            if self.enable_filtering and cls_id not in self.allowed_class_ids:
                continue

            xyxy = box.xyxy[0].cpu().numpy()
            xmin, ymin, xmax, ymax = xyxy
            
            cls_name = self.model.names[cls_id]

            # Populate detection message
            det_msg = CameraDetection()
            det_msg.class_label = cls_name
            det_msg.class_id = cls_id
            det_msg.confidence = conf
            det_msg.xmin = float(xmin)
            det_msg.ymin = float(ymin)
            det_msg.xmax = float(xmax)
            det_msg.ymax = float(ymax)
            
            det_array_msg.detections.append(det_msg)

            # Annotate image
            if annotate_image:
                label = f"{cls_name} {conf:.2f}"
                cv2.rectangle(annotated_frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label, (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections
        self.detection_pub.publish(det_array_msg)

        # Publish annotated image
        if annotate_image:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge Error during publish: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraDetectionNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error initializing node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
