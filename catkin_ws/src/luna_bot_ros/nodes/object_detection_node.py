#!/usr/bin/env python3
"""
YOLOv8 Lunar Detector (Local GPU)
Runs YOLO inference using Ultralytics on local GPU and publishes detections.
"""

import os
import cv2
import json
import rospy
import numpy as np
import threading
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloLunarDetector:
    """Subscribes to camera topics, runs local YOLO inference, publishes detections."""

    def __init__(self):
        rospy.init_node("yolo_lunar_detector")
        rospy.loginfo("YOLOv8 Local Lunar Detector starting")

        # Load local model (best.pt)
        model_path = os.path.join(os.path.dirname(__file__), "best.pt")
        
        if not os.path.exists(model_path):
            rospy.logerr(f"Model not found at {model_path}. Please place your custom .pt file there.")
            self.yolo_available = False
            return
            
        try:
            # Load model onto GPU (if available) automatically
            self.model = YOLO(model_path)
            rospy.loginfo(f"Custom Model loaded: {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            self.yolo_available = False
            return

        # Define class categories
        self.safe_classes = {"clear path", "flag", "antenna"}
        self.obstacle_classes = {"rocks", "buildings", "space capsule"}
        self.caution_classes = {"shadows"}

        self.bridge = CvBridge()
        self.yolo_available = True

        # Publishers
        self.detection_pub = rospy.Publisher("/yolo_detections", String, queue_size=10)

        # Subscribers
        image_topics = [
            "/camera/image_raw",
            "/usb_cam/image_raw",
            "/rover_camera/image_raw",
            "/webcam/image_raw"
        ]
        compressed_topics = [
            "/camera/image_raw/compressed",
            "/usb_cam/image_raw/compressed",
            "/rover_camera/image_raw/compressed",
            "/webcam/image_raw/compressed"
        ]

        for topic in image_topics:
            rospy.Subscriber(topic, Image, self.image_callback)
        for topic in compressed_topics:
            rospy.Subscriber(topic, CompressedImage, self.compressed_callback)

        self.current_image = None
        self.image_lock = threading.Lock()

        # Background detection thread
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()

        rospy.loginfo("YOLOv8 Lunar Detector ready")
        rospy.loginfo(f"Safe classes: {sorted(self.safe_classes)}")
        rospy.loginfo(f"Obstacle classes: {sorted(self.obstacle_classes)}")

    def image_callback(self, msg):
        """Handle uncompressed Image messages."""
        try:
            if "jpeg" in msg.encoding.lower() or "jpg" in msg.encoding.lower():
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if msg.encoding.lower() == "rgb8":
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            if cv_image is not None:
                with self.image_lock:
                    self.current_image = cv_image.copy()
        except Exception as e:
            rospy.logwarn_throttle(5, f"Image callback error: {e}")

    def compressed_callback(self, msg):
        """Handle compressed image messages."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                with self.image_lock:
                    self.current_image = cv_image.copy()
        except Exception as e:
            rospy.logwarn_throttle(5, f"Compressed image callback error: {e}")

    def detection_loop(self):
        """Run inference at a steady rate and publish detections."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.image_lock:
                img = self.current_image.copy() if self.current_image is not None else None

            if img is None:
                rate.sleep()
                continue

            detections = self.detect_objects(img)
            if detections is not None:
                msg = String()
                msg.data = json.dumps(detections)
                self.detection_pub.publish(msg)

            rate.sleep()

    def detect_objects(self, img):
        """Run inference using Local Ultralytics Model."""
        try:
            # Run inference on GPU
            results = self.model.predict(source=img, conf=0.25, verbose=False, device='0')

            detections = []

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x_c, y_c, w, h = box.xywh[0]
                    
                    cls_id = int(box.cls[0])
                    raw_cls = self.model.names[cls_id]
                    cls_lower = raw_cls.lower()
                    
                    conf = float(box.conf[0])

                    is_obs = cls_lower in self.obstacle_classes
                    is_safe = cls_lower in self.safe_classes
                    is_caution = cls_lower in self.caution_classes

                    detection = {
                        "class": cls_lower,
                        "confidence": round(conf, 2),
                        "x": int(x_c),
                        "y": int(y_c),
                        "width": int(w),
                        "height": int(h),
                        "is_obstacle": is_obs,
                        "is_safe": is_safe,
                        "is_caution": is_caution
                    }
                    detections.append(detection)

            obstacles = [d["class"] for d in detections if d["is_obstacle"]]
            others = [d["class"] for d in detections if not d["is_obstacle"] and not d["is_safe"]]

            if obstacles:
                rospy.loginfo_throttle(3, f"Obstacles: {obstacles}")
            elif others:
                rospy.loginfo_throttle(3, f"Ignored Objects: {others}")

            return detections

        except Exception as e:
            rospy.logerr(f"Detection error: {e}")
            return None

def main():
    detector = YoloLunarDetector()
    if detector.yolo_available:
        rospy.spin()


if __name__ == "__main__":
    main()