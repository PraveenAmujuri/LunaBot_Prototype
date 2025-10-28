#!/usr/bin/env python3
"""
YOLOv8 Lunar Detector - Custom model for lunar terrain analysis.
Detects safe zones, obstacles, and caution areas from camera input.
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
from inference_sdk import InferenceHTTPClient


class YoloLunarDetector:
    def __init__(self):
        rospy.loginfo("YOLOv8 Lunar Detector initializing...")

        # Load model configuration
        cache_dir = os.path.expanduser("~/.luna_bot_models")
        config_path = os.path.join(cache_dir, "model_config.json")

        if not os.path.exists(config_path):
            rospy.logerr("Model config JSON not found. Run LunaBotModelSetup first.")
            self.yolo_available = False
            return

        with open(config_path, "r") as f:
            config = json.load(f)

        self.model_id = config.get("model_id")
        self.api_key = config.get("api_key")

        if not self.model_id or not self.api_key:
            rospy.logerr("Model ID or API key missing in config JSON.")
            self.yolo_available = False
            return

        # Initialize YOLO inference client
        self.yolo_client = InferenceHTTPClient(
            api_url="https://detect.roboflow.com",
            api_key=self.api_key
        )

        # Detection class categories
        self.safe_classes = {"clear path", "austroads", "flag", "antenna"}
        self.obstacle_classes = {"rocks", "objects", "space capsule"}
        self.caution_classes = {"shadows"}

        self.bridge = CvBridge()
        self.yolo_available = True

        # Publisher for detections
        self.detection_pub = rospy.Publisher("/yolo_detections", String, queue_size=10)

        # Subscribe to camera image topics
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

        # Image buffer and threading lock
        self.current_image = None
        self.image_lock = threading.Lock()

        # Background detection thread
        self.detection_thread = threading.Thread(
            target=self.detection_loop,
            daemon=True
        )
        self.detection_thread.start()

        rospy.loginfo("YOLOv8 Lunar Detector ready.")
        rospy.loginfo(f"Safe classes: {sorted(self.safe_classes)}")
        rospy.loginfo(f"Obstacle classes: {sorted(self.obstacle_classes)}")
        rospy.loginfo(f"Caution classes: {sorted(self.caution_classes)}")

    # ------------------------------------------------------------------
    # Image Handlers
    # ------------------------------------------------------------------
    def image_callback(self, msg):
        """Handle uncompressed image messages."""
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

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------
    def detection_loop(self):
        """Run inference in a background loop."""
        rate = rospy.Rate(2)  # 2 Hz
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
        """Perform YOLO inference using the configured model."""
        try:
            tmp_path = "/tmp/temp_roboflow_infer.jpg"
            cv2.imwrite(tmp_path, img)
            result = self.yolo_client.infer(tmp_path, model_id=self.model_id)
            os.remove(tmp_path)

            detections = []

            if "predictions" in result:
                for pred in result["predictions"]:
                    cls = pred.get("class", "unknown").lower()
                    conf = float(pred.get("confidence", 0.0))
                    x = int(pred.get("x", 0))
                    y = int(pred.get("y", 0))
                    w = int(pred.get("width", 0))
                    h = int(pred.get("height", 0))

                    detection = {
                        "class": cls,
                        "confidence": conf,
                        "x": x,
                        "y": y,
                        "width": w,
                        "height": h,
                        "is_obstacle": cls in self.obstacle_classes,
                        "is_safe": cls in self.safe_classes,
                        "is_caution": cls in self.caution_classes
                    }
                    detections.append(detection)

                # Summary logs
                obstacles = [d["class"] for d in detections if d["is_obstacle"]]
                safe_zones = [d["class"] for d in detections if d["is_safe"]]

                if obstacles:
                    rospy.loginfo_throttle(3, f"Obstacles detected: {obstacles}")
                elif safe_zones:
                    rospy.loginfo_throttle(5, f"Safe areas detected: {safe_zones}")
                else:
                    rospy.loginfo_throttle(5, f"Detections: {[d['class'] for d in detections]}")

            return detections

        except Exception as e:
            rospy.logerr(f"Detection error: {e}")
            return None


def main():
    rospy.init_node("yolo_lunar_detector")
    detector = YoloLunarDetector()

    if detector.yolo_available:
        rospy.spin()


if __name__ == "__main__":
    main()
