#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import json

class SimpleBehaviorManager:
    def __init__(self):
        rospy.init_node('simple_behavior_manager')
        
        # Publishers
        self.behavior_pub = rospy.Publisher('/robot_behavior', String, queue_size=10)
        self.status_pub = rospy.Publisher('/behavior_status', String, queue_size=10)
        self.early_avoid_pub = rospy.Publisher('/early_avoidance_trigger', String, queue_size=10)  # NEW
        
        # Subscribers
        rospy.Subscriber('/yolo_detections', String, self.yolo_callback)
        rospy.Subscriber('/obstacle_regions', String, self.lidar_callback)
        
        # Behavior rules
        self.behavior_rules = {
            'rocks': {'action': 'avoid', 'priority': 3, 'description': 'Large rocks - must avoid'},
            'objects': {'action': 'avoid', 'priority': 3, 'description': 'Unknown objects - avoid for safety'},
            'shadows': {'action': 'caution', 'priority': 1, 'description': 'Shadow detected - proceed with caution'},
            'space capsule': {'action': 'avoid', 'priority': 4, 'description': 'Space capsule - critical obstacle'},
            'clear path': {'action': 'follow', 'priority': 0, 'description': 'Safe path - continue forward'},
            'austroads': {'action': 'follow', 'priority': 0, 'description': 'Road/path - safe to traverse'},
            'flag': {'action': 'record', 'priority': 2, 'description': 'Flag marker - record position'},
            'antenna': {'action': 'record', 'priority': 2, 'description': 'Antenna detected - record as landmark'},
        }
        
        self.current_behavior = 'idle'
        self.detected_objects = []
        self.immediate_obstacle = False
        
        rospy.loginfo("🧠 Behavior Manager: EARLY DETECTION MODE")
        rospy.Timer(rospy.Duration(1.0), self.publish_status)
    
    def yolo_callback(self, msg):
        """Process YOLOv8 detections - TRIGGER EARLY AVOIDANCE"""
        try:
            detections = json.loads(msg.data)
            self.detected_objects = detections
            
            # Check for obstacles in camera view
            obstacles_detected = []
            for detection in detections:
                obj_class = detection.get('class', 'unknown').lower()
                confidence = detection.get('confidence', 0.0)
                
                if confidence > 0.5 and obj_class in self.behavior_rules:
                    if self.behavior_rules[obj_class]['action'] == 'avoid':
                        obstacles_detected.append(detection)
            
            # TRIGGER EARLY AVOIDANCE based on camera detection
            if obstacles_detected:
                # Determine which side obstacle is on
                avg_x = sum(d.get('x', 320) for d in obstacles_detected) / len(obstacles_detected)
                
                # Image center is typically 320 (for 640x480)
                # If obstacle on left side of image, turn right, and vice versa
                if avg_x < 280:  # Obstacle on LEFT side of image
                    direction = "right"  # Turn RIGHT to avoid
                elif avg_x > 360:  # Obstacle on RIGHT side of image
                    direction = "left"   # Turn LEFT to avoid
                else:  # Obstacle in CENTER
                    # Choose based on which side has more space (from LIDAR)
                    direction = "left"  # Default
                
                # Publish early avoidance trigger
                avoid_msg = String()
                avoid_msg.data = json.dumps({
                    'trigger': 'camera',
                    'direction': direction,
                    'obstacle_count': len(obstacles_detected),
                    'obstacle_types': [d.get('class') for d in obstacles_detected]
                })
                self.early_avoid_pub.publish(avoid_msg)
                
                rospy.logwarn(f"📹 CAMERA: Obstacle ahead! Triggering EARLY avoidance {direction.upper()}")
            
            # Determine behavior
            behavior = self.determine_behavior(detections)
            
            if behavior != self.current_behavior:
                self.current_behavior = behavior
                detected_classes = list(set([d.get('class', 'unknown') for d in detections]))
                rospy.loginfo(f"🎭 Behavior: {behavior} | Detected: {detected_classes}")
                
                behavior_msg = String()
                behavior_msg.data = behavior
                self.behavior_pub.publish(behavior_msg)
            
        except Exception as e:
            rospy.logwarn_throttle(5, f"YOLO callback error: {e}")
    
    def lidar_callback(self, msg):
        """Monitor LIDAR for immediate obstacles"""
        try:
            regions = json.loads(msg.data)
            self.immediate_obstacle = (
                regions.get('front', 10) < 2.0 or
                regions.get('fleft', 10) < 2.0 or
                regions.get('fright', 10) < 2.0
            )
            
            if self.immediate_obstacle:
                rospy.logwarn_throttle(2, "⚠️  LIDAR: Immediate obstacle!")
            
        except Exception as e:
            pass
    
    def determine_behavior(self, detections):
        if self.immediate_obstacle:
            return 'avoid'
        
        if not detections:
            return 'navigate'
        
        highest_priority = -1
        selected_behavior = 'navigate'
        detected_class = None
        
        for detection in detections:
            obj_class = detection.get('class', 'unknown').lower()
            confidence = detection.get('confidence', 0.0)
            
            if confidence < 0.5:
                continue
            
            if obj_class in self.behavior_rules:
                rule = self.behavior_rules[obj_class]
                priority = rule['priority']
                
                if priority > highest_priority:
                    highest_priority = priority
                    selected_behavior = rule['action']
                    detected_class = obj_class
        
        if highest_priority >= 2:
            rospy.loginfo_throttle(3, f"🎯 High priority: {detected_class} → {selected_behavior}")
        
        return selected_behavior
    
    def publish_status(self, event):
        object_counts = {}
        for det in self.detected_objects:
            cls = det.get('class', 'unknown')
            object_counts[cls] = object_counts.get(cls, 0) + 1
        
        status = {
            'current_behavior': self.current_behavior,
            'total_detections': len(self.detected_objects),
            'object_counts': object_counts,
            'immediate_obstacle': self.immediate_obstacle,
            'timestamp': rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = SimpleBehaviorManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
