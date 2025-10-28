#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json

class LidarObstacleRegions:
    def __init__(self):
        rospy.init_node('lidar_obstacle_regions')
        
        # Publisher for obstacle regions
        self.region_pub = rospy.Publisher('/obstacle_regions', String, queue_size=10)
        
        # Subscribe to LIDAR
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Detection thresholds
        self.max_detection_distance = 15.0  # Your laser max range
        self.safe_distance = 10.0           # Treat anything beyond 10m as "clear"
        
        rospy.loginfo("🎯 LIDAR Obstacle Regions (120° FOV, 15m range)")
        rospy.loginfo(f"   Safe distance threshold: {self.safe_distance}m")
    
    def laser_callback(self, msg):
        """
        Process laser scan with 120° FOV
        Divide into 5 regions like the article
        """
        num_readings = len(msg.ranges)
        
        if num_readings == 0:
            return
        
        # Divide 120° FOV into 5 equal regions (24° each)
        # Your laser: angle_min = -60°, angle_max = +60°
        fifth = num_readings // 5
        
        # Define regions (from right to left in laser frame)
        regions = {
            'right':  min(min(msg.ranges[0:fifth]), self.safe_distance),
            'fright': min(min(msg.ranges[fifth:2*fifth]), self.safe_distance),
            'front':  min(min(msg.ranges[2*fifth:3*fifth]), self.safe_distance),
            'fleft':  min(min(msg.ranges[3*fifth:4*fifth]), self.safe_distance),
            'left':   min(min(msg.ranges[4*fifth:num_readings]), self.safe_distance),
        }
        
        # Publish regions as JSON
        region_msg = String()
        region_msg.data = json.dumps(regions)
        self.region_pub.publish(region_msg)
        
        # Log obstacles (only front 3 regions matter for avoidance)
        if (regions['front'] < 4.0 or 
            regions['fleft'] < 3.0 or 
            regions['fright'] < 3.0):
            rospy.loginfo_throttle(2, 
                f"⚠️  Obstacles: F={regions['front']:.1f}m "
                f"FL={regions['fleft']:.1f}m FR={regions['fright']:.1f}m")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = LidarObstacleRegions()
        detector.run()
    except rospy.ROSInterruptException:
        pass
