#!/usr/bin/env python3

"""
SIMPLE REACTIVE NAVIGATION - One file, no complexity!
Works perfectly with 60° FOV laser for obstacle avoidance.
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class SimpleReactiveNav:
    def __init__(self):
        rospy.init_node('simple_reactive_nav')
        
        # Publishers & Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Parameters (TUNABLE!)
        self.max_speed = 0.3          # Max forward speed
        self.turn_speed = 0.6         # Turn speed
        self.safe_distance = 3.5      # React at 3.5m
        self.critical_distance = 2.0  # Emergency at 2.0m
        
        # State
        self.latest_scan = None
        
        # Control timer (5 Hz)
        rospy.Timer(rospy.Duration(0.2), self.control_loop)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("🚀 SIMPLE REACTIVE NAVIGATION STARTED!")
        rospy.loginfo(f"   Max speed: {self.max_speed} m/s")
        rospy.loginfo(f"   Safe distance: {self.safe_distance}m")
        rospy.loginfo(f"   Critical distance: {self.critical_distance}m")
        rospy.loginfo("=" * 60)


    def scan_callback(self, msg):
        """Store latest scan"""
        self.latest_scan = msg


    def control_loop(self, event):
        """Main control logic - runs at 5Hz"""
        
        if self.latest_scan is None:
            rospy.logwarn_throttle(5, "⏳ Waiting for laser scan...")
            return
        
        # Get scan ranges
        ranges = np.array(self.latest_scan.ranges)
        
        # Clean up invalid values
        ranges = np.nan_to_num(ranges, 
                               nan=self.latest_scan.range_max,
                               posinf=self.latest_scan.range_max,
                               neginf=0.0)
        
        # Split into regions (front, left, right)
        num_rays = len(ranges)
        third = num_rays // 3
        
        front_ranges = ranges[third:2*third]      # Center 1/3
        left_ranges = ranges[2*third:]            # Left 1/3
        right_ranges = ranges[:third]             # Right 1/3
        
        # Calculate min distances in each region
        front_min = np.min(front_ranges) if len(front_ranges) > 0 else 999
        left_min = np.min(left_ranges) if len(left_ranges) > 0 else 999
        right_min = np.min(right_ranges) if len(right_ranges) > 0 else 999
        
        front_avg = np.mean(front_ranges) if len(front_ranges) > 0 else 999
        
        # CREATE COMMAND
        cmd = Twist()
        
        # ============================================================
        # DECISION LOGIC - SIMPLE AND RELIABLE
        # ============================================================
        
        # CASE 1: EMERGENCY STOP - Too close!
        if front_min < self.critical_distance:
            rospy.logwarn_throttle(1, f"🛑 EMERGENCY! Front: {front_min:.2f}m")
            cmd.linear.x = -0.1    # Back up slowly
            cmd.angular.z = self.turn_speed if left_min > right_min else -self.turn_speed
        
        # CASE 2: OBSTACLE DETECTED - Avoid
        elif front_min < self.safe_distance:
            rospy.loginfo_throttle(1, f"⚠️  AVOIDING! Front: {front_min:.2f}m L:{left_min:.2f}m R:{right_min:.2f}m")
            
            cmd.linear.x = 0.15    # Slow forward
            
            # Turn toward clearer side
            if left_min > right_min:
                cmd.angular.z = self.turn_speed      # Turn LEFT
                rospy.loginfo_throttle(2, "   ↶ Turning LEFT")
            else:
                cmd.angular.z = -self.turn_speed     # Turn RIGHT
                rospy.loginfo_throttle(2, "   ↷ Turning RIGHT")
        
        # CASE 3: CLEAR PATH - Go forward!
        else:
            rospy.loginfo_throttle(5, f"✅ CLEAR! Front: {front_avg:.2f}m - Moving forward")
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0
        
        # PUBLISH COMMAND
        self.cmd_pub.publish(cmd)


def main():
    try:
        nav = SimpleReactiveNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Simple Reactive Navigation stopped")


if __name__ == '__main__':
    main()
