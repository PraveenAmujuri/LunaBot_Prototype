#!/usr/bin/env python3

"""
Main Navigation Node for Large Rover
Based on: https://github.com/Rad-hi/Obstacle-Avoidance-ROS
"""

from avoider_large_rover import LargeRoverAvoider

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def main():
    # Initialize velocity message
    vel = Twist()
    
    # Create avoider object
    avoider = LargeRoverAvoider(vel)
    
    # Initialize ROS node
    rospy.init_node("Large_Rover_Obstacle_Avoid")
    
    # Subscribe to laser scan
    rospy.Subscriber("/scan", LaserScan, avoider.identify_regions)
    
    # Publisher for movement commands
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    # Control rate - 10Hz (from GitHub article)
    rate = rospy.Rate(10)
    
    rospy.loginfo("🚀 Large Rover Obstacle Avoidance ACTIVE!")
    rospy.loginfo("   Listening to /scan")
    rospy.loginfo("   Publishing to /cmd_vel")
    
    # Main control loop
    while not rospy.is_shutdown():
        vel = avoider.avoid()
        pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Large Rover Avoider stopped")
