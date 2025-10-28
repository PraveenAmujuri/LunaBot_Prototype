#!/usr/bin/env python3

"""
Battery Simulator - For testing emergency return
Simulates gradual battery drain
"""

import rospy
from std_msgs.msg import Float32


def main():
    rospy.init_node('battery_simulator')
    
    battery_pub = rospy.Publisher('/battery_level', Float32, queue_size=1)
    rate = rospy.Rate(0.1)  # Update every 10 seconds
    
    battery_level = 100.0
    drain_rate = 0.5  # % per update (adjust for testing)
    
    rospy.loginfo("🔋 Battery Simulator Started")
    rospy.loginfo(f"   Starting: {battery_level}%")
    rospy.loginfo(f"   Drain rate: {drain_rate}% per 10s")
    
    while not rospy.is_shutdown():
        battery_level -= drain_rate
        battery_level = max(0.0, battery_level)  # Don't go below 0
        
        msg = Float32()
        msg.data = battery_level
        battery_pub.publish(msg)
        
        if battery_level < 10:
            rospy.logwarn_throttle(10, f"🔋 LOW BATTERY: {battery_level:.1f}%")
        elif battery_level > 0:
            rospy.loginfo_throttle(30, f"🔋 Battery: {battery_level:.0f}%")
        
        if battery_level <= 0:
            rospy.logwarn("🔋 BATTERY DEPLETED!")
            break
        
        rate.sleep()


if __name__ == '__main__':
    main()
