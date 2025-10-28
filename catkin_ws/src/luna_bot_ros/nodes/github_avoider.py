#!/usr/bin/env python3

from Avoider import Avoider

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def main():
    vel = Twist()
    
    avoider = Avoider(vel, 
                  obstacle_threshold=15.0,
                  avoid_distance=10.0,           # ← Start avoiding at 5m!
                  emergency_distance=2.0,       # ← Emergency at 2m!
                  normal_lin_vel=0.3,
                  trans_lin_vel=0.15,
                  trans_ang_vel=0.6)
    
    rospy.init_node("Laser_Obs_Avoid_node")
    rospy.Subscriber("/scan", LaserScan, avoider.indentify_regions)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("🚀 SIMPLE Avoider Started")
    rospy.loginfo("   Avoid at: 5m | Emergency at: 2m")
    rospy.loginfo("=" * 60)
    
    while not rospy.is_shutdown():
        vel = avoider.avoid()
        pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
