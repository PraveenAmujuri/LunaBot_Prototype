#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def odom_callback(msg):
    # Create TF broadcaster
    br = tf2_ros.TransformBroadcaster()
    
    # Create transform message
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    
    # Copy position from odometry
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0  # Keep ground plane at z=0
    
    # Copy orientation from odometry
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    
    # Broadcast transform
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    rospy.loginfo("Starting odom to TF broadcaster...")
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()
