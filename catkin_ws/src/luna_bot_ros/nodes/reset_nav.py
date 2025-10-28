#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def reset_navigation():
    """Reset navigation by sending stop command"""
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(0.5)  # Wait for publisher to connect
    
    # Send stop command
    stop_cmd = Twist()
    cmd_pub.publish(stop_cmd)
    
    print("🛑 Navigation reset - rover stopped")

if __name__ == '__main__':
    rospy.init_node('reset_nav', anonymous=True)
    reset_navigation()
