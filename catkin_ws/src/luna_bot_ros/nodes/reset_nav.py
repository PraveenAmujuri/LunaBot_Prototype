#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def reset_navigation():
    """Send a zero-velocity command to stop the rover."""
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(0.5)  # allow publisher to register

    stop_cmd = Twist()  # zero linear and angular velocity
    cmd_pub.publish(stop_cmd)

    print("Navigation reset: rover stopped")

if __name__ == '__main__':
    rospy.init_node('reset_nav', anonymous=True)
    reset_navigation()
