#!/usr/bin/env python3

"""
SIMPLE Wall-Following - TRULY FIXED
- Only resets countdown if direction actually changes
- Keeps 60° FOV (don't change Unity!)
"""

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import tf

class SimpleWallFollower:
    def __init__(self):
        rospy.init_node("simple_wall_follower")
        
        self.SAFE_DIST = 19.0
        self.FORWARD_SPEED = 0.3
        self.TURN_SPEED = 0.4
        self.GOAL_TOLERANCE = 2.0
        
        self.current_goal = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.front = 999
        self.left = 999
        self.right = 999
        
        self.current_turn_direction = None
        self.turn_loops_remaining = 0
        self.MIN_TURN_LOOPS = 10
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.loginfo("🚀 Wall Follower Started (TRUE PERSISTENCE FIX)!")
        rospy.loginfo("   60° FOV - DO NOT change to 360°!")
    
    def goal_callback(self, msg):
        self.current_goal = msg.pose
        self.current_turn_direction = None
        self.turn_loops_remaining = 0
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"🎯 NEW GOAL: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})")
        rospy.loginfo("=" * 60)
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.robot_yaw = yaw
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return
        
        region_size = n // 3
        def get_min(rays):
            valid = [r for r in rays if r < 100 and r != float('inf')]
            return min(valid) if valid else 999
        
        self.left = get_min(ranges[0:region_size])
        self.front = get_min(ranges[region_size:2*region_size])
        self.right = get_min(ranges[2*region_size:n])
    
    def get_distance_to_goal(self):
        if not self.current_goal:
            return float('inf')
        dx = self.current_goal.position.x - self.robot_x
        dy = self.current_goal.position.y - self.robot_y
        return math.sqrt(dx**2 + dy**2)
    
    def get_angle_to_goal(self):
        if not self.current_goal:
            return 0.0
        dx = self.current_goal.position.x - self.robot_x
        dy = self.current_goal.position.y - self.robot_y
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - self.robot_yaw
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def run(self):
        rate = rospy.Rate(10)
        vel = Twist()
        
        while not rospy.is_shutdown():
            if not self.current_goal:
                rospy.loginfo_throttle(5, "⏸️  Waiting for goal...")
                vel.linear.x = 0
                vel.angular.z = 0
                self.cmd_pub.publish(vel)
                rate.sleep()
                continue
            
            dist = self.get_distance_to_goal()
            if dist < self.GOAL_TOLERANCE:
                rospy.loginfo("=" * 60)
                rospy.loginfo("🎉 GOAL REACHED!")
                rospy.loginfo("=" * 60)
                self.current_goal = None
                self.current_turn_direction = None
                self.turn_loops_remaining = 0
                vel.linear.x = 0
                vel.angular.z = 0
                self.cmd_pub.publish(vel)
                rate.sleep()
                continue
            
            angle = self.get_angle_to_goal()
            
            # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
            # WALL-FOLLOWING WITH TRUE PERSISTENCE
            # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
            
            if self.front > self.SAFE_DIST:
                # Clear - reset turn state
                self.current_turn_direction = None
                self.turn_loops_remaining = 0
                
                vel.linear.x = self.FORWARD_SPEED
                if abs(angle) > 0.3:
                    vel.angular.z = self.TURN_SPEED if angle > 0 else -self.TURN_SPEED
                else:
                    vel.angular.z = 0
                
                rospy.loginfo_throttle(2, f"✅ Clear! Forward ({dist:.1f}m)")
            
            else:
                # Blocked - wall-follow
                vel.linear.x = 0.15
                
                # FIXED: Only countdown, don't re-decide until countdown expires
                if self.turn_loops_remaining > 0:
                    # Continue current turn
                    self.turn_loops_remaining -= 1
                    
                    if self.current_turn_direction == "LEFT":
                        vel.angular.z = self.TURN_SPEED
                    else:
                        vel.angular.z = -self.TURN_SPEED
                    
                    if self.turn_loops_remaining % 5 == 0:  # Log every 0.5s
                        rospy.loginfo(f"   {self.current_turn_direction} " +
                                      f"({self.turn_loops_remaining} loops left)")
                
                else:
                    # Countdown expired - decide direction
                    if self.left > self.right + 3.0:
                        new_direction = "LEFT"
                    elif self.right > self.left + 3.0:
                        new_direction = "RIGHT"
                    else:
                        # Keep current or default to LEFT
                        new_direction = self.current_turn_direction if self.current_turn_direction else "LEFT"
                    
                    # CRITICAL FIX: Only reset countdown if direction CHANGED
                    if new_direction != self.current_turn_direction:
                        self.current_turn_direction = new_direction
                        self.turn_loops_remaining = self.MIN_TURN_LOOPS
                        rospy.logwarn(f"⚠️  {new_direction} turn START " +
                                      f"(F:{self.front:.1f} L:{self.left:.1f} R:{self.right:.1f})")
                    else:
                        # Same direction - just reset countdown, don't log
                        self.turn_loops_remaining = self.MIN_TURN_LOOPS
                    
                    # Execute
                    if self.current_turn_direction == "LEFT":
                        vel.angular.z = self.TURN_SPEED
                    else:
                        vel.angular.z = -self.TURN_SPEED
            
            self.cmd_pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = SimpleWallFollower()
        follower.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Stopped")
