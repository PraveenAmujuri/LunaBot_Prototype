#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import math
import json

class ArticleBasedNavigation:
    def __init__(self):
        rospy.init_node('article_based_navigation')
        
        # Publishers & Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/obstacle_regions', String, self.laser_callback)
        
        # State variables
        self.odom = None
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_active = False
        self.regions = None
        
        # Navigation mode
        self.mode = 'idle'
        
        # TUNABLE PARAMETERS - INCREASED FOR EARLY DETECTION
        self.FRONT_THRESHOLD = 5.0   # Start avoiding at 5m (was 3.0)
        self.SIDE_THRESHOLD = 4.0    # Start avoiding at 4m (was 2.5)
        
        rospy.loginfo("🤖 Article-Based Navigation - EARLY DETECTION")
        rospy.loginfo(f"   Front threshold: {self.FRONT_THRESHOLD}m")
        rospy.loginfo(f"   Side threshold: {self.SIDE_THRESHOLD}m")
        
        # Control loop at 10Hz
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
    
    def odom_callback(self, msg):
        self.odom = msg
    
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_active = True
        self.mode = 'navigating'
        rospy.loginfo(f"🎯 New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")
    
    def laser_callback(self, msg):
        """Receive laser regions"""
        try:
            self.regions = json.loads(msg.data)
        except:
            pass
    
    def control_loop(self, event):
        """Main control loop"""
        
        if self.regions is None:
            return
        
        # EARLY DETECTION - Check obstacles at safe distance
        obstacle_detected = (
            self.regions['front'] < self.FRONT_THRESHOLD or
            self.regions['fleft'] < self.SIDE_THRESHOLD or
            self.regions['fright'] < self.SIDE_THRESHOLD
        )
        
        if obstacle_detected:
            # PRIORITY: Avoid obstacles
            self.mode = 'avoiding'
            self.take_action_obstacle_avoidance()
        elif self.goal_active:
            # Navigate to goal
            self.mode = 'navigating'
            self.navigate_to_goal()
        else:
            # Idle
            self.mode = 'idle'
            self.stop()
    
    def take_action_obstacle_avoidance(self):
        """
        8-case obstacle avoidance with EARLY DETECTION
        """
        
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0
        
        front = self.regions['front']
        fleft = self.regions['fleft']
        fright = self.regions['fright']
        
        # Use our early detection thresholds
        d_front = self.FRONT_THRESHOLD  # 5.0m
        d_side = self.SIDE_THRESHOLD    # 4.0m
        
        # Case 1: All clear - go straight
        if front > d_front and fleft > d_side and fright > d_side:
            state = 'Case 1: Clear - Full speed'
            linear_x = 0.5
            angular_z = 0.0
        
        # Case 2: Front blocked ONLY - turn left
        elif front < d_front and fleft > d_side and fright > d_side:
            state = 'Case 2: Front blocked - Turn LEFT'
            linear_x = 0.1  # Slow down
            angular_z = 0.6  # Turn left
        
        # Case 3: Front-right blocked - turn left
        elif front > d_front and fleft > d_side and fright < d_side:
            state = 'Case 3: Right blocked - Gentle LEFT'
            linear_x = 0.3
            angular_z = 0.5
        
        # Case 4: Front-left blocked - turn right
        elif front > d_front and fleft < d_side and fright > d_side:
            state = 'Case 4: Left blocked - Gentle RIGHT'
            linear_x = 0.3
            angular_z = -0.5
        
        # Case 5: Front + right blocked - turn left strong
        elif front < d_front and fleft > d_side and fright < d_side:
            state = 'Case 5: Front+Right - Strong LEFT'
            linear_x = 0.05
            angular_z = 0.7
        
        # Case 6: Front + left blocked - turn right strong
        elif front < d_front and fleft < d_side and fright > d_side:
            state = 'Case 6: Front+Left - Strong RIGHT'
            linear_x = 0.05
            angular_z = -0.7
        
        # Case 7: ALL blocked - turn left very strong
        elif front < d_front and fleft < d_side and fright < d_side:
            state = 'Case 7: ALL BLOCKED - Emergency LEFT'
            linear_x = 0.0  # STOP forward
            angular_z = 0.8  # Strong turn
        
        # Case 8: Both sides blocked, front clear - turn left
        elif front > d_front and fleft < d_side and fright < d_side:
            state = 'Case 8: Sides blocked - Turn LEFT'
            linear_x = 0.2
            angular_z = 0.6
        
        else:
            state = 'Unknown - STOP'
            linear_x = 0.0
            angular_z = 0.0
        
        rospy.loginfo_throttle(0.5, f"⚠️  {state} | F:{front:.1f} FL:{fleft:.1f} FR:{fright:.1f}")
        
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)
    
    def navigate_to_goal(self):
        """Navigate towards goal - only when no obstacles"""
        
        if self.odom is None:
            return
        
        # Get current position
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        
        # Calculate distance
        dx = self.goal_x - px
        dy = self.goal_y - py
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Goal reached check
        if distance < 0.5:
            rospy.loginfo("✅ Goal reached!")
            self.goal_active = False
            self.stop()
            return
        
        # Calculate angle to goal
        q = self.odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        target_yaw = math.atan2(dy, dx)
        error = target_yaw - yaw
        
        # Normalize angle
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        # Control
        msg = Twist()
        
        # Progressive speed control
        if abs(error) > 0.8:
            # Large angle error - turn in place
            msg.linear.x = 0.1
            msg.angular.z = 0.7 * (1.0 if error > 0 else -1.0)
        elif abs(error) > 0.3:
            # Medium angle error - slow forward + turn
            msg.linear.x = 0.25
            msg.angular.z = 0.5 * error
        else:
            # Small angle error - fast forward
            msg.linear.x = min(0.5, distance * 0.5)
            msg.angular.z = 0.3 * error
        
        self.cmd_pub.publish(msg)
    
    def stop(self):
        """Stop the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

if __name__ == '__main__':
    try:
        nav = ArticleBasedNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
