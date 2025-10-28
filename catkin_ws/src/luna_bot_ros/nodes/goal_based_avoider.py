#!/usr/bin/env python3
"""
Goal-Based Navigation + AUTONOMOUS PATROL MODE
- Drives to goals when sent
- Patrols autonomously when no goal (random exploration)
- Avoids obstacles in both modes
- FIXED: Now properly travels 30-40m before changing direction
"""

from avoider_large_rover import LargeRoverAvoider

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import tf
import random


def main():
    rospy.init_node("goal_based_avoider")
    
    # Initialize
    vel = Twist()
    avoider = LargeRoverAvoider(vel, obstacle_threshold=19.0)
    
    # Goal tracking
    current_goal = [None]
    robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    goal_tolerance = 2.0
    
    # Post-avoid tracking
    post_avoid_counter = [0]
    POST_AVOID_LOOPS = 20
    
    # ═══════════════════════════════════════════════════════════
    # PATROL MODE
    # ═══════════════════════════════════════════════════════════
    patrol_mode = [True]  # Start in patrol mode
    patrol_direction = [0.0]  # Current patrol direction
    patrol_change_distance = [30]  # FIXED: Start with 30m default
    last_patrol_position = [{'x': 0.0, 'y': 0.0}]
    patrol_stuck_check = [0]  # NEW: Counter to detect if truly stuck
    # ═══════════════════════════════════════════════════════════
    
    # Subscribers
    def goal_callback(msg):
        current_goal[0] = msg.pose
        post_avoid_counter[0] = 0
        patrol_mode[0] = False
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"🎯 NEW GOAL: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")
        rospy.loginfo(f"   Patrol mode: DISABLED (manual goal active)")
        rospy.loginfo("=" * 60)
    
    def odom_callback(msg):
        robot_pose['x'] = msg.pose.pose.position.x
        robot_pose['y'] = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        robot_pose['yaw'] = yaw
    
    rospy.Subscriber("/scan", LaserScan, avoider.identify_regions)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    # Publisher
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)  # 10Hz
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("🎯 GOAL-BASED AVOIDER + AUTONOMOUS PATROL")
    rospy.loginfo("=" * 60)
    rospy.loginfo("✅ Safe distance: 19m")
    rospy.loginfo("✅ Post-avoid forward drive")
    rospy.loginfo("✅ AUTONOMOUS PATROL when no goal!")
    rospy.loginfo("✅ Patrols in random directions")
    rospy.loginfo("✅ Changes direction every 30-40m")  # FIXED: Updated message
    rospy.loginfo("=" * 60)
    rospy.loginfo("🔍 PATROL MODE ACTIVE - Exploring autonomously...")
    
    # Track previous avoiding state
    was_avoiding = False
    
    # Main loop
    while not rospy.is_shutdown():
        goal = current_goal[0]
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # MODE 1: PATROL MODE (NO GOAL)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if goal is None:
            if not patrol_mode[0]:
                # Just entered patrol mode
                patrol_mode[0] = True
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)  # FIXED: 30-40m range
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0  # Reset stuck counter
                rospy.loginfo("=" * 60)
                rospy.loginfo("🔍 PATROL MODE ACTIVATED - Exploring autonomously")
                rospy.loginfo(f"   Initial direction: {math.degrees(patrol_direction[0]):.1f}°")
                rospy.loginfo(f"   Target distance: {patrol_change_distance[0]}m")
                rospy.loginfo("=" * 60)
            
            # Calculate distance traveled since last direction change
            dx = robot_pose['x'] - last_patrol_position[0]['x']
            dy = robot_pose['y'] - last_patrol_position[0]['y']
            traveled = math.sqrt(dx**2 + dy**2)
            
            # FIXED: Only change direction based on distance traveled, not stuck detection
            # Check if reached target distance
            if traveled >= patrol_change_distance[0]:
                # Pick new random direction
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)  # FIXED: Always 30-40m
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0
                rospy.loginfo("✅ Patrol target reached!")
                rospy.loginfo(f"🔄 New direction: {math.degrees(patrol_direction[0]):.1f}° " +
                            f"(target: {patrol_change_distance[0]}m)")
            
            # OPTIONAL: Very stuck detection (not moving at all for long time)
            # Only trigger if robot hasn't moved much despite trying
            elif avoider.avoiding and traveled < 2 and patrol_stuck_check[0] > 100:
                # Robot is truly stuck (avoiding for 10+ seconds without progress)
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0
                rospy.logwarn("⚠️  Stuck detected - Trying new direction")
                rospy.loginfo(f"🔄 New direction: {math.degrees(patrol_direction[0]):.1f}° " +
                            f"(target: {patrol_change_distance[0]}m)")
            
            # Increment stuck counter if avoiding
            if avoider.avoiding:
                patrol_stuck_check[0] += 1
            else:
                patrol_stuck_check[0] = 0  # Reset when not avoiding
            
            # Calculate angle to patrol direction
            angle_diff = patrol_direction[0] - robot_pose['yaw']
            
            # Normalize angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Determine goal direction for avoider
            if angle_diff > 0.5:
                goal_direction = -1
            elif angle_diff < -0.5:
                goal_direction = 1
            else:
                goal_direction = 0
            
            # Get avoidance command
            vel = avoider.avoid(goal_direction)
            
            # Detect when exited avoiding
            if was_avoiding and not avoider.avoiding:
                post_avoid_counter[0] = POST_AVOID_LOOPS
                rospy.loginfo(f"✅ Path clear - Post-avoid forward ({POST_AVOID_LOOPS} loops)")
            
            was_avoiding = avoider.avoiding
            
            # Post-avoid forward drive
            if post_avoid_counter[0] > 0 and not avoider.avoiding:
                post_avoid_counter[0] -= 1
                vel.linear.x = 0.3
                vel.angular.z = 0.0
            
            # Currently avoiding
            elif avoider.avoiding:
                rospy.loginfo_throttle(2, f"⚠️  Patrol: Avoiding obstacle " +
                                      f"(traveled: {traveled:.1f}m/{patrol_change_distance[0]}m)")
            
            # Navigate toward patrol direction
            elif not avoider.avoiding and post_avoid_counter[0] == 0:
                if abs(angle_diff) > 0.3:
                    vel.angular.z = 0.4 if angle_diff > 0 else -0.4
                    rospy.loginfo_throttle(3, 
                        f"🔍 Patrol: Rotating to {math.degrees(patrol_direction[0]):.1f}° " +
                        f"(angle diff: {math.degrees(angle_diff):.1f}°, {traveled:.1f}m/{patrol_change_distance[0]}m)")
                else:
                    rospy.loginfo_throttle(5, 
                        f"🔍 Patrol: Exploring ({traveled:.1f}m / {patrol_change_distance[0]}m) " +
                        f"direction: {math.degrees(patrol_direction[0]):.1f}°")
            
            pub.publish(vel)
            rate.sleep()
            continue
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # MODE 2: GOAL MODE (MANUAL GOAL ACTIVE)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        
        # Calculate distance to goal
        dx = goal.position.x - robot_pose['x']
        dy = goal.position.y - robot_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # GOAL REACHED
        if distance < goal_tolerance:
            rospy.loginfo("=" * 60)
            rospy.loginfo("🎉 GOAL REACHED!")
            rospy.loginfo("   Returning to PATROL MODE...")
            rospy.loginfo("=" * 60)
            current_goal[0] = None
            patrol_mode[0] = False  # Will trigger patrol mode entry on next loop
            post_avoid_counter[0] = 0
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            pub.publish(vel)
            rate.sleep()
            continue
        
        # Calculate angle to goal
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - robot_pose['yaw']
        
        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Determine goal direction
        if angle_diff > 0.5:
            goal_direction = -1
        elif angle_diff < -0.5:
            goal_direction = 1
        else:
            goal_direction = 0
        
        # Get avoidance command
        vel = avoider.avoid(goal_direction)
        
        # Detect when exited avoiding
        if was_avoiding and not avoider.avoiding:
            post_avoid_counter[0] = POST_AVOID_LOOPS
            rospy.loginfo(f"✅ Starting post-avoid forward drive ({POST_AVOID_LOOPS} loops)")
        
        was_avoiding = avoider.avoiding
        
        # Post-avoid forward drive
        if post_avoid_counter[0] > 0 and not avoider.avoiding:
            post_avoid_counter[0] -= 1
            vel.linear.x = 0.3
            vel.angular.z = 0.0
            rospy.loginfo_throttle(1, f"   Post-avoid forward ({post_avoid_counter[0]} loops left, " +
                                   f"dist: {distance:.2f}m)")
        
        # Currently avoiding
        elif avoider.avoiding:
            rospy.loginfo_throttle(2, f"⚠️  Avoiding (dist to goal: {distance:.2f}m)")
        
        # Navigate toward goal
        elif not avoider.avoiding and post_avoid_counter[0] == 0:
            if abs(angle_diff) > 0.3:
                vel.angular.z = 0.4 if angle_diff > 0 else -0.4
                rospy.loginfo_throttle(2, f"→ Rotating to goal (dist: {distance:.2f}m, " +
                                       f"angle: {math.degrees(angle_diff):.1f}°)")
            else:
                rospy.loginfo_throttle(3, f"✅ Moving to goal (dist: {distance:.2f}m)")
        
        pub.publish(vel)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Goal-Based Avoider stopped")
