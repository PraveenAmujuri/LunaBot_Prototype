#!/usr/bin/env python3
"""
Goal-Based Navigation + Autonomous Patrol Mode
- Drives to goals when sent
- Patrols autonomously when no goal (random exploration)
- Avoids obstacles in both modes
- Travels 30–40m before changing direction
"""

import math
import random
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from avoider_large_rover import LargeRoverAvoider


def main():
    rospy.init_node("goal_based_avoider")
    
    vel = Twist()
    avoider = LargeRoverAvoider(vel, obstacle_threshold=19.0)
    
    current_goal = [None]
    robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    goal_tolerance = 2.0
    
    post_avoid_counter = [0]
    POST_AVOID_LOOPS = 20
    
    # Patrol mode variables
    patrol_mode = [True]
    patrol_direction = [0.0]
    patrol_change_distance = [30]
    last_patrol_position = [{'x': 0.0, 'y': 0.0}]
    patrol_stuck_check = [0]

    # Subscribers
    def goal_callback(msg):
        current_goal[0] = msg.pose
        post_avoid_counter[0] = 0
        patrol_mode[0] = False
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"New goal received: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")
        rospy.loginfo("Patrol mode disabled (manual goal active)")
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
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("Goal-Based Avoider + Autonomous Patrol Initialized")
    rospy.loginfo("=" * 60)
    rospy.loginfo("Safe distance: 19m")
    rospy.loginfo("Post-avoid forward enabled")
    rospy.loginfo("Autonomous patrol when no goal")
    rospy.loginfo("Patrol direction changes every 30–40m")
    rospy.loginfo("=" * 60)
    rospy.loginfo("Patrol mode active – exploring autonomously")

    was_avoiding = False

    while not rospy.is_shutdown():
        goal = current_goal[0]

        # PATROL MODE
        if goal is None:
            if not patrol_mode[0]:
                patrol_mode[0] = True
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0
                rospy.loginfo("=" * 60)
                rospy.loginfo("Patrol mode activated – exploring autonomously")
                rospy.loginfo(f"Initial direction: {math.degrees(patrol_direction[0]):.1f}°")
                rospy.loginfo(f"Target distance: {patrol_change_distance[0]}m")
                rospy.loginfo("=" * 60)
            
            dx = robot_pose['x'] - last_patrol_position[0]['x']
            dy = robot_pose['y'] - last_patrol_position[0]['y']
            traveled = math.sqrt(dx ** 2 + dy ** 2)
            
            if traveled >= patrol_change_distance[0]:
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0
                rospy.loginfo("Patrol segment complete.")
                rospy.loginfo(f"New direction: {math.degrees(patrol_direction[0]):.1f}° "
                              f"(target: {patrol_change_distance[0]}m)")
            
            elif avoider.avoiding and traveled < 2 and patrol_stuck_check[0] > 100:
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0
                rospy.logwarn("Robot appears stuck. Changing direction.")
                rospy.loginfo(f"New direction: {math.degrees(patrol_direction[0]):.1f}° "
                              f"(target: {patrol_change_distance[0]}m)")
            
            patrol_stuck_check[0] += 1 if avoider.avoiding else 0
            
            angle_diff = patrol_direction[0] - robot_pose['yaw']
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if angle_diff > 0.5:
                goal_direction = -1
            elif angle_diff < -0.5:
                goal_direction = 1
            else:
                goal_direction = 0
            
            vel = avoider.avoid(goal_direction)
            
            if was_avoiding and not avoider.avoiding:
                post_avoid_counter[0] = POST_AVOID_LOOPS
                rospy.loginfo(f"Path clear – post-avoid forward ({POST_AVOID_LOOPS} loops)")
            
            was_avoiding = avoider.avoiding
            
            if post_avoid_counter[0] > 0 and not avoider.avoiding:
                post_avoid_counter[0] -= 1
                vel.linear.x = 0.3
                vel.angular.z = 0.0
            elif avoider.avoiding:
                rospy.loginfo_throttle(2, f"Patrol: Avoiding obstacle "
                                          f"(traveled: {traveled:.1f}/{patrol_change_distance[0]}m)")
            elif not avoider.avoiding and post_avoid_counter[0] == 0:
                if abs(angle_diff) > 0.3:
                    vel.angular.z = 0.4 if angle_diff > 0 else -0.4
                    rospy.loginfo_throttle(3, 
                        f"Patrol: Rotating to {math.degrees(patrol_direction[0]):.1f}° "
                        f"(angle diff: {math.degrees(angle_diff):.1f}°, "
                        f"{traveled:.1f}/{patrol_change_distance[0]}m)")
                else:
                    rospy.loginfo_throttle(5,
                        f"Patrol: Moving forward ({traveled:.1f}/{patrol_change_distance[0]}m) "
                        f"direction: {math.degrees(patrol_direction[0]):.1f}°")
            
            pub.publish(vel)
            rate.sleep()
            continue

        # GOAL MODE
        dx = goal.position.x - robot_pose['x']
        dy = goal.position.y - robot_pose['y']
        distance = math.sqrt(dx ** 2 + dy ** 2)
        
        if distance < goal_tolerance:
            rospy.loginfo("=" * 60)
            rospy.loginfo("Goal reached.")
            rospy.loginfo("Returning to patrol mode.")
            rospy.loginfo("=" * 60)
            current_goal[0] = None
            patrol_mode[0] = False
            post_avoid_counter[0] = 0
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            pub.publish(vel)
            rate.sleep()
            continue
        
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - robot_pose['yaw']
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if angle_diff > 0.5:
            goal_direction = -1
        elif angle_diff < -0.5:
            goal_direction = 1
        else:
            goal_direction = 0
        
        vel = avoider.avoid(goal_direction)
        
        if was_avoiding and not avoider.avoiding:
            post_avoid_counter[0] = POST_AVOID_LOOPS
            rospy.loginfo(f"Post-avoid forward drive ({POST_AVOID_LOOPS} loops)")
        
        was_avoiding = avoider.avoiding
        
        if post_avoid_counter[0] > 0 and not avoider.avoiding:
            post_avoid_counter[0] -= 1
            vel.linear.x = 0.3
            vel.angular.z = 0.0
            rospy.loginfo_throttle(1, f"Post-avoid forward ({post_avoid_counter[0]} loops left, "
                                       f"dist: {distance:.2f}m)")
        elif avoider.avoiding:
            rospy.loginfo_throttle(2, f"Avoiding obstacle (distance to goal: {distance:.2f}m)")
        elif not avoider.avoiding and post_avoid_counter[0] == 0:
            if abs(angle_diff) > 0.3:
                vel.angular.z = 0.4 if angle_diff > 0 else -0.4
                rospy.loginfo_throttle(2, f"Rotating to goal (dist: {distance:.2f}m, "
                                           f"angle: {math.degrees(angle_diff):.1f}°)")
            else:
                rospy.loginfo_throttle(3, f"Moving toward goal (dist: {distance:.2f}m)")
        
        pub.publish(vel)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal-Based Avoider stopped.")
