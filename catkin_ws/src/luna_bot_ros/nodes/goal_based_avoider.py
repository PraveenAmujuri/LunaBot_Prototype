#!/usr/bin/env python3
"""
Goal Navigation + Patrol Mode
- Follows user-given goals
- Switches to patrol when no goal is active
- Avoids obstacles in both modes
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

    # Patrol mode state
    patrol_mode = [True]
    patrol_direction = [0.0]
    patrol_change_distance = [30]      # Distance before picking a new direction
    last_patrol_position = [{'x': 0.0, 'y': 0.0}]
    patrol_stuck_check = [0]

    # ---- Subscribers ----
    def goal_callback(msg):
        current_goal[0] = msg.pose
        post_avoid_counter[0] = 0
        patrol_mode[0] = False

        rospy.loginfo("-" * 50)
        rospy.loginfo(f"Goal received: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")
        rospy.loginfo("Patrol mode paused")
        rospy.loginfo("-" * 50)

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

    # ---- Startup Log ----
    rospy.loginfo("-" * 50)
    rospy.loginfo("Navigation + Patrol started")
    rospy.loginfo(f"Safe distance: {avoider.OBSTACLE_DIST}m")
    rospy.loginfo("Patrol mode active")
    rospy.loginfo("-" * 50)

    was_avoiding = False

    # ========================= MAIN LOOP =========================
    while not rospy.is_shutdown():
        goal = current_goal[0]

        # ------------------------ PATROL MODE ------------------------
        if goal is None:

            if not patrol_mode[0]:
                patrol_mode[0] = True
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0

                rospy.loginfo("-" * 50)
                rospy.loginfo("Patrol mode resumed")
                rospy.loginfo(f"Direction: {math.degrees(patrol_direction[0]):.1f}°")
                rospy.loginfo(f"Distance target: {patrol_change_distance[0]}m")
                rospy.loginfo("-" * 50)

            dx = robot_pose['x'] - last_patrol_position[0]['x']
            dy = robot_pose['y'] - last_patrol_position[0]['y']
            traveled = math.sqrt(dx ** 2 + dy ** 2)

            # Pick new patrol direction after finishing segment
            if traveled >= patrol_change_distance[0]:
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0

                rospy.loginfo("Patrol segment complete")
                rospy.loginfo(f"New direction: {math.degrees(patrol_direction[0]):.1f}°")

            # Detect if stuck right after avoiding
            elif avoider.avoiding and traveled < 2 and patrol_stuck_check[0] > 100:
                patrol_direction[0] = random.uniform(-math.pi, math.pi)
                patrol_change_distance[0] = random.randint(30, 40)
                last_patrol_position[0] = {'x': robot_pose['x'], 'y': robot_pose['y']}
                patrol_stuck_check[0] = 0

                rospy.logwarn("Low movement detected — changing direction")
                rospy.loginfo(f"New direction: {math.degrees(patrol_direction[0]):.1f}°")

            patrol_stuck_check[0] += 1 if avoider.avoiding else 0

            # Angle to desired patrol heading
            angle_diff = patrol_direction[0] - robot_pose['yaw']
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Turn left/right/center
            if angle_diff > 0.5:
                goal_direction = -1
            elif angle_diff < -0.5:
                goal_direction = 1
            else:
                goal_direction = 0

            vel = avoider.avoid(goal_direction)

            if was_avoiding and not avoider.avoiding:
                post_avoid_counter[0] = POST_AVOID_LOOPS
                rospy.loginfo(f"Moving forward ({POST_AVOID_LOOPS} loops)")

            was_avoiding = avoider.avoiding

            # Post-avoid short boost
            if post_avoid_counter[0] > 0 and not avoider.avoiding:
                post_avoid_counter[0] -= 1
                vel.linear.x = 0.3
                vel.angular.z = 0.0

            # If not avoiding and not boosting: align to heading
            elif not avoider.avoiding and post_avoid_counter[0] == 0:
                if abs(angle_diff) > 0.3:
                    vel.angular.z = 0.4 if angle_diff > 0 else -0.4
                # Else moving forward normally

            pub.publish(vel)
            rate.sleep()
            continue

        # ------------------------ GOAL MODE ------------------------
        dx = goal.position.x - robot_pose['x']
        dy = goal.position.y - robot_pose['y']
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Reached goal
        if distance < goal_tolerance:
            rospy.loginfo("-" * 50)
            rospy.loginfo("Goal reached")
            rospy.loginfo("Switching to patrol mode")
            rospy.loginfo("-" * 50)

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

        # Give hint to avoider
        if angle_diff > 0.5:
            goal_direction = -1
        elif angle_diff < -0.5:
            goal_direction = 1
        else:
            goal_direction = 0

        vel = avoider.avoid(goal_direction)

        if was_avoiding and not avoider.avoiding:
            post_avoid_counter[0] = POST_AVOID_LOOPS

        was_avoiding = avoider.avoiding

        # Post-avoid assist
        if post_avoid_counter[0] > 0 and not avoider.avoiding:
            post_avoid_counter[0] -= 1
            vel.linear.x = 0.3
            vel.angular.z = 0.0

        # Alignment toward goal
        elif not avoider.avoiding and post_avoid_counter[0] == 0:
            if abs(angle_diff) > 0.3:
                vel.angular.z = 0.4 if angle_diff > 0 else -0.4

        pub.publish(vel)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation stopped.")
