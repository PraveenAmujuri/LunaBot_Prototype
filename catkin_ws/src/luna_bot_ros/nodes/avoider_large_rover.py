#!/usr/bin/env python3

"""
Simple Continuous Avoider - Turns gently until all lasers clear.
"""

import rospy


class LargeRoverAvoider:
    """Continuous obstacle avoidance behavior."""

    def __init__(self, vel_obj, obstacle_threshold=10.0,
                 normal_lin_vel=0.3, avoid_lin_vel=0.2, avoid_ang_vel=0.4):
        """
        :param obstacle_threshold: Distance to start avoiding (m)
        :param normal_lin_vel: Normal forward speed (m/s)
        :param avoid_lin_vel: Speed while avoiding (m/s)
        :param avoid_ang_vel: Turn rate while avoiding (rad/s)
        """
        self.vel_obj = vel_obj
        self.OBSTACLE_DIST = obstacle_threshold
        self.NORMAL_LIN_VEL = normal_lin_vel
        self.AVOID_LIN_VEL = avoid_lin_vel
        self.AVOID_ANG_VEL = avoid_ang_vel

        # Laser data storage
        self.front_dist = 999
        self.left_dist = 999
        self.right_dist = 999
        self.front_left_dist = 999
        self.front_right_dist = 999

        # Avoidance state
        self.avoiding = False

        rospy.loginfo("Simple Continuous Avoider initialized")
        rospy.loginfo(f"Safe distance: {obstacle_threshold} m")
        rospy.loginfo("Turning until all laser regions are clear")

    def identify_regions(self, scan):
        """Split laser scan into 5 regions and record minimum distances."""
        ranges = scan.ranges
        num_rays = len(ranges)
        if num_rays == 0:
            return

        region_size = num_rays // 5

        def get_min_distance(ray_list, default=999):
            valid = [r for r in ray_list if 0 < r < 100 and r != float('inf')]
            return min(valid) if valid else default

        self.front_left_dist = get_min_distance(ranges[0:region_size])
        self.left_dist = get_min_distance(ranges[region_size:2 * region_size])
        self.front_dist = get_min_distance(ranges[2 * region_size:3 * region_size])
        self.right_dist = get_min_distance(ranges[3 * region_size:4 * region_size])
        self.front_right_dist = get_min_distance(ranges[4 * region_size:num_rays])

    def avoid(self, goal_direction=0):
        """Obstacle avoidance logic: turns until all laser regions are clear."""
        obstacles_detected = (
            self.front_dist < self.OBSTACLE_DIST or
            self.front_left_dist < self.OBSTACLE_DIST or
            self.front_right_dist < self.OBSTACLE_DIST or
            self.left_dist < self.OBSTACLE_DIST or
            self.right_dist < self.OBSTACLE_DIST
        )

        if obstacles_detected:
            self.avoiding = True

            left_space = min(self.left_dist, self.front_left_dist)
            right_space = min(self.right_dist, self.front_right_dist)

            if left_space > right_space + 2.0:
                turn_direction = "LEFT"
                angular_vel = self.AVOID_ANG_VEL
            elif right_space > left_space + 2.0:
                turn_direction = "RIGHT"
                angular_vel = -self.AVOID_ANG_VEL
            else:
                if goal_direction <= 0:
                    turn_direction = "LEFT"
                    angular_vel = self.AVOID_ANG_VEL
                else:
                    turn_direction = "RIGHT"
                    angular_vel = -self.AVOID_ANG_VEL

            rospy.loginfo_throttle(0.5,
                f"Avoiding {turn_direction} "
                f"(F:{self.front_dist:.1f} FL:{self.front_left_dist:.1f} "
                f"FR:{self.front_right_dist:.1f} L:{self.left_dist:.1f} R:{self.right_dist:.1f})"
            )

            self.vel_obj.linear.x = self.AVOID_LIN_VEL
            self.vel_obj.angular.z = angular_vel
            return self.vel_obj

        else:
            if self.avoiding:
                rospy.loginfo("Path clear. Resuming normal speed.")
                self.avoiding = False

            self.vel_obj.linear.x = self.NORMAL_LIN_VEL
            self.vel_obj.angular.z = 0.0
            return self.vel_obj
