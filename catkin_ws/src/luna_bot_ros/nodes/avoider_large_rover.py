#!/usr/bin/env python3

"""
LargeRoverAvoider
Simple 5-region LaserScan based obstacle avoidance.
"""

import rospy


class LargeRoverAvoider:
    """Reactive obstacle avoidance using 5 LaserScan regions."""

    def __init__(self, vel_obj, obstacle_threshold=10.0,
                 normal_lin_vel=0.3, avoid_lin_vel=0.2, avoid_ang_vel=0.4):
        """
        Initialize parameters and state.

        Tuning notes:
            obstacle_threshold : Increase = more cautious, Decrease = more aggressive
            normal_lin_vel     : Forward speed when clear
            avoid_lin_vel      : Forward speed during avoidance (keep low for stability)
            avoid_ang_vel      : Turn rate during avoidance (higher = sharper turns)
        """
        self.vel_obj = vel_obj
        self.OBSTACLE_DIST = obstacle_threshold
        self.NORMAL_LIN_VEL = normal_lin_vel
        self.AVOID_LIN_VEL = avoid_lin_vel
        self.AVOID_ANG_VEL = avoid_ang_vel

        # Laser region distance placeholders
        self.front_dist = 999
        self.left_dist = 999
        self.right_dist = 999
        self.front_left_dist = 999
        self.front_right_dist = 999

        self.avoiding = False

        rospy.loginfo("LargeRoverAvoider ready")

    def identify_regions(self, scan):
        """
        Split LaserScan into 5 regions and extract min distances.
        """
        ranges = scan.ranges
        num_rays = len(ranges)
        if num_rays == 0:
            return

        region_size = num_rays // 5

        def get_min_distance(ray_list, default=999):
            # Ignore out-of-range / inf values
            valid = [r for r in ray_list if 0 < r < 100 and r != float('inf')]
            return min(valid) if valid else default

        self.front_left_dist = get_min_distance(ranges[0:region_size])
        self.left_dist = get_min_distance(ranges[region_size:2 * region_size])
        self.front_dist = get_min_distance(ranges[2 * region_size:3 * region_size])
        self.right_dist = get_min_distance(ranges[3 * region_size:4 * region_size])
        self.front_right_dist = get_min_distance(ranges[4 * region_size:num_rays])

    def avoid(self, goal_direction=0):
        """
        Compute avoidance Twist based on region distances.

        Tuning note:
            goal_direction :
                <=0 → prefer left turn,
                >0  → prefer right turn
                (used only when both sides look equal)
        """
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

            # Direction choice
            if left_space > right_space + 2.0:
                angular_vel = self.AVOID_ANG_VEL
                turn_direction = "LEFT"
            elif right_space > left_space + 2.0:
                angular_vel = -self.AVOID_ANG_VEL
                turn_direction = "RIGHT"
            else:
                # Fall back to goal hint
                if goal_direction <= 0:
                    angular_vel = self.AVOID_ANG_VEL
                    turn_direction = "LEFT"
                else:
                    angular_vel = -self.AVOID_ANG_VEL
                    turn_direction = "RIGHT"

            rospy.loginfo_throttle(
                0.5,
                f"Avoiding {turn_direction} "
                f"(F:{self.front_dist:.1f} FL:{self.front_left_dist:.1f} "
                f"FR:{self.front_right_dist:.1f} L:{self.left_dist:.1f} R:{self.right_dist:.1f})"
            )

            self.vel_obj.linear.x = self.AVOID_LIN_VEL
            self.vel_obj.angular.z = angular_vel
            return self.vel_obj

        # Path clear
        if self.avoiding:
            rospy.loginfo("Path clear")
            self.avoiding = False

        self.vel_obj.linear.x = self.NORMAL_LIN_VEL
        self.vel_obj.angular.z = 0.0
        return self.vel_obj
