#!/usr/bin/env python3

"""
Simple Continuous Avoider - Turns gently until all lasers clear
"""

import rospy

class LargeRoverAvoider():
    """Ultra-simple continuous avoider"""
    
    def __init__(self, vel_obj, obstacle_threshold=10.0,
                 normal_lin_vel=0.3, avoid_lin_vel=0.2, avoid_ang_vel=0.4):
        """
        :param obstacle_threshold: Distance to start avoiding (10m)
        :param normal_lin_vel: Normal forward speed (0.3 m/s)
        :param avoid_lin_vel: Speed while avoiding (0.2 m/s)
        :param avoid_ang_vel: Turn rate while avoiding (0.4 rad/s)
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
        
        # Expose avoiding state (for goal navigation)
        self.avoiding = False
        
        rospy.loginfo("🚀 Simple Continuous Avoider initialized")
        rospy.loginfo(f"   Safe distance: {obstacle_threshold}m")
        rospy.loginfo(f"   Turns until ALL lasers clear!")
    
    def identify_regions(self, scan):
        """Split laser into 5 regions"""
        ranges = scan.ranges
        num_rays = len(ranges)
        
        if num_rays == 0:
            return
        
        region_size = num_rays // 5
        
        def get_min_or_max(ray_list, default=999):
            valid = [r for r in ray_list if r < 100 and r != float('inf')]
            return min(valid) if valid else default
        
        self.front_left_dist = get_min_or_max(ranges[0:region_size])
        self.left_dist = get_min_or_max(ranges[region_size:2*region_size])
        self.front_dist = get_min_or_max(ranges[2*region_size:3*region_size])
        self.right_dist = get_min_or_max(ranges[3*region_size:4*region_size])
        self.front_right_dist = get_min_or_max(ranges[4*region_size:num_rays])
    
    def avoid(self, goal_direction=0):
        """
        ULTRA-SIMPLE LOGIC:
        - If ANY laser sees obstacle → turn toward clearer side
        - Keep turning until ALL clear
        - Then go straight
        """
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # CHECK: Is there ANY obstacle in ANY direction?
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        obstacles_detected = (
            self.front_dist < self.OBSTACLE_DIST or
            self.front_left_dist < self.OBSTACLE_DIST or
            self.front_right_dist < self.OBSTACLE_DIST or
            self.left_dist < self.OBSTACLE_DIST or
            self.right_dist < self.OBSTACLE_DIST
        )
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # CASE 1: Obstacle detected - TURN until clear!
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if obstacles_detected:
            self.avoiding = True
            
            # Calculate which side has MORE space
            left_space = min(self.left_dist, self.front_left_dist)
            right_space = min(self.right_dist, self.front_right_dist)
            
            # Decide turn direction
            if left_space > right_space + 2.0:
                # LEFT is clearer → turn LEFT
                turn_direction = "LEFT"
                angular_vel = self.AVOID_ANG_VEL
            elif right_space > left_space + 2.0:
                # RIGHT is clearer → turn RIGHT
                turn_direction = "RIGHT"
                angular_vel = -self.AVOID_ANG_VEL
            else:
                # Equal space → use goal direction
                if goal_direction <= 0:
                    turn_direction = "LEFT"
                    angular_vel = self.AVOID_ANG_VEL
                else:
                    turn_direction = "RIGHT"
                    angular_vel = -self.AVOID_ANG_VEL
            
            # Log
            rospy.loginfo_throttle(0.5, 
                f"⚠️  Avoiding {turn_direction} " +
                f"(F:{self.front_dist:.1f} FL:{self.front_left_dist:.1f} " +
                f"FR:{self.front_right_dist:.1f} L:{self.left_dist:.1f} R:{self.right_dist:.1f})")
            
            # Apply: SLOW FORWARD + GENTLE TURN
            self.vel_obj.linear.x = self.AVOID_LIN_VEL
            self.vel_obj.angular.z = angular_vel
            
            return self.vel_obj
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # CASE 2: ALL CLEAR - Go straight!
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        else:
            if self.avoiding:
                # Just cleared!
                rospy.loginfo("✅ Path CLEAR! All lasers green!")
                self.avoiding = False
            
            # Go straight forward
            self.vel_obj.linear.x = self.NORMAL_LIN_VEL
            self.vel_obj.angular.z = 0.0
            
            return self.vel_obj
