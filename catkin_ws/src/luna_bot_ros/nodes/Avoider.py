#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Avoider():
    ''' ULTRA-SIMPLE obstacle avoidance '''

    Regions_Report = {
                         "front_C": [], "front_L": [], "left_R" : [],
                         "left_C" : [], "left_L" : [], "back_R" : [],
                         "back_C" : [], "back_L" : [], "right_R": [],
                         "right_C": [], "right_L": [], "front_R": [],
                     }

    def __init__(self, vel_obj, 
                 obstacle_threshold=15.0,
                 avoid_distance=5.0,
                 emergency_distance=2.0,
                 regional_angle=30,
                 normal_lin_vel=0.3,
                 trans_lin_vel=0.15,
                 trans_ang_vel=0.6):
        self.vel_obj        = vel_obj
        self.OBSTACLE_DIST  = obstacle_threshold
        self.AVOID_DIST     = avoid_distance
        self.EMERGENCY_DIST = emergency_distance
        self.REGIONAL_ANGLE = regional_angle
        self.NORMAL_LIN_VEL = normal_lin_vel
        self.TRANS_LIN_VEL  = trans_lin_vel
        self.TRANS_ANG_VEL  = trans_ang_vel

    def indentify_regions(self, scan):
        '''Splits laser into 12 regions'''
        
        total_rays = len(scan.ranges)
        fov_degrees = 360
        rays_per_degree = total_rays / fov_degrees
        rays_per_region = int(self.REGIONAL_ANGLE * rays_per_degree)
        
        half_region = int(rays_per_region / 2)
        
        # Front central
        front_rays = list(scan.ranges[-half_region:]) + list(scan.ranges[:half_region])
        self.Regions_Report["front_C"] = [x for x in front_rays 
                                          if x <= self.OBSTACLE_DIST and x != float('inf') and x > 0.1]
        
        # Other regions
        REGIONS = ["front_L", "left_R", "left_C", "left_L", "back_R",
                   "back_C", "back_L", "right_R", "right_C", "right_L", "front_R"]
        
        for i, region in enumerate(REGIONS):
            start_idx = half_region + (rays_per_region * i)
            end_idx = half_region + (rays_per_region * (i + 1))
            
            if end_idx <= total_rays:
                region_rays = scan.ranges[start_idx:end_idx]
            else:
                region_rays = list(scan.ranges[start_idx:])
            
            self.Regions_Report[region] = [x for x in region_rays
                                           if x <= self.OBSTACLE_DIST and x != float('inf') and x > 0.1]

    def avoid(self):
        '''ULTRA-SIMPLE decision logic'''
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Check FRONT regions (3 regions: front_C, front_L, front_R)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        front_regions = ["front_C", "front_L", "front_R"]
        min_front_dist = float('inf')
        
        for region in front_regions:
            if len(self.Regions_Report[region]) > 0:
                dist = min(self.Regions_Report[region])
                if dist < min_front_dist:
                    min_front_dist = dist
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Check SIDE regions (2 regions: left_C, right_C)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        min_left_dist = min(self.Regions_Report["left_C"]) if len(self.Regions_Report["left_C"]) > 0 else float('inf')
        min_right_dist = min(self.Regions_Report["right_C"]) if len(self.Regions_Report["right_C"]) > 0 else float('inf')
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # DECISION LOGIC - 3 STATES
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        
        # STATE 1: EMERGENCY - Reverse!
        if min_front_dist < self.EMERGENCY_DIST or min_left_dist < self.EMERGENCY_DIST or min_right_dist < self.EMERGENCY_DIST:
            rospy.logwarn_throttle(0.5, f"🛑 EMERGENCY! F:{min_front_dist:.1f} L:{min_left_dist:.1f} R:{min_right_dist:.1f}")
            self.vel_obj.linear.x = -0.3    # Reverse!
            self.vel_obj.angular.z = 0.8    # Turn while reversing
            
        # STATE 2: AVOID - Turn away!
        elif min_front_dist < self.AVOID_DIST or min_left_dist < self.AVOID_DIST or min_right_dist < self.AVOID_DIST:
            rospy.logwarn_throttle(0.5, f"⚠️ AVOIDING! F:{min_front_dist:.1f} L:{min_left_dist:.1f} R:{min_right_dist:.1f}")
            
            # Turn AWAY from obstacle
            if min_left_dist < min_right_dist:
                # Obstacle on LEFT → Turn RIGHT
                self.vel_obj.linear.x = 0.15
                self.vel_obj.angular.z = -0.6  # Turn right
            else:
                # Obstacle on RIGHT → Turn LEFT
                self.vel_obj.linear.x = 0.15
                self.vel_obj.angular.z = 0.6   # Turn left
        
        # STATE 3: CLEAR - Go straight!
        else:
            rospy.loginfo_throttle(2, "✅ CLEAR!")
            self.vel_obj.linear.x = self.NORMAL_LIN_VEL
            self.vel_obj.angular.z = 0.0
        
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        
        self.vel_obj.linear.y  = 0
        self.vel_obj.linear.z  = 0
        self.vel_obj.angular.x = 0
        self.vel_obj.angular.y = 0
        
        return self.vel_obj
