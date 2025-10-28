#!/usr/bin/env python3
"""
Mission Manager - OPTIMIZED FOR LARGE ROVER (11m length)
- Larger goal threshold (6m for 11m rover)
- Wider waypoint spacing (10m)
- Adjusted timeout
"""

import rospy
import json
import math
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalID, GoalStatusArray


class MissionManager:
    def __init__(self):
        rospy.loginfo("🎯 Mission Manager Starting...")
        
        # Publishers
        self.status_pub = rospy.Publisher('/mission_status', String, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/battery_level', Float32, self.battery_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/semantic_map', String, self.semantic_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)
        
        # Mission state
        self.battery_level = 100.0
        self.low_battery_threshold = 10.0
        self.mission_mode = "EXPLORATION"
        
        # Base location (starting position)
        self.base_x = None
        self.base_y = None
        self.base_detected = False
        self.first_position_recorded = False
        
        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # ═══════════════════════════════════════════════════════════
        # OPTIMIZED FOR 11-METER ROVER
        # ═══════════════════════════════════════════════════════════
        self.rover_length = 11.0  # meters
        self.path_spacing = 10.0  # ← INCREASED: 10m between waypoints (fewer, more spaced)
        self.goal_reached_distance = 6.0  # ← INCREASED: 6m threshold (about half rover length)
        self.goal_timeout = 60.0  # ← INCREASED: 60s timeout (large rover is slower)
        # ═══════════════════════════════════════════════════════════
        
        # Path tracking
        self.path_history = []
        self.last_path_x = None
        self.last_path_y = None
        
        # Backtracking state
        self.backtrack_index = 0
        self.emergency_triggered = False
        self.current_goal_reached = True
        self.goal_sent_time = None
        
        # Status update timer
        rospy.Timer(rospy.Duration(5.0), self.publish_status)
        
        # Backtracking timer
        rospy.Timer(rospy.Duration(5.0), self.backtrack_timer)  # Check every 5s
        
        rospy.loginfo("✅ Mission Manager Ready!")
        rospy.loginfo(f"   Rover length: {self.rover_length}m (LARGE ROVER)")
        rospy.loginfo(f"   Emergency threshold: {self.low_battery_threshold}%")
        rospy.loginfo(f"   Goal reached distance: {self.goal_reached_distance}m")
        rospy.loginfo(f"   Waypoint spacing: {self.path_spacing}m")
        rospy.loginfo(f"   Goal timeout: {self.goal_timeout}s")


    def battery_callback(self, msg):
        """Monitor battery level and trigger emergency if needed"""
        self.battery_level = msg.data
        
        if self.battery_level < self.low_battery_threshold and not self.emergency_triggered:
            self.emergency_triggered = True
            
            rospy.logwarn("="*60)
            rospy.logwarn("🔋 LOW BATTERY! EMERGENCY BACKTRACK ACTIVATED!")
            rospy.logwarn(f"   Battery: {self.battery_level:.1f}%")
            rospy.logwarn(f"   Current position: ({self.robot_x:.1f}, {self.robot_y:.1f})")
            rospy.logwarn(f"   Base position: ({self.base_x:.1f}, {self.base_y:.1f})")
            rospy.logwarn("="*60)
            
            self.mission_mode = "BACKTRACKING"
            self.initiate_backtrack()


    def odom_callback(self, msg):
        """Track robot position and build path"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Set base to starting position
        if not self.first_position_recorded:
            self.base_x = self.robot_x
            self.base_y = self.robot_y
            self.first_position_recorded = True
            rospy.loginfo(f"🏠 Base set to starting position: ({self.base_x:.1f}, {self.base_y:.1f})")
        
        # Build path from odometry (every 10 meters for large rover)
        if self.last_path_x is None:
            self.path_history.append((self.robot_x, self.robot_y))
            self.last_path_x = self.robot_x
            self.last_path_y = self.robot_y
            rospy.loginfo("🛤️ First waypoint added to path")
        else:
            distance = math.sqrt(
                (self.robot_x - self.last_path_x)**2 + 
                (self.robot_y - self.last_path_y)**2
            )
            
            if distance >= self.path_spacing:
                self.path_history.append((self.robot_x, self.robot_y))
                self.last_path_x = self.robot_x
                self.last_path_y = self.robot_y
                rospy.loginfo(f"🛤️ Waypoint added: {len(self.path_history)} total ({distance:.1f}m from last)")


    def goal_status_callback(self, msg):
        """Monitor goal status"""
        if len(msg.status_list) > 0:
            latest_status = msg.status_list[-1].status
            if latest_status == 3:  # SUCCEEDED
                self.current_goal_reached = True
                rospy.loginfo("✅ Goal status: SUCCEEDED")


    def semantic_callback(self, msg):
        """Detect base location from YOLO detections"""
        try:
            semantic_map = json.loads(msg.data)
            
            if len(semantic_map.get('base', [])) > 0:
                base_detection = semantic_map['base'][0]
                new_base_x = base_detection['x']
                new_base_y = base_detection['y']
                
                if self.base_x is not None:
                    distance_to_start = math.sqrt(
                        (new_base_x - self.base_x)**2 + 
                        (new_base_y - self.base_y)**2
                    )
                    
                    if distance_to_start < 15.0:  # Increased for large rover
                        self.base_x = new_base_x
                        self.base_y = new_base_y
                        self.base_detected = True
                        rospy.loginfo_throttle(10, 
                            f"🏠 Base detected at: ({self.base_x:.1f}, {self.base_y:.1f})")
                
        except Exception as e:
            rospy.logwarn(f"Semantic callback error: {e}")


    def initiate_backtrack(self):
        """Start backtracking along traversed path"""
        
        if self.base_x is None or self.base_y is None:
            rospy.logerr("❌ Base position not set! Cannot return!")
            return
        
        if len(self.path_history) < 2:
            rospy.logwarn("⚠️ No path history! Going direct to base...")
            self.send_goal(self.base_x, self.base_y)
            return
        
        distance_to_base = math.sqrt(
            (self.base_x - self.robot_x)**2 + 
            (self.base_y - self.robot_y)**2
        )
        
        rospy.logwarn("="*60)
        rospy.logwarn("🔙 BACKTRACKING MODE ACTIVATED!")
        rospy.logwarn(f"   Rover length: {self.rover_length}m (LARGE ROVER)")
        rospy.logwarn(f"   Current position: ({self.robot_x:.1f}, {self.robot_y:.1f})")
        rospy.logwarn(f"   Base position: ({self.base_x:.1f}, {self.base_y:.1f})")
        rospy.logwarn(f"   Distance to base: {distance_to_base:.1f}m")
        rospy.logwarn(f"   Path waypoints: {len(self.path_history)} (spaced {self.path_spacing}m apart)")
        rospy.logwarn(f"   Goal threshold: {self.goal_reached_distance}m")
        rospy.logwarn("="*60)
        
        # Start from end of path
        self.backtrack_index = len(self.path_history) - 1
        self.current_goal_reached = True


    def backtrack_timer(self, event):
        """Check if backtracking and send next waypoint when ready"""
        if self.mission_mode != "BACKTRACKING":
            return
        
        if not self.current_goal_reached:
            # Check distance manually + timeout
            if self.backtrack_index >= 0 and self.backtrack_index < len(self.path_history):
                waypoint_x, waypoint_y = self.path_history[self.backtrack_index]
                distance_to_waypoint = math.sqrt(
                    (self.robot_x - waypoint_x)**2 + 
                    (self.robot_y - waypoint_y)**2
                )
                
                # Check if close enough
                if distance_to_waypoint < self.goal_reached_distance:
                    rospy.loginfo(f"✅ Reached waypoint {self.backtrack_index} (dist: {distance_to_waypoint:.1f}m < {self.goal_reached_distance}m)")
                    self.current_goal_reached = True
                    return
                
                # Check timeout
                if self.goal_sent_time is not None:
                    elapsed = (rospy.Time.now() - self.goal_sent_time).to_sec()
                    if elapsed > self.goal_timeout:
                        rospy.logwarn(f"⏱️ Goal timeout ({elapsed:.0f}s > {self.goal_timeout}s)! Forcing next waypoint...")
                        rospy.logwarn(f"   Distance to waypoint: {distance_to_waypoint:.1f}m")
                        self.current_goal_reached = True
                        return
                
                # Still waiting
                if int(elapsed) % 10 == 0:  # Log every 10s
                    rospy.loginfo(f"⏳ Waiting for waypoint {self.backtrack_index}... " +
                                f"dist: {distance_to_waypoint:.1f}m, elapsed: {elapsed:.0f}s/{self.goal_timeout}s")
                return
        
        # Goal reached or timeout - send next waypoint
        self.backtrack_next_waypoint()


    def backtrack_next_waypoint(self):
        """Follow path in reverse"""
        
        # Move backwards through path
        self.backtrack_index -= 1
        
        # Check if reached base
        if self.backtrack_index < 0:
            rospy.logwarn("="*60)
            rospy.logwarn("✅ BACKTRACK COMPLETE! All waypoints visited!")
            rospy.logwarn(f"   Final destination: BASE ({self.base_x:.1f}, {self.base_y:.1f})")
            rospy.logwarn("="*60)
            
            # Send final goal to exact base position
            self.send_goal(self.base_x, self.base_y)
            self.mission_mode = "RETURNING_TO_BASE"
            return
        
        # Get next waypoint
        waypoint_x, waypoint_y = self.path_history[self.backtrack_index]
        
        distance_to_waypoint = math.sqrt(
            (self.robot_x - waypoint_x)**2 + 
            (self.robot_y - waypoint_y)**2
        )
        
        distance_to_base = math.sqrt(
            (self.base_x - self.robot_x)**2 + 
            (self.base_y - self.robot_y)**2
        )
        
        rospy.logwarn("")
        rospy.logwarn("🔙 " + "="*55)
        rospy.logwarn(f"🔙 BACKTRACKING: Waypoint {self.backtrack_index}/{len(self.path_history)-1}")
        rospy.logwarn("🔙 " + "="*55)
        rospy.logwarn(f"   📍 Target: ({waypoint_x:.1f}, {waypoint_y:.1f})")
        rospy.logwarn(f"   📏 Distance: {distance_to_waypoint:.1f}m")
        rospy.logwarn(f"   🔋 Battery: {self.battery_level:.1f}%")
        rospy.logwarn(f"   🏠 Distance to base: {distance_to_base:.1f}m")
        rospy.logwarn(f"   ⚙️  Goal threshold: {self.goal_reached_distance}m")
        rospy.logwarn("🔙 " + "="*55)
        rospy.logwarn("")
        
        self.send_goal(waypoint_x, waypoint_y)
        self.current_goal_reached = False
        self.goal_sent_time = rospy.Time.now()


    def send_goal(self, x, y):
        """Send navigation goal"""
        try:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            
            # Publish multiple times for large rover
            for i in range(5):
                self.goal_pub.publish(goal)
                rospy.sleep(0.1)
            
            rospy.loginfo(f"🎯 Goal sent: ({x:.1f}, {y:.1f})")
            
        except Exception as e:
            rospy.logerr(f"❌ Goal sending error: {e}")


    def publish_status(self, event):
        """Publish mission status"""
        
        if self.base_x is None or self.base_y is None:
            distance_to_base = 0.0
        else:
            distance_to_base = math.sqrt(
                (self.base_x - self.robot_x)**2 + 
                (self.base_y - self.robot_y)**2
            )
        
        status = {
            'battery': self.battery_level,
            'mode': self.mission_mode,
            'base_x': self.base_x if self.base_x is not None else 0.0,
            'base_y': self.base_y if self.base_y is not None else 0.0,
            'base_detected': self.base_detected,
            'robot_x': self.robot_x,
            'robot_y': self.robot_y,
            'distance_to_base': distance_to_base,
            'path_length': len(self.path_history),
            'backtrack_progress': f"{self.backtrack_index}/{len(self.path_history)-1}" if self.mission_mode == "BACKTRACKING" else "N/A",
            'emergency_triggered': self.emergency_triggered,
            'rover_length': self.rover_length
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main():
    rospy.init_node('mission_manager')
    
    rospy.loginfo("="*60)
    rospy.loginfo("🎯 MISSION MANAGER - LARGE ROVER (11m) OPTIMIZED")
    rospy.loginfo("="*60)
    rospy.loginfo("✅ Rover length: 11 meters")
    rospy.loginfo("✅ Monitors battery level")
    rospy.loginfo("✅ Triggers emergency at 10% battery")
    rospy.loginfo("✅ ALWAYS backtracks along traversed path")
    rospy.loginfo("✅ Base = Starting position")
    rospy.loginfo("✅ Waypoint spacing: 10 meters (fewer waypoints)")
    rospy.loginfo("✅ Goal reached: 6m threshold (half rover length)")
    rospy.loginfo("✅ Timeout: 60s per waypoint (slow large rover)")
    rospy.loginfo("="*60)
    
    manager = MissionManager()
    rospy.spin()


if __name__ == '__main__':
    main()
