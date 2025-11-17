#!/usr/bin/env python3
"""
Mission Manager - Optimized for Large Rover (11m length)
- Larger goal threshold (6m)
- Wider waypoint spacing (10m)
- Adjusted timeout (60s)
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
        rospy.loginfo("Mission Manager Starting...")

        # Publishers
        self.status_pub = rospy.Publisher('/mission_status', String, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Subscribers
        rospy.Subscriber('/battery_level', Float32, self.battery_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # rospy.Subscriber('/semantic_map', String, self.semantic_callback)
        rospy.Subscriber('/semantic_detections', String, self.semantic_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)

        # Mission state
        self.battery_level = 100.0
        self.low_battery_threshold = 10.0
        self.mission_mode = "EXPLORATION"

        # Base position
        self.base_x = None
        self.base_y = None
        self.base_detected = False
        self.first_position_recorded = False

        # Rover position
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Parameters for large rover
        self.rover_length = 11.0
        self.path_spacing = 10.0
        self.goal_reached_distance = 6.0
        self.goal_timeout = 60.0

        # Path tracking
        self.path_history = []
        self.last_path_x = None
        self.last_path_y = None

        # Backtracking state
        self.backtrack_index = 0
        self.emergency_triggered = False
        self.current_goal_reached = True
        self.goal_sent_time = None

        # Timers
        rospy.Timer(rospy.Duration(5.0), self.publish_status)
        rospy.Timer(rospy.Duration(5.0), self.backtrack_timer)

        rospy.loginfo("Mission Manager Ready")
        rospy.loginfo(f"Rover length: {self.rover_length}m")
        rospy.loginfo(f"Emergency battery threshold: {self.low_battery_threshold}%")
        rospy.loginfo(f"Goal reached distance: {self.goal_reached_distance}m")
        rospy.loginfo(f"Waypoint spacing: {self.path_spacing}m")
        rospy.loginfo(f"Goal timeout: {self.goal_timeout}s")

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def battery_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level < self.low_battery_threshold and not self.emergency_triggered:
            self.emergency_triggered = True
            rospy.logwarn("LOW BATTERY - Emergency backtrack initiated")
            self.mission_mode = "BACKTRACKING"
            self.initiate_backtrack()

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        if not self.first_position_recorded:
            self.base_x = self.robot_x
            self.base_y = self.robot_y
            self.first_position_recorded = True
            rospy.loginfo(f"Base set to starting position: ({self.base_x:.1f}, {self.base_y:.1f})")

        if self.last_path_x is None:
            self.path_history.append((self.robot_x, self.robot_y))
            self.last_path_x = self.robot_x
            self.last_path_y = self.robot_y
        else:
            distance = math.hypot(self.robot_x - self.last_path_x, self.robot_y - self.last_path_y)
            if distance >= self.path_spacing:
                self.path_history.append((self.robot_x, self.robot_y))
                self.last_path_x = self.robot_x
                self.last_path_y = self.robot_y
                rospy.loginfo(f"Waypoint added: {len(self.path_history)} total ({distance:.1f}m apart)")

    def goal_status_callback(self, msg):
        if len(msg.status_list) > 0 and msg.status_list[-1].status == 3:
            self.current_goal_reached = True
            rospy.loginfo("Goal reached successfully")

    def semantic_callback(self, msg):
        try:
            semantic_map = json.loads(msg.data)
            if len(semantic_map.get('base', [])) > 0:
                base_detection = semantic_map['base'][0]
                new_base_x = base_detection['x']
                new_base_y = base_detection['y']

                if self.base_x is not None:
                    distance_to_start = math.hypot(new_base_x - self.base_x, new_base_y - self.base_y)
                    if distance_to_start < 15.0:
                        self.base_x = new_base_x
                        self.base_y = new_base_y
                        self.base_detected = True
                        rospy.loginfo_throttle(10, f"Base detected at: ({self.base_x:.1f}, {self.base_y:.1f})")
        except Exception as e:
            rospy.logwarn(f"Semantic callback error: {e}")

    # ------------------------------------------------------------
    # Backtracking
    # ------------------------------------------------------------
    def initiate_backtrack(self):
        if self.base_x is None or self.base_y is None:
            rospy.logerr("Base position not set. Cannot backtrack.")
            return

        if len(self.path_history) < 2:
            rospy.logwarn("No path history. Returning directly to base.")
            self.send_goal(self.base_x, self.base_y)
            return

        rospy.logwarn("Backtracking initiated.")
        self.backtrack_index = len(self.path_history) - 1
        self.current_goal_reached = True

    def backtrack_timer(self, event):
        if self.mission_mode != "BACKTRACKING":
            return

        if not self.current_goal_reached:
            if 0 <= self.backtrack_index < len(self.path_history):
                waypoint_x, waypoint_y = self.path_history[self.backtrack_index]
                distance_to_waypoint = math.hypot(self.robot_x - waypoint_x, self.robot_y - waypoint_y)

                if distance_to_waypoint < self.goal_reached_distance:
                    rospy.loginfo(f"Reached waypoint {self.backtrack_index}")
                    self.current_goal_reached = True
                    return

                if self.goal_sent_time is not None:
                    elapsed = (rospy.Time.now() - self.goal_sent_time).to_sec()
                    if elapsed > self.goal_timeout:
                        rospy.logwarn("Goal timeout. Moving to next waypoint.")
                        self.current_goal_reached = True
                        return
            return

        self.backtrack_next_waypoint()

    def backtrack_next_waypoint(self):
        self.backtrack_index -= 1

        if self.backtrack_index < 0:
            rospy.logwarn("Backtrack complete. Returning to base.")
            self.send_goal(self.base_x, self.base_y)
            self.mission_mode = "RETURNING_TO_BASE"
            return

        waypoint_x, waypoint_y = self.path_history[self.backtrack_index]
        self.send_goal(waypoint_x, waypoint_y)
        self.current_goal_reached = False
        self.goal_sent_time = rospy.Time.now()

    # ------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------
    def send_goal(self, x, y):
        try:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0

            for _ in range(5):  # Reliable publishing for large rover
                self.goal_pub.publish(goal)
                rospy.sleep(0.1)

            rospy.loginfo(f"Goal sent: ({x:.1f}, {y:.1f})")
        except Exception as e:
            rospy.logerr(f"Goal sending error: {e}")

    # ------------------------------------------------------------
    # Status Publisher
    # ------------------------------------------------------------
    def publish_status(self, event):
        if self.base_x is None or self.base_y is None:
            distance_to_base = 0.0
        else:
            distance_to_base = math.hypot(self.base_x - self.robot_x, self.base_y - self.robot_y)

        status = {
            'battery': self.battery_level,
            'mode': self.mission_mode,
            'base_x': self.base_x or 0.0,
            'base_y': self.base_y or 0.0,
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
    rospy.loginfo("Mission Manager - Large Rover (11m) Mode Active")
    manager = MissionManager()
    rospy.spin()


if __name__ == '__main__':
    main()
