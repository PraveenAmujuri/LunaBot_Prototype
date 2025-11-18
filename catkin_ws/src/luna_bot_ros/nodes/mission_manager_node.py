#!/usr/bin/env python3
"""
Mission Manager - Large Rover Configuration (11m)
Coordinates backtracking, waypoint recording, base detection,
and emergency return-to-base behavior.
"""

import rospy
import json
import math
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalID, GoalStatusArray


class MissionManager:
    """Coordinates mission logic: exploration, base detection, and backtracking."""

    def __init__(self):
        rospy.loginfo("Mission Manager starting")

        # Publishers
        self.status_pub = rospy.Publisher('/mission_status', String, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Subscribers
        rospy.Subscriber('/battery_level', Float32, self.battery_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/semantic_detections', String, self.semantic_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)

        # Mission state
        self.battery_level = 100.0
        self.low_battery_threshold = 10.0  # (Tuning note: increase if rover carries heavy payload)
        self.mission_mode = "EXPLORATION"

        # Base position
        self.base_x = None
        self.base_y = None
        self.base_detected = False
        self.first_position_recorded = False

        # Rover pose
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Rover parameters
        self.rover_length = 11.0
        self.path_spacing = 10.0        # (Tuning note: decrease for tighter coverage patterns)
        self.goal_reached_distance = 6.0  # (Tuning note: adjust for rover turning radius)
        self.goal_timeout = 60.0        # (Tuning note: increase in uneven terrain)

        # Path history
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

        rospy.loginfo("Mission Manager ready")
        rospy.loginfo(f"Goal radius: {self.goal_reached_distance} m")
        rospy.loginfo(f"Waypoint spacing: {self.path_spacing} m")
        rospy.loginfo(f"Goal timeout: {self.goal_timeout} s")

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def battery_callback(self, msg):
        """Monitor battery and trigger emergency backtracking if needed."""
        self.battery_level = msg.data

        if self.battery_level < self.low_battery_threshold and not self.emergency_triggered:
            rospy.logwarn("Battery low — switching to backtracking mode")
            self.emergency_triggered = True
            self.mission_mode = "BACKTRACKING"
            self.initiate_backtrack()

    def odom_callback(self, msg):
        """Record rover pose and store path waypoints."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # First recorded position becomes "base"
        if not self.first_position_recorded:
            self.base_x = self.robot_x
            self.base_y = self.robot_y
            self.first_position_recorded = True
            rospy.loginfo(f"Base position set: ({self.base_x:.1f}, {self.base_y:.1f})")

        # Add waypoints as rover moves
        if self.last_path_x is None:
            self.path_history.append((self.robot_x, self.robot_y))
            self.last_path_x = self.robot_x
            self.last_path_y = self.robot_y
        else:
            distance = math.hypot(self.robot_x - self.last_path_x,
                                  self.robot_y - self.last_path_y)

            if distance >= self.path_spacing:
                self.path_history.append((self.robot_x, self.robot_y))
                self.last_path_x = self.robot_x
                self.last_path_y = self.robot_y
                rospy.loginfo(f"Waypoint added ({len(self.path_history)} total)")

    def goal_status_callback(self, msg):
        """Confirm goal completion."""
        if len(msg.status_list) > 0 and msg.status_list[-1].status == 3:
            rospy.loginfo("Goal reached")
            self.current_goal_reached = True

    def semantic_callback(self, msg):
        """Update base position if detected by YOLO."""
        try:
            semantic_map = json.loads(msg.data)
            base_items = semantic_map.get('base', [])

            if len(base_items) == 0:
                return

            det = base_items[0]
            new_base_x, new_base_y = det['x'], det['y']

            # Accept detections near the originally recorded base
            if self.base_x is not None:
                offset = math.hypot(new_base_x - self.base_x,
                                    new_base_y - self.base_y)

                if offset < 15.0:  # (Tuning note: adjust based on YOLO stability)
                    self.base_x = new_base_x
                    self.base_y = new_base_y
                    self.base_detected = True
                    rospy.loginfo_throttle(10, f"Base detected at ({self.base_x:.1f}, {self.base_y:.1f})")

        except Exception as e:
            rospy.logwarn(f"Semantic callback error: {e}")

    # ---------------------------------------------------------------------
    # Backtracking Logic
    # ---------------------------------------------------------------------
    def initiate_backtrack(self):
        """Prepare to walk the recorded path in reverse."""
        if self.base_x is None:
            rospy.logerr("Base not set — cannot backtrack")
            return

        if len(self.path_history) < 2:
            rospy.logwarn("Insufficient path data — sending direct goal to base")
            self.send_goal(self.base_x, self.base_y)
            return

        rospy.logwarn("Backtracking started")
        self.backtrack_index = len(self.path_history) - 1
        self.current_goal_reached = True

    def backtrack_timer(self, event):
        """Handle navigation between backtrack waypoints."""
        if self.mission_mode != "BACKTRACKING":
            return

        if not self.current_goal_reached:
            if 0 <= self.backtrack_index < len(self.path_history):
                wx, wy = self.path_history[self.backtrack_index]
                distance = math.hypot(self.robot_x - wx, self.robot_y - wy)

                # Check arrival
                if distance < self.goal_reached_distance:
                    rospy.loginfo(f"Reached waypoint {self.backtrack_index}")
                    self.current_goal_reached = True
                    return

                # Timeout handling
                if self.goal_sent_time:
                    elapsed = (rospy.Time.now() - self.goal_sent_time).to_sec()
                    if elapsed > self.goal_timeout:
                        rospy.logwarn("Goal timeout — moving to next waypoint")
                        self.current_goal_reached = True
                        return
            return

        self.backtrack_next_waypoint()

    def backtrack_next_waypoint(self):
        """Move to the previous waypoint in the path history."""
        self.backtrack_index -= 1

        if self.backtrack_index < 0:
            rospy.logwarn("Backtracking complete — returning to base")
            self.send_goal(self.base_x, self.base_y)
            self.mission_mode = "RETURNING_TO_BASE"
            return

        wx, wy = self.path_history[self.backtrack_index]
        self.send_goal(wx, wy)
        self.current_goal_reached = False
        self.goal_sent_time = rospy.Time.now()

    # ---------------------------------------------------------------------
    # Goal Publishing
    # ---------------------------------------------------------------------
    def send_goal(self, x, y):
        """Send a simple goal to move_base (reliable multi-publish)."""
        try:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0

            # Publish several times to ensure move_base receives it
            for _ in range(5):
                self.goal_pub.publish(goal)
                rospy.sleep(0.1)

            rospy.loginfo(f"Goal sent: ({x:.1f}, {y:.1f})")
        except Exception as e:
            rospy.logerr(f"Goal send error: {e}")

    # ---------------------------------------------------------------------
    # Status Reporting
    # ---------------------------------------------------------------------
    def publish_status(self, event):
        """Publish JSON mission state for the dashboard."""
        if self.base_x is None:
            distance_to_base = 0.0
        else:
            distance_to_base = math.hypot(self.base_x - self.robot_x,
                                          self.base_y - self.robot_y)

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
            'backtrack_progress': (
                f"{self.backtrack_index}/{len(self.path_history)-1}"
                if self.mission_mode == "BACKTRACKING" else "N/A"
            ),
            'emergency_triggered': self.emergency_triggered,
            'rover_length': self.rover_length
        }

        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rospy.init_node('mission_manager')
    rospy.loginfo("Mission Manager loaded")
    MissionManager()
    rospy.spin()


if __name__ == '__main__':
    main()
