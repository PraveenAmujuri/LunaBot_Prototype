#!/usr/bin/env python3
"""
Enhanced SLAM Mapper with YOLO Integration
------------------------------------------
Features:
- Marks detected rocks, base, flags, antennas on map with unique values
- Tracks path history for backtracking
- Persists map between runs
- Publishes semantic map (obstacles + landmarks)
- TF broadcasting for RViz
- Duplicate detection prevention
"""

import rospy
import json
import numpy as np
import os
import pickle
import time
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class EnhancedMapper:
    def __init__(self):
        rospy.loginfo("Starting Enhanced SLAM Mapper...")

        # Publishers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.semantic_map_pub = rospy.Publisher('/semantic_map', String, queue_size=1)
        self.path_pub = rospy.Publisher('/traversed_path', Path, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/yolo_detections', String, self.yolo_callback)

        self.tf_broadcaster = tf.TransformBroadcaster()

        # Map configuration
        self.map_width = 1200
        self.map_height = 1200
        self.resolution = 0.1  # meters per cell
        self.origin_x = -60.0
        self.origin_y = -60.0
        self.map_save_path = '/tmp/lunabot_slam_map.pkl'

        # Load or initialize map
        self.occupancy_grid = self.load_map()

        # Semantic layers
        self.semantic_map = {'rocks': [], 'base': [], 'flags': [], 'antennas': []}
        self.duplicate_threshold = 10.0
        self.duplicate_count = 0

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.first_odom = True
        self.last_robot_x = 0.0
        self.last_robot_y = 0.0
        self.position_jump_threshold = 5.0

        # Path
        self.path_history = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # Timers
        rospy.Timer(rospy.Duration(0.05), self.broadcast_tf)
        rospy.Timer(rospy.Duration(30.0), self.save_map_timer)
        self.update_rate = rospy.Duration(0.5)
        self.last_update = rospy.Time.now()

        rospy.loginfo(f"Mapper ready | Map: {self.map_width}x{self.map_height} @ {self.resolution} m/cell")
        rospy.loginfo(f"Duplicate threshold: {self.duplicate_threshold} m | Map path: {self.map_save_path}")

    # --------------------------------------------------------------------------
    # Map persistence
    # --------------------------------------------------------------------------

    def load_map(self):
        if os.path.exists(self.map_save_path):
            try:
                with open(self.map_save_path, 'rb') as f:
                    data = pickle.load(f)
                    rospy.loginfo(f"Loaded existing map from {self.map_save_path} (age: {data.get('timestamp', 'unknown')})")
                    return data['occupancy_grid']
            except Exception as e:
                rospy.logwarn(f"Could not load map: {e}")
        rospy.loginfo("Initialized new empty map.")
        return np.full((self.map_height, self.map_width), -1, dtype=np.int8)

    def save_map(self):
        try:
            data = {
                'occupancy_grid': self.occupancy_grid,
                'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                'map_width': self.map_width,
                'map_height': self.map_height,
                'resolution': self.resolution,
                'origin_x': self.origin_x,
                'origin_y': self.origin_y
            }
            with open(self.map_save_path, 'wb') as f:
                pickle.dump(data, f)
            rospy.loginfo_throttle(60, f"Map saved to {self.map_save_path}")
        except Exception as e:
            rospy.logwarn(f"Map save error: {e}")

    def save_map_timer(self, _):
        self.save_map()

    # --------------------------------------------------------------------------
    # TF broadcasting
    # --------------------------------------------------------------------------

    def broadcast_tf(self, _):
        try:
            now = rospy.Time.now()
            self.tf_broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), now, "odom", "map")
            if self.odom_received:
                quat = quaternion_from_euler(0, 0, self.robot_yaw)
                self.tf_broadcaster.sendTransform(
                    (self.robot_x, self.robot_y, 0), quat, now, "base_link", "odom"
                )
        except Exception as e:
            rospy.logerr(f"TF broadcast error: {e}")

    # --------------------------------------------------------------------------
    # Odometry and path
    # --------------------------------------------------------------------------

    def odom_callback(self, msg):
        try:
            new_x, new_y = msg.pose.pose.position.x, msg.pose.pose.position.y
            dx, dy = abs(new_x - self.last_robot_x), abs(new_y - self.last_robot_y)
            jump = np.sqrt(dx**2 + dy**2)

            if self.odom_received and not self.first_odom and jump > self.position_jump_threshold:
                rospy.logwarn("Unity reset detected. Clearing map.")
                self.occupancy_grid.fill(-1)
                self.semantic_map = {'rocks': [], 'base': [], 'flags': [], 'antennas': []}
                self.path_history.clear()
                self.path_msg.poses.clear()
                self.duplicate_count = 0
                self.publish_map()

            q = msg.pose.pose.orientation
            _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.robot_x, self.robot_y = new_x, new_y
            self.last_robot_x, self.last_robot_y = new_x, new_y
            self.odom_received = True
            self.first_odom = False

            # Update path every 0.5m
            if not self.path_history or np.hypot(new_x - self.path_history[-1][0], new_y - self.path_history[-1][1]) > 0.5:
                self.path_history.append((new_x, new_y, self.robot_yaw))
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = new_x
                pose.pose.position.y = new_y
                pose.pose.orientation = msg.pose.pose.orientation
                self.path_msg.poses.append(pose)
                self.path_msg.header.stamp = rospy.Time.now()
                self.path_pub.publish(self.path_msg)

        except Exception as e:
            rospy.logwarn(f"Odom callback error: {e}")

    # --------------------------------------------------------------------------
    # YOLO integration
    # --------------------------------------------------------------------------

    def yolo_callback(self, msg):
        try:
            detections = json.loads(msg.data)
            for det in detections:
                cls, conf = det.get('class', '').lower(), det.get('confidence', 0.0)
                est_range = 5.0
                det_x = self.robot_x + est_range * np.cos(self.robot_yaw)
                det_y = self.robot_y + est_range * np.sin(self.robot_yaw)

                category = None
                value = 0
                size = 0

                if 'rock' in cls or 'boulder' in cls:
                    category, value, size = 'rocks', 75, 3
                elif 'base' in cls or 'station' in cls:
                    category, value, size = 'base', 50, 5
                elif 'flag' in cls or 'marker' in cls:
                    category, value, size = 'flags', 25, 2
                elif 'antenna' in cls or 'tower' in cls:
                    category, value, size = 'antennas', 25, 2

                if not category:
                    continue

                if any(
                    self.calculate_distance(det_x, det_y, obj['x'], obj['y']) < self.duplicate_threshold
                    for obj in self.semantic_map[category]
                ):
                    self.duplicate_count += 1
                    continue

                self.semantic_map[category].append({'x': det_x, 'y': det_y, 'confidence': conf, 'class': cls})
                self.mark_object_on_map(det_x, det_y, value, size)

            semantic_msg = String(data=json.dumps(self.semantic_map))
            self.semantic_map_pub.publish(semantic_msg)

        except Exception as e:
            rospy.logwarn(f"YOLO callback error: {e}")

    # --------------------------------------------------------------------------
    # Map updates
    # --------------------------------------------------------------------------

    def scan_callback(self, msg):
        if not self.odom_received:
            return
        now = rospy.Time.now()
        if (now - self.last_update) < self.update_rate:
            return
        self.last_update = now

        try:
            self.update_map_from_scan(msg)
            self.publish_map()
        except Exception as e:
            rospy.logerr(f"Scan callback error: {e}")

    def update_map_from_scan(self, scan):
        robot_x, robot_y = self.world_to_map(self.robot_x, self.robot_y)
        if not (0 <= robot_x < self.map_width and 0 <= robot_y < self.map_height):
            rospy.logwarn_throttle(5, f"Robot out of map bounds: ({robot_x}, {robot_y})")
            return

        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                obs_x = self.robot_x + r * np.cos(angle + self.robot_yaw)
                obs_y = self.robot_y + r * np.sin(angle + self.robot_yaw)
                mx, my = self.world_to_map(obs_x, obs_y)
                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    if self.occupancy_grid[my, mx] not in [25, 50, 75]:
                        self.occupancy_grid[my, mx] = 100
                    self.mark_free_line(robot_x, robot_y, mx, my)
            angle += scan.angle_increment

    def mark_free_line(self, x0, y0, x1, y1):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
        err = dx - dy
        while x0 != x1 or y0 != y1:
            if 0 <= x0 < self.map_width and 0 <= y0 < self.map_height:
                if self.occupancy_grid[y0, x0] == -1:
                    self.occupancy_grid[y0, x0] = 0
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def mark_object_on_map(self, wx, wy, value, size=3):
        try:
            cx, cy = self.world_to_map(wx, wy)
            for dx in range(-size, size + 1):
                for dy in range(-size, size + 1):
                    if dx * dx + dy * dy <= size * size:
                        mx, my = cx + dx, cy + dy
                        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                            self.occupancy_grid[my, mx] = value
        except Exception as e:
            rospy.logwarn(f"Mark object error: {e}")

    def world_to_map(self, wx, wy):
        return int((wx - self.origin_x) / self.resolution), int((wy - self.origin_y) / self.resolution)

    def calculate_distance(self, x1, y1, x2, y2):
        return np.hypot(x2 - x1, y2 - y1)

    def publish_map(self):
        try:
            grid = OccupancyGrid()
            grid.header.stamp = rospy.Time.now()
            grid.header.frame_id = 'map'
            grid.info.resolution = self.resolution
            grid.info.width = self.map_width
            grid.info.height = self.map_height
            grid.info.origin.position.x = self.origin_x
            grid.info.origin.position.y = self.origin_y
            grid.info.origin.orientation.w = 1.0
            grid.data = self.occupancy_grid.flatten().tolist()
            self.map_pub.publish(grid)

            stats = self.get_detection_stats()
            rospy.loginfo_throttle(
                10,
                f"Map | Pose=({self.robot_x:.1f},{self.robot_y:.1f}) | Path={len(self.path_history)} | "
                f"Detections={stats['total_detections']} | Duplicates={stats['duplicates_prevented']}"
            )
        except Exception as e:
            rospy.logerr(f"Map publish error: {e}")

    def get_detection_stats(self):
        total = sum(len(v) for v in self.semantic_map.values())
        return {
            'total_rocks': len(self.semantic_map['rocks']),
            'total_base': len(self.semantic_map['base']),
            'total_flags': len(self.semantic_map['flags']),
            'total_antennas': len(self.semantic_map['antennas']),
            'total_detections': total,
            'duplicates_prevented': self.duplicate_count
        }

    def shutdown(self):
        rospy.loginfo("Saving map before shutdown...")
        self.save_map()
        stats = self.get_detection_stats()
        rospy.loginfo(f"Final stats: {stats}")


def main():
    rospy.init_node('enhanced_mapper', anonymous=False)
    try:
        mapper = EnhancedMapper()
        rospy.on_shutdown(mapper.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mapper shutting down.")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")


if __name__ == '__main__':
    main()
