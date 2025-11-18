#!/usr/bin/env python3
"""
Semantic Annotator - Overlays YOLO detections on RTAB-Map
"""

import rospy
import json
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf


class SemanticAnnotator:
    """Annotates RTAB-Map with YOLO detections (rocks, base, flags, antennas)."""

    def __init__(self):
        rospy.loginfo("Semantic Annotator starting")

        # Publishers
        self.semantic_pub = rospy.Publisher('/semantic_map', OccupancyGrid, queue_size=1, latch=True)
        self.semantic_json_pub = rospy.Publisher('/semantic_detections', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.rtabmap_callback)
        rospy.Subscriber('/yolo_detections', String, self.yolo_callback)

        # TF listener for corrected poses
        self.tf_listener = tf.TransformListener()

        # Storage
        self.rtabmap_grid = None
        self.map_info = None
        self.semantic_layers = {'rocks': [], 'base': [], 'flags': [], 'antennas': []}

        # Tuning notes:
        #   duplicate_threshold: distance (m) to consider detections duplicates (increase to be stricter)
        #   est_range: assumed detection distance in front of rover when YOLO gives image coords
        self.duplicate_threshold = 10.0
        self.duplicate_count = 0

        rospy.loginfo("Semantic Annotator ready - listening to RTAB-Map and YOLO")

    def rtabmap_callback(self, msg):
        """Cache RTAB-Map grid and map info."""
        self.rtabmap_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def yolo_callback(self, msg):
        """Process YOLO detections and annotate the map."""
        if self.rtabmap_grid is None:
            return

        try:
            detections = json.loads(msg.data)

            # Get corrected robot pose from TF
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_x, robot_y = trans[0], trans[1]
                _, _, robot_yaw = tf.transformations.euler_from_quaternion(rot)
            except Exception:
                rospy.logwarn_throttle(5, "TF unavailable; skipping YOLO detections")
                return

            for det in detections:
                cls = det.get('class', '').lower()
                conf = det.get('confidence', 0.0)

                # Estimate detection position relative to robot
                est_range = 5.0  # Tuning note: change if detections are closer/farther in your setup
                det_x = robot_x + est_range * np.cos(robot_yaw)
                det_y = robot_y + est_range * np.sin(robot_yaw)

                # Class mapping to map values/sizes (values match occupancy grid coding used elsewhere)
                category, value, size = None, 0, 0
                if 'rock' in cls or 'boulder' in cls or 'objects' in cls:
                    category, value, size = 'rocks', 75, 3
                elif 'base' in cls or 'station' in cls or 'capsule' in cls:
                    category, value, size = 'base', 50, 5
                elif 'flag' in cls or 'marker' in cls:
                    category, value, size = 'flags', 25, 2
                elif 'antenna' in cls or 'tower' in cls:
                    category, value, size = 'antennas', 25, 2

                if not category:
                    continue

                # Duplicate filtering
                if any(self.distance(det_x, det_y, obj['x'], obj['y']) < self.duplicate_threshold
                       for obj in self.semantic_layers[category]):
                    self.duplicate_count += 1
                    continue

                # Store detection
                self.semantic_layers[category].append({
                    'x': det_x, 'y': det_y,
                    'confidence': conf, 'class': cls
                })

            # Publish combined map and JSON detections
            self.publish_semantic_map()
            semantic_json = String(data=json.dumps(self.semantic_layers))
            self.semantic_json_pub.publish(semantic_json)

        except Exception as e:
            rospy.logwarn(f"YOLO processing error: {e}")

    def publish_semantic_map(self):
        """Overlay YOLO annotations on RTAB-Map and publish combined occupancy grid."""
        if self.rtabmap_grid is None or self.map_info is None:
            return

        combined_grid = np.copy(self.rtabmap_grid)

        # Overlay semantic detections
        for category, detections in self.semantic_layers.items():
            value = {'rocks': 75, 'base': 50, 'flags': 25, 'antennas': 25}[category]
            size = {'rocks': 3, 'base': 5, 'flags': 2, 'antennas': 2}[category]

            for det in detections:
                cx, cy = self.world_to_map(det['x'], det['y'])
                for dx in range(-size, size + 1):
                    for dy in range(-size, size + 1):
                        if dx * dx + dy * dy <= size * size:
                            mx, my = cx + dx, cy + dy
                            if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
                                combined_grid[my, mx] = value

        # Publish occupancy grid
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.info = self.map_info
        msg.data = combined_grid.flatten().tolist()
        self.semantic_pub.publish(msg)

        total_detections = sum(len(v) for v in self.semantic_layers.values())
        rospy.loginfo_throttle(10, f"Semantic map published | Detections: {total_detections} | Duplicates: {self.duplicate_count}")

    def world_to_map(self, wx, wy):
        """Convert world coordinates to map grid indices."""
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return mx, my

    def distance(self, x1, y1, x2, y2):
        """Euclidean distance (2D)."""
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


if __name__ == '__main__':
    rospy.init_node('semantic_annotator')
    annotator = SemanticAnnotator()
    rospy.spin()
