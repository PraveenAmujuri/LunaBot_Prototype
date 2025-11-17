#!/usr/bin/env python3
"""
Semantic Annotator - Overlays YOLO detections on RTAB-Map with visual markers
"""

import rospy
import json
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf


class SemanticAnnotator:
    def __init__(self):
        rospy.loginfo("🗺️ Starting Semantic Annotator for RTAB-Map...")
        
        # Publishers
        self.semantic_pub = rospy.Publisher('/semantic_map', OccupancyGrid, queue_size=1, latch=True)
        self.semantic_json_pub = rospy.Publisher('/semantic_detections', String, queue_size=1)
        self.marker_pub = rospy.Publisher('/semantic_markers', MarkerArray, queue_size=1, latch=True)
        
        # Subscribers
        rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.rtabmap_callback)
        rospy.Subscriber('/yolo_detections', String, self.yolo_callback)
        
        # TF listener for corrected poses
        self.tf_listener = tf.TransformListener()
        
        # Storage
        self.rtabmap_grid = None
        self.map_info = None
        self.semantic_layers = {'rocks': [], 'base': [], 'flags': [], 'antennas': []}
        self.duplicate_threshold = 10.0
        self.duplicate_count = 0
        self.marker_id = 0
        
        rospy.loginfo("✅ Semantic Annotator ready - subscribing to RTAB-Map + YOLO")
        
    def rtabmap_callback(self, msg):
        """Store RTAB-Map's base grid"""
        self.rtabmap_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
    def yolo_callback(self, msg):
        """Add YOLO detections as semantic annotations"""
        if self.rtabmap_grid is None:
            return
            
        try:
            detections = json.loads(msg.data)
            
            # Get corrected robot pose from TF (RTAB-Map corrected position)
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_x, robot_y = trans[0], trans[1]
                _, _, robot_yaw = tf.transformations.euler_from_quaternion(rot)
            except:
                rospy.logwarn_throttle(5, "⚠️ TF lookup failed, skipping detections")
                return
            
            for det in detections:
                cls = det.get('class', '').lower()
                conf = det.get('confidence', 0.0)
                
                # Estimate detection position (5m in front of rover)
                est_range = 5.0
                det_x = robot_x + est_range * np.cos(robot_yaw)
                det_y = robot_y + est_range * np.sin(robot_yaw)
                
                # Classify detection - UPDATED to match your YOLO classes
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
                
                # Check for duplicates
                if any(self.distance(det_x, det_y, obj['x'], obj['y']) < self.duplicate_threshold
                       for obj in self.semantic_layers[category]):
                    self.duplicate_count += 1
                    continue
                
                # Store detection
                self.semantic_layers[category].append({
                    'x': det_x, 'y': det_y, 
                    'confidence': conf, 'class': cls
                })
                
            # Publish combined map
            self.publish_semantic_map()
            
            # Publish visual markers
            self.publish_semantic_markers()
            
            # Publish JSON detections for mission manager
            semantic_json = String(data=json.dumps(self.semantic_layers))
            self.semantic_json_pub.publish(semantic_json)
            
        except Exception as e:
            rospy.logwarn(f"❌ YOLO processing error: {e}")
    
    def publish_semantic_map(self):
        """Publish RTAB-Map grid + YOLO semantic annotations"""
        if self.rtabmap_grid is None or self.map_info is None:
            return
        
        # Copy RTAB-Map base grid
        combined_grid = np.copy(self.rtabmap_grid)
        
        # Overlay YOLO detections
        for category, detections in self.semantic_layers.items():
            value = {'rocks': 75, 'base': 50, 'flags': 25, 'antennas': 25}[category]
            size = {'rocks': 3, 'base': 5, 'flags': 2, 'antennas': 2}[category]
            
            for det in detections:
                cx, cy = self.world_to_map(det['x'], det['y'])
                for dx in range(-size, size + 1):
                    for dy in range(-size, size + 1):
                        if dx*dx + dy*dy <= size*size:
                            mx, my = cx + dx, cy + dy
                            if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
                                combined_grid[my, mx] = value
        
        # Publish combined map
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.info = self.map_info
        msg.data = combined_grid.flatten().tolist()
        self.semantic_pub.publish(msg)
        
        total_detections = sum(len(v) for v in self.semantic_layers.values())
        rospy.loginfo_throttle(10, f"🗺️ Semantic map | Detections: {total_detections} | Duplicates filtered: {self.duplicate_count}")
    
    def publish_semantic_markers(self):
        """Publish colored 3D markers for YOLO detections"""
        marker_array = MarkerArray()
        self.marker_id = 0
        
        # Color map for different object types
        colors = {
            'rocks': (1.0, 0.0, 0.0),      # Red
            'base': (0.0, 1.0, 0.0),       # Green
            'flags': (1.0, 1.0, 0.0),      # Yellow
            'antennas': (0.0, 0.5, 1.0)    # Blue
        }
        
        for category, detections in self.semantic_layers.items():
            r, g, b = colors.get(category, (1.0, 1.0, 1.0))
            
            for det in detections:
                # Main object marker
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = category
                marker.id = self.marker_id
                self.marker_id += 1
                
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                marker.pose.position.x = det['x']
                marker.pose.position.y = det['y']
                marker.pose.position.z = 0.5
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.8
                marker.scale.y = 0.8
                marker.scale.z = 1.0
                
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = 0.7
                
                marker.lifetime = rospy.Duration(0)  # Persist forever
                
                # Text label
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.ns = category + "_label"
                text_marker.id = self.marker_id
                self.marker_id += 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose.position.x = det['x']
                text_marker.pose.position.y = det['y']
                text_marker.pose.position.z = 1.8
                
                text_marker.scale.z = 0.4
                
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                text_marker.text = f"{det['class'].upper()}\n{det['confidence']:.2f}"
                text_marker.lifetime = rospy.Duration(0)
                
                marker_array.markers.append(marker)
                marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        rospy.loginfo_throttle(15, f"🏷️ Published {len(marker_array.markers)//2} semantic markers")
    
    def world_to_map(self, wx, wy):
        """Convert world coordinates to map grid coordinates"""
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return mx, my
    
    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


if __name__ == '__main__':
    rospy.init_node('semantic_annotator')
    annotator = SemanticAnnotator()
    rospy.spin()
