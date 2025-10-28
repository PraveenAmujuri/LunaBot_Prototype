#!/usr/bin/env python3
"""
Enhanced SLAM Mapper with YOLO Integration - COMPLETE VERSION WITH DUPLICATE PREVENTION
- Marks detected rocks, base, flags on map with DIFFERENT COLORS
- Tracks path history for backtracking
- PERSISTS map data (doesn't clear on restart)
- Publishes semantic map (obstacles + landmarks)
- Fixed TF broadcasting for RViz visualization
- PREVENTS DUPLICATE DETECTIONS (checks distance before marking)
"""

import rospy
import json
import numpy as np
import os
import pickle
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class EnhancedMapper:
    def __init__(self):
        rospy.loginfo("🗺️  Enhanced SLAM Mapper Starting...")
        
        # Publishers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.semantic_map_pub = rospy.Publisher('/semantic_map', String, queue_size=1)
        self.path_pub = rospy.Publisher('/traversed_path', Path, queue_size=1, latch=True)
        
        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/yolo_detections', String, self.yolo_callback)
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Map parameters (LARGER MAP FOR ±100m range)
        self.map_width = 1200
        self.map_height = 1200
        self.resolution = 0.1  # 10cm per cell
        
        # Map origin (centered)
        self.origin_x = -60.0  # -(1200 * 0.1 / 2)
        self.origin_y = -60.0
        
        # ═══════════════════════════════════════════════════════════
        # MAP PERSISTENCE: Load existing map or create new
        # ═══════════════════════════════════════════════════════════
        self.map_save_path = '/tmp/lunabot_slam_map.pkl'
        self.load_map()  # Try to load existing map
        # ═══════════════════════════════════════════════════════════
        
        # Semantic layer (detected objects)
        self.semantic_map = {
            'rocks': [],      # [{x, y, confidence}, ...]
            'base': [],       # [{x, y, confidence}, ...]
            'flags': [],      # Landmarks
            'antennas': [],   # Landmarks
        }
        
        # ═══════════════════════════════════════════════════════════
        # DUPLICATE DETECTION SETTINGS
        # ═══════════════════════════════════════════════════════════
        self.duplicate_threshold = 10  # meters (objects within this distance are considered same)
        self.duplicate_count = 0  # Track how many duplicates were prevented
        # ═══════════════════════════════════════════════════════════
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        
        # Path tracking
        self.path_history = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        # Auto-reset detection
        self.last_robot_x = 0.0
        self.last_robot_y = 0.0
        self.position_jump_threshold = 5.0  # Increased to 5m to avoid false triggers
        self.first_odom = True
        
        # Update control
        self.last_update = rospy.Time.now()
        self.update_rate = rospy.Duration(0.5)
        
        # Map save timer (save every 30 seconds)
        rospy.Timer(rospy.Duration(30.0), self.save_map_timer)
        
        # TF timer
        rospy.Timer(rospy.Duration(0.05), self.broadcast_tf)
        
        rospy.loginfo("✅ Enhanced Mapper Ready!")
        rospy.loginfo(f"   Map: {self.map_width}x{self.map_height} @ {self.resolution}m/cell")
        rospy.loginfo(f"   Coverage: {self.map_width * self.resolution}m x {self.map_height * self.resolution}m")
        rospy.loginfo(f"   Duplicate detection threshold: {self.duplicate_threshold}m")
        rospy.loginfo(f"   Map saved to: {self.map_save_path}")


    # ═══════════════════════════════════════════════════════════
    # MAP PERSISTENCE FUNCTIONS
    # ═══════════════════════════════════════════════════════════
    
    def load_map(self):
        """Load existing map from disk if available"""
        try:
            if os.path.exists(self.map_save_path):
                with open(self.map_save_path, 'rb') as f:
                    data = pickle.load(f)
                    self.occupancy_grid = data['occupancy_grid']
                    rospy.loginfo(f"✅ Loaded existing map from {self.map_save_path}")
                    rospy.loginfo(f"   Map age: {data.get('timestamp', 'unknown')}")
            else:
                # Create new empty map
                self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
                rospy.loginfo("📝 Created new empty map")
        except Exception as e:
            rospy.logwarn(f"⚠️ Could not load map: {e}")
            self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
    
    
    def save_map_timer(self, event):
        """Auto-save map every 30 seconds"""
        self.save_map()
    
    
    def save_map(self):
        """Save current map to disk"""
        try:
            import time
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
            
            rospy.loginfo_throttle(60, f"💾 Map saved to {self.map_save_path}")
        except Exception as e:
            rospy.logwarn(f"⚠️ Could not save map: {e}")
    
    # ═══════════════════════════════════════════════════════════


    def broadcast_tf(self, event):
        """Broadcast TF transforms"""
        try:
            current_time = rospy.Time.now()
            
            # map -> odom (identity)
            self.tf_broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                current_time,
                "odom",
                "map"
            )
            
            # odom -> base_link (robot position)
            if self.odom_received:
                quat = quaternion_from_euler(0, 0, self.robot_yaw)
                self.tf_broadcaster.sendTransform(
                    (self.robot_x, self.robot_y, 0.0),
                    quat,
                    current_time,
                    "base_link",
                    "odom"
                )
        except Exception as e:
            rospy.logerr(f"❌ TF broadcast error: {e}")


    def odom_callback(self, msg):
        """Track robot position and detect resets"""
        try:
            new_x = msg.pose.pose.position.x
            new_y = msg.pose.pose.position.y
            
            # Detect ONLY large position jumps (Unity hard reset)
            if self.odom_received and not self.first_odom:
                dx = abs(new_x - self.last_robot_x)
                dy = abs(new_y - self.last_robot_y)
                jump = np.sqrt(dx**2 + dy**2)
                
                if jump > self.position_jump_threshold:
                    rospy.logwarn("="*60)
                    rospy.logwarn("🔄 UNITY HARD RESET DETECTED!")
                    rospy.logwarn(f"   Position jump: {jump:.1f}m")
                    rospy.logwarn("   Clearing map and starting fresh")
                    rospy.logwarn("="*60)
                    
                    # Clear everything on hard reset
                    self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
                    self.semantic_map = {'rocks': [], 'base': [], 'flags': [], 'antennas': []}
                    self.path_history = []
                    self.path_msg.poses = []
                    self.duplicate_count = 0
                    self.publish_map()
            
            self.last_robot_x = new_x
            self.last_robot_y = new_y
            self.robot_x = new_x
            self.robot_y = new_y
            
            # Extract yaw
            q = msg.pose.pose.orientation
            _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Add to path history (every 0.5m)
            if len(self.path_history) == 0 or \
               np.sqrt((new_x - self.path_history[-1][0])**2 + 
                      (new_y - self.path_history[-1][1])**2) > 0.5:
                self.path_history.append((new_x, new_y, self.robot_yaw))
                
                # Update path message
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = new_x
                pose.pose.position.y = new_y
                pose.pose.position.z = 0.0
                pose.pose.orientation = msg.pose.pose.orientation
                self.path_msg.poses.append(pose)
                
                self.path_msg.header.stamp = rospy.Time.now()
                self.path_pub.publish(self.path_msg)
            
            self.odom_received = True
            self.first_odom = False
            
        except Exception as e:
            rospy.logwarn(f"⚠️ Odom callback error: {e}")


    # ═══════════════════════════════════════════════════════════
    # YOLO CALLBACK WITH DUPLICATE PREVENTION
    # ═══════════════════════════════════════════════════════════
    
    def yolo_callback(self, msg):
        """Process YOLO detections and mark on map (WITH DUPLICATE PREVENTION)"""
        try:
            detections = json.loads(msg.data)
            
            for det in detections:
                cls = det.get('class', '').lower()
                conf = det.get('confidence', 0.0)
                
                # Estimate world position
                est_range = 5.0
                det_x = self.robot_x + est_range * np.cos(self.robot_yaw)
                det_y = self.robot_y + est_range * np.sin(self.robot_yaw)
                
                # ───────────────────────────────────────────────────
                # DUPLICATE DETECTION PREVENTION
                # Check if this object is already marked nearby
                # ───────────────────────────────────────────────────
                
                is_duplicate = False
                
                if 'rock' in cls or 'boulder' in cls:
                    # Check against existing rocks
                    for existing_rock in self.semantic_map['rocks']:
                        distance = self.calculate_distance(det_x, det_y, existing_rock['x'], existing_rock['y'])
                        
                        if distance < self.duplicate_threshold:
                            is_duplicate = True
                            self.duplicate_count += 1
                            rospy.logdebug(f"🔁 Duplicate rock ignored (distance: {distance:.2f}m)")
                            break
                    
                    if not is_duplicate:
                        detection_entry = {
                            'x': det_x,
                            'y': det_y,
                            'confidence': conf,
                            'class': cls
                        }
                        self.semantic_map['rocks'].append(detection_entry)
                        self.mark_object_on_map(det_x, det_y, 75, size=3)  # 75 = Orange rocks
                        rospy.loginfo(f"🪨 NEW Rock marked at ({det_x:.1f}, {det_y:.1f}) conf={conf:.2f}")
                
                elif 'capsule' in cls or 'base' in cls or 'station' in cls:
                    # Check against existing base detections
                    for existing_base in self.semantic_map['base']:
                        distance = self.calculate_distance(det_x, det_y, existing_base['x'], existing_base['y'])
                        
                        if distance < self.duplicate_threshold:
                            is_duplicate = True
                            self.duplicate_count += 1
                            rospy.logdebug(f"🔁 Duplicate base ignored (distance: {distance:.2f}m)")
                            break
                    
                    if not is_duplicate:
                        detection_entry = {
                            'x': det_x,
                            'y': det_y,
                            'confidence': conf,
                            'class': cls
                        }
                        self.semantic_map['base'].append(detection_entry)
                        self.mark_object_on_map(det_x, det_y, 50, size=5)  # 50 = Green base
                        rospy.loginfo(f"🏠 NEW Base marked at ({det_x:.1f}, {det_y:.1f}) conf={conf:.2f}")
                
                elif 'flag' in cls or 'marker' in cls:
                    # Check against existing flags
                    for existing_flag in self.semantic_map['flags']:
                        distance = self.calculate_distance(det_x, det_y, existing_flag['x'], existing_flag['y'])
                        
                        if distance < self.duplicate_threshold:
                            is_duplicate = True
                            self.duplicate_count += 1
                            rospy.logdebug(f"🔁 Duplicate flag ignored (distance: {distance:.2f}m)")
                            break
                    
                    if not is_duplicate:
                        detection_entry = {
                            'x': det_x,
                            'y': det_y,
                            'confidence': conf,
                            'class': cls
                        }
                        self.semantic_map['flags'].append(detection_entry)
                        self.mark_object_on_map(det_x, det_y, 25, size=2)  # 25 = Flag marker
                        rospy.loginfo(f"🚩 NEW Flag marked at ({det_x:.1f}, {det_y:.1f}) conf={conf:.2f}")
                
                elif 'antenna' in cls or 'tower' in cls:
                    # Check against existing antennas
                    for existing_antenna in self.semantic_map['antennas']:
                        distance = self.calculate_distance(det_x, det_y, existing_antenna['x'], existing_antenna['y'])
                        
                        if distance < self.duplicate_threshold:
                            is_duplicate = True
                            self.duplicate_count += 1
                            rospy.logdebug(f"🔁 Duplicate antenna ignored (distance: {distance:.2f}m)")
                            break
                    
                    if not is_duplicate:
                        detection_entry = {
                            'x': det_x,
                            'y': det_y,
                            'confidence': conf,
                            'class': cls
                        }
                        self.semantic_map['antennas'].append(detection_entry)
                        self.mark_object_on_map(det_x, det_y, 25, size=2)
                        rospy.loginfo(f"📡 NEW Antenna marked at ({det_x:.1f}, {det_y:.1f}) conf={conf:.2f}")
            
            # Publish semantic map (only after checking all detections)
            semantic_msg = String()
            semantic_msg.data = json.dumps(self.semantic_map)
            self.semantic_map_pub.publish(semantic_msg)
            
        except Exception as e:
            rospy.logwarn(f"⚠️ YOLO callback error: {e}")
    
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        dx = x2 - x1
        dy = y2 - y1
        return np.sqrt(dx**2 + dy**2)
    
    # ═══════════════════════════════════════════════════════════


    def mark_object_on_map(self, world_x, world_y, value, size=3):
        """
        Mark a detected object on the occupancy grid
        
        Args:
            world_x, world_y: World coordinates
            value: Map cell value (50=base, 75=rock, 25=flag, 100=obstacle)
            size: Radius in cells (3 = ~30cm radius for rocks)
        """
        try:
            center_x, center_y = self.world_to_map(world_x, world_y)
            
            # Mark a circular area
            for dx in range(-size, size + 1):
                for dy in range(-size, size + 1):
                    if dx*dx + dy*dy <= size*size:  # Circle check
                        mx = center_x + dx
                        my = center_y + dy
                        
                        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                            self.occupancy_grid[my, mx] = value
                            
        except Exception as e:
            rospy.logwarn(f"⚠️ Mark object error: {e}")


    def scan_callback(self, msg):
        """Process laser scan and update occupancy grid"""
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
            rospy.logerr(f"❌ Scan callback error: {e}")


    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map grid coordinates"""
        map_x = int((world_x - self.origin_x) / self.resolution)
        map_y = int((world_y - self.origin_y) / self.resolution)
        return map_x, map_y


    def update_map_from_scan(self, scan_msg):
        """Update occupancy grid from laser scan data"""
        robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)
        
        if not (0 <= robot_map_x < self.map_width and 0 <= robot_map_y < self.map_height):
            rospy.logwarn_throttle(5, f"⚠️ Robot out of map bounds: ({robot_map_x}, {robot_map_y})")
            return
        
        angle = scan_msg.angle_min
        
        for range_val in scan_msg.ranges:
            if scan_msg.range_min < range_val < scan_msg.range_max:
                obs_x = self.robot_x + range_val * np.cos(angle + self.robot_yaw)
                obs_y = self.robot_y + range_val * np.sin(angle + self.robot_yaw)
                
                obs_map_x, obs_map_y = self.world_to_map(obs_x, obs_y)
                
                if 0 <= obs_map_x < self.map_width and 0 <= obs_map_y < self.map_height:
                    # Don't overwrite YOLO detections (50, 75, 25)
                    current_value = self.occupancy_grid[obs_map_y, obs_map_x]
                    if current_value not in [50, 75, 25]:
                        self.occupancy_grid[obs_map_y, obs_map_x] = 100  # Obstacle
                    
                    self.mark_free_line(robot_map_x, robot_map_y, obs_map_x, obs_map_y)
            
            angle += scan_msg.angle_increment


    def mark_free_line(self, x0, y0, x1, y1):
        """Mark free space using Bresenham's algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while x != x1 or y != y1:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                # Only mark as free if unknown AND not a YOLO detection
                current_value = self.occupancy_grid[y, x]
                if current_value == -1:  # Unknown
                    self.occupancy_grid[y, x] = 0  # Free space
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy


    def get_detection_stats(self):
        """Get detection statistics"""
        return {
            'total_rocks': len(self.semantic_map['rocks']),
            'total_base': len(self.semantic_map['base']),
            'total_flags': len(self.semantic_map['flags']),
            'total_antennas': len(self.semantic_map['antennas']),
            'total_detections': sum([
                len(self.semantic_map['rocks']),
                len(self.semantic_map['base']),
                len(self.semantic_map['flags']),
                len(self.semantic_map['antennas'])
            ]),
            'duplicates_prevented': self.duplicate_count
        }


    def publish_map(self):
        """Publish occupancy grid map"""
        try:
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = rospy.Time.now()
            grid_msg.header.frame_id = 'map'
            
            grid_msg.info.resolution = self.resolution
            grid_msg.info.width = self.map_width
            grid_msg.info.height = self.map_height
            grid_msg.info.origin.position.x = self.origin_x
            grid_msg.info.origin.position.y = self.origin_y
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0
            
            grid_msg.data = self.occupancy_grid.flatten().tolist()
            
            self.map_pub.publish(grid_msg)
            
            # Get statistics
            stats = self.get_detection_stats()
            
            rospy.loginfo_throttle(10, 
                f"🗺️ Map | Rover: ({self.robot_x:.1f}, {self.robot_y:.1f}) | " +
                f"Path: {len(self.path_history)} pts | " +
                f"Detections: {stats['total_detections']} " +
                f"(Rocks: {stats['total_rocks']}, Base: {stats['total_base']}) | " +
                f"Duplicates prevented: {stats['duplicates_prevented']}")
            
        except Exception as e:
            rospy.logerr(f"❌ Map publish error: {e}")


    def shutdown(self):
        """Save map on shutdown"""
        rospy.loginfo("💾 Saving map before shutdown...")
        self.save_map()
        
        stats = self.get_detection_stats()
        rospy.loginfo("=" * 60)
        rospy.loginfo("📊 FINAL STATISTICS:")
        rospy.loginfo(f"   Total rocks detected: {stats['total_rocks']}")
        rospy.loginfo(f"   Total base detections: {stats['total_base']}")
        rospy.loginfo(f"   Total flags detected: {stats['total_flags']}")
        rospy.loginfo(f"   Total antennas detected: {stats['total_antennas']}")
        rospy.loginfo(f"   Duplicate detections prevented: {stats['duplicates_prevented']}")
        rospy.loginfo(f"   Path waypoints: {len(self.path_history)}")
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ Map saved successfully!")


def main():
    rospy.init_node('enhanced_mapper', anonymous=False)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("🗺️  ENHANCED SLAM MAPPER WITH DUPLICATE PREVENTION")
    rospy.loginfo("=" * 60)
    rospy.loginfo("✅ Builds persistent occupancy grid map")
    rospy.loginfo("✅ Marks YOLO detections on map:")
    rospy.loginfo("   🪨 Rocks = Orange (value 75)")
    rospy.loginfo("   🏠 Base = Green (value 50)")
    rospy.loginfo("   🚩 Flags = Markers (value 25)")
    rospy.loginfo("✅ Prevents duplicate detections (1.5m threshold)")
    rospy.loginfo("✅ Tracks path history for backtracking")
    rospy.loginfo("✅ Map persists between runs")
    rospy.loginfo("✅ Auto-saves every 30 seconds")
    rospy.loginfo("=" * 60)
    
    try:
        mapper = EnhancedMapper()
        rospy.on_shutdown(mapper.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Mapper shutting down")
    except Exception as e:
        rospy.logerr(f"❌ Fatal error: {e}")


if __name__ == '__main__':
    main()
