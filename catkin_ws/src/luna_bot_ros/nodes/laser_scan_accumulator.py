#!/usr/bin/env python3
"""
LaserScan Accumulator - Shows complete exploration coverage
Accumulates all laser scan data to visualize full mapped area
"""

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import struct


class LaserScanAccumulator:
    def __init__(self):
        rospy.loginfo("🔦 Starting LaserScan Accumulator...")
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Storage for accumulated points
        self.points = []
        self.max_points = 1000000  # Store up to 1 million points
        
        # Publisher
        self.cloud_pub = rospy.Publisher(
            '/accumulated_laser_scan', 
            PointCloud2, 
            queue_size=1, 
            latch=True
        )
        
        # Subscribe to laser scan
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Publish timer (update visualization every second)
        rospy.Timer(rospy.Duration(1.0), self.publish_cloud)
        
        self.frame_count = 0
        rospy.loginfo("✅ LaserScan Accumulator ready - subscribing to /scan")
    
    def scan_callback(self, scan_msg):
        """Accumulate laser scan into global map"""
        try:
            # Get transform from laser frame to map
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                scan_msg.header.frame_id,  # Source frame
                scan_msg.header.stamp,
                rospy.Duration(0.5)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"⚠️ TF lookup failed: {e}")
            return
        
        # Extract transform
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        
        # Extract yaw from quaternion
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        # Convert quaternion to yaw (2D rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Process each laser reading
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            # Filter valid ranges
            if scan_msg.range_min < r < scan_msg.range_max:
                # Calculate point in laser frame
                lx = r * np.cos(angle)
                ly = r * np.sin(angle)
                
                # Transform to map frame
                mx = tx + lx * np.cos(yaw) - ly * np.sin(yaw)
                my = ty + lx * np.sin(yaw) + ly * np.cos(yaw)
                mz = tz
                
                # Store point (x, y, z, intensity)
                intensity = int(255 * (1.0 - r / scan_msg.range_max))
                self.points.append((mx, my, mz, intensity))
            
            angle += scan_msg.angle_increment
        
        # Limit total points (keep most recent)
        if len(self.points) > self.max_points:
            self.points = self.points[-self.max_points:]
        
        self.frame_count += 1
        if self.frame_count % 10 == 0:
            rospy.loginfo(f"📊 Accumulated {len(self.points):,} laser points from {self.frame_count} scans")
    
    def publish_cloud(self, event=None):
        """Publish accumulated point cloud"""
        if len(self.points) == 0:
            return
        
        # Create PointCloud2 header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        
        # Define point fields (x, y, z, intensity)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.UINT8, 1),
        ]
        
        # Pack points into binary data
        cloud_data = []
        for x, y, z, intensity in self.points:
            # Pack: 3 floats + 1 uint8 = 13 bytes per point
            cloud_data.append(struct.pack('fffB', x, y, z, intensity))
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(self.points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 13  # bytes per point
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join(cloud_data)
        
        self.cloud_pub.publish(msg)
        rospy.loginfo_throttle(30, f"🗺️ Publishing {len(self.points):,} accumulated laser scan points")


if __name__ == '__main__':
    rospy.init_node('laser_scan_accumulator')
    accumulator = LaserScanAccumulator()
    rospy.spin()
