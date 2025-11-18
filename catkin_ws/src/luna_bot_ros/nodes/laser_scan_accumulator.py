#!/usr/bin/env python3
"""
LaserScan Accumulator
- Collects all LaserScan points in global frame
- Produces a full exploration coverage point cloud
"""

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import struct


class LaserScanAccumulator:
    def __init__(self):
        rospy.loginfo("LaserScan Accumulator starting...")

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Storage for accumulated points
        self.points = []

        self.max_points = 1000000  
        # Tuning note:
        #   Increase for higher resolution maps
        #   Lower if memory usage needs to be reduced

        # Publisher
        self.cloud_pub = rospy.Publisher(
            '/accumulated_laser_scan',
            PointCloud2,
            queue_size=1,
            latch=True
        )

        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publish every 1 second
        rospy.Timer(rospy.Duration(1.0), self.publish_cloud)

        self.frame_count = 0
        rospy.loginfo("Subscribed to /scan")


    def scan_callback(self, scan_msg):
        """Accumulate transformed LaserScan points into global frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                rospy.Duration(0.5)   # Tuning note: increase if TF is delayed
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:

            rospy.logwarn_throttle(5, f"TF lookup failed: {e}")
            return

        # Translation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # Rotation → yaw
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Walk through laser rays
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:

            if scan_msg.range_min < r < scan_msg.range_max:

                # Convert to laser frame
                lx = r * np.cos(angle)
                ly = r * np.sin(angle)

                # Rotate + translate into map frame
                mx = tx + lx * np.cos(yaw) - ly * np.sin(yaw)
                my = ty + lx * np.sin(yaw) + ly * np.cos(yaw)
                mz = tz

                # Simple intensity falloff
                intensity = int(255 * (1.0 - r / scan_msg.range_max))

                self.points.append((mx, my, mz, intensity))

            angle += scan_msg.angle_increment

        # Keep buffer within max size
        if len(self.points) > self.max_points:
            self.points = self.points[-self.max_points:]

        self.frame_count += 1
        if self.frame_count % 10 == 0:
            rospy.loginfo(f"Accumulated {len(self.points):,} points")


    def publish_cloud(self, event=None):
        """Publish accumulated point cloud."""
        if len(self.points) == 0:
            return

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.UINT8, 1)
        ]

        cloud_data = [
            struct.pack('fffB', x, y, z, intensity)
            for x, y, z, intensity in self.points
        ]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(self.points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 13
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join(cloud_data)

        rospy.loginfo_throttle(30, f"Publishing {len(self.points):,} accumulated points")
        self.cloud_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('laser_scan_accumulator')
    LaserScanAccumulator()
    rospy.spin()
