#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def publish_static_transforms():
    rospy.init_node('static_tf_publisher')
    
    br = tf2_ros.StaticTransformBroadcaster()
    
    transforms = []
    
    # RGB camera transform
    rgb_tf = TransformStamped()
    rgb_tf.header.stamp = rospy.Time.now()
    rgb_tf.header.frame_id = "base_link"
    rgb_tf.child_frame_id = "camera_color_optical_frame"
    rgb_tf.transform.translation.x = 0.0
    rgb_tf.transform.translation.y = 0.0
    rgb_tf.transform.translation.z = 0.5
    rgb_tf.transform.rotation.x = 0.0
    rgb_tf.transform.rotation.y = 0.0
    rgb_tf.transform.rotation.z = 0.0
    rgb_tf.transform.rotation.w = 1.0
    transforms.append(rgb_tf)
    
    # Depth camera transform
    depth_tf = TransformStamped()
    depth_tf.header.stamp = rospy.Time.now()
    depth_tf.header.frame_id = "base_link"
    depth_tf.child_frame_id = "camera_depth_optical_frame"
    depth_tf.transform.translation.x = 0.0
    depth_tf.transform.translation.y = 0.0
    depth_tf.transform.translation.z = 0.5
    depth_tf.transform.rotation.x = 0.0
    depth_tf.transform.rotation.y = 0.0
    depth_tf.transform.rotation.z = 0.0
    depth_tf.transform.rotation.w = 1.0
    transforms.append(depth_tf)
    
    # LIDAR transform (ADD THIS)
    lidar_tf = TransformStamped()
    lidar_tf.header.stamp = rospy.Time.now()
    lidar_tf.header.frame_id = "base_link"
    lidar_tf.child_frame_id = "lidar_link"
    lidar_tf.transform.translation.x = 0.0
    lidar_tf.transform.translation.y = 0.0
    lidar_tf.transform.translation.z = 0.3  # Lidar is 0.3m above base
    lidar_tf.transform.rotation.x = 0.0
    lidar_tf.transform.rotation.y = 0.0
    lidar_tf.transform.rotation.z = 0.0
    lidar_tf.transform.rotation.w = 1.0
    transforms.append(lidar_tf)
    
    br.sendTransform(transforms)
    
    rospy.loginfo("✅ Static TF transforms published:")
    rospy.loginfo("   base_link -> camera_color_optical_frame")
    rospy.loginfo("   base_link -> camera_depth_optical_frame")
    rospy.loginfo("   base_link -> lidar_link")
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_static_transforms()
    except rospy.ROSInterruptException:
        pass
