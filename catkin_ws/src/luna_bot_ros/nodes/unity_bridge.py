#!/usr/bin/env python
import rospy
import json
import threading
import time
from websocket import create_connection
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class UnityBridge:
    def __init__(self):
        rospy.init_node('unity_bridge', anonymous=True)
        
        # ROS Subscribers (ROS → Unity)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # ROS Publishers (Unity → ROS)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.unity_hazard_pub = rospy.Publisher('/unity_hazard', String, queue_size=10)
        
        # ROSBridge connection
        self.ws_url = "ws://172.22.54.111:9090"  # Your WSL IP
        self.ws = None
        self.connected = False
        
        # Start connection
        self.connect_to_unity()
        
        rospy.loginfo("🌉 Unity Bridge started - connecting ROS nodes to Unity")

    def connect_to_unity(self):
        """Connect to Unity ROSBridge"""
        max_retries = 5
        retry_count = 0
        
        while retry_count < max_retries and not rospy.is_shutdown():
            try:
                rospy.loginfo(f"🔄 Connecting to Unity ROSBridge (attempt {retry_count + 1}/{max_retries})")
                self.ws = create_connection(self.ws_url, timeout=5)
                self.connected = True
                rospy.loginfo("✅ Connected to Unity ROSBridge")
                
                # Subscribe to Unity topics
                self.ws.send(json.dumps({"op": "subscribe", "topic": "/odom"}))
                self.ws.send(json.dumps({"op": "subscribe", "topic": "/hazard"}))
                
                # Start listening in separate thread
                listen_thread = threading.Thread(target=self.listen_to_unity)
                listen_thread.daemon = True
                listen_thread.start()
                break
                
            except Exception as e:
                retry_count += 1
                rospy.logwarn(f"❌ Connection failed: {e}")
                if retry_count < max_retries:
                    rospy.loginfo("⏳ Retrying in 2 seconds...")
                    time.sleep(2)
                else:
                    rospy.logerr("❌ Failed to connect to Unity after all attempts")

    def cmd_vel_callback(self, msg):
        """Forward ROS cmd_vel to Unity rover"""
        if self.connected and self.ws:
            try:
                # FIXED: Send directly to /cmd_vel (Unity listens to this)
                unity_cmd = {
                    "op": "publish",
                    "topic": "/cmd_vel",
                    "msg": {
                        "linear": {"x": msg.linear.x, "y": 0, "z": 0},
                        "angular": {"x": 0, "y": 0, "z": msg.angular.z}
                    }
                }
                
                self.ws.send(json.dumps(unity_cmd))
                rospy.loginfo(f"📤 Bridge sent to Unity /cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
                
            except Exception as e:
                rospy.logwarn(f"❌ Failed to send cmd_vel to Unity: {e}")
                self.connected = False

    def goal_callback(self, msg):
        """Forward ROS goals to Unity for visualization"""
        if self.connected and self.ws:
            try:
                unity_msg = {
                    "op": "publish",
                    "topic": "/move_base_simple/goal",
                    "msg": {
                        "pose": {
                            "position": {
                                "x": msg.pose.position.x,
                                "y": msg.pose.position.y,
                                "z": msg.pose.position.z
                            }
                        }
                    }
                }
                self.ws.send(json.dumps(unity_msg))
                rospy.loginfo(f"🎯 Goal sent to Unity: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})")
            except Exception as e:
                rospy.logwarn(f"❌ Failed to send goal to Unity: {e}")

    def listen_to_unity(self):
        """Listen for messages from Unity"""
        rospy.loginfo("👂 Started listening to Unity messages")
        
        while not rospy.is_shutdown() and self.connected:
            try:
                if self.ws:
                    result = self.ws.recv()
                    data = json.loads(result)
                    
                    topic = data.get("topic")
                    if topic == "/odom":
                        self.handle_unity_odom(data.get("msg", {}))
                    elif topic == "/hazard":
                        self.handle_unity_hazard(data.get("msg", {}))
                        
            except Exception as e:
                rospy.logwarn(f"❌ Unity listen error: {e}")
                self.connected = False
                break
        
        rospy.loginfo("👂 Stopped listening to Unity")

    def handle_unity_odom(self, msg):
        """Convert Unity odometry to ROS odometry"""
        try:
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            
            # Extract pose
            pose_data = msg["pose"]["pose"]
            odom.pose.pose.position.x = pose_data["position"]["x"]
            odom.pose.pose.position.y = pose_data["position"]["y"]
            odom.pose.pose.position.z = pose_data["position"]["z"]
            
            odom.pose.pose.orientation.x = pose_data["orientation"]["x"]
            odom.pose.pose.orientation.y = pose_data["orientation"]["y"]
            odom.pose.pose.orientation.z = pose_data["orientation"]["z"]
            odom.pose.pose.orientation.w = pose_data["orientation"]["w"]
            
            self.odom_pub.publish(odom)
            
        except Exception as e:
            rospy.logwarn(f"❌ Odom conversion error: {e}")

    def handle_unity_hazard(self, msg):
        """Forward Unity hazards to ROS"""
        try:
            hazard_msg = String()
            hazard_msg.data = msg.get("data", "Unknown Unity hazard")
            self.unity_hazard_pub.publish(hazard_msg)
            rospy.logwarn(f"🚨 Unity hazard received: {hazard_msg.data}")
        except Exception as e:
            rospy.logwarn(f"❌ Hazard handling error: {e}")

if __name__ == '__main__':
    try:
        bridge = UnityBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Unity Bridge shutting down")
