#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from luna_bot_ros.msg import Hazard
import json

class SensorNode:
    def __init__(self):
        rospy.Subscriber('/env/status', String, self.env_cb)
        self.alert_pub = rospy.Publisher('/alerts', Hazard, queue_size=10)
        rospy.loginfo("Sensor node started.")

    def env_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logwarn("Received invalid JSON in /env/status message.")
            return

        current_time = rospy.Time.now().to_sec()

        if data.get('leak', False):
            h = Hazard()
            h.type = 'leak'
            h.severity = 'critical'
            h.description = 'Cabin leak detected'
            h.timestamp = current_time
            self.alert_pub.publish(h)

        if data.get('temp', 0) > -10:
            h = Hazard()
            h.type = 'high_temp'
            h.severity = 'warning'
            h.description = 'Temperature high'
            h.timestamp = current_time
            self.alert_pub.publish(h)

        if data.get('oxygen', 100) < 18.0:
            h = Hazard()
            h.type = 'low_oxygen'
            h.severity = 'critical'
            h.description = 'Low O2'
            h.timestamp = current_time
            self.alert_pub.publish(h)

if __name__ == '__main__':
    rospy.init_node('sensor_node', anonymous=True)
    try:
        sensor_instance = SensorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass