#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def callback(data):
	publisher.publish("I heard: " + str(data.data))


rospy.init_node("assignment1_publisher_subscriber")

publisher = rospy.Publisher("/assignment1_publisher_subscriber", String)
rospy.Subscriber("/yaw", Float32, callback)

rospy.spin()
