#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

count =0

def callback(msg):
	global count
	#nur jedes 10. wird gepublished
	if(count >10):
	    if(msg.data <-10):
		    error_pub.publish(75)
	    elif(msg.data <15):
		   error_pub.publish(89)
	    elif(msg.data >15):
		  error_pub.publish(95)
	    #rate = rospy.Rate(100)
	    count =0
	    print msg.data
	count= count +1 # rate.sleep()	


    
rospy.init_node("steer_control")
error_sub = rospy.Subscriber("/error", Float32, callback)
error_pub = rospy.Publisher("/steer", Float32, queue_size=1)
rospy.spin()

