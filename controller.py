#!/usr/bin/env python
# coding=utf-8
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import random
import numpy.linalg as numpy
import numpy as np
from std_msgs.msg import Float32

class image_converter:

	def __init__(self):
#############	#zum testen 
		self.image_pub = rospy.Publisher("/control/",Image, queue_size=1) 
#############		
		self.error_pub = rospy.Publisher("/error", Float32, queue_size=1)
		rate = rospy.Rate(100)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1, buff_size=2**24)
		rate.sleep()
	
	def callback(self,data):
		
		array = []
		help_array =[]
		try:
			image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		#gray	
		image1=cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
		
		
		height = image1.shape[0]
		width = image1.shape[1]
		
		#get coordinates for ransac
		for y in range(200, height-1): 
			for x in range(200, width-200):
		
				rgb = image1[y,x]

				if (rgb>= (200) ):   
			    		
					help_array.append((x,y))
			#mittleren Punkt der weissen Linie nehmen		
			if (help_array != []):		
				array.append(help_array[(len(help_array)/2)-1])
			#print help_array, len(help_array), "hhhh " , array
			help_array = []	    
						    	
		print len(array)
		
		#begin of ransac
		n = 60
		d = 1
		t1 = len(array)/4
		set_final=[]
		set_tmp=[]
		a_final=(0,0)
		b_final=(0,0)
		while(n>0):
			set_tmp = []
			a=random.choice(array)
			b=random.choice(array)
			while(b==a):
				b=random.choice(array)
			for i in range(0,len(array)):
				a_np = np.asarray(a)
				b_np = np.asarray(b)
				c = np.asarray(array[i])
				d1 = numpy.norm(np.cross(b_np-a_np, a_np-c))/numpy.norm(b_np-a_np)
				if(d1<d):
					set_tmp.append(array[i])
			if(len(set_tmp)>=t1 and len(set_tmp)>len(set_final)):
				a_final = a
				b_final = b
				set_final = set_tmp
			n-=1

####################	#nur zum testen			
		print a_final
		print b_final
		cv2.line(image1,a_final,b_final,(0),3)
		cv2.imwrite('img_lines.png',image1)
#####################
		m1 = (b_final[1]-a_final[1])/float(b_final[0]-a_final[0])
		b1 = a_final[1]-m1*a_final[0]
		print "\n"	
		print "m: "
		print m1
		print "b: "
		print b1
		print "\n"
		
		row = 240
		column = 320
		
		intersection = int((row - b1) /m1)
		#intersection
		image1[row,intersection] = 150 
		
		error = intersection - column

		print error
		#publish error 
		self.error_pub.publish(error)	
		
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(image1, "mono8"))
		except CvBridgeError as e:
			        	print(e)

def main(args):

  rospy.init_node('controller', anonymous=True)    
    #Objekt ic erstellen
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
