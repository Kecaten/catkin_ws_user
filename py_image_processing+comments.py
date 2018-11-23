#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
  #publish Bild zu /image_processing/bin_img
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1) 
#ROS bild zu OpenCV bild oder anders rum
    self.bridge = CvBridge()
    #Bild erhalten von /app/camera/rgb/image_raw
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
    #ROS bild in OpenCV bild konvertieren (bgr8: CV_8UC3, color image with blue-green-red color order)
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    #(input, flag --> BGR zu Gray) HSV:Farbwert, Farbsättigung, Hellwert
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 245
    #retinal, bild =(image, wert für die Klassifizierung der Pixelwerte, Wert den Pixel mit höherem Wert annehmen sollen, fkt)
    #cv2.THRESH_BINARY = zwei Farben: schwarz, weiß bei bi_gray_min=127 bi_gray_max=255
    #pixelwert gibt helligkeit & Farbe an; 255=weiß 0=schwarz, 245=hellgrau
    #--> schwarz weiß machen
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    #gauss
    MAX_KERNEL_LENGTH = 2;
    i= 5
    #Schärfe einstellen
    dst=cv2.GaussianBlur(cv_image,(5,5),0,0)

    #edge
    dx = 1;
    dy = 1;
    ksize = 3; #1,3,5,7
    scale = 1
    delta = 0
    #Kanten erkennen cv2.CV_8UC1 ist Output typ CV_8UC1= 8-bit single channel array
    #border_default = am Rand ist das Bild gespiegelt 
    edge_img=cv2.Sobel(thresh1, cv2.CV_8UC1, dx, dy, ksize, scale, delta, cv2.BORDER_DEFAULT)

    #bi_rgb
    r_max = 244;
    r_min = 0;
    g_max = 255;
    g_min = 0;
    b_max = 255;
    b_min = 0;
    #splitten um an den einzelnen Farben r,g,b zu arbeiten
    b,g,r = cv2.split(cv_image)
    #img.shape gibt Zeilen, Spalten, Channels aus; bei grayen Bildern nur Zeilen, Spalten
    for j in range(cv_image.shape[0]):
      for i in range(cv_image.shape[1]):
        if (r[j,i] >= r_min and r[j,i] <= r_max):
          if (g[j,i] >= g_min and g[j,i] <= g_max):
            if (b[j,i] >= b_min and b[j,i] <= b_max):
              r[j,i]=0
              g[j,i]=0
              b[j,i]=0
            else:
              r[j,i]=255
              g[j,i]=255
              b[j,i]=255
    #dann alles wieder zusammenfügen
    bi_rgb = cv2.merge((b,g,r))

    #bi_hsv
    h_max = 255;
    h_min = 0;
    s_max = 255;
    s_min= 0;
    v_max = 252;
    v_min = 0;
    #Farbe von BGR zu HSV
    hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    h,s,v = cv2.split(hsv)

    for j in xrange(hsv.shape[0]):
      for i in xrange(hsv.shape[1]):
        if  (v[j,i]>= v_min and v[j,i]<= v_max and s[j,i]>= s_min and s[j,i]<= s_max and h[j,i]>= h_min and h[j,i]<= h_max):
          h[j,i]=0
          s[j,i]=0
          v[j,i]=0
        else:
          h[j,i]=255
          s[j,i]=255
          v[j,i]=255

    bi_hsv = cv2.merge((h,s,v))

    # titles = ['Original Image', 'GRAY','BINARY','GAUSS','EDGE','BI_RGB','BI_HSV']
    # images = [cv_image, gray, thresh1,dst,edge_img,bi_rgb,bi_hsv]
    #
    # for i in xrange(7):
    #   plt.subplot(2,4,i+1),plt.imshow(images[i],'gray')
    #   plt.title(titles[i])
    #   plt.xticks([]),plt.yticks([])
    #
    # plt.show()
    # print("Done")

    try:
    #bild publishen
    #thresh1 --> schwarz weißes Bild
    #thresh1 durch gray ersetzen für ein graues bild
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)
      

def main(args):
#node image_converter
  rospy.init_node('image_converter', anonymous=True)
  #Objekt ic erstellen
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)