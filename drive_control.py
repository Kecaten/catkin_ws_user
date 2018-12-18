#!/usr/bin/env python
# coding=utf-8
# --- imports ---
import rospy
from math import sqrt
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import pylab as py
# --- definitions ---
epsilon = 0.05   # allowed inaccuracy for distance calculation
speed_rpm = 150
last_odom = None
is_active = False
distance = 0.30
time = [] 
steer_value =[] 
dist = 0
def callbackOdom(msg):
    global last_odom
    last_odom = msg
 
 
def waitForFirstOdom():
    while not rospy.is_shutdown() and last_odom is None:
        rospy.loginfo(
            "%s: No initial odometry message received. Waiting for message...",
            rospy.get_caller_id())
        rospy.sleep(1.0)
 
 
def callbackForward(msg): 
  
   
    now = rospy.get_rostime()
    
    #arrays for plotting
    time.append(now.secs)
    steer_value.append(msg.data)
    drive(distance, "callbackForward", speed_rpm, msg.data)
 
def drive(distance, command, speed, angle):
    global is_active
 
    rospy.loginfo("%s: Running %s(%f)", rospy.get_caller_id(), command, distance)
    if distance <= 0:
        rospy.logerr(
            "%s: Error, distance argument has to be > 0! %f given",
            rospy.get_caller_id(),
            distance)
        return
 
    pub_info.publish("BUSY")
    if is_active:
        rospy.logwarn(
            "%s: Warning, another command is still active! Please wait and try again.",
            rospy.get_caller_id())
        return
 
    is_active = True
 
    # stop the car and set desired steering angle + speed
    pub_speed.publish(0)
    pub_stop_start.publish(1)
    rospy.sleep(1)
    pub_steering.publish(angle)
    pub_stop_start.publish(0)
    rospy.sleep(1)
    pub_speed.publish(speed)
 
    start_pos = last_odom.pose.pose.position
    current_distance = 0
 
    while not rospy.is_shutdown() and current_distance < (distance - epsilon):
        current_pos = last_odom.pose.pose.position
        current_distance = sqrt(
            (current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
        rospy.sleep(0.1)
 
    pub_speed.publish(0)
    is_active = False
    current_pos = last_odom.pose.pose.position
    current_distance = sqrt((current_pos.x - start_pos.x)
                            ** 2 + (current_pos.y - start_pos.y)**2)
    pub_info.publish("FINISHED")
        
    rospy.loginfo(
        "%s: Finished %s(%f)\nActual travelled distance = %f %f",
        rospy.get_caller_id(),
        command,
        distance,
        current_distance, angle)
    rospy.loginfo("%s %s", time, steer_value) 
rospy.init_node("simple_drive_control")
 
# read parameters from parameter server (launch file or rosparam set), if
# existing
if rospy.has_param("speed_rpm"):
    speed_rpm = rospy.get_param("speed_rpm")
 
# create subscribers and publishers
sub_odom = rospy.Subscriber("odom", Odometry, callbackOdom, queue_size=100)
# wait for first odometry message, till adverting subscription of commands
waitForFirstOdom()
sub_forward = rospy.Subscriber(
    "/steer", Float32, callbackForward, queue_size=10)

pub_stop_start = rospy.Publisher(
    "manual_control/stop_start",
    Int16,
    queue_size=100)
pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=100)
pub_steering = rospy.Publisher("steering",UInt8,queue_size=1)
pub_info = rospy.Publisher("simple_drive_control/info", String, queue_size=100)
 
rospy.loginfo(rospy.get_caller_id() + ": started!")
 #plotting
print time
print steer_value
py.plot(time, steer_value) 
rospy.spin()