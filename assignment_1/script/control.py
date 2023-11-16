#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from assignment_1.msg import RobotVision
from geometry_msgs.msg import Twist
import numpy as np
#import time

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

state = "search_marker"
id_list = [15, 13, 12, 11]
id_marker = 0

vision_id = 0
camera_center = []
marker_center = []
marker_top_right = []
marker_top_left = []
marker_bottom_left = []
marker_bottom_right = []

r = 0

def vision_cb(vision_msg):

    global vision_id, camera_center, marker_center, marker_top_right, marker_top_left, marker_bottom_left, marker_bottom_right, id_marker
    
    if(vision_msg.id == id_marker):
    
	    vision_id = vision_msg.id
	    camera_center = vision_msg.camera_center
	    marker_center = vision_msg.marker_center
	    marker_top_right = vision_msg.marker_top_right
	    marker_top_left = vision_msg.marker_top_left
	    marker_bottom_left = vision_msg.marker_bottom_left
	    marker_bottom_right = vision_msg.marker_bottom_right


def get_close_marker():

    global vision_id, camera_center, marker_center, marker_top_right, marker_top_left, marker_bottom_left, marker_bottom_right, pub, state, id_marker, id_list
    
    target = 180
    
    velocity = Twist()
    
    x_cord = marker_top_right[0] - marker_bottom_right[0]
    y_cord = marker_top_left[1] - marker_bottom_left[1]
    
    side = np.sqrt(np.power(x_cord,2) + np.power(y_cord,2))
    
    linear_gain = 0.002
    angular_gain = 0.002
          
    angular_error = camera_center[0] - marker_center[0]
    linear_error = target - side 
          
    velocity.linear.x = linear_gain * linear_error
    velocity.angular.z = angular_gain * angular_error
          
    pub.publish(velocity)
    
    print(side)
    
    if(side >= target):
           print("Reached!")
           id_marker = id_list.pop()
           state = "search_marker"
    
    
def search_marker():

    global id_list, state, pub, vision_id, id_marker,r
    
    if (len(id_list) == 0):
          exit()
    #else:
          #id_marker = id_list.pop()
          
    velocity = Twist()
    
    velocity.angular.z = 0
    velocity.linear.x = 0
    pub.publish(velocity)
    
    if(id_marker != vision_id):
         #print("Looking for the right marker")
         
         print("id_mark: ", id_marker)
         print("vision_id: ", vision_id)
         
         velocity.angular.z = -0.4
    
         pub.publish(velocity)
    else: 
         
         state = "get_close_marker"

def main():

    global state,r, id_list, id_marker

    r = rospy.Rate(1)
 
    rospy.Subscriber('info_vision', RobotVision, vision_cb)

    id_marker = id_list.pop()
    
    while not rospy.is_shutdown():
             if(state == "search_marker"):
                   search_marker()
             elif(state == "get_close_marker"):
                   get_close_marker()
             r.sleep()
    



if __name__=='__main__':

	try:
		rospy.init_node('control') 
		
		main()
		
		
	except rospy.ROSInterruptException:
		
		print("Error client")
		exit()
