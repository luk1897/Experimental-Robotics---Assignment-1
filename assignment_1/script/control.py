#!/usr/bin/env python

"""
.. module: counter
   :platform unix
   :synopsis: Python module for control the robot and get close to the markers.
   
.. moduleauthor:: Luca Petruzzello <S5673449@studenti.unige.it>

This ROS node is used for controlling the angular and linear velocity of the robot through /cmd_vel topic and for getting informations regarding the markers through the /info_vision topic

Subscribes to:
  **info_vision**

Publisher to:
  **/cmd_vel**

"""

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from assignment_1.msg import RobotVision
from geometry_msgs.msg import Twist
import numpy as np

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10) 

"""
global variable for publishing to /cmd_vel

"""

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

"""
global variables regarding the state and the vision informations

"""

def vision_cb(vision_msg):

    """
    
    	Function for getting information (id, camera center, marker center and corners) regarding markers from vision node.
    
    	Args: 
    	vision_msg (RobotVision): RobotVision message
    	
    	Returns: None
    
    """

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

    """
    
    	Function for getting close to the marker and extracting the next goal id
    
    	Args: None
    	
    	Returns: None
    
    """

    global vision_id, camera_center, marker_center, marker_top_left, marker_bottom_left, pub, state, id_marker
    
    target = 175    # marker side to reach
    
    velocity = Twist()
    
    x_cord = marker_top_left[0] - marker_bottom_left[0]  # x coordinate of left corner of a marker
    y_cord = marker_top_left[1] - marker_bottom_left[1]  # y coordinate of left corner of a marker
    
    side = np.sqrt(np.power(x_cord,2) + np.power(y_cord,2))  # computing the side
    
    linear_gain = 0.0025
    angular_gain = 0.002
          
    angular_error = camera_center[0] - marker_center[0]  # angular error between camera center and marker center
    linear_error = target - side                 # linear error between the robot and a marker        
          
    velocity.linear.x = linear_gain * linear_error    # computing the right linear velocity
    velocity.angular.z = angular_gain * angular_error # computing the right angular velocity
          
    pub.publish(velocity)  # publish the velocity to /cmd_vel topic
    
    print("Reaching the target!")
    
    if(side >= target):   # marker side in the camera reached the target
           print("Reached!")
           if(id_marker != 15):  # if the id is not the last (15)
              id_marker = id_list.pop()  # extract the id from the list
              state = "search_marker"  # change the state
           else:
              rospy.signal_shutdown("exit")  # a way to close the node

    
    
def search_marker():

    """
    
    	Function for allowing the robot to rotate until it finds the desired id marker
    
    	Args: None
    	
    	Returns: None
    
    """

    global state, pub, vision_id, id_marker
          
    velocity = Twist()
    
    if(id_marker != vision_id):    # the id marker is not the desired one
         
         print("Looking for the target!")
         
         velocity.angular.z = -0.5   # getting on rotating
    
         pub.publish(velocity)
         
    else:                   # the id marker is the desired one
         
         velocity.angular.z = 0     # stopping all velocities
         velocity.linear.x = 0  
         pub.publish(velocity)
         
         state = "get_close_marker"   # changing state

def main():

    """
    
    	Function for setting the frequency with the robot works, the subscriber to info_vision topic, extracting the first id marker from the list and calling the states
    
    	Args: None
    	
    	Returns: None
    
    """

    global state, id_marker
    
    r = rospy.Rate(20)  # robot works with 20 Hz
 
    rospy.Subscriber('info_vision', RobotVision, vision_cb)  # sub to info_vision
    
    id_marker = id_list.pop()  # extracting the first id marker
    
    while not rospy.is_shutdown():
             if(state == "search_marker"):  #looking for the marker
                   search_marker()        
             elif(state == "get_close_marker"):   # getting close to the marker
                   get_close_marker()
             r.sleep()
    
    



if __name__=='__main__':

	try:
		rospy.init_node('control') 
		
		main()
		
		
	except rospy.ROSInterruptException:
		
		print("Error client")
		exit()
