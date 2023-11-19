#!/usr/bin/env python

"""

.. module: planning_client
   :platform unix
   :synopsis: Python module for planning.
   
.. moduleauthor:: Luca Petruzzello <S5673449@studenti.unige.it>

This ROS node is used for getting the informations (height and width) regarding the camera (through /camera/color/camera_info topic) and the informations (id, center and corners) regarding the markers (through /camera/color/image_raw topic).

Subscribes to:
  **/camera/color/image_raw**
  **/camera/color/camera_info**

Publishes to:
  **info_vision**
  

"""

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from assignment_1.msg import RobotVision

cam_center_x = 0
cam_center_y = 0

pub = rospy.Publisher('info_vision', RobotVision, queue_size = 10) 

"""
global variables for setting the publisher to info_vision and computing the center of the camera

"""


def camera_cb(camera_msg):

    """
    
    Function for computing the camera center.
    
    Args: camera_msg (CameraInfo): camera message with its informations
    	
    Returns: None
    
    """

    global cam_center_x, cam_center_y

    cam_center_x = camera_msg.width / 2  # computing the x coordinate of the center
    cam_center_y = camera_msg.height / 2   # computing the y coordinate of the center


def img_cb(img_msg):

    """
    
    Function for making the image usable with aruco, computing and sending all the informations regarding the markers (id, center, corners)
    
    Args: img_msg (Image): image message with information about a marker
    	
    Returns: None
    
    """

    global cam_center_x, cam_center_y, pub

    bridge = CvBridge()  
    image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8') #bridge for transforming the image

    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL) # right dictionary
    parameters = aruco.DetectorParameters_create()           # parameters useful for aruco
    
    corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=parameters) # getting corners and id
    

    if ids is not None:
    
    
        marker_center_x = (corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]) / 4  # compute x coordinate center of a marker doing the average between all the corners
        marker_center_y = (corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]) / 4   # compute y coordinate center of a marker doing the average between all the corners
        
        camera_center = [cam_center_x, cam_center_y]     
        marker_center = [marker_center_x, marker_center_y]
        
        #all the four corners
        
        top_right = [corners[0][0][0][0], corners[0][0][0][1]] 
        top_left = [corners[0][0][1][0], corners[0][0][1][1]]
        bottom_left = [corners[0][0][2][0], corners[0][0][2][1]]
        bottom_right = [corners[0][0][3][0], corners[0][0][3][1]]
        
        
        info_msg = RobotVision()
        
        # sending all the informations to info_vision
        
        info_msg.id = int(ids[0][0])
        info_msg.camera_center = camera_center
        info_msg.marker_center = marker_center
        info_msg.marker_top_right = top_right
        info_msg.marker_top_left = top_left
        info_msg.marker_bottom_left = bottom_left
        info_msg.marker_bottom_right = bottom_right
        
        print(ids[0][0])
	
        pub.publish(info_msg)
        
        
    else:
    	print("None")
        

def main():

    """
    
    Function used for setting the two publishers.
    
    Args: None
    	
    Returns: None
    
    """

       
    rospy.Subscriber('/camera/color/image_raw', Image, img_cb)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_cb)
    
    rospy.spin()

if __name__=='__main__':

	try:
		rospy.init_node('robot_vision') 
		
		main()
		
		
	except rospy.ROSInterruptException:
		
		print("Error client")
		exit()
