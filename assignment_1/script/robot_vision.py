#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from assignment_1.msg import RobotVision

cam_center_x = 0
cam_center_y = 0

pub = rospy.Publisher('info_vision', RobotVision, queue_size = 10) 


def camera_cb(camera_msg):

    global cam_center_x, cam_center_y

    cam_center_x = camera_msg.width / 2
    cam_center_y = camera_msg.height / 2


def img_cb(img_msg):

    global cam_center_x, cam_center_y, pub

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()
    
    corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    

    if ids is not None:
    
    
        marker_center_x = (corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]) / 4
        marker_center_y = (corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]) / 4
        
        camera_center = [cam_center_x, cam_center_y]
        marker_center = [marker_center_x, marker_center_y]
        
        top_right = [corners[0][0][0][0], corners[0][0][0][1]]
        top_left = [corners[0][0][1][0], corners[0][0][1][1]]
        bottom_left = [corners[0][0][2][0], corners[0][0][2][1]]
        bottom_right = [corners[0][0][3][0], corners[0][0][3][1]]
        
        
        info_msg = RobotVision()
        
        info_msg.id = ids[0][0]
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
