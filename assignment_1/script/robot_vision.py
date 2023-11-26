#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from assignment_1.msg import RobotVision
import numpy as np

cam_center_x = 0
cam_center_y = 0

info_msg = RobotVision()

def camera_cb(camera_msg):
    """
    Calculates the camera center coordinates
    """

    global cam_center_x, cam_center_y

    cam_center_x = camera_msg.width / 2
    cam_center_y = camera_msg.height / 2


def img_cb(img_msg):
    """
    Publishes the information about
     - marker id
     - marker center coordinates
     - marker corners coordinates
    """

    global cam_center_x, cam_center_y, info_msg

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()
    
    corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    info_msg.ids = [] if not np.array(ids).tolist() else [ids[0][0]]
    
    if ids is not None:
        marker_center_x = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4
        marker_center_y = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4
                
        info_msg.camera_center = [cam_center_x, cam_center_y]
        info_msg.marker_center = [marker_center_x, marker_center_y]
        info_msg.marker_top_right = [corners[0][0][0][0], corners[0][0][0][1]]
        info_msg.marker_top_left = [corners[0][0][1][0], corners[0][0][1][1]]
        info_msg.marker_bottom_left = [corners[0][0][2][0], corners[0][0][2][1]]
        info_msg.marker_bottom_right = [corners[0][0][3][0], corners[0][0][3][1]]

def main():
    """
    Subscribes to camera topics
     - image_raw
     - camera_info
    Publishes the topic /info_vision containing
     - marker id,
     - coordinates in pixels of the camera center, marker center, 4 marker corners
    """

    vision_rate = rospy.get_param("vision_rate")
    rate = rospy.Rate(vision_rate)

    # subscribe to camera topics
    rospy.Subscriber('/camera/color/image_raw', Image, img_cb)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_cb)

    # publisher with data about collected image
    pub = rospy.Publisher('info_vision', RobotVision, queue_size = 10) 
    
    rospy.loginfo("robot_vision node initialized")

    # to control the publishing rate
    # could simulate delay in camera transmission
    while (not rospy.is_shutdown()):
        pub.publish(info_msg)

        rate.sleep()

if __name__=='__main__':

    try:
        rospy.init_node('robot_vision')
        main()
        
    except rospy.ROSInterruptException as e:
        print(e)
