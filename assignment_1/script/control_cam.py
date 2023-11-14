#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from assignment_1.msg import RobotVision
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import math

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
cam_pub = rospy.Publisher('/astra_joint_velocity_controller/command', Float64, queue_size = 10)

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

robot_orientation = 0

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
	    
def odom_cb(odom_msg):

    global robot_orientation
    
    orientation = odom_msg.pose.pose.orientation
    
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(quaternion)
    
    robot_orientation = yaw
    
    #robot_orientation = odom_msg.twist.twist.z
     


def get_close_marker():

    global vision_id, camera_center, marker_center, marker_top_right, marker_top_left, marker_bottom_left, marker_bottom_right, pub, state, cam_pub, robot_orientation
    
    target = 175
    
    velocity = Twist()
    
    x_cord = marker_top_right[0] - marker_bottom_right[0]
    y_cord = marker_top_left[1] - marker_bottom_left[1]
    
    side = np.sqrt(np.power(x_cord,2) + np.power(y_cord,2))
    
    linear_gain = 0.003
    angular_gain = 0.003
    
    angular_error = camera_center[0] - marker_center[0]
    cam_pub.publish(0.01 * angular_error)
    
    angular_body_cam_error = marker_center[0] - robot_orientation
    velocity.angular.z = angular_gain * angular_body_cam_error
    
    #print(angular_body_cam_error)
    #print(marker_center[0])
    print(robot_orientation)
    #print(angular_body_cam_error)
    
    if(angular_body_cam_error <= 0.1):
       #print("hello")
       velocity.angular.z = 0
       pub.publish(velocity)
    
    #linear_error = target - side
    #velocity.linear.x = linear_gain * linear_error
    pub.publish(velocity)
    
    #print(side)
    
    if(side >= target):
           print("Reached!")
           state = "search_marker"  

    
    
def search_marker():

    global id_list, state, vision_id, id_marker, cam_pub
    
    if (len(id_list) == 0):
          exit()
    else:
          id_marker = id_list.pop()
          
    cam_pub.publish(0.0)
    
    while(id_marker != vision_id):
         #print("Looking for the right marker")
         
         #print(id_marker)
         
         cam_pub.publish(-1.5)
         
    state = "get_close_marker"

def main():

    global state
 
    rospy.Subscriber('info_vision', RobotVision, vision_cb)
    rospy.Subscriber('/odom', Odometry, odom_cb)
    
    while not rospy.is_shutdown():
             if(state == "search_marker"):
                   search_marker()
             elif(state == "get_close_marker"):
                   get_close_marker()
            
    
    



if __name__=='__main__':

	try:
		rospy.init_node('control') 
		
		main()
		
		
	except rospy.ROSInterruptException:
		
		print("Error client")
		exit()
