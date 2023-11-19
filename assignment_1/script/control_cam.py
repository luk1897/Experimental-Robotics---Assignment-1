#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo, JointState
from assignment_1.msg import RobotVision
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
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
cam_angle = 0

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
	 
    robot_orientation = odom_msg.twist.twist.angular.z
    
    
    
def joint_state_cb(js_msg):

    global cam_angle
    
    cam_angle = js_msg.position[0]
    


def get_close_marker():

    global vision_id, camera_center, marker_center, marker_top_left, marker_bottom_left, pub, state, cam_pub, robot_orientation, cam_angle, id_marker
    
    target = 175
    
    velocity = Twist()
    
    x_cord = marker_top_left[0] - marker_bottom_left[0]
    y_cord = marker_top_left[1] - marker_bottom_left[1]
    
    side = np.sqrt(np.power(x_cord,2) + np.power(y_cord,2))
    
    linear_gain = 0.0022
    angular_gain = 0.4
    cam_gain = 0.01
    
    angular_error = camera_center[0] - marker_center[0]
    cam_pub.publish((cam_gain * angular_error) - robot_orientation )
    
    velocity.angular.z =  angular_gain * cam_angle

    if(np.abs(cam_angle) <= 0.1):
       linear_error = target - side
       velocity.linear.x = linear_gain * linear_error
       
    pub.publish(velocity)
    
    print("Reaching the target!")
    
    if(side >= target):
           print("Reached!")
           if(id_marker != 15):
              id_marker = id_list.pop()
              state = "search_marker"  
           else:
              rospy.signal_shutdown("exit")  

    
    
def search_marker():

    global state, vision_id, id_marker, cam_pub
    
    if(id_marker != vision_id):
         
         print("Looking for the target!")
         
         cam_pub.publish(-0.5)
    else:
         cam_pub.publish(0.0)
        
         state = "get_close_marker"

def main():

    global state, id_marker
    
    r = rospy.Rate(20)
 
    rospy.Subscriber('info_vision', RobotVision, vision_cb)
    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.Subscriber('/joint_states', JointState, joint_state_cb)
    
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
