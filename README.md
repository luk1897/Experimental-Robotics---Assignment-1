# Experimental_Robotics - Assignment_1

## IMPORTANT 

In the simulation, the camera frame the side of the marker reached a maximum of 175 pixels because, by increasing the threshold, the camera was unable to detect the marker.
In the real robot, on the other hand, we set the threshold to 185, so that the robot is a little closer to the marker and the simulation and real robot are almost equal.

## Project Goal

Given four markers, the aim is to get the robot in front of them in such a way that the side of each marker is at least 175 pixels in the camera frame. Finally, after testing the code in simulation, our goal was to run it on the real robot.

## How to install and run

### Install

It is necessary to have ROS. Follow the instructions from [wiki.ros.org](http://wiki.ros.org/).

If you are using the professor's Docker Image, add the line ```source /opt/ros/noetic/setup.bash``` to the .bashrc file.

You need to use Git. Run: ```sudo apt-get install git```

Then run on your shell: ```git clone https://github.com/luk1897/Experimental_Robotics-Assignment_1```

Then install xterm because it is used to check what the robot is doing. Run ```sudo apt-get install xterm```

Finally, you need to have aruco in your pc. Run ```git clone https://github.com/CarmineD8/aruco_ros ```

### Run

Run this command on your shell: ```roslaunch assignment_1 assignment1.launch```

## Environment

![immagine](https://github.com/luk1897/Experimental_Robotics-Assignment_1/assets/80416766/a2aa6bef-f815-477f-9023-456ed6b273f9)

This is the entire environment in which we worked.

## Nodes

### ROBOT VISION NODE

This ROS node is used for getting the informations (height and width) regarding the camera (through /camera/color/camera_info topic) and the informations (id, center and corners) regarding the markers (through /camera/color/image_raw topic). In the main we set the subscribers.

#### camera_cb

```python
def camera_cb(camera_msg):

    global cam_center_x, cam_center_y

    cam_center_x = camera_msg.width / 2  # computing the x coordinate of the center
    cam_center_y = camera_msg.height / 2   # computing the y coordinate of the center
  ```
  
  Function for computing the camera center.
  
  #### Img_cb
  
  ```python
 
def img_cb(img_msg):

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
```

Function for making the image usable with aruco, computing and sending all the informations regarding the markers (id, center and corners).


### CONTROLLER NODE

This node allows the robot to turn in search of the desired marker and then to reach it. The logic is implemented in the main.

#### vision_cb

``` python
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
``` 
Function for getting information (id, camera center, marker center and corners) regarding markers from vision node through /info_vision topic.

#### get_close_marker

``` python
def get_close_marker():

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
```
Function for getting close to the marker and extracting the next goal id

#### search_marker

``` python
def search_marker():

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
```
Function for allowing the robot to rotate until it finds the desired id marker.

## Messages

### RobotVision.msg

![immagine](https://github.com/luk1897/Experimental_Robotics-Assignment_1/assets/80416766/ffdf61a3-5fe2-4dca-b9f1-2084d5ec7764)

This message is fundamental to share the informations regarding the camera and the markers from robot_vision node to control node.

## Flowchart

## Outline of the assignment and the whole environment

![immagine](https://github.com/luk1897/Experimental_Robotics-Assignment_1/assets/80416766/a7a052b5-3889-42d9-8acd-7c1102a4a1d7)

## Differences between simulation and real robot
* The linear gain was reduced from 0.0025 to 0.002 in the real robot.
* The angular velocity was reduced from -0.5 to -0.4 in the real robot.
* Of course, there is no need for gazebo and urdf parts in the real robot.
* The threshold to reach a marker was increased from 175 pixels to 185 pixels in the real robot.

## Results

https://github.com/luk1897/Experimental_Robotics-Assignment_1/assets/80416766/d64546dd-3616-4faf-9d98-949b5a38abfb


##  Possibile improvements
	
	
	
	
	
	
	
	
