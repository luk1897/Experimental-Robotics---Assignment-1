# Experimental_Robotics - Assignment_1

## IMPORTANT 

In the simulation, the camera frame the side of the marker reached a maximum of 175 pixels because, by increasing the threshold, the camera was unable to detect the marker.
In the real robot, on the other hand, we set the threshold to 185, so that the robot is a little closer to the marker and the simulation and real robot are almost equal.

## Project Goal

Given four markers, the aim is to get the robot in front of them in such a way that the side of each marker is at least 175 pixels in the camera frame.

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

### VISION NODE

This node allows the robot to turn in search of the desired marker and then to reach it.

#### Publish_function

```python
def publish_function(od):

	pub=rospy.Publisher('/posvel',Posvel,queue_size=10) # publisher on /posvel topic
	
	info=Posvel() 
	
	info.x=od.pose.pose.position.x
	
	info.y=od.pose.pose.position.y 
	
	info.vel_x=od.twist.twist.linear.x 
	
	info.vel_z=od.twist.twist.angular.z 
	
	pub.publish(info)
  ```
  
  This function for publish Posvel custom message (created with odometry information) on /posvel topic.
  
  #### Menu
  
  ```python
  def menu(client):

	print("MENU\n")
	print("1. Choose your goal\n")
	print("2. Cancel your goal\n")
	print("3. Number of reached and cancelled goals\n")
	print("4. Exit\n")
	
	print("Insert your choice: \n")
	choice=int(input())
	
	if(choice == 1):
		goal = position() #getting goal coordinates
		client.send_goal(goal) #sending goal to the server
		print("Goal sent!\n")
		
	elif(choice == 2):
		client.cancel_goal() #cancelling goal
		print("\nGoal cancelled!\n")
		
	elif(choice == 3):
		rospy.wait_for_service('counter') #synchronizing with service node
		
		service=rospy.ServiceProxy('counter',Counter) #request to the service node
		
		counter=service("ok") # used a message "ok" to avoid any problem with empty message
	
	elif(choice == 4):
		print("\nExiting!\n")
		exit()
```

The function for choosing the goal, cancelling it and showing the number of reached and cancelled goals.

#### Position

``` python
def position():
	
	print("Insert x value: " )
	
	x=float(input())
	
	print("Insert y value: " )
	
	y=float(input())
	
	print("\nGoal: (%s,%s) "%(x,y))
	print("\n")
	
	goal=PlanningGoal()  #initialising goal to the message PlanningGoal
	
	goal.target_pose.pose.position.x=x #set the x goal position
		
	goal.target_pose.pose.position.y=y #set the y goal position
	
	return goal
```

The function is useful for getting goal coordinates.


### Controller NODE

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
Function for getting information (id, camera center, marker center and corners) regarding markers from vision node.

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

## Flowchart

## Outline of the assignment and the whole environment

![immagine](https://github.com/luk1897/Experimental_Robotics-Assignment_1/assets/80416766/a7a052b5-3889-42d9-8acd-7c1102a4a1d7)



##  Possibile improvements
	
	
	
	
	
	
	
	
