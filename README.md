# Experimental_Robotics - Assignment_1

## IMPORTANT 

In the simulation, the camera frame the side of the marker reached a maximum of 175 pixels because, by increasing the threshold, the camera was unable to detect the marker.
In the real robot, on the other hand, we set the threshold to 185, so that the robot is a little closer to the marker and the simulation and real robot are almost equal.I



## Project Goal

Given a simulation, create an action client which gives the user the possibility to decide the robot's goal and which takes information from the topic /odom.
Create a service node that returns how many times goals have been reached or cancelled.
Create a node that takes the information taken from the first node and prints out the distance to goal and the average speed.

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

### PLANNING CLIENT NODE

This node gives the user the possibility to decide the robot's goal and which takes information from the topic /odom.

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

#### Output 

![client_console](https://user-images.githubusercontent.com/80416766/211173202-07822132-67b0-4e38-8657-f566f7a6d977.png)

### COUNTER NODE

This is a service node that returns how many times goals have been reached or cancelled.

#### Res_cb

``` python
def res_cb(res):

	global reach_counter, cancel_counter
	
	if(res.status.status==3):
		reach_counter+=1
	elif(res.status.status==2):
		cancel_counter+=1
``` 
This function is used for computing how many goals are reached or cancelled.

#### Count_printer

``` python
def count_printer(string):

	return CounterResponse(reach_counter,cancel_counter) #
```
This function is useful for sending the response.

#### Output

![service_console](https://user-images.githubusercontent.com/80416766/211222756-76957959-6b36-47a7-b584-08780695b37c.png)


### INFORMATION NODE

This node a node takes the information taken from the first node and prints out the distance to goal and the average speed wiht a rate of 1 Hz, this frequence is parameter passed to this node through the launcher.

#### Posvel_cb

``` python
def posvel_cb(current):

	global distance_x,distance_y,average_vel_x,average_vel_z,vel_x_list,vel_z_list
	
	dist_x=goal_x-current.x 
	
	dist_y=goal_y-current.y 
	
	distance_x=round(abs(dist_x),4)
	
	distance_y=round(abs(dist_y),4)
	
	vel_x_list.append(current.vel_x) 
	
	average_vel_x=sum(vel_x_list)/len(vel_x_list) 
	
	vel_z_list.append(current.vel_z) 
	
	average_vel_z=sum(vel_z_list)/len(vel_z_list)
```

This function is used for getting distance from current goal and velocity average.

#### Goal_cb

```python
def goal_cb(pos):
	global goal_x,goal_y

	goal_x=pos.goal.target_pose.pose.position.x	#x goal position
	goal_y=pos.goal.target_pose.pose.position.y	#y goal position
```
This Function is useful for getting current goal.

#### Output

![info_console](https://user-images.githubusercontent.com/80416766/211173208-e70b2e37-eec2-4620-828d-3e0900c6ea8b.png)

## Flowchart

## Outline of the assignment and the whole environment




##  Possibile improvements
	
	
	
	
	
	
	
	
