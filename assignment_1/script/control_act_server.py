#! /usr/bin/env python

import rospy
import actionlib
import math
import assignment_1.msg as a1_msgs
from assignment_1.msg import RobotVision
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RobotCtrl_base(object):
    """
    Base class for Robot Controller. It provides:
     - common members,
     - publisher on /cmd_vel to control the robot,
     - subscriber callback to get data from the camera
    """

    def __init__(self):

        # target marker id
        self._target_id = None

        # get process rate
        self._rate = rospy.get_param("process_rate")

        # subscriber for getting camera info
        self._vision_sub = None

        # timeout [s] for the first camera update
        self._camera_timeout = 2.0

        # camera joint name
        self._camera_joint_name = "astra_joint"

        # camera data
        self._camera_info = RobotVision()

        # publisher of the velocity reference control to the camera joint
        self._ctr_camera_cmd_vel_pub = rospy.Publisher(
                                            '/robot_camera_joint_vel_ctrl/command',
                                            Float64,
                                            queue_size = 10
                                        )

        
    def vision_callback(self, vision_msg: RobotVision):
        """
        Stores the camera data from the camera topic /info_vision

        :param vision_msg: camera data
        :type vision_msg: RobotVision

        :return: None
        """

        # filter out all messages not related to the target marker
        self._camera_info.ids = vision_msg.ids
        if( vision_msg.ids and vision_msg.ids[0] == self._target_id):
            self._camera_info = vision_msg


class RobotCtrlServer_searchMarkerId(RobotCtrl_base):
    """
    Action server controller for searching a target marker id
    """
    def __init__(self):

        # initialize parent class
        super().__init__()

        # robot orientation around z-axis
        self._robot_orientation = 0
        
        # camera orientation around z-axis
        self._camera_orientation = 0

        # create feedback message
        self._feedback = a1_msgs.RobotCtrl_searchFeedback()
        
        # create result message
        self._result = a1_msgs.RobotCtrl_searchResult()
        
        # create action server
        self._act_server = actionlib.SimpleActionServer(
                                "robotCtrl_search",
                                a1_msgs.RobotCtrl_searchAction,
                                execute_cb=self.act_server_callback,
                                auto_start=False
                            )
        
        # start action server
        self._act_server.start()

    def act_server_callback(self, goal: a1_msgs.RobotCtrl_searchGoal):
        """
        Handles the goal requests to look for a target marker id

        :param: goal: goal request from the client
        :type goal: assignment.msg.RobotCtrl_searchGoal

        :return: None
        """

        # subscriber to camera topic
        self._vision_sub = rospy.Subscriber(
                                'info_vision',
                                RobotVision,
                                self.vision_callback
                            )

        # get target marker id to look for
        self._target_id = goal.id
        
        try:
            # wait for the first update from the camera to come
            self._camera_info = rospy.wait_for_message('info_vision',RobotVision, timeout=self._camera_timeout)
        
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.loginfo(e)
            return False
        
        success = self.search_markerId(self._target_id)
        self._act_server.set_succeeded(result = a1_msgs.RobotCtrl_searchResult(success))
        
        # subscription not needed anymore. Remove it
        self._vision_sub.unregister()


    def search_markerId(self, id: int):
        """
        Looks for the target marker id rotating the robot

        :param: id: id of the target marker to look for
        :type id: int

        :return: Bool: True if it finds the requested target marker id, False otherwise
        """

        r = rospy.Rate(self._rate)

        # velocity reference control
        camera_angular_z = Float64(0.0)

        # send velocity command
        self._ctr_camera_cmd_vel_pub.publish(camera_angular_z)

        while(not self._camera_info.ids or id != self._camera_info.ids[0]):

            # rospy.loginfo("control - looking for marker id: %d" % id)

            # check that the goal has not been requested to be cleared
            if self._act_server.is_preempt_requested():
                rospy.loginfo("control_act_server search - Searching goal preempted")

                # stop looking for target marker id
                camera_angular_z.data = 0
                self._ctr_camera_cmd_vel_pub.publish(camera_angular_z)

                # preempt current goal
                self._act_server.set_preempted()
                return False
            
            # send feedback on current id found by the camera
            # if self._camera_info.id != None:
            self._feedback.ids = self._camera_info.ids
            self._act_server.publish_feedback(self._feedback)

            # keep on looking
            camera_angular_z.data = 0.5
            self._ctr_camera_cmd_vel_pub.publish(camera_angular_z)

            r.sleep()
        
        # send feedback on current id found by the camera
        # if self._camera_info.id != None:
        self._feedback.ids = self._camera_info.ids
        self._act_server.publish_feedback(self._feedback)

        # stop looking for target marker id
        camera_angular_z.data = 0
        self._ctr_camera_cmd_vel_pub.publish(camera_angular_z)
        
        return True


class RobotCtrlServer_reachMarkerId(RobotCtrl_base):
    """
    Action Server controller for reaching the target marker
    within a threshold
    """
    def __init__(self):
        
        # initialize parent class
        super().__init__()

        # keep on getting close till the target marker side size
        # is lower than the threshold
        self._marker_side_th = None        

        # publisher of the velocity reference control to the robot
        self._ctr_cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # create feedback message
        self._feedback = a1_msgs.RobotCtrl_reachFeedback()
        
        # create result message
        self._result = a1_msgs.RobotCtrl_reachResult()
        
        # subscriber to joint state for camera orientation
        self._camera_joint_state_sub = rospy.Subscriber(
                                            '/joint_states',
                                            JointState,
                                            self.robot_joint_state_callback
                                        )
        
        # create action server
        self._act_server = actionlib.SimpleActionServer(
                                "robotCtrl_reach",
                                a1_msgs.RobotCtrl_reachAction,
                                execute_cb=self.act_server_callback,
                                auto_start=False
                            )
        
        # start action server
        self._act_server.start()


    def robot_joint_state_callback(self, camera_joint: JointState):
        """
        Callback function to acquire the angular camera position with respect
        to the robot body frame

        :param: camera_joint: states of the camera joint, such as position, velocity, effort
        :type camera_joint: JointState

        :return: None
        """
        self._camera_orientation = camera_joint.position[
                                        camera_joint.name.index(self._camera_joint_name)
                                    ]


    def act_server_callback(self, goal: a1_msgs.RobotCtrl_reachGoal):
        """
        Handles the goal requests to reach a target marker id

        :param: goal: goal request from the client
        :type goal: assignment.msg.RobotCtrl_reachGoal

        :return: None
        """
    
        # subscriber to camera topic
        self._vision_sub = rospy.Subscriber(
                                'info_vision',
                                RobotVision,
                                self.vision_callback
                            )

        # subscriber to joint state for camera orientation
        self._camera_joint_state_sub = rospy.Subscriber(
                                            '/joint_states',
                                            JointState,
                                            self.robot_joint_state_callback
                                        )

        # set target marker id used to filter out the image from the camera
        self._target_id = goal.id
        self._marker_side_th = goal.marker_side_th

        try:
            # wait for the first update from the camera to come
            self._camera_info = rospy.wait_for_message('info_vision',RobotVision, timeout=self._camera_timeout)
        
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.loginfo(e)
            return False

        success = self.reach_marker_id(self._marker_side_th)
        self._act_server.set_succeeded(result = a1_msgs.RobotCtrl_reachResult(success))
        
    
    def get_marker_side(self):
        """
        Provides the maximum vertical marker side seen from the camera

        :return: float: maximum vertical marker side seen from the camera
        """
        x_cord_right = self._camera_info.marker_top_right[0] - self._camera_info.marker_bottom_right[0]
        y_cord_right = self._camera_info.marker_top_right[1] - self._camera_info.marker_bottom_right[1]

        x_cord_left = self._camera_info.marker_top_left[0] - self._camera_info.marker_bottom_left[0]
        y_cord_left = self._camera_info.marker_top_left[1] - self._camera_info.marker_bottom_left[1]
        
        # consider just vertical side, since horizontal one could be seen sideways, thus,
        # in some case, could never reach the threshold
        marker_side_right = math.sqrt(math.pow(x_cord_right,2) + math.pow(y_cord_right,2))
        marker_side_left = math.sqrt(math.pow(x_cord_left,2) + math.pow(y_cord_left,2))

        return max(marker_side_right, marker_side_left)


    def reach_marker_id(self, marker_side_th: float):
        """
        Gets close to the target marker up to a threshold.
        It keeps the target marker in the middle of the camera vision

        :param marker_side_th: threshold to stop the approach to the target marker
        :type marker_side_th: float

        :return: Bool: True if marker reached within the threshold, False otherwise
        """
        
        # control rate
        r = rospy.Rate(self._rate)
        # r = rospy.Rate(2)

        marker_side = 0

        # robot proportional gains
        robot_linear_gain = 0.003
        robot_angular_gain = 2

        # camera proportional gain
        camera_angular_gain = 0.003

        # robot velocity reference
        robot_velocity = Twist()

        # camera velocity reference
        camera_velocity_z = Float64(0)

        # get maximum vertical marker side
        marker_side = self.get_marker_side()

        self._feedback.id = self._target_id
    
        while (marker_side < marker_side_th):

            # check that the goal has not been requested to be cleared
            if self._act_server.is_preempt_requested():
                rospy.loginfo("control_act_server reach - Getting close to target marker preempted")
                # preempt current goal
                self._act_server.set_preempted()
                return False
            
            # get maximum vertical marker side
            marker_side = self.get_marker_side()
        
            # send feedback on the current marker size
            self._feedback.marker_side = marker_side
            self._act_server.publish_feedback(self._feedback)

            # robot errors
            linear_error = marker_side_th - marker_side
            robot_angular_error = math.fmod(self._camera_orientation, 2*math.pi)

            # robot velocity commands
            # if there is an error in the orientation of the robot with respect to the camera position,
            # the linear velocity is attenuated to give priority to the orientation movement
            robot_direction = math.cos(robot_angular_error)**3
            robot_velocity.linear.x = robot_linear_gain * linear_error * robot_direction
            # robot angular velocity, scaled with respect to the marker side with reference to the marker side threshold
            robot_velocity.angular.z = robot_angular_gain * robot_angular_error * marker_side / marker_side_th

            # publish robot velocity command
            self._ctr_cmd_vel_pub.publish(robot_velocity)

            # camera error
            camera_angular_error = self._camera_info.camera_center[0] - self._camera_info.marker_center[0]

            # camera velocity command
            # compensated with respect to the orientation movement of the robot
            camera_velocity_z.data = camera_angular_gain * camera_angular_error - \
                                     robot_velocity.angular.z

            # publish camera velocity command
            self._ctr_camera_cmd_vel_pub.publish(camera_velocity_z)

            # send feedback on the current marker size
            self._feedback.marker_side = marker_side
            self._act_server.publish_feedback(self._feedback)

            # cycle timing
            r.sleep()
        
        # target marker reached
        robot_velocity.linear.x = 0
        robot_velocity.angular.z = 0
        self._ctr_cmd_vel_pub.publish(robot_velocity)

        # subscriptions not needed anymore. Remove it
        self._vision_sub.unregister()
        self._camera_joint_state_sub.unregister()

        return True


def main():
    """
    Main function of the node
      - initialize the node
      - creates action server object for ``robotCtrl_search``
      - creates action server object for ``robotCtrl_reach``

    :return: None
    """
    rospy.init_node("robot_controller_action_server")
        
    ctr_act_server_searchMarkerId = RobotCtrlServer_searchMarkerId()
    ctr_act_server_reachMarkerId = RobotCtrlServer_reachMarkerId()

    rospy.loginfo("robot_controller_action_server initialized")

    rospy.spin()


if __name__ == "__main__":
    main()

