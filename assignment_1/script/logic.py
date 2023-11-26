#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import assignment_1.msg as a1_msgs
from assignment_1.msg import LogicState

ctr_goal_status_sub = None  # subscriber to get controller goal status
ctrl_client_search = None   # client of control action server for searching a marker id
ctrl_client_reach = None    # client of control action server for reaching a marker id
state = None                # state machine status
marker_side_th = 175        # threshold for limit on getting close to the marker
current_marker_id = None    # marker id currently looked for
cycle_rate = None


def search_marker_id_done_cb(goalStatus: GoalStatus, result: a1_msgs.RobotCtrl_searchResult):
    """
    Updates the logic state machine status to send the get close to the
    target market goal when the target marker id is found

    :param goalStatus: goal status of the current goal
    :type goalStatus: GoalStatus
    :param result: search action result
    
    :return: None
    """
    
    global state

    rospy.loginfo("logic - search result: " + str(result.success))
    if (result.success):
        state = LogicState.SEND_REACH_ID
    else:
        rospy.loginfo("************* ERROR ****************")
        rospy.loginfo("Something went wrong while looking for the target marker id")
        state = LogicState.FINISH


def reach_marker_done_cb(goalStatus: GoalStatus, result: a1_msgs.RobotCtrl_reachResult):
    """
    Updates the logic state machine status to send the id of the new target marker
    when the robot is close enough to the current target marker

    :param goalStatus: goal status of the current goal
    :type goalStatus: GoalStatus
    :param result: reach action result
    :type result: RobotCtrl_reachResult

    :return: None
    """
    
    global state
    
    rospy.loginfo("logic - reach result: " + str(result.success))

    if (result.success):
        state = LogicState.SEND_SEARCH_ID
    else:
        rospy.loginfo("************* ERROR ****************")
        rospy.loginfo("Something went wrong while getting close to the target marker")
        state = LogicState.FINISH

def terminate_node():
    """
    Sends the shutdown signal to the node

    :return: None
    """
    
    rospy.signal_shutdown("\nShutting down node on user request...")


def start(ids: list):
    """
    Defines the state machine for handling the different phases of the process:

      * 0: select and send the target marker id to look for
      * 1: waiting for finding target marker id
      * 2: send command to get closer to target marker id
      * 3: waiting for getting close to marker id
      * 4: terminate the node

    :param ids: list of marker id to look for
    :type ids: list

    :return: None
    """

    global ctrl_client_reach, state, current_marker_id, cycle_rate

    # passing an empty list finishes the process
    if ids:
        state = LogicState.SEND_SEARCH_ID
    else:
        state = LogicState.FINISH

    while(not rospy.is_shutdown()):
        
        # select target marker id and send request
        # to search for such a marker id
        if state == LogicState.SEND_SEARCH_ID:
            if ids:
                current_marker_id = ids.pop(0)
                # create goal for looking for target marker id
                ctr_goal_search = a1_msgs.RobotCtrl_searchGoal(
                                    id = current_marker_id
                                )

                # send the goal to action server
                ctrl_client_search.send_goal(
                                        ctr_goal_search,
                                        done_cb = search_marker_id_done_cb
                                    )

                # state = LogicState.WAIT_FOR_GOAL_SEARCH_ACTIVE
                state = LogicState.LOOKING_FOR_ID
            
            # all target markers have been reached
            else:
                state = LogicState.FINISH

        # wait for target marker id to be found
        elif state == LogicState.LOOKING_FOR_ID:
            # waiting for finding target marker id
            pass

        # send request to get close to target marker
        elif state == LogicState.SEND_REACH_ID:

            # create goal for getting close to target marker
            ctr_goal_reach = a1_msgs.RobotCtrl_reachGoal(
                            id = current_marker_id,
                            marker_side_th = marker_side_th
                        )

            # send goal to action server
            ctrl_client_reach.send_goal(
                                    ctr_goal_reach,
                                    done_cb = reach_marker_done_cb
                                )

            state = LogicState.GETTING_CLOSE

        elif state == LogicState.GETTING_CLOSE:
            # waiting for getting close enough to the marker
            pass

        elif state == LogicState.FINISH:
            terminate_node()
        
        # exception handling for rate whne the process is terminated
        try:
            # keep cycle time
            cycle_rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            print(e)


def search_feedback_callback(feedback: a1_msgs.RobotCtrl_searchFeedback):
    """
    Prints the feedback of current marker id seen by the camera

    :param: feedback:  message containing the marker id currently seen by
                       the camera
    :type feedback: assignment_.msg.RobotCtrl_searchFeedback

    :return: None
    """
    
    rospy.loginfo("search feedback - marker id seen: " + str(feedback.feedback.ids))


def reach_feedback_callback(feedback: a1_msgs.RobotCtrl_reachFeedback):
    """
    Prints the feedback of current marker id and marker side

    :param: feedback: message containing the marker id and marker side size
    :type feedback: assignment_1.msg.RobotCtrl_reachFeedback
    
    :return: None
    """
    
    rospy.loginfo("reach feedback - target marker id: %d" % feedback.feedback.id)
    rospy.loginfo("reach feedback - target marker side: %.1f" % feedback.feedback.marker_side)


def main():
    """
    Main function of the node
      - initializes the node
      - creates client for action server ``robotCtrl_search``
      - creates client for action server ``robotCtrl_reach``
      - launches function `start` with the list of marker ids to look for.
        This function handles the process phases.

    :return: None
    """

    global ctr_goal_status_sub, ctrl_client_search, ctrl_client_reach, cycle_rate

    # initialize node
    rospy.init_node("robot_logic")
    
    # client for search action server controller
    ctrl_client_search = actionlib.SimpleActionClient(
                    "robotCtrl_search",
                    a1_msgs.RobotCtrl_searchAction
                )

    rospy.loginfo("Waiting for action server 'robotCtrl_search'...")
    
    # client for reach action server controller
    ctrl_client_reach = actionlib.SimpleActionClient(
                    "robotCtrl_reach",
                    a1_msgs.RobotCtrl_reachAction
                )

    rospy.loginfo("Waiting for action server 'robotCtrl_reach'...")

    # wait for the search controller server to be started
    ctrl_client_search.wait_for_server()
    rospy.loginfo("action server 'robotCtrl_search' found")
    
    # wait for the reach controller server to be started
    ctrl_client_reach.wait_for_server()
    rospy.loginfo("action server 'robotCtrl_reach' found")

    # subscriber to the feedback of the search action server
    ctrl_search_feedback_sub = rospy.Subscriber(
                                                "/robotCtrl_search/feedback",
                                                a1_msgs.RobotCtrl_searchFeedback,
                                                search_feedback_callback
                                            )
    
    # subscriber to the feedback of the reach action server
    ctrl_reach_feedback_sub = rospy.Subscriber(
                                                "/robotCtrl_reach/feedback",
                                                a1_msgs.RobotCtrl_reachFeedback,
                                                reach_feedback_callback
                                            )

    # ordered list of marker ids to look for
    ids = [11, 12, 13, 15]

    # get process rate
    freq_rate = rospy.get_param("process_rate")
    # initialize logic rate
    cycle_rate = rospy.Rate(freq_rate)

    rospy.loginfo("robot_logic node initialized")

    # start looking for markers
    start(ids)

    rospy.spin()


if __name__ == "__main__":
    # waiting time for GUI to load
    rospy.sleep(7)

    main()
        
