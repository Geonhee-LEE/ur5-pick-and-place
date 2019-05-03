#! /usr/bin/env python
"""--------------------------------------------------------------------
This Node/Script shows an example on how to use a `SimpleActionClient` instance 
to control a Robotiq 2 Finger gripper.

Parameters:
    action_name: Name of the action advertised by the `ActionServer` controlling the gripper.

@author: Daniel Felipe Ordonez Apraez
@email: daniels.ordonez@gmail.com
--------------------------------------------------------------------"""

import rospy

# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from robotiq_2f_gripper_msgs.msg import RobotiqGripperCommand

def operate_gripper():
    global goal

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    # Wait until grippers are ready to take command
    robotiq_client.wait_for_server()

    rospy.logwarn("Client test: Starting sending goals")
    ## Manually set all the parameters of the gripper goal state.
    ######################################################################################
    
    #goal = CommandRobotiqGripperGoal()
    goal.emergency_release = False
    goal.stop = False
    goal.position = 0.10
    goal.speed = 0.1
    goal.force = 5.0

    # Sends the goal to the gripper.
    robotiq_client.send_goal(goal)
    # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
    robotiq_client.wait_for_result()
    
    # Use pre-defined functions for robot gripper manipulation.
    #####################################################################################
    '''
    while not rospy.is_shutdown():
        Robotiq.close(robotiq_client, block=True)   # Close and wait until completion
        Robotiq.open(robotiq_client, block=False)   # Open and do not block thread while completing goal

        Robotiq.goto(robotiq_client, pos=0.00, speed=0.1, force=100 , block=True)
        Robotiq.goto(robotiq_client, pos=0.04, speed=0.01, force=10)
        Robotiq.goto(robotiq_client, pos=0.011, speed=0.01, force=0 , block=True)
        Robotiq.goto(robotiq_client, pos=0.08, speed=0.11, force=200 , block=True)
        # Robotiq.goto(robotiq_client, pos=0.06, speed=0.0, force=0)
        # break
    '''

    # 
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        #rospy.loginfo("Goto")
        Robotiq.goto(robotiq_client, pos=goal.position, speed=goal.speed, force=goal.force , block=True)
        rate.sleep()
    # Prints out the result of executing the action
    return robotiq_client.get_result()  # A FibonacciResult


def gripperCB(RobotiqGripperCommand):
    global goal
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", RobotiqGripperCommand.position)
    
    goal.emergency_release = RobotiqGripperCommand.emergency_release
    goal.stop = RobotiqGripperCommand.stop
    goal.position = RobotiqGripperCommand.position # 0 ~ 0.1
    goal.speed = RobotiqGripperCommand.speed
    goal.force = RobotiqGripperCommand.force

    if goal.position < 0.011:
        goal.position = 0.011
    elif goal.position > 0.1:
        goal.position = 0.1

    if goal.speed == 0:
        goal.speed = 0.1
     
    if goal.force == 0:
        goal.force = 10


if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('robotiq_2f_client')
    rospy.Subscriber("gripper_command", RobotiqGripperCommand, gripperCB)

    goal = CommandRobotiqGripperGoal()
    result = operate_gripper()
    rospy.spin()