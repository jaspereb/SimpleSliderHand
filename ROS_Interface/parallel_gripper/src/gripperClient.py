#! /usr/bin/env python

# Example client for using the gripper. The feedback and result variables are not yet implemented. 
# The state (goal) consists of a letter followed by a number
# c - close
# o - open
# dxxxx - distance, in mm
# txxxx - torque, out of 1023
# sxxxx - speed, out of 1023

# Feedback value is the velocity (0 to 2047)
# Returned value is the final load (0 to 2047) and flag if that is above a threshold

import rospy
import actionlib
import parallel_gripper.msg
import time
import signal
import sys

def signal_handler(sig, frame):
    print('Closing Gripper Client')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def gripper_client(actionType,actionValue):
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('gripper', parallel_gripper.msg.actuateGripperAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for gripper server")
    client.wait_for_server()
    print("Connected to gripper server")
    
    # Creates a goal to send to the action server.
    goal = parallel_gripper.msg.actuateGripperGoal(actionType=actionType, actionValue=actionValue)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('gripper_client_py')
        actionValue = 0
        print("Press Enter to Move Gripper")

        while(1):
            #Open
            raw_input()
            actionType = 'o'
            result = gripper_client(actionType,actionValue)
            print(result)

            #Set Torque
            raw_input()
            actionType = 't'
            actionValue = 400
            result = gripper_client(actionType,actionValue)
            print(result)

            #Close
            raw_input()
            actionType = 'c'
            result = gripper_client(actionType,actionValue)
            print(result)

            #Open
            raw_input()
            actionType = 'o'
            result = gripper_client(actionType,actionValue)
            print(result)

            #Close to Width 100mm
            raw_input()
            actionType = 'd'
            actionValue = 100
            result = gripper_client(actionType,actionValue)
            print(result)

            #Open
            raw_input()
            actionType = 'o'
            result = gripper_client(actionType,actionValue)
            print(result)
        
    except rospy.ROSInterruptException:
        rospy.logerr("Gripper client interrupted by ROS")
