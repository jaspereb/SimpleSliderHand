#!/usr/bin/env python  
import roslib
roslib.load_manifest('parallel_gripper')
import rospy
import actionlib
import parallel_gripper.msg
import serial
import signal
import sys
import time

ser = serial.Serial('/dev/serial/by-id/usb-ROBOTIS_CO._LTD._ROBOTIS_Virtual_COM_Port-if00', 57600, timeout=1)  # open serial port
forceThreshold = 400 #Threshold to consider something being gripped

def signal_handler(sig, frame):
    print('Closing Gripper Server')
    # open gripper before exiting
    ser.write(str.encode('o\n'))
    ser.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class GripperAction(object):
    # create messages that are used to publish feedback/result
    _feedback = parallel_gripper.msg.actuateGripperFeedback()
    _result = parallel_gripper.msg.actuateGripperResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, parallel_gripper.msg.actuateGripperAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        #print("Got gripper request to:")
        #print(goal.actionType)

        #Goal: 0-Open, 1-Closed, 2-Mid
        if(goal.actionType == 'c'):
            ser.write(str.encode('c\n'))
            rospy.loginfo("Closing gripper")
        elif(goal.actionType == 'o'):
            ser.write(str.encode('o\n'))
            rospy.loginfo("Opening gripper")
        elif(goal.actionType == 'd'):
            cmdStr = 'd{}\n'.format(goal.actionValue)
            ser.write(str.encode(cmdStr))
            rospy.loginfo("Moving gripper to width of {}mm".format(goal.actionValue))
        elif(goal.actionType == 't'):
            if(goal.actionValue > 1023):
                goal.actionValue = 1023
            cmdStr = 't{}\n'.format(goal.actionValue)
            ser.write(str.encode(cmdStr))
            rospy.loginfo("Setting gripper torque to {}".format(goal.actionValue))
        elif(goal.actionType == 's'):
            if(goal.actionValue > 1023):
                goal.actionValue = 1023
            cmdStr = 's{}\n'.format(goal.actionValue)
            ser.write(str.encode(cmdStr))
            rospy.loginfo("Setting gripper speed to {}".format(goal.actionValue))
        else:
            rospy.logerr("Unknown gripper action goal")

        #While velocity is non-zero, wait
        velocity = 1000
        while(abs(velocity) > 10):
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(str.encode('i\n'))
            status = ser.readline()
            status = status.split(",")
            velocity = int(status[1])
            position = int(status[0])
            finalForce = int(status[2])
            self._feedback.velocity = velocity
            self._as.publish_feedback(self._feedback)

        if(finalForce > forceThreshold):
            applyingForce = 1
        else:
            applyingForce = 0

        self._result.finalForce = finalForce
        self._result.applyingForce = applyingForce
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('gripper')
    server = GripperAction(rospy.get_name())
    print("Created gripper server")
    rospy.spin()
