#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

## Persistent problem with indentation 

import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import MyoArm

########## Data Enums ###########
# MyoArm.arm___________________ #
#    UNKNOWN        = 0     #
#    RIGHT          = 1     #
#    Left           = 2     #
# MyoArm.xdir___________________#
#    UNKNOWN        = 0     #
#    X_TOWARD_WRIST = 1     #
#    X_TOWARD_ELBOW = 2     #
# myo_gest UInt8________________#
#    REST           = 0     #
#    FIST           = 1     #
#    WAVE_IN        = 2     #
#    WAVE_OUT       = 3     #
#    FINGERS_SPREAD = 4     #
#    THUMB_TO_PINKY = 5     #
#    UNKNOWN        = 255   #
#################################


if __name__ == '__main__':

    global armState
    global xDirState
    armState = 0;
    rospy.init_node('turtlebot_myo_move', anonymous=True)

    turtlebotPub = rospy.Publisher("directs", String, queue_size=10)


    # set the global arm states
    def setArm(data):

        armState = data.arm
        xDirState = data.xdir
    rospy.sleep(0.1)

    # Use the calibrated Myo gestures to drive the turtle
    def drive(gest):
    rospy.set_param('~guard_action',gest.data)

        if gest.data == 1: # FIST
        turtlebotPub.publish("go back")
    elif gest.data == 2: # WAVE_IN, RIGHT arm
        turtlebotPub.publish("go left")
        elif gest.data == 2 and armState == 2: #WAVE_IN, LEFT arm
        turtlebotPub.publish("go right")
        # elif gest.data == 3 and armState == 1: #WAVE_OUT, RIGHT arm
    elif gest.data == 3: # WAVE_OUT, RIGHT arm
        turtlebotPub.publish("go right")
        elif gest.data == 3 and armState == 2: # WAVE_OUT, LEFT arm
        turtlebotPub.publish("go left")
        elif gest.data == 4: # FINGERS_SPREAD
        turtlebotPub.publish("go forward")

    rospy.Subscriber("myo_arm", MyoArm, setArm)
    rospy.Subscriber("myo_gest", UInt8, drive)
    rospy.loginfo('Please sync the Myo')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

