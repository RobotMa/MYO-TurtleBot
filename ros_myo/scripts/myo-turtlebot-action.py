#!/usr/bin/env python

## get ros parameter '~guard_action' to accomplish 
## continuous gesture-velocity control

import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import MyoArm

########## Data Enums ###########
# MyoArm.arm___________________ #
#    UNKNOWN        = 0 	#
#    RIGHT          = 1		#
#    Left           = 2		#
# MyoArm.xdir___________________#
#    UNKNOWN        = 0		#
#    X_TOWARD_WRIST = 1		#
#    X_TOWARD_ELBOW = 2		#
# myo_gest UInt8________________#
#    REST           = 0		#
#    FIST           = 1		#
#    WAVE_IN        = 2		#
#    WAVE_OUT       = 3		#
#    FINGERS_SPREAD = 4		#
#    THUMB_TO_PINKY = 5		#
#    UNKNOWN        = 255	#
#################################
global manual 
manual = False

def myo_turtlebot_action():
    global manual
    tbPub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
    # controlPub = rospy.Publisher("/manual", String, queue_size=8)
    rospy.init_node('turtlebot_myo_action', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        # Use the received ros paramter to drive the turtlebot
        guard_action = rospy.get_param('/myo_turtlebot_move/guard_action')
        if guard_action == 5: # Trigger for manual/auto control
            manual = not manual

        if manual == True:
            rospy.set_param('/control_mode',"manual")
            if guard_action == 1:
                tbPub.publish(Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, -0)))
            elif guard_action == 2:
                tbPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.8)))
            elif guard_action == 3:
                tbPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.8)))
            elif guard_action == 4:
                tbPub.publish(Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0)))
        elif manual == False:
            rospy.set_param('/control_mode',"auto")

        rate.sleep()

if __name__ == '__main__':
    # manual = False
    try:
        myo_turtlebot_action()
    except rospy.ROSInterruptException:
        pass
