#! /usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty

MOTOR_STATE = True

def motor_callback(msg):
    global MOTOR_STATE
    MOTOR_STATE = msg.data

if __name__=='__main__':
    rospy.init_node('emergency_recovery')
    sub = rospy.Subscriber('/RosAria/motors_state', Bool, motor_callback)

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        if MOTOR_STATE == False:
            rospy.wait_for_service('/RosAria/enable_motors')
            try:
                enable_motors_serv = rospy.ServiceProxy('/RosAria/enable_motors', Empty)
                resp1 = enable_motors_serv()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        r.sleep()