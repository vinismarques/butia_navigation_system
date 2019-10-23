import rospy
from std_msgs.msg import Bool

flag = False

def callBackEM(response):
    global flag
    if response.data:
        flag = True
    else:
        flag = False
        

if __name__ == "__main__":
    rospy.init_node("enable_motors")
    motors_sub = rospy.Subscriber("/RosAria/motors_state", Bool, callBackEM, queue_size=1)
    rospy.wait_for_service("/RosAria/enable_motors")
    try:
        enable_motors = rospy.ServiceProxy("/RosAria/enable_motors", Bool)
    except rospy.ServiceException, e:
        print "Service call failed %s"%e
    while not rospy.is_shutdown():
        if not flag:
            enable_motors(True)