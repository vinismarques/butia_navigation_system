#! /usr/bin/env python3
 
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import LaserScan
import math
import statistics

FRONT_STATE = {
    'NOTHING': 0,
    'OPEN': 1,
    'CLOSED': 2,
}

ANGLE_GAP = 8
VARIATION_THRESHOLD = 1.0
compute_reference = False
reference_distance = 0.0
last_front_state = FRONT_STATE['NOTHING'] #0 = nothing, 1 = open, 2 = closed

def handle_reset_reference_distance(req):
    global compute_reference
    compute_reference = True
    return EmptyResponse()

def compute_min_and_max_ranges(msg):
    angle_gap = math.radians(ANGLE_GAP)
    
    if msg.angle_min < 0:
        angle_min = msg.angle_min*(-1)
    else:
        angle_min = msg.angle_min
    
    angle_max = msg.angle_max
    angle_range = angle_min + angle_max
    
    rpr = (len(msg.ranges)/angle_range) #readings per radians
    range_gap = rpr*angle_gap
    range_position_min = int((len(msg.ranges)/2)-range_gap)
    range_position_max = int((len(msg.ranges)/2)+range_gap)

    return range_position_min, range_position_max

def compute_reference_distance(msg):
    range_position_min, range_position_max = compute_min_and_max_ranges(msg)

    ranges = msg.ranges[range_position_min:range_position_max+1]
    ranges_median = statistics.median(ranges)
    reference_distance = ranges_median
    return reference_distance

def callback(msg):
    global compute_reference
    global last_front_state
    global reference_distance

    if compute_reference:
        reference_distance = compute_reference_distance(msg)
        compute_reference = False
        last_front_state = FRONT_STATE['NOTHING'] 
    else:
        range_position_min, range_position_max = compute_min_and_max_ranges(msg)
        ranges = msg.ranges[range_position_min:range_position_max+1]
        ranges_median = statistics.median(ranges)

        if ranges_median > (reference_distance + VARIATION_THRESHOLD):
            reference_distance = 0.0
            if last_front_state == FRONT_STATE['NOTHING'] or last_front_state == FRONT_STATE['CLOSED']:
                pub.publish(Bool(True))
                rospy.loginfo('FRONT IS OPEN.')
                last_front_state = FRONT_STATE['OPEN']
        else:
            if last_front_state == FRONT_STATE['NOTHING'] or last_front_state == FRONT_STATE['OPEN']:
                pub.publish(Bool(False))
                rospy.loginfo('FRONT IS CLOSED.')
                last_front_state = FRONT_STATE['CLOSED']

if __name__ == '__main__':
    rospy.init_node('verify_front_state')
    ANGLE_GAP = rospy.get_param('/butia_navigation/bns/angle_gap', ANGLE_GAP)
    VARIATION_THRESHOLD = rospy.get_param('/butia_navigation/bns/variation_threshold', VARIATION_THRESHOLD)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/butia_navigation/bns/front_state', Bool, queue_size=1)
    server = rospy.Service('/butia_navigation/bns/reset_reference_distance', Empty, handle_reset_reference_distance)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()