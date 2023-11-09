#!/usr/bin/env python
import rospy

CAMERA_TOPIC = ""

def lane_det_node():
    rospy.init_node('lane_det_node')

    # rospy.Subscriber(CAMERA_TOPIC)

    rospy.spin()

if __name__ == '__main__':
    lane_det_node()