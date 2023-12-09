#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

def publish_fake_message():
    """
    Publishes a fake /rtabmap/localization_pose message. Used for testing.
    """
    rospy.init_node('fake_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/rtabmap/localization_pose', PoseWithCovarianceStamped, queue_size=10)
    
    rate = rospy.Rate(5)  # Adjust the rate as needed (e.g., every 0.2 seconds)

    while not rospy.is_shutdown():
        fake_pose_msg = PoseWithCovarianceStamped()
        # Populate the fake message with desired values
        # For example:
        fake_pose_msg.header.stamp = rospy.Time.now()
        fake_pose_msg.pose.pose.position.x = 1.0
        fake_pose_msg.pose.pose.position.y = 2.0
        fake_pose_msg.pose.pose.position.z = 3.0

        # Publish the fake message
        pub.publish(fake_pose_msg)

        # Sleep for the desired interval
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_message()
    except rospy.ROSInterruptException:
        pass
