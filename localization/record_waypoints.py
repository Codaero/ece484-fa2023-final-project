import numpy as np
import pandas as pd
import rospy
import sys
import signal
import time
from datetime import datetime
from threading import Lock

from geometry_msgs.msg import PoseWithCovarianceStamped

# Global variables
waypoints = np.empty((0, 3), dtype=int)
latest_message = None

# Global constants
SAMPLING_FREQUENCY = 1

def save_waypoints_to_csv(ignored, ignored_again):
    """
    Behavior to execute when a ctrl-c signal is sent to the process
    """
    global waypoints
    fname = f"record_waypoints_{datetime.now().strftime('%d_%H_%M')}.csv"
    np.savetxt(fname, waypoints, delimiter=',', fmt='%f')
    print(f"\nSaved real-time recording to `{fname}`")
    sys.exit(0) # Exit cleanly


def ekf_callback(msg):
    """
    Sets latest message
    """
    global latest_message
    latest_message = msg


def quaternion_to_euler(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]


def record_waypoints():
    """
    Listens to the localization_pose topic and records the waypoints at 1Hz. Used to get an accurate csv of the track.
    """
    global latest_message
    global waypoints

    rospy.init_node('record_waypoints', anonymous=True)
    pose_subscriber = rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, ekf_callback)

    last_sample = None
    rate = rospy.Rate(SAMPLING_FREQUENCY) 
    while True:
        rate.sleep()

        # Check that we have a new sample, otherwise sleep for another second
        if latest_message is None or (last_sample is not None and latest_message.header.seq == last_sample.header.seq):
            continue

        # Add new sample to list of waypoints
        pose = latest_message.pose.pose
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        euler = quaternion_to_euler(*quat)
        waypoints = np.append(waypoints, [[pose.position.x, pose.position.y, euler[2]]], axis=0)
        last_sample = latest_message


if __name__ == "__main__":
    signal.signal(signal.SIGINT,save_waypoints_to_csv) # Register our ctrl-c handler
    try:
        record_waypoints()
    except rospy.ROSInterruptException:
        save_waypoints_to_csv(0, 0) # inputs ignored