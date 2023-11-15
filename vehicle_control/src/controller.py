import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from simple_pid import PID
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler

import time


def pi_clip(angle):
    """function to map angle error values between [-pi, pi]"""
    if angle > 0:
        if angle > math.pi:
            return angle - 2*math.pi
    else:
        if angle < -math.pi:
            return angle + 2*math.pi
    return angle

class vehicleController():

    pid_angle = PID(-2, -0.5, -0.0, setpoint=0.0, output_limits=(-5, 5))
    pid_angle.error_map = pi_clip #function to map angle errror values between -pi and pi

    pid_velocity = PID(-20, -10, -0.0, setpoint=0.0, output_limits=(0, 2))

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        self.prev_alpha = 0
        # self.dt = 


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)

    def get_images(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def compute_lateral_error(self, image):
        


        return angle_error


    def longititudal_PID_controller(self, lateral_error):

        target_velocity = self.pid_velocity(lateral_error, dt=self.dt)

        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def lateral_PID_controller(self, angle_error):

        steering_angle = self.pid_angle(angle_error, dt = self.dt)

        return steering_angle


    def execute(self):

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz

        # print(acceleration)
        self.prev_vel = curr_vel

        print(curr_x, curr_y)

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)