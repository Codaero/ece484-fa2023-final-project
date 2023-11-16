import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from simple_pid import PID
import math
from util import euler_to_quaternion, quaternion_to_euler
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
from std_msgs.msg import String, Bool, Float32, Float64, Float32MultiArray

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt

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

        self.dt = 0.1
        self.rate = rospy.Rate(30)
        self.target_throttle = 0.3
        self.lane_orientation_sub = rospy.Subscriber('/lane_orientation', Float32, self.control_callback)
        self.lane_orientation = 0.0
        # self.steering_angle = 0.0
        # self.ackermann_msg = AckermannDrive()
        # self.ackermann_msg.steering_angle = 0.0



        # -------------------- PACMod setup --------------------
        self.gem_enable    = False
        self.pacmod_enable = True

        # GEM vehicle enable
        self.enable_sub = rospy.Subscriber('/pacmod/as_rx/enable', Bool, self.pacmod_enable_callback)
        # self.enable_cmd = Bool()
        # self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second



    def control_callback(self, msg):
        self.lane_orientation = msg.data
        print(self.lane_orientation)

    # PACMod enable callback function
    def pacmod_enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def compute_lateral_error(self, image):
        pass

    def longititudal_PID_controller(self, lateral_error):

        target_velocity = self.pid_velocity(lateral_error, dt=self.dt)
        pass

    # Task 3: Lateral Controller (Pure Pursuit)
    def lateral_PID_controller(self, lane_orentation):
        steering_angle = self.pid_angle(lane_orentation, dt = self.dt) #radians
        return steering_angle

    
    # Start PACMod interface
    def start_pacmod(self):
        print("test")
        if(self.pacmod_enable == True):
            print("running 1")
            if (self.gem_enable == False):
                print("running 2")
                # ---------- Enable PACMod ----------

                # enable forward gear
                self.gear_cmd.ui16_cmd = 3

                # enable brake
                self.brake_cmd.enable  = True
                self.brake_cmd.clear   = False
                self.brake_cmd.ignore  = False
                self.brake_cmd.f64_cmd = 0.0

                # enable gas 
                self.accel_cmd.enable  = True
                self.accel_cmd.clear   = False
                self.accel_cmd.ignore  = False
                self.accel_cmd.f64_cmd = 0.0

                self.gear_pub.publish(self.gear_cmd)
                print("Foward Engaged!")

                self.turn_pub.publish(self.turn_cmd)
                print("Turn Signal Ready!")
                
                self.brake_pub.publish(self.brake_cmd)
                print("Brake Engaged!")

                self.accel_pub.publish(self.accel_cmd)
                print("Gas Engaged!")

                self.gem_enable = True

            else: 
                steering_angle = self.lateral_PID_controller(self.lane_orientation)
                print ("running 3")
                if (steering_angle <= 45 and steering_angle >= -45):
                    self.turn_cmd.ui16_cmd = 1
                elif(steering_angle > 45):
                    self.turn_cmd.ui16_cmd = 2 # turn left
                else:
                    self.turn_cmd.ui16_cmd = 0 # turn right

                self.accel_cmd.f64_cmd = 0.33
                self.steer_cmd.angular_position = 0

                self.accel_pub.publish(self.accel_cmd)
                self.steer_pub.publish(self.steer_cmd)
                
        self.rate.sleep()


def execute():
    rospy.init_node('pacmod_control_node', anonymous=True)
    controller = vehicleController()

    while not rospy.core.is_shutdown():
        try:
            controller.start_pacmod()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    execute()