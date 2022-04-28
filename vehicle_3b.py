#!/usr/bin/env python3
"""
This code is based on the code "kick_blue_ball.py" given at:
https://github.com/AlexandrLucas/COM3528/blob/master/com3528_examples/src/kick_blue_ball.py

"""
# Imports
##########################
import os
from re import S
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library

import rospy  # ROS Python interface
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message

import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################

class MiRoClient:
    """
    Script settings below
    """
    TICK = 0.02  # This is the update interval for the main control loop in secs
    ##NOTE The following option is relevant in MiRoCODE
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        Unmodified from kick_blue_ball.py
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.vel_pub.publish(msg_cmd_vel)

    def callback_light_sens(self, intensity):
        """
        Get light sensor readings from Miro
        Convert light sensor ROS message to a usable form
        Array gives [FRONT LEFT, FRONT RIGHT, REAR LEFT, REAR RIGHT] as sensor order
        """
        # Step 1. get light sensor intensity -> from callback
        # Convert ROS specific MultiArray format into python usable array
        intensity_data = intensity.data

        # Step 2. convert intensity into usable movement speed
        # For vehicle 3b, the LEFT sensor value is proportional to the RIGHT motor speed, and vice versa
        # So just using the front sensors:
        f_left_intensity = intensity_data[0]
        f_right_intensity = intensity_data[1]
        
        speed_left  = 1/f_right_intensity
        speed_right  = 1/f_left_intensity

        # Step 3. execute movement
        self.drive(speed_left, speed_right)

    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("vehicle_3b", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # Create new subscriber to recieve light sensor, with associated callback
        self.sub_light_sens = rospy.Subscriber(
            topic_base_name + "/sensors/light", 
            Float32MultiArray, 
            self.callback_light_sens, 
            queue_size=1
        )
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

    def loop(self):
        """
        Main control loop
        """

        print("MiRo implementation of Braitenberg Vehicle 3b, press CTRL+C to halt...")
        while not rospy.core.is_shutdown():
            rospy.sleep(self.TICK)


# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop
