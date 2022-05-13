#!/usr/bin/env python3
"""
This code is based on the code "kick_blue_ball.py" given at:
https://github.com/AlexandrLucas/COM3528/blob/master/
com3528_examples/src/kick_blue_ball.py

"""
# Imports
##########################
import os
from re import S
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library

# ROS Python interface
import rospy
from std_msgs.msg import Float32MultiArray
# ROS cmd_vel (velocity control) message
from geometry_msgs.msg import TwistStamped

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
        Get light sensor readings from Miro and store them so that they're
        accessable to the light sensing threshold devices
        Convert light sensor ROS message to a usable form
        Array gives [FRONT LEFT, FRONT RIGHT, REAR LEFT, REAR RIGHT] as sensor
         order
        """
        # Step 1. get light sensor intensity -> from callback
        # Convert ROS specific MultiArray format into python usable array
        intensity_data = intensity.data
        # Step 2. record the info for each light sensor
        self.light_left = intensity_data[0]
        self.light_right = intensity_data[1]

    def __init__(self):
        # PROPERTIES
        self.light_left = 0.0
        self.light_right = 0.0

        # TOPICS
        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("vehicle_4a", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # Create new subscriber to recieve light sensor, with associated callback
        self.light_sensor = rospy.Subscriber(
            topic_base_name + "/sensors/light",
            Float32MultiArray,
            self.callback_light_sens,
            queue_size=1
            )
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        # Choose a behaviour for the robot by choosing a deivce tree
        self.find_moderate_light()

    """
    DEVICES
    These methods define threshold device trees and set them as current robot
    behaviour.
    Each method must set self.left_wheel_driver and self.right_wheel_driver.
    A sensor tree must have only light sensors as its leaves.
    """
    def find_moderate_light(self):
        """
        With this, each wheel turns only if the corresponding sensor value is
        NOT between 0.4 and 0.6. The robot will find a moderately bright light
        and face towards it
        """
        sensor_1 = Light_Sensor(0.6, self, positive=False, side=1)
        sensor_2 = Light_Sensor(0.4, self, positive=True, side=1)
        sensor_3 = Light_Sensor(0.6, self, positive=False, side=2)
        sensor_4 = Light_Sensor(0.4, self, positive=True, side=2)
        left_inputs = [(sensor_1, True), (sensor_2, True)]
        right_inputs = [(sensor_3, True), (sensor_4, True)]
        self.left_wheel_driver = Threhold_Device(left_inputs, 1)
        self.right_wheel_driver = Threhold_Device(right_inputs, 1)

    def update_speeds(self):
        """
        Check the current values of the devices connected to the left and right
        wheels and set them to move or not
        """
        # The miro's max speed is 0.4
        left_speed = self.left_wheel_driver.get_output()*0.4
        right_speed = self.right_wheel_driver.get_output()*0.4
        self.drive(left_speed, right_speed)

    def loop(self):
        """
        Main control loop
        """
        print("MiRo implementation of Braitenberg Vehicle 5, press CTRL+C to "
              +"halt...")
        while not rospy.core.is_shutdown():
            rospy.sleep(self.TICK)
            self.update_speeds()

#------------------------------------------------------------------------------#
class Threhold_Device:
    """
    Threshold devices that can be combined to create various behaviour patterns
    """
    def __init__(self, inputs, threshold_val, positive=True):
        """
        Default constructor.
        Parameters:
        - inputs: A list of the other devices that feed into this one, as a
            tuple (device, inhibitory), where inhibitory is a boolean that
            declares whether the connection is inhibitory or not
        - threshold_val: The net number of positive signals needed for this
            this device to activate its threshold behaviour
        - positive: If True, the device activates above the threshold value; if
            False, it instead deactivates when above it
        """
        # Set properties
        self.inputs = inputs
        self.threshold_val = threshold_val
        self.positive = positive

    def get_output(self):
        """
        Gets the current output value of this node based on its input nodes
        """
        # Calculate the net input
        input = 0
        for device, positive in self.inputs:
            if positive:
                input += device.get_output()
            else:
                input -= device.get_output()
        # Choose what to return based on own behaviour setting
        if ((input > self.threshold_val and self.positive)
          or (input <= self.threshold_val and not self.positive)):
            return 1
        else:
            return 0


class Light_Sensor(Threhold_Device):
    """
    The light sensing variant of the the threshol device. Should be used to
    create input paths.
    """

    def __init__(self, threshold_val, parent, positive=True,
                 side=1):
        """
        Modified constructor for the light sensing variant. Needs to communicate
        with ROS since it reads the light value from the MIRO sensors
        Parameters
        - threshold_val: The LIGHT value at which the node should activate. Note
            that although light is technically in the range 0-1, in practice
            it's basically always 0.5-1
        - parent: The MiRoClient that created this device and therefore holds
            the light sensor values
        - positive: If True, the device activates above the threshold value; if
            False, it instead deactivates when below it
        - side: Which side camera to use. 1 for left, 2 for right
        """
        super().__init__([], threshold_val, positive)
        self.parent = parent
        self.side = side

    def get_output(self):
        """
        Gets the current output value of this node based on the current light
        levels and its defined behaviour
        """
        # Get the correct light sensor
        if self.side == 1:
            input = self.parent.light_left
        else:
            input = self.parent.light_right

        # Choose what to return based on own behaviour setting
        if ((input > self.threshold_val and self.positive)
          or (input <= self.threshold_val and not self.positive)):
            return 1
        else:
            return 0


# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop
