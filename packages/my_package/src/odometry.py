import numpy as np
import rospy
from duckietown_msgs.msg import WheelEncoderStamped

class Odometry():

    def __init__(self):
        self.L = 0.1

        rospy.Subscriber('/bestduckbot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_left_encoder)
        rospy.Subscriber('/bestduckbot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_right_encoder)

    def callback_left_encoder(self, data):
        self.left_encoder = data.data

    def callback_right_encoder(self, data):
        self.right_encoder = data.data

    def calculate_odometry(self):
        prev_tick_left = self.left_encoder
        prev_tick_right = self.right_encoder
    
        ticks_per_revolution = 135
        alpha = 2 * np.pi / ticks_per_revolution

        ticks_left = self.left_encoder
        ticks_right = self.right_encoder

        delta_ticks_left = ticks_left - prev_tick_left
        delta_ticks_right = ticks_right - prev_tick_right

        rotation_wheel_left = alpha * delta_ticks_left
        rotation_wheel_right = alpha * delta_ticks_right

        prev_tick_left = ticks_left
        prev_tick_right = ticks_right

        wheel_radius = 0.0335

        distance_left = wheel_radius * rotation_wheel_left
        distance_right = wheel_radius * rotation_wheel_right

        distance_traveled = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / self.L

        return distance_traveled, delta_theta

