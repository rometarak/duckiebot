#!/usr/bin/env python3

import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
import time
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, LEDPattern
from sensor_msgs.msg import Range
speed = WheelsCmdStamped()

x0 = y0 = 0 # meters
theta0 = 0 # radians

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        self.distance = 0.0

        self.time_left = 0
        self.time_right = 0

        self.left_encoder = 0.0
        self.right_encoder = 0.0

        self.ticks_left = 0
        self.ticks_right = 0

        self.prev_tick_left = 0
        self.prev_tick_right = 0

        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0

        self.delta_ticks_left = 0
        self.delta_ticks_right = 0

        self.L = 0.1 #  Distance between the center of the two wheels, expressed in meters

        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0

        self.prev_int = 0
        self.prev_e = 0

        self.omega = 0
        self.v0 = 0.5   #Kiirus

        self.e = 0
        self.e_int = 0
        self.e_der = 0

        self.theta_hat = 0
        self.delta_t = 1
        self.t0 = time.time()

        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Publisher('messageTopic', String, queue_size=10)
        rospy.Subscriber('/bestduckbot/front_center_tof_driver_node/range', Range, self.callback)
        rospy.Subscriber('/bestduckbot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_left_encoder)
        rospy.Subscriber('/bestduckbot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_right_encoder)
        rospy.Subscriber('/bestduckbot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.time_left)
        rospy.Subscriber('/bestduckbot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.time_right)
        rospy.Subscriber('/bestduckbot/led_emitter_node/led_pattern', LEDPattern, self.led_pattern)

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def callback(self, data):
        self.distance = data.range
    def callback_left_encoder(self, data):
        self.left_encoder = data.data
    def callback_right_encoder(self, data):
        self.right_encoder = data.data
    def time_left(self, data):
        self.time_left = data.header.seq
    def time_right(self, data):
        self.time_right = data.header.seq
    def led_pattern(self, data):
        self.led_pattern = data.header.seq

    def poseEstimation( self, R, # radius of wheel (assumed identical) - this is fixed in simulation, and will be imported from your saved calibration for the physical robot
        x_prev, # previous x estimate - assume given
        y_prev, # previous y estimate - assume given
        theta_prev, # previous orientation estimate - assume given
        delta_phi_left, # left wheel rotation (rad)
        delta_phi_right): # right wheel rotation (rad)
        """
            Calculate the current Duckiebot pose using dead reckoning approach.
            Returns x,y,theta current estimates:
                x_curr, y_curr, theta_curr (:double: values)
        """
        # Define wheel radii [m]
        # r = 0 # make different than zero if you have reason to believe the wheels are of different sizes.
        R_left = R # * (1-r)
        R_right = R # * (1+r)
        # Define distance travelled by each wheel [m]
        d_left = R_left * delta_phi_left
        d_right = R_right * delta_phi_right
        # Define distance travelled by the robot, in body frame [m]
        d_A = (d_left + d_right)/2
        # Define rotation of the robot [rad]
        Dtheta = (d_right - d_left)/self.L
        # Define distance travelled by the robot, in world frame [m]
        Dx = d_A * np.cos(theta_prev)
        Dy = d_A * np.sin(theta_prev)
        # Update pose estimate
        self.x_curr = x_prev + Dx
        self.y_curr = y_prev + Dy
        self.theta_curr = theta_prev + Dtheta

    def run(self):
        """
        Args:
            v_0 (:double:) linear Duckiebot speed (given).
            theta_ref (:double:) reference heading pose
            theta_hat (:double:) the current estiamted theta.
            prev_e (:double:) tracking error at previous iteration.
            prev_int (:double:) previous integral error term.
            delta_t (:double:) time interval since last call.
        returns:
            v_0 (:double:) linear velocity of the Duckiebot 
            omega (:double:) angular velocity of the Duckiebot
            e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
            e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
        """

        rate = rospy.Rate(20) # 20Hz

        self.prev_tick_left = self.left_encoder
        self.prev_tick_right = self.right_encoder
    
        while not rospy.is_shutdown():
            
            self.prev_e = self.e
            self.e_int = self.prev_int + self.e*self.delta_t   # integral of the error
            self.e = 4.5 - self.theta_hat   # Tracking 
            # anti-windup - preventing the integral error from growing too much
            self.e_int = max(min(self.e_int,2),-2)
            # derivative of the error
            self.e_der = (self.e - self.prev_e)/self.delta_t

            # controller coefficients
            #Kp = rospy.get_param("/p")
            #Ki = rospy.get_param("/i")
            #Kd = rospy.get_param("/d")
            Kp = 0.1233
            Ki = 0.022
            Kd = 10



            # PID controller for omega
            self.omega = Kp*self.e + Ki*self.e_int + Kd*self.e_der

            bus = SMBus(1)
            read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)
            
            #arvutab theta refi(keskpunkti välja, milleks on 4.5)
            line_values = []
            for i, value in enumerate(read):
                if value =='1':
                    line_values.append(i + 1)
            if len(line_values) >= 1:
                self.theta_hat = sum(line_values)/len(line_values)
        
            
        
            N_tot = 135 # total number of ticks per revolution
            alpha = 2 * np.pi / N_tot # wheel rotation per tick in radians

            print(f"The angular resolution of our encoders is: {np.rad2deg(alpha)} degrees")
            self.ticks_left = self.left_encoder
            self.ticks_right = self.right_encoder

            self.delta_ticks_left = self.ticks_left-self.prev_tick_left # delta ticks of left wheel 
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right # delta ticks of right wheel 

            self.rotation_wheel_left = alpha * self.delta_ticks_left # total rotation of left wheel 
            self.rotation_wheel_right = alpha * self.delta_ticks_right # total rotation of right wheel 

            self.ticks_right = self.right_encoder
            self.ticks_left = self.left_encoder

            self.delta_ticks_left = self.ticks_left-self.prev_tick_left # delta ticks of left wheel
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right # delta ticks of right wheel

            self.rotation_wheel_left = alpha * self.delta_ticks_left # total rotation of left wheel
            self.rotation_wheel_right = alpha * self.delta_ticks_right # total rotation of right wheel

            print(f"The left wheel rotated: {np.rad2deg(self.rotation_wheel_left)} degrees")
            print(f"The right wheel rotated: {np.rad2deg(self.rotation_wheel_right)} degrees")

            # What is the radius of your wheels? 
            R = 0.0335 # insert value measured by ruler, in *meters*

            # What is the distance travelled by each wheel?
            d_left = R * self.rotation_wheel_left 
            d_right = R * self.rotation_wheel_right

            print(f"The left wheel travelled: {d_left} meters")
            print(f"The right wheel rotated: {d_right} meters")
         
            # How much has the robot travelled? 
            d_A = (d_left + d_right)/2
            print(f"The robot has travelled: {d_A} meters")

            # What is the baseline length of your robot? 
            self.L = 0.1 #  Diste center of the two wheels, expressed in meters

            # How much has the robot rotated? ance between th
            Delta_Theta = (d_right-d_left)/self.L # expressed in radians
            print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
            print("-------------------------------", self.left_encoder)
            print("-------------------------------", self.right_encoder)
            
            speed.vel_left = self.v0 - self.omega
            speed.vel_right = self.v0 + self.omega
            

            #POOLIKUD KOODID SIIN ALL KOMMENTAARIDES
            """ if self.distance < 0.25:
                speed.vel_right = 0.0
                speed.vel_left = 0.3
            elif self.distance > 25 and abs(speed.vel_right - 0.3) < 0.01 and abs(speed.vel_left - 0) < 0.01:
                speed.vel_right = 0.3
                speed.vel_right = 0 """

                

            """ total_distance = d_A + 0.60
            target_distance = 0
            target_distance1 = 0
            target_distance2 = 0
            if self.distance < 0.25: # tof sensoriga 15 cm objektist peatub
                while self.distance < total_distance:
                    # 90 kraadi paremale
                    speed.vel_right = 0.0
                    speed.vel_left = 0.3
                    rospy.sleep(0.5)
                    # 15 cm otse
                    speed.vel_right = 0.3
                    speed.vel_left = 0
                    total_distance = d_A + 0.60
                    target_distance = d_A + 0.15
                    if d_A < target_distance:
                        self.omega = 0.0
                        self.v0 = 0.1
                    # 90 kraadi vasakule
                    elif d_A > target_distance:
                        speed.vel_right = 0.3
                        speed.vel_left = 0
                        rospy.sleep(1)
                        target_distance1 = d_A + 0.15
                        if d_A < target_distance1:
                            self.omega = 0.0
                            self.v0 = 0.1
                        elif d_A > target_distance1:
                            speed.vel_right = 0.3
                            speed.vel_left = 0
                            rospy.sleep(1)
                            target_distance2 = d_A + 0.15
                            if d_A < target_distance2:
                                self.omega = 0.0
                                self.v0 = 0.1
                            elif d_A > target_distance2:
                                speed.vel_right = 0
                                speed.vel_left = 0.3
                                rospy.sleep(1)
                                break """     
                

            #lihtsalt prindid, et infot kätte saada kogu aeg
            print(read)
            print(self.omega)
            print(self.distance)
            bus.close()
            self.pub.publish(speed)
            rate.sleep()

            t1 = time.time()
            self.delta_t = self.t0 - t1
            self.prev_int = self.e_int
            
    

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()


