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

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        self.distance = 0.0
        self.left_encoder = 0.0
        self.right_encoder = 0.0
        self.delta_t = 1
        self.omega = 0

        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/bestduckbot/front_center_tof_driver_node/range', Range, self.callback)
        rospy.Subscriber('/bestduckbot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_left_encoder)
        rospy.Subscriber('/bestduckbot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_right_encoder)
        rospy.Subscriber('/bestduckbot/led_emitter_node/led_pattern', LEDPattern, self.led_pattern)

    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    
    def shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)

    def callback(self, data):
        self.distance = data.range

    def callback_left_encoder(self, data):
        self.left_encoder = data.data
        
    def callback_right_encoder(self, data):
        self.right_encoder = data.data

    def led_pattern(self, data):
        self.led_pattern = data.header.seq
   
    def pid_controller(self):
        bus = SMBus(1)
        read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)
        
        #arvutab theta refi keskpunkti välja(otse on 4.5)
        line_values = []
        for i, value in enumerate(read):
            if value =='1':
                line_values.append(i + 1)
        if len(line_values) >= 1:
            theta_hat = sum(line_values)/len(line_values)
        if len(line_values) == 0:
            theta_hat = 4

        pose_estimation = 4.5
        prev_int = 0
 
        e = pose_estimation - theta_hat 
        e_int = prev_int + e*self.delta_t
        prev_int = e_int                                        #integral of the error
        prev_e = e                                              #Tracking
        e_int = max(min(e_int,2),-2)                            # anti-windup - preventing the integral error from growing too much       
        e_der = (e - prev_e)/self.delta_t                       #derivative of the error
        

        # controller coefficients
        #Kp = rospy.get_param("/p")
        #Ki = rospy.get_param("/i")
        #Kd = rospy.get_param("/d")
        Kp = 0.1233
        Ki = 0.022
        Kd = 10

        self.omega = Kp*e + Ki*e_int + Kd*e_der                 #PID controller for omega
        print(line_values)
    def run(self):
        #----------------------------------------------MUUTUJAD--------------------------------------------------------
        v0 = 0.5                                                #Kiirus
        L = 0.1                                                 #Distance between the center of the two wheels, expressed in meters
        #delta_t = 1
        t0 = time.time()

        prev_tick_left = self.left_encoder
        prev_tick_right = self.right_encoder
    
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
            bus = SMBus(1)
            #----------------------------------------------ODOMEETRIA--------------------------------------------------------
            N_tot = 135                                         #total number of ticks per revolution
            alpha = 2 * np.pi / N_tot                           #wheel rotation per tick in radians

            ticks_left = self.left_encoder
            ticks_right = self.right_encoder

            delta_ticks_left = ticks_left-prev_tick_left        # delta ticks of left wheel 
            delta_ticks_right = ticks_right-prev_tick_right     # delta ticks of right wheel 

            rotation_wheel_left = alpha * delta_ticks_left      # total rotation of left wheel 
            rotation_wheel_right = alpha * delta_ticks_right    # total rotation of right wheel 

            ticks_right = self.right_encoder
            ticks_left = self.left_encoder

            delta_ticks_left = ticks_left-prev_tick_left        # delta ticks of left wheel
            delta_ticks_right = ticks_right-prev_tick_right     # delta ticks of right wheel

            rotation_wheel_left = alpha * delta_ticks_left      #rotation_wheel_left = vasak ratas on kokku rotateerunud
            rotation_wheel_right = alpha * delta_ticks_right    #rotation_wheel_right = parem ratas on kokku rotateerunud

            R = 0.0335                                          #Rataste raadius meetrites
            d_left = R * rotation_wheel_left                    #d_left = Distants läbitud vasakul rattal
            d_right = R * rotation_wheel_right                  #d_right = Distants läbitud paremal rattal
         
            d_A = (d_left + d_right)/2                          #d_A = Roboti läbitud tee
        
            Delta_Theta = (d_right-d_left)/L                    #Delta_Theta = Mitu kraadi robot keeranud on
            #print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
            #print("-------------------------------", self.left_encoder)
            #print("-------------------------------", self.right_encoder)
            self.pid_controller()
            speed.vel_left = v0 - self.omega
            speed.vel_right = v0 + self.omega
            

            #----------------------------------------------KASTIST MÖÖDA MINEMINE--------------------------------------------------------
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
                        omega = 0.0
                        v0 = 0.1
                    # 90 kraadi vasakule
                    elif d_A > target_distance:
                        speed.vel_right = 0.3
                        speed.vel_left = 0
                        rospy.sleep(1)
                        target_distance1 = d_A + 0.15
                        if d_A < target_distance1:
                            omega = 0.0
                            v0 = 0.1
                        elif d_A > target_distance1:
                            speed.vel_right = 0.3
                            speed.vel_left = 0
                            rospy.sleep(1)
                            target_distance2 = d_A + 0.15
                            if d_A < target_distance2:
                                omega = 0.0
                                v0 = 0.1
                            elif d_A > target_distance2:
                                speed.vel_right = 0
                                speed.vel_left = 0.3
                                rospy.sleep(1)
                                break """     
                
            
            bus.close()
            self.pub.publish(speed)
            rate.sleep()

            t1 = time.time()
            self.delta_t = t0 - t1
            #prev_int = e_int

if __name__ == '__main__':
    speed = WheelsCmdStamped()
    x0 = y0 = 0 # meters
    theta0 = 0 # radians
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()