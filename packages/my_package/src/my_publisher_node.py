#!/usr/bin/env python3

# -----------------IMPORTED FILES---------------------
#import odometry
import change_lane
import pidcontroller
import around_the_box
from sensor_msgs.msg import Imu
import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
import time
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        self.linear_acceleration = 0.0
        self.distance = 8
        self.left_encoder = 0.0
        self.right_encoder = 0.0
        self.v0 = 0.2                                           #Kiirus
        self.L = 0.1                                            #Distance between the center of the two wheels, expressed in meters

        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/bestduckbot/front_center_tof_driver_node/range', Range, self.callback)
        rospy.Subscriber('/bestduckbot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_left_encoder)
        rospy.Subscriber('/bestduckbot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_right_encoder)
        rospy.Subscriber('/bestduckbot/imu_node/imu_data', Imu, self.imu_data)

    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    
    def shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)

    def imu_data(self, data):
        self.linear_acceleration = data.linear_acceleration.x
    
    def callback(self, data):
        self.distance = data.range

    def callback_left_encoder(self, data):
        self.left_encoder = data.data
        
    def callback_right_encoder(self, data):
        self.right_encoder = data.data

    def run(self):
        t0 = time.time()

        prev_tick_left = self.left_encoder
        prev_tick_right = self.right_encoder
    
        rate = rospy.Rate(25) # 20Hz
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

            rotation_wheel_left = alpha * delta_ticks_left      #rotation_wheel_left = vasak ratas on kokku rotateerunud
            rotation_wheel_right = alpha * delta_ticks_right    #rotation_wheel_right = parem ratas on kokku rotateerunud

            R = 0.0335                                          #Rataste raadius meetrites
            d_left = R * rotation_wheel_left                    #d_left = Distants läbitud vasakul rattal
            d_right = R * rotation_wheel_right                  #d_right = Distants läbitud paremal rattal
         
            d_A = (d_left + d_right)/2                          #d_A = Roboti läbitud tee
        
            Delta_Theta = (d_right-d_left)/self.L               #Delta_Theta = Mitu kraadi robot keeranud on
            
            #pidcontroller.pidcontroller() returnib omega
            speed.vel_left = self.v0 - pidcontroller.pid_controller()
            speed.vel_right = self.v0 + pidcontroller.pid_controller()
            
            if self.distance < 0.25:
                #Kutsun välja kastist mööda minemise funktsiooni
                #around_the_box.around_box()
                speed.vel_left = 0
                speed.vel_right = 0

            #Lühema raja valimine
            change_lane.change_lane()

            print(pidcontroller.pid_controller())
            #print(self.distance)
            #print("Linear Acceleration X: ", self.linear_acceleration)
            bus.close()
            self.pub.publish(speed)
            rate.sleep()

            t1 = time.time()
            self.delta_t = t0 - t1

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