#!/usr/bin/env python3

# -----------------IMPORTED FILES---------------------
#import odometry
#import imu_calibration
#import change_lane
import pidcontroller
import around_the_box
import change_lane
import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
import time
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import sys

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        self.distance = 8
        self.left_encoder = 0.0
        self.right_encoder = 0.0
        self.v0 = 0.3                                           #Kiirus
        self.L = 0.1                                            #Distance between the center of the two wheels, expressed in meters
        self.flag = 0
        self.turn_left = [[1,2,3,4],[1,2,3,4,5],[1,2,3,4,5,6]]
        self.turn_right = [[5,6,7,8],[4,5,6,7,8],[3,4,5,6,7,8]]
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        rospy.Subscriber('/bestduckbot/front_center_tof_driver_node/range', Range, self.callback)
        rospy.Subscriber('/bestduckbot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_left_encoder)
        rospy.Subscriber('/bestduckbot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.callback_right_encoder)

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
    
    def run(self):
        t0 = time.time()

        prev_tick_left = self.left_encoder
        prev_tick_right = self.right_encoder
    
        rate = rospy.Rate(25) # 20Hz
        while not rospy.is_shutdown():
            bus = SMBus(1)
            #----------------------------------------------ODOMEETRIA--------------------------------------------------------
            #N_tot = 135                                         #total number of ticks per revolution
            #alpha = 2 * np.pi / N_tot                           #wheel rotation per tick in radians

            #ticks_left = self.left_encoder
            #ticks_right = self.right_encoder

            #delta_ticks_left = ticks_left-prev_tick_left        # delta ticks of left wheel 
            #delta_ticks_right = ticks_right-prev_tick_right     # delta ticks of right wheel 

            #rotation_wheel_left = alpha * delta_ticks_left      # total rotation of left wheel 
            #rotation_wheel_right = alpha * delta_ticks_right    # total rotation of right wheel 

            #ticks_right = self.right_encoder
            #ticks_left = self.left_encoder

            #rotation_wheel_left = alpha * delta_ticks_left      #rotation_wheel_left = vasak ratas on kokku rotateerunud
            #rotation_wheel_right = alpha * delta_ticks_right    #rotation_wheel_right = parem ratas on kokku rotateerunud

            #R = 0.0335                                          #Rataste raadius meetrites
            #d_left = R * rotation_wheel_left                    #d_left = Distants läbitud vasakul rattal
            #d_right = R * rotation_wheel_right                  #d_right = Distants läbitud paremal rattal
         
            #d_A = (d_left + d_right)/2                          #d_A = Roboti läbitud tee
        
            #Delta_Theta = (d_right-d_left)/self.L               #Delta_Theta = Mitu kraadi robot keeranud on

            #90 kraadiste nurkade pööramise loogika
            flag = 0
            if pidcontroller.get_line_values() in self.turn_right:
                speed.vel_left = 0.4
                speed.vel_right = 0
                self.pub.publish(speed)
                flag = 1
            if pidcontroller.get_line_values() in self.turn_left:
                speed.vel_left = 0
                speed.vel_right = 0.4
                self.pub.publish(speed)
                flag = 0
            
            if pidcontroller.get_line_values() == [] and flag == 1:
                speed.vel_left = 0.4
                speed.vel_right = 0
                self.pub.publish(speed)
            if pidcontroller.get_line_values() == [] and flag == 0:
                speed.vel_left = 0
                speed.vel_right = 0.4
                self.pub.publish(speed)
            if pidcontroller.get_line_values() == []:
                speed.vel_left = 0.3
                speed.vel_right = 0.3
            t1 = time.time()
            #pidcontroller.pidcontroller() returnib omega
            speed.vel_left = self.v0 - pidcontroller.pid_controller(t0,t1)
            speed.vel_right = self.v0 + pidcontroller.pid_controller(t0,t1)
            
            if self.distance < 0.25:
                #Kutsun välja kastist mööda minemise funktsiooni
                around_the_box.around_box()

            #Lühema raja valimine
            line_values = [[1,4],[1,2,4],[1,4,5],[1,2,4,5],[2,3,5,6],[2,3,6]]
            if pidcontroller.get_line_values() in line_values:
                change_lane.change_lane()
        
            bus.close()
            self.pub.publish(speed)
            rate.sleep()

            

if __name__ == '__main__':
    speed = WheelsCmdStamped()
    #x0 = y0 = 0 # meters
    #theta0 = 0 # radians
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()