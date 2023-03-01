#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
from smbus2 import SMBus
speed = WheelsCmdStamped()
class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def run(self):
        flag = 0
        # publish message every 1 second
        rate = rospy.Rate(20) # 1Hz
        while not rospy.is_shutdown():

            bus = SMBus(1)
            read = int('{:08b}'.format(bus.read_byte_data(62, 17))[::-1], 2)

            if read <= 223 and read > 0:
                speed.vel_left = 0.6
                speed.vel_right = 0
                flag = 0

            if read >= 251 and read < 255:
                speed.vel_left = 0
                speed.vel_right = 0.6
                flag = 1

            if read > 223 and read < 251:
                speed.vel_left = 0.7
                speed.vel_right = 0.7
                
            if read == 0:
                speed.vel_right = -0.3
                speed.vel_left = -0.3

            if read == 255 and flag == 1:
                speed.vel_left = 0
                speed.vel_right = 0.6
            if read == 255 and flag == 0:
                speed.vel_right = 0
                speed.vel_left = 0.6

            print(read)
            bus.close()
            self.pub.publish(speed)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()







if read <= 4 and read > 0:
                speed.vel_left = 0
                speed.vel_right = float(-0.2)
            elif read >= 32 and read < 255:
                speed.vel_right = 0
                speed.vel_left = float(-0.2)
            elif read > 4 and read < 32:
                speed.vel_left = float(-0.2)
                speed.vel_right = float(-0.2)
            elif read == 0 or read == 255:
                speed.vel_left = 0
                speed.vel_right = 0
            self.pub.publish(speed)
            rate.sleep()
