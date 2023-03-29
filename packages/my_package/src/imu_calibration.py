#!/usr/bin/env python3
import time
import rospy
from sensor_msgs.msg import Imu
from duckietown.dtros import DTROS, NodeType

class ImuCalibration(DTROS):
    def __init__(self, node_name):
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0
        self.linear_acceleration_x = 0.0
        self.linear_acceleration_y = 0.0
        self.linear_acceleration_z = 0.0
        self.i = 0
        super(ImuCalibration, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/bestduckbot/imu_node/imu_data', Imu, self.imu_data)

    def imu_data(self, data):
        self.angular_velocity_x = data.angular_velocity.x
        self.angular_velocity_y = data.angular_velocity.y
        self.angular_velocity_z = data.angular_velocity.z
        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_y = data.linear_acceleration.y
        self.linear_acceleration_z = data.linear_acceleration.z

        
        while self.i<=1000:
            self.i+=1
            self.linear_x += self.linear_acceleration_x 
            self.linear_y += self.linear_acceleration_y
            self.linear_z += self.linear_acceleration_z

            self.angular_x += self.angular_velocity_x
            self.angular_y += self.angular_velocity_y
            self.angular_z += self.angular_velocity_z

    def run(self):
        while not rospy.is_shutdown():
            print("---------------------------------------------------")
            print("angular x keskmine on: ", self.angular_x/1000)
            print("angular y keskmine on: ", self.angular_y/1000)
            print("angular z keskmine on: ", self.angular_z/1000)
            print("LINEAR X KESKMINE ON: ", self.linear_x/1000)
            print("LINEAR Y KESKMINE ON: ", self.linear_y/1000)
            print("LINEAR Z KESKMINE ON: ", self.linear_z/1000)
            time.sleep(5)

if __name__ == '__main__':
    # create the node
    node = ImuCalibration(node_name='imucalibration')
    # run node
    node.run()
    # keep spinning
    rospy.spin()