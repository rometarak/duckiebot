#!/usr/bin/env python3
import time
from sensor_msgs.msg import Imu
from duckietown.dtros import DTROS, NodeType
import rospy
from nav_msgs.msg import Odometry
import math
from math import sin, cos, pi
import tf 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from smbus2 import SMBus

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
        pub = rospy.Publisher('odom', Odometry, queue_size=10)
        odom_broadcaster = tf.TransformBroadcaster()
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        x = 0.0
        y = 0.0
        th = 0.0
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #print("---------------------------------------------------")
            #print("angular x keskmine on: ", self.angular_x/1000)
            #rint("angular y keskmine on: ", self.angular_y/1000)
            #print("angular z keskmine on: ", self.angular_z/1000)
            #print("LINEAR X KESKMINE ON: ", self.linear_x/1000)
            #print("LINEAR Y KESKMINE ON: ", self.linear_y/1000)
            #print("LINEAR Z KESKMINE ON: ", self.linear_z/1000)
            gyro_xout = self.angular_x / 1000 #EI KASUTA
            gyro_yout = self.angular_y #EI KASUTA
            gyro_zout = self.angular_z

            acc_xout = self.linear_x
            acc_yout = self.linear_y
            acc_zout = self.linear_z / 1000 #EI KASUTA

            #angular
            #gyroskop_xout = self.read_word_2c(0x43)
            #gyroskop_yout = self.read_word_2c(0x45)
            #gyroskop_zout = self.read_word_2c(0x47)

            #acceleration
            #accleration_xout = self.read_word_2c(0x3b)
            #accleration_yout = self.read_word_2c(0x3d)
            #accleration_zout = self.read_word_2c(0x3f)

            accleration_xout_skaliert = acc_xout
            accleration_yout_skaliert = acc_yout
            accleration_zout_skaliert = acc_zout
            current_time = rospy.Time.now()

            vx = accleration_xout_skaliert 
            vy = accleration_yout_skaliert
            vth = gyro_zout
            dt = (current_time - last_time).to_sec()
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )
            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
            pub.publish(odom)
            last_time = current_time

if __name__ == '__main__':
    # create the node
    node = ImuCalibration(node_name='imucalibration')
    # run node
    node.run()
    # keep spinning
    rospy.spin()