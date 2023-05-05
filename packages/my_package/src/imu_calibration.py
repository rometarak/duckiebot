#!/usr/bin/env python3
import time
from sensor_msgs.msg import Imu
from duckietown.dtros import DTROS, NodeType
import rospy
from nav_msgs.msg import Odometry
from math import sin, cos
import tf 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

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
        self.current_time = 0.0
        self.last_time = 0.0
        
        
        super(ImuCalibration, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.Subscriber('/bestduckbot/imu_node/imu_data', Imu, self.imu_data)

    def imu_data(self, data):
        self.angular_velocity_x = data.angular_velocity.x
        self.angular_velocity_y = data.angular_velocity.y
        self.angular_velocity_z = data.angular_velocity.z
        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_y = data.linear_acceleration.y
        self.linear_acceleration_z = data.linear_acceleration.z
        self.current_time = time.time()
        
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

        x = 0.0
        distance_distance_y = 0.0
        distance_distance_x = 0.0
        y = 0.0
        th = 0.0
        delta_time = 0.0

        rate = rospy.Rate(15) # 10hz
        while not rospy.is_shutdown():
            rospy.set_param("~ang_vel_offset", [self.angular_velocity_x/1000, self.angular_velocity_y/1000,self.angular_velocity_z/1000])
            rospy.get_param("~accel_offset", [self.linear_acceleration_x/1000,self.linear_acceleration_y/1000,self.linear_acceleration_z/1000])
            #print("---------------------------------------------------")
            #print("angular x keskmine on: ", self.angular_x/1000)
            #print("angular y keskmine on: ", self.angular_y/1000)
            #print("angular z keskmine on: ", self.angular_z/1000)
            #print("LINEAR X KESKMINE ON: ", self.linear_x/1000)
            #print("LINEAR Y KESKMINE ON: ", self.linear_y/1000)
            #print("LINEAR Z KESKMINE ON: ", self.linear_z/1000)
            acc_y = self.linear_acceleration_y
            acc_x = self.linear_acceleration_x
            vel_z = self.angular_velocity_z

            delta_time = self.current_time - self.last_time
            dth = vel_z * delta_time
            th = th + dth

            #Y telg
            if delta_time < 1600000000.0 and delta_time != 0:
                dvy = acc_y * delta_time
                if dvy >= -0.2 and dvy <= 0.1:
                    vy = 0
                vy = round(vy + dvy,1)
                distance_y = round(vy,2) * delta_time
                distance_distance_y = distance_distance_y + (distance_y*cos(th))

                #print("kiirendus: ",acc_y,"\n","delta_time: ",delta_time,"\n","kiirus: ",vx)
                print("y distants ",distance_distance_y)

            #X telg
            if delta_time < 1600000000.0 and delta_time != 0:
                dvx = acc_x * delta_time
                if dvx >= -0.2 and dvx <= 0.1:
                    vx = 0
                vx = round(vx + dvx,1)
                distance_x = round(vx,2) * delta_time
                distance_distance_x = distance_distance_x + (distance_x*cos(th))

                #print("kiirendus: ",acc_x,"\n","delta_time: ",delta_time,"\n","kiirus: ",vx)
                print("x distants", distance_distance_x)
                                                                                                 #KIIRUSE ARVUTAMISE VALEMID
                                                                                                   #speed = distance ÷ time. (speed = kiirendus * deltatime)
                                                                                                    #distance = speed × time.
                                                                                                    #time = distance ÷ speed.
            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )
            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x,y,0), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(acc_x, vel_z, 0), Vector3(0, 0, vel_z))
            pub.publish(odom)
            
            self.last_time = self.current_time
            #vx = 0
            rate.sleep()
            
if __name__ == '__main__':
    # create the node
    node = ImuCalibration(node_name='imucalibration')
    # run node
    node.run()
    # keep spinning
    rospy.spin()








    #acc_x = kiirus
    #accx = kiirendus
    #dacc_x = kiiruse muutus
    #dt = deltatime
    #x = distants
    #dx = distantsi muutus
