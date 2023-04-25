#!/usr/bin/env python3

#-------------------------------------- IMPORTIMISED ----------------------------------------------#
import os
import rospy
import numpy as np
from time import sleep
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, ColorRGBA
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range


#-------------------------------------- KOODI PEAMINE FUNKTSIONAALSUS ----------------------------------------------#

speed = WheelsCmdStamped()
avoiding = False #Kas robot on mööda sõitmas takistusest


class MyPublisherNode(DTROS): #Klass

    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.start_vel = 0.4  #Roboti algne kiirus
        self.range = 1
        self.right = 0
        self.left = 0
        self.timeL = 0
        self.timeR = 0

        #Info vastu võtmine ja saatmine
        self.pub = rospy.Publisher('bestestduckiebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber('/bestestduckiebot/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/bestestduckiebot/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/bestestduckiebot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)

        self.bus = SMBus(1)
        self.ticks_left = 0
        self.prev_tick_left = 0
        self.ticks_right = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.baseline_wheel2wheel = 0.1 #Laius kahe ratta vahel, meetrites
        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0
        self.prev_int = 0
        self.prev_e = 0
        self.theta_ref = 0
        self.lastCall = 0
        self.In_al = 0
        self.avoiding = False

        self.position = 0
        self.lastturn = 0
        self.loops = 0
        self.short = 0

        self.prev_info = 0o00011000 #Jätab meelde eelmise joone lugeri info
        self.option = 0
        self.errorlist = []

        #Joone järgimis anduri võimalikud väärtused mida on vaja sõitmiseks joonel
        self.rightvalues = [[0, 1, 1, 1, 1, 1, 1, 1], [0, 0, 1, 1, 1, 1, 1, 1], [0, 0, 0, 1, 1, 1, 1, 1], [0, 0, 0, 0, 1, 1, 1, 1], [0, 0, 0, 0, 0, 1, 1, 1], [0, 0, 0, 0, 0, 0, 1, 1], [0, 0, 0, 0, 0, 0, 0, 1]]
        self.leftvalues = [[1, 1, 1, 1, 1, 1, 1, 0], [1, 1, 1, 1, 1, 1, 0, 0], [1, 1, 1, 1, 1, 0, 0, 0], [1, 1, 1, 1, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0, 0]]
        self.shortroute = [[1, 0, 0, 1, 0, 0, 0, 0], [1, 1, 0, 1, 1, 0, 0, 0], [1, 0, 1, 1, 0, 0, 0, 0], [0, 1, 1, 0, 1, 1, 0, 0]]
        self.turningpoint = [ [0, 0, 1, 1, 1, 1, 0, 0], [0, 1, 1, 1, 1, 0, 0, 0], [0, 0, 0, 1, 1, 1, 1, 0]]
        
        #Roboti sulgemis errori eemaldamine
    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown)
    
        #Roboti sulgemisel saadetakse need käsud
    def shutdown(self):
        speed.vel_right = 0
        speed.vel_left = 0
        self.pub.publish(speed)
    
        
    def callback(self, data):
        self.range = data.range
        #rospy.loginfo("Kuulen: %s", data.range)
    def rightwheel(self, data):
        self.right = data.data
        #rospy.loginfo("Parem ratas: %s", data.data)
        self.timeR = data.header.seq
    def leftwheel(self, data):
        self.left = data.data
        #rospy.loginfo("Vasak ratas: %s", data.data)
        self.timeL = data.header.seq

    #def time_leftwheel(self, data):
        
    #def time_rightwheel(self, data):
        

#-------------------------------------- MÖÖDA SÕITMINE ----------------------------------------------#


    #Funktsioon mis seina tuvastusel hakkab seinast ümberpõiget sooritama ja otsib peale manööverdamist joone üles ning jätkab sõitu
    def go_around(self):
        print("Going around!")
        sleep(0.5) #Peale igat käsku on paus, parandab roboti liikumist
        right_start = self.right
        #print(":::::::::::::::: %s", right_start)
        parem = self.right - right_start
        print(parem)

        left_start = self.left
        vasak = self.left - left_start
        #print(":::::::::::::::: %s", left_start)
        print(vasak)

        while parem <= 60:
            print("Turning left:")
            print("vasak: ", vasak)
            print("parem: ", parem)
            speed.vel_right = 0.07
            speed.vel_left = 0.0
            self.pub.publish(speed)
            parem = self.right - right_start

        if parem > 60:
            speed.vel_right = 0.0
            speed.vel_left = 0.0
            self.pub.publish(speed)
            sleep(0.5)
            while vasak < 200:

                print("Driving forward:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = 0.5
                speed.vel_left = 0.5
                self.pub.publish(speed)
                vasak = self.left - left_start
                print(vasak)

            if vasak >= 200:
                vasak = self.left - left_start
                print("Stopping:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
                sleep(0.5)

                while vasak < 350:
                    print("Turning left:")
                    print("vasak: ", vasak)
                    print("parem: ", parem)
                    vasak = self.left - left_start
                    speed.vel_right = 0.0
                    speed.vel_left = 0.07
                    self.pub.publish(speed)

                if vasak >= 350:
                    print("Stopping:")
                    print("vasak: ", vasak)
                    print("parem: ", parem)
                    speed.vel_right = 0.0
                    speed.vel_left = 0.0
                    self.pub.publish(speed)
                    sleep(0.5)

            while vasak >= 350 and vasak < 500:
                print("Driving forward:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = 0.5
                speed.vel_left = 0.5
                self.pub.publish(speed)
                vasak = self.left - left_start
                print(vasak)

            if vasak >= 500:
                print("Stopping:")
                print("vasak: ", vasak)
                print("parem: ", parem)
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
        self.avoiding = False



#-------------------------------------- 90 KRAADISED KURVID / LÜHEM RADA VALIK ----------------------------------------------#

        #Äkiline parem põõrde funktsioon sooritab 90 kraadiseid põõrdeid
    def sharp_right(self):
        #print("starting to turn right")
        speed.vel_right = 0.0
        speed.vel_left = 0.2
        self.pub.publish(speed)
        while sum(self.errorlist) == 0 or self.errorlist in self.rightvalues:
            #print("Sharp right!")
            temp = self.bus.read_byte_data(62, 17)
            self.theta_ref = bin(temp)[2:].zfill(8)
            self.errorlist = list(map(int, self.theta_ref))            
            
            speed.vel_right = 0.0
            speed.vel_left = 0.2
            self.pub.publish(speed)
            #print("Right: ", self.theta_ref)
            self.lastturn = 1

        #Äkiline vasak põõrde funktsioon sooritab 90 kraadiseid põõrdeid
    def sharp_left(self):
        #print("starting to turn left")
        speed.vel_right = 0.2
        speed.vel_left = 0.0
        self.pub.publish(speed)

        while sum(self.errorlist) == 0 or self.errorlist in self.leftvalues:
            #print("Sharp left!")
            temp = self.bus.read_byte_data(62, 17)
            self.theta_ref = bin(temp)[2:].zfill(8)
            self.errorlist = list(map(int, self.theta_ref))            
            
            speed.vel_right = 0.2
            speed.vel_left = 0.0
            self.pub.publish(speed)
            #print("Left: ", self.theta_ref)
            self.lastturn = 2

        #Lühema raja valimine, valib lühema raja tuvastatdes joone ääres olevat ruutu mille järgi teab kumal pool on õige valik ning keerab õigesse suunda
    def short_path(self):
        print("Taking the short path!")
        while self.errorlist[0] == 1 or self.errorlist[1] == 1 or self.errorlist[2] == 1 and self.errorlist[5] == 0:
            temp = self.bus.read_byte_data(62, 17)
            self.theta_ref = bin(temp)[2:].zfill(8)
            self.errorlist = list(map(int, self.theta_ref))
                
            speed.vel_right = 0.2
            speed.vel_left = 0.0
            self.pub.publish(speed)
        self.short = 0


#-------------------------------------- ODUMEETRIA ----------------------------------------------#


        #Odumeetria arvutab välja roboti orentatsiooni ja sõitmist
    def Odometry(self):
        try:
            temp = self.bus.read_byte_data(62, 17)
        except:
            print("Ei saa andmeid lugeda!!!")
            #message = str(temp)
        #rospy.loginfo("Line follower: %s" % message)
        N_tot = 135 # total number of ticks per revolution
        alpha = 2 * np.pi / N_tot # wheel rotation per tick in radians
        #print(f"The angular resolution of our encoders is: {np.rad2deg(alpha)} degrees")
        self.ticks_right = self.right
        self.ticks_left = self.left
        self.delta_ticks_left = self.ticks_left-self.prev_tick_left #Vasaku ratta delta tickid
        self.delta_ticks_right = self.ticks_right-self.prev_tick_right #Parema ratta deltsa tickid
        self.rotation_wheel_left = alpha * self.delta_ticks_left #Kogu vasaku ratta põõre
        self.rotation_wheel_right = alpha * self.delta_ticks_right #Kogu parema ratta põõre
        #print(f"The left wheel rotated: {np.rad2deg(self.rotation_wheel_left)} degrees")
        #print(f"The right wheel rotated: {np.rad2deg(self.rotation_wheel_right)} degrees")

        R = 0.0345           # Joonlauaga sisestatud väärtus, *meetrites*
        d_left = R * self.rotation_wheel_left
        d_right = R * self.rotation_wheel_right
        #print(f"The left wheel travelled: {d_left} meters")
        #print(f"The right wheel rotated: {d_right} meters")

        #Kui palju on robot keeranud?
        Delta_Theta = (d_right-d_left)/self.baseline_wheel2wheel # expressed in radians
        #print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
        self.prev_tick_left = self.ticks_left
        self.prev_tick_right = self.ticks_right
        #print("Left wheel time: ", self.timeL)
        #print("Right wheel time: ", self.timeR)

        #Joone sensori info teisendamine binaar koodi
        self.theta_ref = bin(temp)[2:].zfill(8)
        #print(self.theta_ref)

        #Delta T arvutamine
        self.delta_t = self.timeL - self.lastCall + 1
        self.lastCall = self.timeL
        #print("DELTA T : ", self.delta_t)
    

#-------------------------------------- PID KONTROLLERI ARVUTUSED ----------------------------------------------#


        #Joone järgimine PID kontrolleri abil, roboti errori arvutamine
    def PID_calc(self):

        #Joone järgimine
        Kp = 0.045
        Ki = 0.02 
        Kd = 1.2    #Vähendab roboti ujumist 

        self.prev_info = self.theta_ref

        #P - proportional
        self.position = sum(self.line_values) / len(self.line_values)
        error = 4.5 - self.position

        #I - integral
        self.In_al = self.In_al + (self.delta_t * error) #in_al = integral
        #print("integral: ", self.In_al)
        #anti-windup - väldib integraalvea liigset suurenemist
        self.In_al = max(min(self.In_al, 1.8), -1.8)
        #print("integral + anti-windup: ", self.In_al)

        #D - derivative
        err_der = (error - self.prev_e) / self.delta_t
        Kpe = Kp * error
        Kie = Ki * self.In_al
        Kde = Kd * err_der
        #print("Kpe: ", Kpe)
        self.PID = Kpe + Kie + Kde
        self.prev_e = error


#-------------------------------------- RUN FUNKTSIOON ----------------------------------------------#


    def run(self):
#Väljastab info iga sekund|| niipalju kordi
#                         \/
        rate = rospy.Rate(30) # 1Hz
        
        while not rospy.is_shutdown():

            #Odomeetria
            self.Odometry()

            #Ümber takistuse
            '''
            if self.range < 0.25:
                print("A wall appeared!")
                speed.vel_right = 0.0
                speed.vel_left = 0.0
                self.pub.publish(speed)
                self.avoiding = True
                self.go_around()
                if self.avoiding == False:
                    pass
            '''
            
            
            self.line_values = []
            for i, value in enumerate(self.theta_ref):
                if value =='1':
                    self.line_values.append(i + 1)
            #print(self.line_values)
            self.errorlist = list(map(int, self.theta_ref))
            #print("errorlist = ", self.errorlist)
            

            #D = (Error - previous error) / delta t

            if self.errorlist in self.rightvalues:  #Äkiline parem põõre
                #print("func start right")
                self.sharp_right()
               
            if self.errorlist in self.leftvalues:    #Äkiline vasak põõre
                #print("func start left")
                self.sharp_left()
                
            if self.line_values == [] and self.lastturn == 1:    #Äkiline parem põõre           self.position < 2 
                speed.vel_right = 0.0           
                speed.vel_left = 0.3
                self.pub.publish(speed)

            if self.line_values == [] and self.lastturn == 2:   #Äkiline vasak põõre     self.position > 7 
                speed.vel_right = 0.3
                speed.vel_left = 0.0
                self.pub.publish(speed)
            
            if len(self.line_values) != 0 and self.theta_ref is not self.rightvalues and self.theta_ref is not self.leftvalues:

                if self.errorlist in self.shortroute:
                    self.short = 1
                
                if self.short == 1 and self.errorlist[1] == 1 or self.errorlist in self.turningpoint:
                    self.short_path()
                self.PID_calc()
                
                speed.vel_right = self.start_vel + self.PID
                speed.vel_left = self.start_vel - self.PID
                #print(speed.vel_right)
                #print(speed.vel_left)
                #print("PID")
                self.pub.publish(speed)
                self.loops = self.loops + 1
                if self.loops > 50:
                    self.loops = 0
                    self.lastturn = 0
            rate.sleep()
        
if __name__ == '__main__':
    #Teeb nodi
    node = MyPublisherNode(node_name='my_publisher_node')
    #Jooksutamis node
    node.run()
    #Jääb kordama
    rospy.spin()