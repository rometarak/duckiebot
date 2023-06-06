from smbus2 import SMBus
import rospy

def get_line_values():
    bus = SMBus(1)
    read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)

    line_values = []
    for i, value in enumerate(read):
        if value =='1':
            line_values.append(i + 1)
    
    return line_values

def get_theta():
    bus = SMBus(1)
    read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)

    line_values = []
    for i, value in enumerate(read):
        if value =='1':
            line_values.append(i + 1)
    if len(line_values) >= 1:
        theta_hat = sum(line_values)/len(line_values)
    if len(line_values) == 0:
        theta_hat = 4

    return theta_hat

def pid_controller(t0,t1):
        delta_t = 1
        pose_estimation = 4.5
        prev_int = 0
 
        e = pose_estimation - get_theta()
        e_int = prev_int + e*delta_t
        prev_int = e_int                                        #integral of the error
        prev_e = e                                              #Tracking
        e_int = max(min(e_int,2),-2)                            # anti-windup - preventing the integral error from growing too much       
        e_der = (e - prev_e)/delta_t                            #derivative of the error

        # controller coefficients
        #Kp = rospy.get_param("/p")          #rosparam set /p 0.068
        #Ki = rospy.get_param("/i")          #rosparam set /i 0.029
        #Kd = rospy.get_param("/d")          #rosparam set /d 0.27
        Kp = 0.068
        Ki = 0.029 
        Kd = 0.27
        
        delta_t = t0-t1
        omega = Kp*e + Ki*e_int + Kd*e_der                 #PID controller for omega
        
        return omega