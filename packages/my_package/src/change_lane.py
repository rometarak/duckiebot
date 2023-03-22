import time
import rospy
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
speed = WheelsCmdStamped()
def change_lane():
    bus = SMBus(1)
    read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)

    line_values = []
    for i, value in enumerate(read):
        if value =='1':
            line_values.append(i + 1)

    if line_values == 1.2:
        time.sleep(0.5)
        speed.vel_left = 0
        speed.vel_right = 0.3
        pub.publish(speed)