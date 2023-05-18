import time
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
speed = WheelsCmdStamped()


def change_lane():
        # Drive a little bit forward
        # And then turn left to switch to the shorter lane
        speed.vel_left = 0.3
        speed.vel_right = 0.5
        pub.publish(speed)
        time.sleep(0.8)