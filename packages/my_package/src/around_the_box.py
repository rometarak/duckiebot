import time
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
speed = WheelsCmdStamped()
pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
def around_box():
    # Turn 90 degrees right in 0.65 second
    speed.vel_right = -0.02
    speed.vel_left = 0.4
    pub.publish(speed)
    time.sleep(0.6)
    # Go straight for 2.2 meters
    speed.vel_right = 0.3
    speed.vel_left = 0.3
    pub.publish(speed)
    time.sleep(2.3)
    # Turn 90 degrees left in 0.65 second
    speed.vel_right = 0.3
    speed.vel_left = -0.1
    pub.publish(speed)
    time.sleep(0.4)
    # Go straight for 2.2 meters
    speed.vel_right = 0.4
    speed.vel_left = 0.4
    pub.publish(speed)
    time.sleep(2.2)
    # Turn 90 degrees right in 0.4 second
    speed.vel_right = -0.02
    speed.vel_left = 0.35
    pub.publish(speed)
    time.sleep(0.4)
