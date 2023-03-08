import time
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
speed = WheelsCmdStamped()
pub = rospy.Publisher('bestduckbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

def around_box():
    # Turn 90 degrees right in 2 second
    speed.vel_right = 0.0
    speed.vel_left = 0.3
    pub.publish(speed)
    time.sleep(1)
    # Go straight for 0.15 meters
    speed.vel_right = 0.3
    speed.vel_left = 0.3
    pub.publish(speed)
    time.sleep(1)
    # Turn 90 degrees left in 2 second
    speed.vel_right = 0.3
    speed.vel_left = 0.0
    pub.publish(speed)
    time.sleep(1)
    # Go straight for 0.15 meters
    speed.vel_right = 0.3
    speed.vel_left = 0.3
    pub.publish(speed)
    time.sleep(2)
    # Turn 90 degrees left in 2 second
    speed.vel_right = 0.0
    speed.vel_left = 0.3
    pub.publish(speed)
    time.sleep(1)