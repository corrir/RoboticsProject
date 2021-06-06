#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt



DEG2RAD = (M_PI / 180.0)
RAD2DEG = (180.0 / M_PI)

CENTER = 0
LEFT = 1
RIGHT = 2

LINEAR_VELOCITY = 0.3
ANGULAR_VELOCITY = 1.5

GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN = 2
TB3_LEFT_TURN = 3

class turtleBot3Controller:

    def __init__(self):
        """Initialization."""
        rospy.init_node('turtlebot3_random_exploration')

    cmd_vel_topic_name = rospy.resolve_name('cmd_vel_topic_name', caller_id=None)

    rospy.loginfo(cmd_vel_topic_name)

    # self.velocity_publisher = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    # self.laser_subscriber = rospy.Subscriber('/turtle2/pose', Pose, self.update_pose)



if __name__ == '__main__':
    controller = turtleBot3Controller()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass