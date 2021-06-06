#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi, isinf

# id for different laser orientations
CENTER = 0
LEFT = 1
RIGHT = 2

# linear and angular velocity constants
LINEAR_VELOCITY = 0.3
ANGULAR_VELOCITY = 1.5

# robot states 

# initial state: based on current position and LiDAR data, decide move direction
GET_DIRECTION = 0

# move forward state
DRIVE_FORWARD = 1

# turn right state
RIGHT_TURN = 2

# turn left state
LEFT_TURN = 3

ESCAPE_RANGE = 30 * pi / 180.0

class turtleBot3Controller:

    def __init__(self):
        """Initialization."""
        rospy.init_node('turtlebot3_random_exploration')

        # distances
        self.forward_threshold = 0.7
        self.side_threshold = 0.6

        # current robot state
        self.curr_state = 0

        # laser data
        self.scan_data = [0.0, 0.0, 0.0]

        # pose data
        self.pose_data = 0
        self.prev_pose_data = 0


        # init velocity publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # init laser sensor subscriber
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.get_laser_data)

        # set rospy rate to 125
        self.rate = rospy.Rate(125)

        # init odometry subscriber
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.get_pose_data)


    # retrieve the position using trigonometry and the orientation of the robot
    def get_pose_data(self, data):
        orientation = data.pose.pose.orientation
        
        sin_y = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        
        cos_y = 1 - 2 * (orientation.y**2 + orientation.z**2)
        self.pose_data = atan2(sin_y, cos_y)

 
    # get the value of the laser from the 0, 30 and -30 degree sensors
    def get_laser_data(self, data):
        for i, angle in enumerate([0, 30, 330]):
            # if the sensor does not reach an object (=inf), save the max range data of the sensor
            if isinf(data.ranges[angle]):
                self.scan_data[i] = data.range_max
            else:
                self.scan_data[i] = data.ranges[angle]


    def update_pose(self, linear, angular):
        cmd_vel = Twist()

        # Linear velorcity in the x-axis.
        cmd_vel.linear.x = linear

        # Angular velocity in the z-axis.
        cmd_vel.angular.z = angular

        # Publishing our cmd_vel
        self.velocity_publisher.publish(cmd_vel)


    # make the robot move with a randomic movement without letting it crash
    def autonomous_move(self):

        # decide wich action to perform (state 0)
        if self.curr_state == GET_DIRECTION:

            # if we don't have obstacles in front
            if self.scan_data[CENTER] > self.forward_threshold:

                # if there is an obstable on the left, turn right
                if self.scan_data[LEFT] < self.side_threshold:
                    self.prev_pose_data = self.pose_data
                    self.curr_state = RIGHT_TURN

                # if there is an obstable on the right, turn left
                elif self.scan_data[RIGHT] < self.side_threshold:
                    self.prev_pose_data = self.pose_data
                    self.curr_state = LEFT_TURN

                # no obstacles around the robot (left, right and front).
                # just drive forward
                else:
                    self.curr_state = DRIVE_FORWARD

            # if we have an obstacle(s) in front, turn right
            if self.scan_data[CENTER] < self.forward_threshold:
                self.prev_pose_data = self.pose_data
                self.curr_state = RIGHT_TURN

        # drive forward (state 1)
        elif self.curr_state == DRIVE_FORWARD:
            self.update_pose(LINEAR_VELOCITY, 0)
            self.curr_state = GET_DIRECTION

        # turn right (state 2)
        elif self.curr_state == RIGHT_TURN:
            # check for the escape threshold, used to define a margin 
            # from which we are sure that the robot can rotate without 
            # being stuck. If we will get stuck, don't turn
            if abs(self.prev_pose_data - self.pose_data) >= ESCAPE_RANGE:
                self.curr_state = GET_DIRECTION
            else:
                self.update_pose(0, -1 * ANGULAR_VELOCITY)
        
        # turn left (state 3)
        elif self.curr_state == LEFT_TURN:
            # same as turn right
            if abs(self.prev_pose_data - self.pose_data) >= ESCAPE_RANGE:
                self.curr_state = GET_DIRECTION
            else:
                self.update_pose(0, ANGULAR_VELOCITY)
        else:
            self.curr_state = GET_DIRECTION


    # Move TB3 until ctrl^c is pressed
    def run(self):
        while not rospy.is_shutdown():
            self.autonomous_move()
            self.rate.sleep()



if __name__ == '__main__':
    controller = turtleBot3Controller()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass