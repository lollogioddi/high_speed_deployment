#!/usr/bin/env python2.7
import rospy
import tf
import numpy as np
import sys
import traceback
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class DataSaver:
    def __init__(self):
        try:
            rospy.init_node('data_saver', anonymous=True)

            # Subscribers
            self.sub = rospy.Subscriber('/pelican/msf_core/odometry', Odometry, self.odometry_callback)
            self.sub_clock = rospy.Subscriber("/clock", Clock, self.clock_callback)
            self.pose_sub = rospy.Subscriber('/pelican/command/trajectory', MultiDOFJointTrajectory, self.trajectory_callback)

            # Publishers
            self.pub_time = rospy.Publisher('/time_in_sec', Float64, queue_size=10)
            self.pub_position = rospy.Publisher('/pelican/position', Vector3, queue_size=10)
            self.pub_position_error = rospy.Publisher('/pelican/pos_err', Vector3, queue_size=10)
            self.pub_angular_position = rospy.Publisher('/pelican/angular_pos', Vector3, queue_size=10)
            self.pub_command_trajectory = rospy.Publisher('/pelican/comm_pos', Vector3, queue_size=10)

            self.current_time = Clock()
            self.command_trajectory = MultiDOFJointTrajectory()
            self.odometry = Odometry()

            self.rate = rospy.Rate(400)
        except Exception as e:
            rospy.logerr("Initialization error: {}\n{}".format(e, traceback.format_exc()))
            sys.exit(1)

    def clock_callback(self, msg):
        self.current_time = msg

    def trajectory_callback(self, msg):
        self.command_trajectory = msg

    def odometry_callback(self, msg):
        self.odometry = msg

    def publish(self, position, pos_err, euler_orientation, time, command_position):
        self.pub_time.publish(time)
        self.pub_position.publish(Vector3(*position))
        self.pub_position_error.publish(Vector3(*pos_err))
        self.pub_angular_position.publish(Vector3(*euler_orientation))
        self.pub_command_trajectory.publish(Vector3(*command_position))

    def process_and_publish(self):
        if self.command_trajectory.points:
            command_position = np.array([
                self.command_trajectory.points[0].transforms[0].translation.x,
                self.command_trajectory.points[0].transforms[0].translation.y,
                self.command_trajectory.points[0].transforms[0].translation.z
            ])

            position = np.array([
                self.odometry.pose.pose.position.x,
                self.odometry.pose.pose.position.y,
                self.odometry.pose.pose.position.z
            ])

            orientation = [
                self.odometry.pose.pose.orientation.x,
                self.odometry.pose.pose.orientation.y,
                self.odometry.pose.pose.orientation.z,
                self.odometry.pose.pose.orientation.w
            ]
            euler_orientation = tf.transformations.euler_from_quaternion(orientation)

            pos_err = command_position - position

            time = self.current_time.clock.secs + self.current_time.clock.nsecs * 1e-9

            self.publish(position, pos_err, euler_orientation, time, command_position)

if __name__ == '__main__':
    try:
        node = DataSaver()
        while not rospy.is_shutdown():
            node.process_and_publish()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node terminated.")