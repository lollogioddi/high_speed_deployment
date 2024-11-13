#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from std_msgs.msg import String
import rospy
import time
import random  # Import the random module
# Explicitly add the path to the trajectory_msgs package
import sys
sys.path.append("/opt/ros/melodic/share")
#from mav_msgs.msg import conversions
#from mav_msgs.msg import default_topics
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import math
import tf
import numpy as np

odometry = None
def odometry_callback(msg):
    global odometry
    odometry = msg

def unpause_gazebo():
    # wait for the service to be available
    rospy.wait_for_service('/gazebo/unpause_physics')
    # check for 10 seconds if the system is unpaused
    i = 0
    unpaused = False
    while i <= 10 and not unpaused:
        try:
            rospy.loginfo("Wait for 1 second before trying to unpause Gazebo again.")
            time.sleep(1)  # Attendere un secondo
            unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause_gazebo()  # Tentare di riprendere Gazebo
            unpaused = True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        i += 1

    if not unpaused:
        rospy.logfatal("Could not wake up Gazebo.")
        return -1
    else:
        rospy.loginfo("Unpaused the Gazebo simulation.")
def main():
    rospy.init_node('hovering_example')
    # To get a handle to the current ROS node
    nh = rospy.get_namespace()
    # Create a private node handle for accessing node parameters
    nh_private = rospy.get_namespace()

    #if a drone different from the pelican is used, the publisher topic has to be changed accordingly
    trajectory_pub = rospy.Publisher('/pelican/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    motor_speed_pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size=10)
    odometry_sub = rospy.Subscriber('/pelican/odometry_sensor1/odometry', Odometry, odometry_callback)
    rospy.loginfo("Started hovering example.")

    time.sleep(1.0)

    trajectory_msg = MultiDOFJointTrajectory()
    trajectory_msg.header.stamp = rospy.Time.now()
    motor_speed_msg = Actuators()
    motor_speed_msg.header.stamp = rospy.Time.now()
    
    # unpause gazebo command
    # unpause_gazebo()

    while odometry is None and not rospy.is_shutdown():
        rospy.sleep(0.1) 
    rospy.loginfo("Odometry data received.")
    # lines to extract desired position from current position and stay put
    time.sleep(2)
    if odometry is not None:
        position = odometry.pose.pose.position
        desired_position = Vector3(0.0, 0.0, 0.0)
        desired_position.x = position.x
        desired_position.y = position.y
        desired_position.z = position.z
    # desired_position = Vector3(0.0, 0.0, 10.0)
    desired_roll =  0.0 # Sostituisci con il tuo valore desiderato
    desired_pitch = 0.0 # Sostituisci con il tuo valore desiderato
    desired_yaw = 1.0 # Sostituisci con il tuo alore desiderato
    # Converti roll, pitch, yaw in quaternioni
    quaternion_np = tf.transformations.quaternion_from_euler(desired_roll, desired_pitch, desired_yaw, 'sxyz')
    # Convert the numpy array to a Quaternion message
    quaternion_msg = Quaternion(*quaternion_np)

    point = MultiDOFJointTrajectoryPoint()
    transform = Transform(translation=desired_position, rotation=quaternion_msg)
    velocities = Twist(linear=Vector3(0.0,0.0,0.0), angular = Vector3(0.0,0.0,0.0))
    accelerations = Twist(linear=Vector3(0.0,0.0,0.0), angular =Vector3(0.0,0.0,0.0))
    point.transforms.append(transform)
    point.velocities.append(velocities)
    point.accelerations.append(accelerations)
    trajectory_msg.points.append(point)

    rospy.loginfo("Publishing waypoint on namespace %s: [%f, %f, %f].", rospy.get_namespace(), desired_position.x, desired_position.y, desired_position.z)
    trajectory_pub.publish(trajectory_msg)

    rospy.spin()
    # Shutdown the ROS node
    rospy.signal_shutdown("Shutting down the node")
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
