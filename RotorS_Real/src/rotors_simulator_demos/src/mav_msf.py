#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from std_msgs.msg import String
import rospy
import time
# Explicitly add the path to the trajectory_msgs package
import sys
sys.path.append("/opt/ros/melodic/share")
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sensor_fusion_comm.srv import InitScale, InitScaleRequest
import tf
import numpy as np

odometry = None
msf = None
def odometry_callback(msg):
    global odometry
    odometry = msg

def msf_callback(msg):
    global msf
    msf = msg

def append_string_to_file(file_name, value1, value2):
        # Generate a file of two columns with the input values
        path = "/home/lollogioddi/Desktop"
        full_path = path + file_name
        with open(full_path, 'a') as file:
            file.write("{}\t{}\n".format(value1, value2))

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
    rospy.init_node('mav_falling')
    # To get a handle to the current ROS node
    nh = rospy.get_namespace()
    # Create a private node handle for accessing node parameters
    nh_private = rospy.get_namespace()

    #if a drone different from the pelican is used, the publisher topic has to be changed accordingly
    odometry_sub = rospy.Subscriber('/pelican/odometry_sensor1/odometry', Odometry, odometry_callback)
    msf_sub = rospy.Subscriber('/pelican/msf_core/odometry', Odometry, msf_callback)

    rospy.loginfo("Starting falling.")
    
    # unpause gazebo command
    unpause_gazebo()
    
    while odometry is None and not rospy.is_shutdown():
        rospy.sleep(0.1) 
    rospy.loginfo("Odometry data received.")
    try:
        scale_value = 1.0  
        init_scale = rospy.ServiceProxy('msf/pose_sensor/initialize_msf_scale', InitScale)
        # Create a request object
        request = InitScaleRequest(scale=scale_value)
        response = init_scale(request)
        if response.result == "Initialized scale":
            rospy.loginfo("MSF scale successfully initialized.")
            rospy.sleep(1.0)
        else:
            rospy.logwarn("MSF scale initialization may have failed: %s", response.result)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
    

    rospy.spin()
    # Shutdown the ROS node
    rospy.signal_shutdown("Shutting down the node")
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
