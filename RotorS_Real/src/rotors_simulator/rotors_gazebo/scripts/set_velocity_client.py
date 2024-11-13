#!/usr/bin/env python
import rospy
from rotors_comm.srv import SetInitialVelocities
from sys import argv

def set_velocities_client(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    rospy.wait_for_service('/gazebo_client/set_initial_velocities')
    try:
        set_initial_velocities = rospy.ServiceProxy('/gazebo_client/set_initial_velocities', SetInitialVelocities)
        resp = set_initial_velocities(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
        return resp.success, resp.message
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    if len(argv) == 7:
        linear_x = float(argv[1])
        linear_y = float(argv[2])
        linear_z = float(argv[3])
        angular_x = float(argv[4])
        angular_y = float(argv[5])
        angular_z = float(argv[6])
        print("Requesting initial velocities...")
        success, message = set_velocities_client(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
        print("Response from service: Success: {}, Message: '{}'".format(success, message))
    else:
        print("Usage: set_velocity_client.py linear_x linear_y linear_z angular_x angular_y angular_z")

