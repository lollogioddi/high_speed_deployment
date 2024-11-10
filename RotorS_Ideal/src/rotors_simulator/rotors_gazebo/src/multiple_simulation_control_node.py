#!/usr/bin/env python2.7

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess
import signal
import os
import psutil
import time
import random
from std_srvs.srv import Empty as EmptyService

gazebo_pid = None

# linear_velocity_bounds = {
#     'x': (10, 15.0),  # Example bounds for linear x velocity
#     'y': (10.0, 15.0),
#     'z': (20.0, 30.0)
# }

# angular_velocity_bounds = {
#     'x': (20, 25),  # Example bounds for angular x velocity
#     'y': (20, 25),
#     'z': (20, 25)
# }

def generate_random_velocity(bounds):
    return random.uniform(bounds[0], bounds[1])

def is_gazebo_running():
    for proc in psutil.process_iter():
        if proc.name() in ['gzserver', 'gzclient']:
            return True
    return False

def stop_gazebo(req):
    try:
        # Kill gzserver process
        process = subprocess.Popen(['pkill', '-9', 'gzserver'])
        process.wait()
        rospy.loginfo("gzserver process killed successfully.")
        
        # Kill gzclient process (if running)
        subprocess.Popen(['pkill', '-9', 'gzclient'])
        process.wait()
        rospy.loginfo("gzclient process killed successfully.")
        
        return TriggerResponse(success=True, message="Gazebo processes killed successfully.")
    except subprocess.CalledProcessError as e:
        rospy.logerr("Failed to kill Gazebo processes: {}".format(e))
        return TriggerResponse(success=False, message="Failed to kill Gazebo processes.")

    
def start_gazebo_world(req): 
    try:   
        global gazebo_pid
        proc = subprocess.Popen(["roslaunch", "rotors_gazebo", "empty_world.launch", "gui:=false"])
        gazebo_pid = proc.pid
        rospy.loginfo("Started Gazebo with PID {}".format(proc.pid))
        rospy.sleep(3)  # Give Gazebo some time to launch
        return TriggerResponse(success=True, message="Gazebo world started")
    except Exception as e:
        rospy.logerr("Failed to launch Gazebo world: {}".format(e))
        return TriggerResponse(success=False, message="Failed to launch Gazebo world")

def set_initial_velocities(req):
    try:
        rospy.loginfo("Setting initial velocities...")
        
        # Generate random initial velocities within the specified boundaries
        # linear_x = generate_random_velocity(linear_velocity_bounds['x'])
        # linear_y = generate_random_velocity(linear_velocity_bounds['y'])
        # linear_z = generate_random_velocity(linear_velocity_bounds['z'])
        # angular_x = generate_random_velocity(angular_velocity_bounds['x'])
        # angular_y = generate_random_velocity(angular_velocity_bounds['y'])
        # angular_z = generate_random_velocity(angular_velocity_bounds['z'])
        linear_x = 30
        linear_y = 0
        linear_z = 0
        angular_x = 25
        angular_y = 25
        angular_z = 25
        # Call the velocity_publisher_once script with the generated velocities
        subprocess.Popen([
            "rosrun", "rotors_gazebo", "set_initial_velocity_node.py",
            str(linear_x), str(linear_y), str(linear_z),
            str(angular_x), str(angular_y), str(angular_z)
        ])

        rospy.sleep(1)  # Give some time for the velocities to be set
        rospy.loginfo("Initial velocities set: linear=({}, {}, {}), angular=({}, {}, {})".format(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z))
        return TriggerResponse(success=True, message="Initial velocities set successfully.")    
    except Exception as e:
        rospy.logerr("Failed to set initial velocities: {}".format(e))
        return TriggerResponse(success=False, message="Failed to set initial velocities: {}".format(e))

def start_simulation(req):
    try:
        if not is_gazebo_running():
            rospy.loginfo("Gazebo is not running. Starting Gazebo...")
            stop_gazebo()  # Stop any existing unhealthy Gazebo processes
            if not start_gazebo_world():  # Start Gazebo with an empty world
                return TriggerResponse(success=False, message="Failed to launch Gazebo world")
        
        # Spawn multiple pelican drones
        subprocess.Popen(["roslaunch", "rotors_gazebo", "multiple_drones.launch",
                          "mav_name:=pelican", "controller_type:=QuaternionBased_spherical_SMC.py"
                          "enable_my_logging:=false", "gains_file:=_for_tuning"])

        # Unpause Gazebo physics
        #unpause_gazebo()

        return TriggerResponse(success=True, message="Simulation started")
    except Exception as e:
        rospy.logerr("Failed to start simulation: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def start_data_saver_nodes(req):
    try:
        rospy.loginfo("Starting data saver nodes for all pelicans...")
        subprocess.Popen(["roslaunch", "rotors_gazebo", "multiple_data_saver_node.launch"])
        rospy.sleep(5)  # Give some time for the nodes to start
        return TriggerResponse(success=True, message="Data saver nodes started")
    except Exception as e:
        rospy.logerr("Failed to start data saver nodes: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def stop_simulation(req):
    try:
        rospy.loginfo("Stopping simulation...")
        process = subprocess.Popen(["pkill", "-f", "roslaunch rotors_gazebo multiple_drones.launch"])
        process.wait()
        rospy.loginfo("roslaunch process killed successfully.")
        return TriggerResponse(success=True, message="Simulation stopped")
    except Exception as e:
        rospy.logerr("Failed to stop simulation: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def reset_simulation(req):
    try:
        rospy.loginfo("Resetting simulation...")
        subprocess.Popen(["rosservice", "call", "/gazebo/pause_physics"])
        #subprocess.Popen(["rosservice", "call", "/gazebo/delete_model", "{model_name: pelican}"])
        #subprocess.Popen(["rosservice", "call", "/gazebo/reset_simulation"])
        rospy.sleep(2)
        return TriggerResponse(success=True, message="Simulation reset")
    except Exception as e:
        rospy.logerr("Failed to reset simulation: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def stop_gazebo2(req):
    try:
        # Kill gzserver process
        process = subprocess.Popen(['pkill', '-9', 'gzserver'])
        process.wait()
        rospy.loginfo("gzserver process killed successfully.")
        
        # Kill gzclient process (if running)
        subprocess.Popen(['pkill', '-9', 'gzclient'])
        process.wait()
        rospy.loginfo("gzclient process killed successfully.")
        
        return TriggerResponse(success=True, message="Gazebo processes killed successfully.")
    except subprocess.CalledProcessError as e:
        rospy.logerr("Failed to kill Gazebo processes: {}".format(e))
        return TriggerResponse(success=False, message="Failed to kill Gazebo processes.")

def unpause_gazebo():
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpaused = False
    for i in range(10):  # Try for 10 seconds
        try:
            rospy.loginfo("Trying to unpause Gazebo...")
            unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', EmptyService)
            unpause_gazebo()  # Attempt to unpause Gazebo
            unpaused = True
            break
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        time.sleep(1)

    if not unpaused:
        rospy.logfatal("Could not unpause Gazebo.")
        return -1
    else:
        rospy.loginfo("Unpaused the Gazebo simulation.")

def main():
    rospy.init_node('multiple_simulation_control_node')
    rospy.Service('start_gazebo_world', Trigger, start_gazebo_world)
    rospy.Service('set_initial_velocities', Trigger, set_initial_velocities)    
    rospy.Service('start_simulation', Trigger, start_simulation)
    rospy.Service('start_data_saver_nodes', Trigger, start_data_saver_nodes)
    rospy.Service('stop_simulation', Trigger, stop_simulation)
    rospy.Service('reset_simulation', Trigger, reset_simulation)
    rospy.Service('stop_gazebo2', Trigger, stop_gazebo2)
    rospy.spin()

if __name__ == "__main__":
    main()
