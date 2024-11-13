#!/usr/bin/env python2.7

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess
import signal
import os
import psutil
import requests
import csv

initial_velocities = []
current_velocity_index = 0

# Function to read initial velocities from a CSV file
def read_initial_velocities(file_path='/home/lollogioddi/Desktop/bash_related_files/initial_velocities.csv'):
    global initial_velocities
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header
        for row in reader:
            velocities = list(map(float, row))
            initial_velocities.append(velocities)

def is_gazebo_running():
    for proc in psutil.process_iter():
        if proc.name() in ['gzserver', 'gzclient']:
            return True
    return False

def stop_gazebo(req):
    global gazebo_pid
    if gazebo_pid is None:
        rospy.logerr("No Gazebo PID found")
        return TriggerResponse(success=False, message="No Gazebo PID found")
    try:
        proc = psutil.Process(gazebo_pid)
        rospy.loginfo("Terminating gzserver with PID {}".format(proc.pid))
        proc.terminate()  # Gracefully terminate the process
        try:
            proc.wait(timeout=3)
        except psutil.TimeoutExpired:
            rospy.logwarn("Process {} did not terminate in time. Killing it.".format(proc.pid))
            proc.kill()  # Forcefully terminate if not terminated gracefully
        gazebo_pid = None  # Reset the PID
        return TriggerResponse(success=True, message="Simulation stopped")
    except Exception as e:
        rospy.logerr("Failed to stop gazebo: {}".format(e))
        return TriggerResponse(success=False, message=str(e))


def start_gazebo_world_msf(req): 
    try:   
        global gazebo_pid
        proc = subprocess.Popen(["roslaunch", "rotors_gazebo", "empty_world.launch"])
        gazebo_pid = proc.pid
        rospy.loginfo("Started Gazebo with PID {}".format(proc.pid))
        rospy.sleep(3)  # Give Gazebo some time to launch
        return TriggerResponse(success=True, message="Gazebo world started")
    except Exception as e:
        rospy.logerr("Failed to launch Gazebo world: {}".format(e))
        return TriggerResponse(success=False, message="Failed to launch Gazebo world")



def start_simulation_msf(req):
    try:
        if not is_gazebo_running():
            rospy.loginfo("Gazebo is not running. Starting Gazebo...")
            stop_gazebo()  # Stop any existing unhealthy Gazebo processes
            if not start_gazebo_world_msf():  # Start Gazebo with an empty world
                return TriggerResponse(success=False, message="Failed to launch Gazebo world")

        subprocess.Popen(["roslaunch", "rotors_simulator_demos", "mav_hovering_example_msf.launch",
                          "mav_name:=pelican", "controller_type:=Lee_decoupled_controller_evolution.py",
                          "enable_my_logging:=false", "gains_file:=_for_tuning_2"])

        return TriggerResponse(success=True, message="Simulation started")
    except Exception as e:
        rospy.logerr("Failed to start simulation: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def start_data_saver_node_msf(req):
    try:
        rospy.loginfo("Starting data saver node...")
        subprocess.Popen(["roslaunch", "rotors_gazebo", "data_saver_node.launch"])
        rospy.sleep(5)  # Give some time for the nodes to start
        return TriggerResponse(success=True, message="Data saver node started")
    except Exception as e:
        rospy.logerr("Failed to start data saver node: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def stop_simulation_msf(req):
    try:
        rospy.loginfo("Stopping simulation...")
        subprocess.Popen(["pkill", "-f", "roslaunch rotors_simulator_demos mav_hovering_example_msf.launch"])
        rospy.sleep(2)
        return TriggerResponse(success=True, message="Simulation stopped")
    except Exception as e:
        rospy.logerr("Failed to stop simulation: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def reset_simulation_msf(req):
    try:
        rospy.loginfo("Resetting simulation...")
        subprocess.Popen(["rosservice", "call", "/gazebo/pause_physics"])
        #subprocess.Popen(["rosservice", "call", "/gazebo/delete_model", "{model_name: pelican}"])
        s#ubprocess.Popen(["rosservice", "call", "/gazebo/reset_simulation"])
        rospy.sleep(2)
        return TriggerResponse(success=True, message="Simulation reset")
    except Exception as e:
        rospy.logerr("Failed to reset simulation: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def stop_gazebo2_msf(req):
    global gazebo_pid
    if gazebo_pid is None:
        rospy.logerr("No Gazebo PID found")
        return TriggerResponse(success=False, message="No Gazebo PID found")
    try:
        proc = psutil.Process(gazebo_pid)
        rospy.loginfo("Terminating gzserver with PID {}".format(proc.pid))
        proc.terminate()  # Gracefully terminate the process
        try:
            proc.wait(timeout=3)
        except psutil.TimeoutExpired:
            rospy.logwarn("Process {} did not terminate in time. Killing it.".format(proc.pid))
            proc.kill()  # Forcefully terminate if not terminated gracefully
        gazebo_pid = None  # Reset the PID
        return TriggerResponse(success=True, message="Simulation stopped")
    except Exception as e:
        rospy.logerr("Failed to stop gazebo: {}".format(e))
        return TriggerResponse(success=False, message=str(e))
    
def set_initial_velocities(req):
    global current_velocity_index  # Use the global index
    try:
        # Get initial velocities for this iteration
        if initial_velocities:
            velocities = initial_velocities[current_velocity_index]
            current_velocity_index = (current_velocity_index + 1) % len(initial_velocities)
        else:
            velocities = [5, 0, 20, 5, -2, 8]  # Default values if the file is empty or all velocities have been used

        # Run the set_initial_velocity_node.py script with the velocities
        velocity_str = " ".join(map(str, velocities))
        set_velocity_command = 'rosrun rotors_gazebo set_initial_velocity_node.py {}'.format(velocity_str)
        subprocess.Popen(set_velocity_command, shell=True)
        rospy.sleep(2)  # Give some time for the node to set velocities
        return TriggerResponse(success=True, message="Velocities imprinted")
    except Exception as e:
        rospy.logerr("Failed to set initial velocities: {}".format(e))
        return TriggerResponse(success=False, message=str(e))
def main():
    os.environ['ROS_MASTER_URI'] = 'http://localhost:11320'
    os.environ['GAZEBO_MASTER_URI'] = 'http://localhost:11352'
    rospy.init_node('simulation_control_node_msf')
    read_initial_velocities()
    rospy.Service('start_gazebo_world_msf', Trigger, start_gazebo_world_msf)
    rospy.Service('set_initial_velocities', Trigger, set_initial_velocities)
    rospy.Service('start_simulation_msf', Trigger, start_simulation_msf)
    rospy.Service('start_data_saver_node_msf', Trigger, start_data_saver_node_msf)
    rospy.Service('stop_simulation_msf', Trigger, stop_simulation_msf)
    rospy.Service('reset_simulation_msf', Trigger, reset_simulation_msf)
    rospy.Service('stop_gazebo2_msf', Trigger, stop_gazebo2_msf)
    rospy.spin()

if __name__ == "__main__":
    main()
