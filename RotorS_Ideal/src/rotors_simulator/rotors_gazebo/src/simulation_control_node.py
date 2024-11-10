#!/usr/bin/env python2.7

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess
import signal
import os
import psutil
import requests

gazebo_pid = None

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


def start_gazebo_world(req): 
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

def start_simulation(req):
    try:
        if not is_gazebo_running():
            rospy.loginfo("Gazebo is not running. Starting Gazebo...")
            stop_gazebo()  # Stop any existing unhealthy Gazebo processes
            if not start_gazebo_world():  # Start Gazebo with an empty world
                return TriggerResponse(success=False, message="Failed to launch Gazebo world")

        subprocess.Popen(["roslaunch", "rotors_gazebo", "multiple_drones.launch",
                          "mav_name:=pelican", "controller_type:=QuaternionBased_Spherical_SMC.py",
                          "enable_my_logging:=false", "gains_file:=_for_tuning"])    
        # subprocess.Popen(["roslaunch", "rotors_gazebo", "mav_hovering_example.launch",
        #                   "mav_name:=pelican", "controller_type:=QuaternionBased_Spherical_SMC.py",
        #                   "enable_my_logging:=false", "gains_file:=_for_tuning"])

        return TriggerResponse(success=True, message="Simulation started")
    except Exception as e:
        rospy.logerr("Failed to start simulation: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def start_data_saver_node(req):
    try:
        rospy.loginfo("Starting data saver node...")
        subprocess.Popen(["roslaunch", "rotors_gazebo", "data_saver_node.launch"])
        rospy.sleep(5)  # Give some time for the nodes to start
        return TriggerResponse(success=True, message="Data saver node started")
    except Exception as e:
        rospy.logerr("Failed to start data saver node: {}".format(e))
        return TriggerResponse(success=False, message=str(e))

def stop_simulation(req):
    try:
        rospy.loginfo("Stopping simulation...")
        subprocess.Popen(["pkill", "-f", "roslaunch rotors_gazebo multiple_drones.launch"])
        # subprocess.Popen(["pkill", "-f", "roslaunch rotors_gazebo mav_hovering_example.launch"])
        rospy.sleep(2)
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
    
def main():
    rospy.init_node('simulation_control_node')
    rospy.Service('start_gazebo_world', Trigger, start_gazebo_world)    
    rospy.Service('start_simulation', Trigger, start_simulation)
    rospy.Service('start_data_saver_node', Trigger, start_data_saver_node)
    rospy.Service('stop_simulation', Trigger, stop_simulation)
    rospy.Service('reset_simulation', Trigger, reset_simulation)
    rospy.Service('stop_gazebo2', Trigger, stop_gazebo2)
    rospy.spin()

if __name__ == "__main__":
    main()
