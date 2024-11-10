import rospy
from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Trigger
import numpy as np
import os
import time
from itertools import izip_longest

class MetricsListener:
    def __init__(self, drone_index):
        self.received = False
        self.overshoots = []
        self.settling_times = []
        self.namespace = 'pelican_{}'.format(drone_index + 1)
        rospy.Subscriber('/{}/performance_metrics'.format(self.namespace), Float64MultiArray, self.callback)
        rospy.loginfo("Subscribed to /{}/performance_metrics".format(self.namespace))
    
    def callback(self, msg):
        self.overshoots = list(msg.data[:6])
        self.settling_times = list(msg.data[6:])
        self.received = True
        rospy.loginfo("Received performance metrics for {}: {}, {}".format(self.namespace, self.overshoots, self.settling_times))

metrics_listeners = [MetricsListener(i) for i in range(4)]  # Assuming 4 drones

def evaluate_population(individuals):
    num_drones = 4
    subpopulations = [individuals[i::num_drones] for i in range(num_drones)]
    all_fitnesses = [None] * len(individuals)

    print("Evaluating population")

    for subpop_set in izip_longest(*subpopulations, fillvalue=None):
        for listener in metrics_listeners:
            listener.received = False
            listener.overshoots = []
            listener.settling_times = []

        for drone_index, individual in enumerate(subpop_set):
            if individual is not None:
                rospy.loginfo("Updating drone {} with individual: {}".format(drone_index + 1, individual))
                update_drone_configuration(individual, drone_index)

        rospy.sleep(1)

        if not start_gazebo():
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue

        if not start_collective_simulation():
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue

        rospy.sleep(15)

        if not stop_and_reset_collective_simulation():
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue

        for drone_index in range(num_drones):
            stop_sim_pub = rospy.Publisher('/pelican_{}/stop_simulation_signal'.format(drone_index + 1), Bool, queue_size=10)
            rospy.loginfo("Stopping signal sent: {}".format(drone_index + 1))
            rospy.sleep(1)
            stop_sim_pub.publish(True)

        rospy.sleep(10)

        for drone_index, individual in enumerate(subpop_set):
            if individual is not None:
                fitness = calculate_fitness_for_drone(drone_index)
                if fitness is None:
                    fitness = (1e6, 1e6)
                index = individuals.index(individual)
                all_fitnesses[index] = fitness

        for drone_index in range(num_drones):
            shutdown_pub = rospy.Publisher('/pelican_{}/shutdown_data_saver'.format(drone_index + 1), Bool, queue_size=10)
            rospy.loginfo("Shutdown data saver node {}".format(drone_index + 1))
            rospy.sleep(1)
            shutdown_pub.publish(True)

        if not call_service('/stop_gazebo2', Trigger):
            rospy.logerr("Failed to stop gazebo: large cost returned")
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue

    return all_fitnesses

def assign_high_cost_to_all(subpop_set, all_fitnesses, individuals):
    for drone_index, individual in enumerate(subpop_set):
        if individual is not None:
            index = individuals.index(individual)
            all_fitnesses[index] = (1e6, 1e6)

def update_drone_configuration(individual, drone_index):
    p_gains = individual[:3]
    d_gains = individual[3:]
    yaml_path = '/home/lollogioddi/catkin_ws/src/rotors_simulator/rotors_gazebo/resource/lee_controller_pelican_for_tuning_{}.yaml'.format(drone_index + 1)
    python_script_path = '/home/lollogioddi/Desktop/bash_related_files/modify_PID_matlab.py'
    command_str = 'python2.7 {} {} {} {} {} {} {} {}'.format(
        python_script_path, yaml_path, p_gains[0], p_gains[1], p_gains[2], d_gains[0], d_gains[1], d_gains[2])
    os.system(command_str)

def start_gazebo():
    max_retries = 3
    retry_delay = 5
    for attempt in range(max_retries):
        if call_service('/start_gazebo_world', Trigger):
            return True
        rospy.logwarn("Attempt {}/{} to start Gazebo world failed, retrying in {} seconds...".format(attempt + 1, max_retries, retry_delay))
        rospy.sleep(retry_delay)
    rospy.logerr("Failed to start Gazebo world after {} attempts: large cost returned")
    return False

def start_collective_simulation():
    if not call_service('/start_simulation', Trigger) or not call_service('/start_data_saver_nodes', Trigger):
        rospy.logerr("Failed to start simulation: large cost returned")
        return False
    return True

def stop_and_reset_collective_simulation():
    if not call_service('/stop_simulation', Trigger) or not call_service('/reset_simulation', Trigger):
        rospy.logerr("Failed to stop or reset simulation: large cost returned")
        return False
    return True

def calculate_fitness_for_drone(drone_index):
    metrics_listener = metrics_listeners[drone_index]
    timeout = 20
    start_time = time.time()
    
    while not metrics_listener.received and (time.time() - start_time) < timeout:
        rospy.logerr("received still in: {}".format(metrics_listener.received))
        rospy.sleep(0.1)

    rospy.loginfo("Before checking, drone {}: received={}, overshoots={}, settling_times={}".format(
        drone_index + 1, metrics_listener.received, metrics_listener.overshoots, metrics_listener.settling_times))
    if not metrics_listener.received:
        rospy.logerr("Failed to receive performance metrics in time for drone {}.".format(drone_index + 1))
        return (1e6, 1e6)

    metrics_listener.settling_times = [
        80 / (1 + np.exp(-0.01 * (measure - 40))) if measure == 80 else measure 
        for measure in metrics_listener.settling_times
    ]
    
    overshoot = sum(metrics_listener.overshoots)
    settling_time = sum(metrics_listener.settling_times)
    return (overshoot, settling_time)

def call_service(service_name, service_type):
    try:
        rospy.wait_for_service(service_name, timeout=10)
        service_client = rospy.ServiceProxy(service_name, service_type)
        response = service_client()
        return response.success
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call to {} failed: {}".format(service_name, e))
        return False
