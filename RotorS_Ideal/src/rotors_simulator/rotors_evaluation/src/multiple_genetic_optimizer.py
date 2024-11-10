#!/usr/bin/env python2.7
import random
import numpy as np
import pandas as pd
from deap import base, creator, tools, algorithms
import matplotlib.pyplot as plt
from deap.benchmarks.tools import hypervolume
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float64MultiArray, Bool
import os
import traceback
import pickle
import threading
import time
from itertools import izip_longest

# Initialize ROS node
rospy.init_node('multiple_genetic_algorithm_optimizer', anonymous=True)

# Define the bounds for the gains
BOUND_LOW = 0.0
BOUND_UP = 6.0
N_GENES = 12

# Define the number of individuals in the population
POP_SIZE = 40  # Equivalent to MU
MAX_GEN = 250  # Equivalent to NGEN
CXPB = 0.8
MUTPB = 0.06
RETRY_LIMIT = 3

# Define your known good gains
known_good_gains = known_good_gains = []



                    
# Create the toolbox with the right parameters
creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0, -1.0))  # Adjust weights for multi-objective optimization
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, BOUND_LOW, BOUND_UP)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, N_GENES)
toolbox.register("population", tools.initRepeat, list, toolbox.individual, POP_SIZE)

# Register the genetic operators
toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=MUTPB)
toolbox.register("select", tools.selNSGA2)

# Callback to receive the performance metrics
class MetricsListener:
    def __init__(self, drone_index):
        self.received = False
        self.overshoots = []
        self.settling_times = []
        self.rmse_values = []
        self.namespace = 'pelican_{}'.format(drone_index + 1)
        rospy.Subscriber('/{}/performance_metrics'.format(self.namespace), Float64MultiArray, self.callback)
        rospy.loginfo("Subscribed to /{}/performance_metrics".format(self.namespace))
    
    def callback(self, msg):
        self.overshoots = list(msg.data[:6])
        self.settling_times = list(msg.data[6:12])
        self.rmse_values = list(msg.data[12:])
        self.received = True
        rospy.loginfo("Received performance metrics for {}: {}, {}, {}".format(self.namespace, self.overshoots, self.settling_times, self.rmse_values))

# Create a list of metric listeners for each drone
metrics_listeners = [MetricsListener(i) for i in range(4)]  # Assuming 4 drones

def evaluate_population(individuals):
    num_drones = 3
    # Divide the population into subpopulations, each for one drone
    subpopulations = [individuals[i::num_drones] for i in range(num_drones)]
    all_fitnesses = [None] * len(individuals)  # Initialize a list to store fitness for each individual

    print("Evaluating population")

    # Iterate over each set of individuals corresponding to each drone
    for subpop_set in izip_longest(*subpopulations, fillvalue=None):
        # Reset metrics listener before starting a new evaluation
        for listener in metrics_listeners:
            listener.received = False
            listener.overshoots = []
            listener.settling_times = []
            listener.rmse_values = []

        # Update each drone's configuration with its respective individual
        for drone_index, individual in enumerate(subpop_set):
            if individual is not None:  # Skip if there is no individual in the subpopulation
                rospy.loginfo("Updating drone {} with individual: {}".format(drone_index + 1, individual))
                update_drone_configuration(individual, drone_index)

        # Ensure subscribers are ready
        rospy.sleep(1)

        # Start Gazebo
        if not start_gazebo():
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue
        
        # Set initial velocities
        if not call_service('/set_initial_velocities', Trigger):
            rospy.logerr("Failed to set initial velocities")
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue
        
        # Start simulation for all drones together
        if not start_collective_simulation():
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue

        # Wait for simulation to complete
        rospy.sleep(60)  # Or the necessary time for your simulation

        # Stop and reset the simulation
        if not stop_and_reset_collective_simulation():
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue

        # Signal data_saver to compute metrics for each drone
        for drone_index in range(num_drones):
            stop_sim_pub = rospy.Publisher('/pelican_{}/stop_simulation_signal'.format(drone_index + 1), Bool, queue_size=10)
            rospy.loginfo("Stopping signal sent: {}".format(drone_index + 1))
            rospy.sleep(1)  # Give some time for the publisher to connect
            stop_sim_pub.publish(True)

        # Wait for the metrics to be published
        rospy.sleep(10)  # Wait a bit for the data saver to process and publish the metrics

        # Collect and calculate fitnesses for all drones
        for drone_index, individual in enumerate(subpop_set):
            if individual is not None:
                fitness = calculate_fitness_for_drone(drone_index)
                if fitness is None:
                    fitness = (1e6, 1e6, 1e6)
                index = individuals.index(individual)
                all_fitnesses[index] = fitness
                # Check if the sum of performance metrics is below the threshold
                if sum(fitness) < 150:
                    rospy.loginfo("Fitness sum under 200, gains saved for future analysis")
                    save_gains_and_performance(individual, fitness, '/your/directory/SMC_genetic/good_gains_and_performances_lee_SMC.pkl')

        # Signal the data saver node to shutdown
        for drone_index in range(num_drones):
            shutdown_pub = rospy.Publisher('/pelican_{}/shutdown_data_saver'.format(drone_index + 1), Bool, queue_size=10)
            rospy.loginfo("Shutdown data saver node {}".format(drone_index + 1))
            rospy.sleep(1)  # Give some time for the publisher to connect
            shutdown_pub.publish(True)

        if not call_service('/stop_gazebo2', Trigger):
            rospy.logerr("Failed to stop gazebo: large cost returned")
            assign_high_cost_to_all(subpop_set, all_fitnesses, individuals)
            continue

    # Check if all drones provided metrics
    reevaluate_indices = []
    for i, fitness in enumerate(all_fitnesses):
        if fitness == (1e6, 1e6, 1e6):  # High cost indicates a need for reevaluation
            drone_index = i % num_drones
            reevaluate_indices.append((i, drone_index))
            rospy.loginfo("Drone {} didn't provide metrics".format(drone_index + 1))

    # # Reevaluate the drones that did not provide metrics
    # for i, drone_index in reevaluate_indices:
    #     individual = individuals[i]
    #     fitness = reevaluate_individual(individual, drone_index)
    #     all_fitnesses[i] = fitness

    return all_fitnesses

def assign_high_cost_to_all(subpop_set, all_fitnesses, individuals):
    for drone_index, individual in enumerate(subpop_set):
        if individual is not None:
            index = individuals.index(individual)
            all_fitnesses[index] = (1e6, 1e6, 1e6)

def update_drone_configuration(individual, drone_index):
    # Function to update configuration files or parameters for each drone
    p_gains = individual[:3]
    d_gains = individual[3:6]
    pos_gains = individual[6:9]
    vel_gains = individual[9:]
    yaml_path = '/your/directory/catkin_ws/src/rotors_simulator/rotors_gazebo/resource/lee_controller_pelican_for_tuning_{}.yaml'.format(drone_index + 1)
    python_script_path = '/your/directory/additional_files/modify_PID.py'
    command_str = 'python2.7 {} {} {} {} {} {} {} {} {} {} {} {} {} {}'.format(
        python_script_path, yaml_path, p_gains[0], p_gains[1], p_gains[2], d_gains[0], d_gains[1], d_gains[2], pos_gains[0], pos_gains[1], pos_gains[2], vel_gains[0], vel_gains[1], vel_gains[2])
    os.system(command_str)

def start_gazebo():
    # Retry mechanism for starting the Gazebo world
    max_retries = 3
    retry_delay = 5  # seconds
    for attempt in range(max_retries):
        if call_service('/start_gazebo_world', Trigger):
            return True
        rospy.logwarn("Attempt {}/{} to start Gazebo world failed, retrying in {} seconds...".format(attempt + 1, max_retries, retry_delay))
        rospy.sleep(retry_delay)
    rospy.logerr("Failed to start Gazebo world after {} attempts: large cost returned")
    return False

def start_collective_simulation():
    # Start the simulation for all drones
    if not call_service('/start_simulation', Trigger) or not call_service('/start_data_saver_nodes', Trigger):
        rospy.logerr("Failed to start simulation: large cost returned")
        return False
    return True

def stop_and_reset_collective_simulation():
    # Stop and reset the simulation for all drones
    if not call_service('/stop_simulation', Trigger) or not call_service('/reset_simulation', Trigger):
        rospy.logerr("Failed to stop or reset simulation: large cost returned")
        return False
    return True

def calculate_fitness_for_drone(drone_index):
    # Assuming metrics listeners are correctly set up and collecting data
    metrics_listener = metrics_listeners[drone_index]
    timeout = 20  # Set a timeout for waiting
    start_time = time.time()
    
    # Wait loop to wait for the received flag to be set
    while not metrics_listener.received and (time.time() - start_time) < timeout:
        rospy.logerr("received still in: {}".format(metrics_listener.received))
        rospy.sleep(0.1)  # Sleep for a short time before checking again

    # Debug print before checking the received flag
    rospy.loginfo("Before checking, drone {}: received={}, overshoots={}, settling_times={}, rmse_values={}".format(
        drone_index + 1, metrics_listener.received, metrics_listener.overshoots, metrics_listener.settling_times, metrics_listener.rmse_values))
    if not metrics_listener.received:
        rospy.logerr("Failed to receive performance metrics in time for drone {}.".format(drone_index + 1))
        return (1e6, 1e6, 1e6)

    # Handle infinite settling time using a smooth penalty function
    metrics_listener.settling_times = [
        80 / (1 + np.exp(-0.01 * (measure - 40))) if measure == float('inf') else measure 
        for measure in metrics_listener.settling_times
    ]
    
    metrics_listener.rmse_values = [
        80 / (1 + np.exp(-0.01 * (measure - 40))) if measure == float('inf') else measure 
        for measure in metrics_listener.rmse_values
    ]

    # Debug print before checking the received flag
    rospy.loginfo("After checking, drone {}: overshoots={}, settling_times={}, rmse_values={}".format(
        drone_index + 1, metrics_listener.overshoots, metrics_listener.settling_times, metrics_listener.rmse_values))
    # Calculate fitness from received metrics
    overshoot = sum(metrics_listener.overshoots)
    settling_time = sum(metrics_listener.settling_times)
    rmse_values = sum(metrics_listener.rmse_values)

    # Debug print before checking the received flag
    rospy.loginfo("Fitness values, drone {}: overshoots={}, settling_times={}, rmse_values={}".format(
        drone_index + 1, overshoot, settling_time, rmse_values))
    
    return (overshoot, settling_time, rmse_values)

def reevaluate_individual(individual, drone_index):
    metrics_listener = metrics_listeners[drone_index]
    for attempt in range(RETRY_LIMIT):
        try:
            # Similar logic to evaluate_individual, but for reevaluation
            update_drone_configuration(individual, drone_index)

            # Start simulation for all drones together again
            if not start_collective_simulation():
                return (1e6, 1e6, 1e6)  # High cost if simulation fails to start

            # Wait for simulation to complete
            rospy.sleep(15)  # Or the necessary time for your simulation

            # Stop and reset the simulation
            if not stop_and_reset_collective_simulation():
                return [(1e6, 1e6, 1e6)] * len(individual)  # High cost for all if unable to stop/reset
            
            #Signal data_saver to compute metrics for each drone
            for drone_index in range(4):
                stop_sim_pub = rospy.Publisher('/pelican_{}/stop_simulation_signal'.format(drone_index + 1), Bool, queue_size=10)
                rospy.sleep(1)  # Give some time for the publisher to connect
                stop_sim_pub.publish(True)

            # Wait for the metrics to be published
            rospy.sleep(5)  # Wait a bit for the data saver to process and publish the metrics

            # Collect and calculate fitness for the specific drone
            if not metrics_listener.received:
                rospy.logerr("Failed to receive performance metrics in time for drone {}.".format(drone_index + 1))
                continue

            # Handle infinite settling time using a smooth penalty function
            metrics_listener.settling_times = [
                80 / (1 + np.exp(-0.01 * (time - 40))) if time == float('inf') else time 
                for time in metrics_listener.settling_times
            ]

            metrics_listener.rmse_values = [
                80 / (1 + np.exp(-0.01 * (time - 40))) if measure == float('inf') else measure 
                for measure in metrics_listener.rmse_values
            ]

            overshoot = sum(metrics_listener.overshoots)
            settling_time = sum(metrics_listener.settling_times)
            rmse_values = sum(metrics_listener.rmse_values)
            metrics_listener.received = False  # Reset for next evaluation

            return (overshoot, settling_time, rmse_values)
        except Exception as e:
            rospy.logerr("Error in reevaluation for drone {}: {}. Attempt {}/{}".format(drone_index + 1, e, attempt + 1, RETRY_LIMIT))
            rospy.sleep(5)  # Wait before retrying

    return (1e6, 1e6, 1e6)  # Return high cost if all retries fail

# Helper function to call ROS services
def call_service(service_name, service_type):
    try:
        rospy.wait_for_service(service_name, timeout=10)
        service_client = rospy.ServiceProxy(service_name, service_type)
        response = service_client()
        return response.success
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call to {} failed: {}".format(service_name, e))
        return False

# Register the evaluation function
toolbox.register("evaluate", evaluate_population)

def log_results(logbook, filename='multiple_genetic_algorithm_log_lee_SMC.csv'):
    # Extracting generations, min cost, and avg cost from the logbook
    gen = logbook.select("gen")
    min_ = logbook.select("min")
    avg = logbook.select("avg")

    # Check if the logbook contains the necessary data
    if not gen or not min_ or not avg:
        print('Logbook data missing or incomplete, proceeding with available data')

    # Creating a DataFrame to save the results
    data = {'Generation': gen}
    if min_:
        data['Min Cost'] = min_
    if avg:
        data['Avg Cost'] = avg

    df = pd.DataFrame(data)
    df.to_csv(filename, index=False)
    print('Logged results to {}'.format(filename))

def plot_convergence(logbook):
    # Extracting generations, avg cost, and min cost from the logbook
    gen = logbook.select("gen")
    avg = logbook.select("avg")
    min_ = logbook.select("min")

    if not gen:
        print("Logbook data is empty, cannot plot convergence.")
        return

    # Plotting convergence
    if avg:
        plt.plot(gen, avg, label="Average", linestyle="--")
    if min_:
        plt.plot(gen, min_, label="Minimum", linestyle="-.")
    
    plt.title("Convergence")
    plt.xlabel('Generation')
    plt.ylabel('Fitness')
    plt.legend()
    plt.show()

# Function to plot hypervolume
def plot_hypervolume(hyper):
    if not hyper:
        print("Hypervolume data is empty, cannot plot.")
    plt.plot(hyper, 'bo')
    plt.title('Hypervolume')
    plt.xlabel('Generation')
    plt.ylabel('Hypervolume')
    plt.show()
    
# Save checkpoint
def save_checkpoint(population, logbook, gen, pareto, hyper):
    checkpoint = {
        'population': population,
        'logbook': logbook,
        'gen': gen,
        'pareto': pareto,
        'hyper': hyper,
    }
    with open('checkpoint_lee_SMC.pkl', 'wb') as f:
        pickle.dump(checkpoint, f)

# Load checkpoint
def load_checkpoint():
    try:
        with open('checkpoint_lee_SMC.pkl', 'rb') as f:
            checkpoint = pickle.load(f)
        return checkpoint['population'], checkpoint['logbook'], checkpoint['gen'], checkpoint['pareto'], checkpoint['hyper']
    except (IOError, KeyError):
        return None

def load_gains_from_pickle():
    with open('/home/lollogioddi/Desktop/bash_related_files/SMC_genetic/population.pkl', 'rb') as file:
        gains = pickle.load(file)
    return gains

def initialize_population(pop_size, gain_range, known_gains):
    population = []
    # Include the known good gains in the initial population
    #last_gains = load_gains_from_pickle()
    last_gains = known_gains
    for gains in last_gains:
        population.append(creator.Individual(gains))
    # Fill the rest of the population with random gains
    remaining_size = pop_size - len(last_gains)
    for _ in range(remaining_size):
        individual = [random.uniform(gain_range[0], gain_range[1]) for _ in range(N_GENES)]
        population.append(creator.Individual(individual))
    return population

def save_best_individual(individual, filename):
    with open(filename, 'wb') as f:
        pickle.dump(individual, f)
    rospy.loginfo("Saved best individual to {}".format(filename))

def save_population(population, filename):
    with open(filename, 'wb') as f:
        pickle.dump(population, f)
    rospy.loginfo("Saved population to {}".format(filename))

def save_gains_and_performance(gains, performance, filename):
    data = {'gains': gains, 'performance': performance}
    with open(filename, 'ab') as f:  # Use 'ab' to append to the file
        pickle.dump(data, f)
    rospy.loginfo("Saved gains and performance to {}".format(filename))

def main():
    try:
        checkpoint = load_checkpoint()

        if checkpoint:
            population, logbook, start_gen, pareto, hyper = checkpoint
            for ind in population:
                rospy.loginfo("Initialized Individual - Length: {}, Values: {}".format(len(ind), ind))
        else:
            population = initialize_population(POP_SIZE, (BOUND_LOW, BOUND_UP), known_good_gains)
            for ind in population:
                rospy.loginfo("Initialized Individual - Length: {}, Values: {}".format(len(ind), ind))
            logbook = tools.Logbook()
            logbook.header = "gen", "std", "min", "avg"
            start_gen = 0
            pareto = tools.ParetoFront()
            hyper = []

            # Register statistics
            stats = tools.Statistics(lambda ind: ind.fitness.values)
            stats.register("avg", np.mean)
            stats.register("std", np.std)
            stats.register("min", np.min)
            stats.register("max", np.max)

            # Evaluate the initial population
            fitnesses = evaluate_population(population)
            print('Fitness in main: {}'.format(fitnesses))
            for ind, fit in zip(population, fitnesses):
                rospy.loginfo("Initial fitness type for individual {}: {}, value: {}".format(ind, type(fit), fit))
                if fit is None:
                    fit = (1e6, 1e6, 1e6)
                assert isinstance(fit, tuple) and len(fit) == 3, "Each fitness value should be a tuple with 3 elements"
                ind.fitness.values = fit

            # Compile statistics about the population
            record = stats.compile(population)
            logbook.record(gen=start_gen, **record)

        # Begin the evolution
        for gen in range(start_gen + 1, MAX_GEN + 1):
            # Select the next generation individuals
            offspring = tools.selNSGA2(population, len(population))
            offspring = [toolbox.clone(ind) for ind in offspring]

            # Apply crossover and mutation on the offspring
            for ind1, ind2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < CXPB:
                    toolbox.mate(ind1, ind2)
                    del ind1.fitness.values
                    del ind2.fitness.values

                if random.random() < MUTPB:
                    toolbox.mutate(ind1)
                    toolbox.mutate(ind2)
                    del ind1.fitness.values
                    del ind2.fitness.values

            # Evaluate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = evaluate_population(invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                rospy.loginfo("Fitness type for individual {}: {}, value: {}".format(ind, type(fit), fit))
                if fit is None:
                    fit = (1e6, 1e6, 1e6)
                assert isinstance(fit, tuple) and len(fit) == 3, "Each fitness value should be a tuple with 3 elements"
                ind.fitness.values = fit

            # Gather all the fitnesses in one list and print the stats
            population = toolbox.select(population + offspring, POP_SIZE)

            pareto.update(population)

            # Ensure reference point has the same dimensionality as the Pareto front
            ref_point = [2e7] * len(population[0].fitness.values)
            hyper.append(hypervolume(pareto, ref_point))

            record = tools.Statistics(lambda ind: ind.fitness.values).compile(population)
            logbook.record(gen=gen, **record)
            print(logbook.stream)

            # Save checkpoint
            save_checkpoint(population, logbook, gen, pareto, hyper)

        # Get the best individual
        best_ind = tools.selBest(population, 1)[0]
        print('Best individual: {}'.format(best_ind))
        print('Fitness: {}'.format(best_ind.fitness.values[0]))

        # Save the best individual
        save_best_individual(best_ind, '/your/directory/SMC_genetic/best_individual_lee_SMC.pkl')

        # Optionally, save the entire population
        save_population(population, '/your/directory/SMC_genetic/population_lee_SMC.pkl')

        # Log the results
        log_results(logbook, filename='/your/directory/SMC_genetic/multiple_genetic_algorithm_log_lee_SMC.csv')

        # Plot convergence, hypervolume, and Pareto front
        plot_convergence(logbook)
        plot_hypervolume(hyper)
    except Exception as e:
        rospy.logerr("An error occurred in the main function: {}".format(e))
        rospy.logerr("Full exception: {}".format(traceback.format_exc()))  # <-- Full traceback
        # Add any additional cleanup or logging here if necessary

if __name__ == "__main__":
    main()
