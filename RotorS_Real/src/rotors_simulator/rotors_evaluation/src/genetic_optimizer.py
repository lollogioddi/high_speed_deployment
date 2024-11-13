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
import pickle

# Initialize ROS node
rospy.init_node('genetic_algorithm_optimizer_msf', anonymous=True)

# Define the bounds for the gains
BOUND_LOW = 0.01
BOUND_UP = 2.00
N_GENES = 6

# Define the number of individuals in the population
POP_SIZE = 20  # Equivalent to MU
MAX_GEN = 250  # Equivalent to NGEN
CXPB = 0.9
MUTPB = 0.1
RETRY_LIMIT = 3

# Create the toolbox with the right parameters
creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, BOUND_LOW, BOUND_UP)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, N_GENES)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Register the genetic operators
toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=MUTPB)
toolbox.register("select", tools.selNSGA2)

# Callback to receive the performance metrics
class MetricsListener:
    def __init__(self):
        self.received = False
        self.overshoots = []
        self.settling_times = []
        rospy.Subscriber('/performance_metrics_msf', Float64MultiArray, self.callback)
    
    def callback(self, msg):
        data = msg.data
        self.overshoots = list(data[:6])
        self.settling_times = list(data[6:])
        self.received = True

metrics_listener = MetricsListener()

# Define the evaluation function
def evaluate(individual):
    for attempt in range(RETRY_LIMIT):
        try:
            p_gains = individual[:3]
            d_gains = individual[3:]

            # Update the YAML file with the new gains
            yaml_path = '/home/lollogioddi/RotorS_2/src/rotors_simulator/rotors_gazebo/resource/lee_controller_pelican_for_tuning_2.yaml'
            python_script_path = '/home/lollogioddi/Desktop/bash_related_files/modify_PID_matlab_2.py'
            command_str = 'python2.7 {} {} {} {} {} {} {} {}'.format(
                python_script_path, yaml_path, p_gains[0], p_gains[1], p_gains[2], d_gains[0], d_gains[1], d_gains[2])
            status = os.system(command_str)
            if status != 0:
                raise RuntimeError('Failed to execute the Python script.')

            # Retry mechanism for starting the Gazebo world
            max_retries = 3
            retry_delay = 5  # seconds
            for attempt in range(max_retries):
                if call_service('/start_gazebo_world_msf', Trigger):
                    break
                rospy.logwarn("Attempt {}/{} to start Gazebo world failed, retrying in {} seconds...".format(attempt + 1, max_retries, retry_delay))
                rospy.sleep(retry_delay)
            else:
                rospy.logerr("Failed to start Gazebo world after {} attempts: large cost returned".format(max_retries))
                return (1e6, 1e6)  # Return a large cost if the service call fails
    
            # Start the simulation
            if not call_service('/start_simulation_msf', Trigger) or not call_service('/start_data_saver_node_msf', Trigger):
                rospy.logerr("Failed to start simulation: large cost returned")
                continue

            # Wait for the simulation to complete
            rospy.sleep(15)

            # Stop the simulation
            if not call_service('/stop_simulation_msf', Trigger):
                rospy.logerr("Failed to stop simulation: large cost returned")
                continue

            # Reset the simulation
            if not call_service('/reset_simulation_msf', Trigger):
                rospy.logerr("Failed to reset simulation: large cost returned")
                continue

            # Signal data_saver to compute metrics
            stop_sim_pub = rospy.Publisher('/stop_simulation_signal_msf', Bool, queue_size=10)
            rospy.sleep(1)  # Give some time for the publisher to connect
            stop_sim_pub.publish(True)

            # Wait for the metrics to be published
            rospy.sleep(5)  # Wait a bit for the data saver to process and publish the metrics

            if not metrics_listener.received:
                rospy.logerr("Failed to receive performance metrics in time.")
                continue

            # Handle infinite settling time using a smooth penalty function
            metrics_listener.settling_times = [
                80 / (1 + np.exp(-0.01 * (time - 40))) if time == 80 else time 
                for time in metrics_listener.settling_times
            ]
            
            # Extract the performance metrics
            overshoot = sum(metrics_listener.overshoots)
            settling_time = sum(metrics_listener.settling_times)

            # Signal the data saver node to shutdown
            shutdown_pub = rospy.Publisher('/shutdown_data_saver_msf', Bool, queue_size=10)
            rospy.sleep(1)  # Give some time for the publisher to connect
            shutdown_pub.publish(True)

            if not call_service('/stop_gazebo2_msf', Trigger):
                rospy.logerr("Failed to stop gazebo: large cost returned")
                continue

            return (overshoot, settling_time)
        except Exception as e:
            rospy.logerr("Error in evaluation: {}. Attempt {}/{}".format(e, attempt + 1, RETRY_LIMIT))
            rospy.sleep(5)  # Wait before retrying

    # Return a large cost if all retries fail
    return (1e6, 1e6)

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
toolbox.register("evaluate", evaluate)

def log_results(logbook, filename='genetic_algorithm_log_msf.csv'):
    # Extracting generations, min cost, and avg cost from the logbook
    gen = logbook.select("gen")
    min_ = logbook.select("min")
    avg = logbook.select("avg")

    # Creating a DataFrame to save the results
    df = pd.DataFrame({'Generation': gen, 'Min Cost': min_, 'Avg Cost': avg})
    df.to_csv(filename, index=False)
    print('Logged results to {}'.format(filename))

def plot_convergence(logbook):
    # Extracting generations, avg cost, and min cost from the logbook
    gen = logbook.select("gen")
    avg = logbook.select("avg")
    min_ = logbook.select("min")

    # Plotting convergence
    plt.plot(gen, avg, label="Average", linestyle="--")
    plt.plot(gen, min_, label="Minimum", linestyle="-.")
    plt.title("Convergence")
    plt.xlabel('Generation')
    plt.ylabel('Fitness')
    plt.legend()
    plt.show()

# Function to plot hypervolume
def plot_hypervolume(hyper):
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
    with open('checkpoint_msf.pkl', 'wb') as f:
        pickle.dump(checkpoint, f)

# Load checkpoint
def load_checkpoint():
    try:
        with open('checkpoint_msf.pkl', 'rb') as f:
            checkpoint = pickle.load(f)
        return checkpoint['population'], checkpoint['logbook'], checkpoint['gen'], checkpoint['pareto'], checkpoint['hyper']
    except (IOError, KeyError):
        return None

# Run the genetic algorithm
def main():
    checkpoint = load_checkpoint()

    if checkpoint:
        population, logbook, start_gen, pareto, hyper = checkpoint
        rospy.loginfo("Resuming from generation {}".format(start_gen))
    else:
        population = toolbox.population(n=POP_SIZE)
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
        fitnesses = list(map(toolbox.evaluate, population))
        for ind, fit in zip(population, fitnesses):
            ind.fitness.values = fit

        # Compile statistics about the population
        record = tools.Statistics(lambda ind: ind.fitness.values).compile(population)
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

            toolbox.mutate(ind1)
            toolbox.mutate(ind2)
            del ind1.fitness.values
            del ind2.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
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

    # Log the results
    log_results(logbook, filename='/home/lollogioddi/Desktop/bash_related_files/genetic_algorithm_log_msf.csv')

    # Plot convergence, hypervolume, and Pareto front
    plot_convergence(logbook)
    plot_hypervolume(hyper)

if __name__ == "__main__":
    main()
