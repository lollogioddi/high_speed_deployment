#!/usr/bin/env python2.7
import sys
import os

# Add the path to the scripts folder of the rotors_gazebo package
sys.path.append('/home/lollogioddi/catkin_ws/src/rotors_simulator/rotors_gazebo/scripts')
sys.path.append('/home/lollogioddi/Desktop/bash_related_files/abc_algorithm')

import random
import numpy as np
import pandas as pd
import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, Bool
import time
from itertools import izip_longest
from evaluate_population import evaluate_population  # Import from the scripts directory
from ABC import ABC
from Config import Config
from Reporter import Reporter

# Initialize ROS node
rospy.init_node('multiple_ABC_algorithm_optimizer', anonymous=True)

# Define the bounds for the gains
BOUND_LOW = 0.0
BOUND_UP = 10.0
N_GENES = 6

def main():
    config = Config([])
    abc = ABC(config)
    start_time = time.time() * 1000
    abc.initial()
    abc.memorize_best_source()

    while not abc.stopping_condition():
        abc.send_employed_bees()
        abc.calculate_probabilities()
        abc.send_onlooker_bees()
        abc.memorize_best_source()
        abc.send_scout_bees()
        abc.increase_cycle()

    abc.globalTime = time.time() * 1000 - start_time

    # Report the results
    Reporter([abc])

    # Log the results
    print('Best gains: {}'.format(abc.globalParams))
    print('Best fitness: {}'.format(abc.globalOpt))

if __name__ == "__main__":
    main()
