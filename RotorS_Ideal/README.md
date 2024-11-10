# high_speed_deployment
!!!!!!!!!!!!!!!!!Disclaimer: I apologize for any grammar error and typo you might find in here. In case of doubt, if you simply don't understand something or it doesn't work as expected, you can contact me at 'lorenzo2.giordano@mail.polimi.it'. If I don't answer, I invite you to write to me again, I simply didn't see it.!!!!!!!!!!!!!!!!!!

[Introduction to the UAV Control Dynamics Setup]
-
The control dynamics setup was developed to analyze and optimize the stabilization of unmanned aerial vehicles (UAVs), when deployed from a moving fixed-wing aircraft. It was specifically focused on the Pelican quadcopter but it can be extended to any other UAVs configuration.
The aim was to enable rapid, robust recovery and control of the UAV in high-velocity conditions, making it suitable for various real-world applications such as disaster response and surveillance.

This setup is built on the RotorS simulator, a UAV simulation environment developed by ETH Zurich (https://github.com/ethz-asl/rotors_simulator), which provides a highly realistic platform for simulating UAV flight dynamics. To support high-speed deployment scenarios, customized features are added to the RotorS environment, allowing the simulation of deployment-specific conditions such as initial velocity disturbances and environmental effects.

This setup was crucial for:
- Control Strategy Testing: Various advanced control algorithms were implemented and compared, such as Sliding Mode Control, Lee Geometric Control, and Hybrid Controls;
- Gain Tuning Optimization: Heuristic method as Genetic Algorithm (GA) was implemented for fine-tuning of controller gains;
- Delay Optimization: Optimal delay for activating the control system was determined analyzing the natural decay of oscillations in the quadcopter’s dynamics.

IMPORTANT: I used
- ubuntu 18.04
- Gazebo9
- RosMelodic
- python2.7.17 for Ros files
- python3.8 for additional files

In this git repository (link) you will find 3 main directories
- RotorS_Ideal;
- RotorS_Real;
- additional_files;
and  four main documents:
- 2016_rotors.pdf: explains the RotorS platform and how to install it from scratch if you have any problem or simply don't want my set up (I strongly invite you to read it);
- Aero.Next: Optimizing UAV Control Dynamics for Enhanced Stabilization in High-Velocity Aerial Deployments.pdf: my master thesis with all the information and the details of my research (for example the control strategies in detail);
- executive_summary_LG.pdf: the executive summary of the thesis for a more general ideal;
- Master_thesis_presentation_LG: the slides of the dissertation's presentation;
------------------------------------
That being said, let's start.

We have 2 different directories:
- one for the Ideal sensor scenario
- one for the Real sensor scenario
The reason why I used two different folders, actually the second one also contains the entire standard RotorS package with Ideal sensor, is because messing around with the simulator I had to reinstall it multiple times and so when I started messing around with the integration of the Real sensors, It sounded like a good idea not to risk the already unstable and unpredictable Ideal sensor working environment.
If you prefere to have the two things in one directory I suggest you to copy and paste the file you need for the Idea sensor in the Real sensor folder. Most of the files are already there (for example everything related to the controllers) but some adjustment might be needed for the rotors_gazebo package.

---------------------------------------------------------------------------------------------------------

[INDEX]
-
This document is divided in:

- Download

- Ideal Sensor files description
   | how to launch and use
   | how do I restart the simulation   
   | GA gains tuning

- Real sensor files decriptions
   | how to launch and use
   | how do I restart the simulation

- Additional files

---------------------------------------------------------------------------------------------------------

[DOWNLOAD]
-
In order to make them work, you have to
- Download Ros Melodic as described in here: 
- Dowload Gazebo (I was using Gazebo9);
- Create in your home directory for each scenario (for example: RotorS_Ideal and RotorS_Real);
- Dowload the right 'src' folder in the right directory you just created;
- Open a terminal window. Go on that main directory you just created and run 'catkin build';
- now you have to run: source ~/your_folder_name/devel/setup.bash;

If it doesn't work you have to follow the official dowload instruction of RotorS ( you should find them in '2016_rotors.pdf') and manually add the additional files (the one presented later on) and adjust each 'CMakeLists.txt' and 'package.xml'; 

---------
I'm now going through all the main directories highlighting the additions you do not have in the RotorS standard package.
Later on, I'm explaining how to launch and use the simulator and the tuning method.

---------------------------------------------------------------------------------------------------------

[IDEAL SENSOR FOLDER]
-
a) rotors_control
    | src
   Here you find the controller files.

- Lee_position_controller.py:
  The classical version of the Lee Geometric Control with Position and Attitude control active from the beginning;

- Lee_position_controller_with_profiler.py:
  It insert a profiler to check the time required from each part of the code to run;

- Lee_position_controller_evolution.py:
  The evolution of the Lee Geometric approach to address more complex and aggressive aerial operations;

- Lee_decoupled_controller.py:
  The classical version of the Lee Geometric Control but the Position control activates only after attitude stabilization. A constant values of vertical acceleration is used in the initial stages. For aggressive stabilization at high speed you have to use this one or it's evolution and not the Lee_position_controller.py (or evolution) because the initial position error is too high and disruptive for the control strategy. The desired position is set in-flight only after stabilization. you do not choose the linear position in space, only the angular position;

- Lee_decoupled_controller_evolution.py:
  The evolution of the Lee Geometric approach to address more complex and aggressive aerial operations. It's valid everything said in the previous point (this one is the second best control strategy);

- QuaternionBased_Spherical_SMC.py:
  The Sliding Mode Controller. It uses quaternions instead of euler angles for attitude. It's valid everything said in Decoupled versions;

- Hybrid_controller.py:
  Control strategy to blend together the Lee and the SMC. First aggressive attitude stabilization with SMC and seconf phase with Lee for stable flight (needs further tuning and a smoother phase switching function). It's valid everything said in Decoupled versions;

- Gain_scheduled_controller.py:
  It uses Lee Evolution. Two phases. One with aggressive set of gains and one with stable flight  gains. It's valid everything said in Decoupled versions. Best overall result;

b) rotors_description
    | urdf
   Here you find the drone configuration files

- multirotor_base.xacro: 
  In here I added two plugins to the drone. If you want to remove or change them look for:
   - set_link_velocity_plugin (set's the initial speeds imprinted to the drone when the simulation start, rotors turned off)
   - gazebo_wind_plugin (set's the wind conditions)

c) rotors_evaluation 
    |src
   - data_saver.py: it collects, processes, and publishes UAV position, orientation, and errors from various topics. It calculates performance metrics including overshoots, settling times, and RMSE, and publishes them;

   - genetic_optimizer.py: it's the genetic algorithm used for tuning the gains. It's the singol drone use;

   - multiple_genetic_optimizer.py: It's the multiple drones use;

   - multiple_ABC_otimizer: It containes an Artificial Bee Colony algorithm. The idea is to use it for gains tuning. It was never tested. It still needs to be compleated. Have fun;


d) rotors_gazebo
 [Launch]
  - empty_world.launch: It launched gazebo;

  - mav_hovering_example.launch and spawn_mav.launch: They are responsible for spawning the drone on gazebo and start the simulation. Here you specify the drone and controller type and the .py file that starts the simulation and imprint the desired commands. Thanks to 'enable_my_logging' you can use 'data_saver_node' to save the desired topics.  

  - control.launch: it launches the 'simulation_control_node' (genetic algorithm use with one drone only);

  - data_saver_node.launch: it launches the 'data_saver_node';
  
  - multiple_control.launch: it launches 'simulation_control_node' (genetic algorithm use with multiple drone);

  - multiple_data_saver_node.launch: it launches multiple 'data_saver_node', one for each namespace. In my case up to 4 drone for cpu limitaions;

  - multiple_drones.launch: it's the equivalent of 'mav_hovering_example.launch' but for launching multiple drones, each one with a different namespace, on the same Gazebo window. In my case up to 4 drone for cpu limitaions. Very useful during tuning with GA or simply to test at the same time.

 [Resource]
  - lee_controller_pelican_for_tuning.yaml: There are 5 of those without and with numbers from 1 to 4. It's the file from which the controllers read the gains. Each file was used by one drone during the tuning process. You can put inside whethever value you want if you keep the structure. If you change the names of the gains group you have to change it also in the controller (initial part of the controller files);

  - pelican.yaml: it contains the pelican vehicle parameters;

 [Scripts]
  - set_initial_velocity_node.py: it sets the initial linear and angular velocities (using the plugin). If you want to use it you have to run it (in a specific order later explained) as: 'rosrun rotors_gazebo set_initial_velocity_node.py velX velY velZ angX angY angZ';

 [src]
  - hovering_example.py: it containes the intial commands and starts the simulation;

  - simulation_control_node.py: Used in the single drone gains tuning with GA. It's responsible for launching the correct files for the simulation, launching and killing Gazebo and resetting the simulation. It allows to simulate iterativelly without blocks;

  - multiple_simulation_control_node.py: same as above but used in multiple drones gains tuning with GA;
-----------------------------------------------
HOW TO LAUNCH AND USE:
-
[in flight deployment]
- in 'spawn_mav.launch' change the x,y,z linear and angular position as desired. The drone will be spawned in that positon and with that initial attitude;

- open a terminal window and launch Gazebo: 'roslaunch rotors_gazebo empty_world.launch' (if you don't need to see the drone, set the gui:=false to reduce the cpu load, the risk of Gazebo to crash and speed up the loading process);

- open a new terminal window and give the velocities you want your drone to be subjected at the start of the simulation: 'rosrun rotors_gazebo set_initial_velocity_node.py velX velY velZ angX angY angZ';

-open a new terminal window and launch: 'roslaunch rotors_gazebo mav_hovering_example.py'. you can specify which type of drone and controller you want and if you want to save simulation data, for example: 'roslaunch rotors_gazebo mav_hovering_example.py mav_name:=pelican controller_type:=Gain_scheduled.py enable_my_logging:=true';

the Gazebo window will not follow the drone automatically and standard view is at 0,0,0 position, so if you are spawning it at a different location (for example I was doing 0,0,200m) you might want to go in 'rotors_gazebo/src/hovering_example.py' and comment the 'unpause_gazebo()' call. The physic of the system will remain paused and you will have the time to go to the Gazebo gui, click on your drone object and go to 'Follow' to move to the point in space where the drone is. 
Then, you will have to open a new terminal window and run 'rosservice call gazebo/unpause_physics' to start the simulation.

IMPORTANT: Every time I changed something in a script I had to run again 'catkin build' to apply the change to the environment. That didn't applied only with the launch and yaml files.

[how do I restart the simulation]
For what I saw, the cleanest way is to kill everything and launch in sequence again.
You can also kill the 'roslaunch rotors_gazebo mav_hovering_example.py' and the 'rosrun rotors_gazebo set_initial_velocity_node.py' (but it should automatically kill itself) and then run in this sequence and only in this sequence:
1. 'rosservice call gazebo/pause_physics'
2. 'rosservice call /gazebo/delete_model '{model_name: pelican}'
3. 'rosservice call /gazebo/reset_simulation'
then, again:
1. set initial velocities: 'rosrun rotors_gazebo set_initial_velocity_node.py'
2. launch the drone files: 'roslaunch rotors_gazebo mav_hovering_example.py'
You do not have to close Gazebo like that and can speed up the simulation process, but it might lead to errors because Gazebo might not be cleaning itself properly at each iteration and expecially with multiple drones I suggest to close everything and restart.
--------------------------------------
[GA gains tuning]
In order to use the GA for tuning the controller parameters you have to:
- firtly, launch the 'simulation_control_node' with: 'roslaunch rotors_gazebo control.launch' for single drone and 'roslaunch rotors_gazebo multiple_control.launch' for multiple drones;

- then, open a new terminal window and launch the 'Genetic Algorithm' with: 'rosrun rotors_evaluation genetic_optimizer.py' for single drone and 'rosrun rotors_evaluation multiple_genetic_optimizer.py' for multiple drones;

In the algorithm file you have to select the path where you want to save the final results, the good results you get on the way and most importantly the checkpoints file. With the checkpoints if the system crashes for any reason you can simply close all the terminal windows and launch everything again, starting from where it crashed.

---------------------------------------------------------------------------------------------------------

[REAL SENSOR FOLDER]
-
What changes in here is the presence of the packages for the sensor fusion approach that simulate real data (for more details on it, read the '2016_rotors.pdf', the '2013_IROS_lynen_modular_sensor_fusion.pdf' in the 'ethzasl_msf' package and the corresponding chapter in my Thesis doc) and of the 'rotors_simulator_demos' package that containes the launch file you need.
you have:
- 'rotors_simulator_demos/launch/mav_hovering_example_msf.launch' that is the equivalent of the 'mav_hovering_example.launch in the Ideal case;

- 'rotors_simulator_demos/src/hovering_example_msf.py' that is the equivalent of the 'hovering_example.py' in the Ideal case;

All the rest is the same and all the cosideration done for the Ideal sensor are still valid.

---------------------------------------
HOW TO LAUNCH AND USE:
-
[in flight deployment]

- in the rotors_gazebo/launch package, in 'spawn_mav.launch' change the x,y,z linear and angular position as desired. The drone will be spawned in that positon and with that initial attitude;

- open a terminal window and launch Gazebo: 'roslaunch rotors_gazebo empty_world.launch' (if you don't need to see the drone, set the gui:=false to reduce the cpu load, the risk of Gazebo to crash and speed up the loading process);

- open a new terminal window and give the velocities you want your drone to be subjected at the start of the simulation: 'rosrun rotors_gazebo set_initial_velocity_node.py velX velY velZ angX angY angZ';
  
-open a new terminal window and launch: 'roslaunch rotors_simulator_demos mav_hovering_example_msf.py'. you can specify which type of drone and controller you want and id you want to save simulation data, for example: 'roslaunch rotors_simulator_demos mav_hovering_example_msf.py mav_name:=pelican controller_type:=Gain_scheduled.py enable_my_logging:=true';

-------------------------------------
[how do I restart the simulation]
Same as in the Ideal sensor case

---------------------------------------------------------------------------------------------------------
[ADDITIONAL FILES]
-
In the 'additional_file' folder you have some scipt that are useful to read and analyse the simulation data saved.
During the simulation logging, data are recorded in rosbags. Those bags might need some refinement since some topics publish not perfectly sinchronized.
you have:
- from_bag_to_csv.py: it converts the rosbag in a csv;

- data_cleaner.py / clean_data_redundancy.py / filtering_data.py: they clean the data preparing them for analysis, removing multiple timestamps, assuring the correct sampling and interpolating where is needed;

Those scipts will probably need to be adjusted to your specific use.


Good luck!!




