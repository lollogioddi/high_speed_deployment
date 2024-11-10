/* IDEAL ODOMETRY
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the LicensY
/pelican/ground_truth/twist.Twist.Linear.Ze is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


nav_msgs::Odometry odometry;
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Estrarre il messaggio di odometria
   odometry = *msg;
  //std::cout << "ODOMETRY" << odometry << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  /*while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  } */

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(1.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  //tentativo di introdurre il sensing del free falling
  ros::Subscriber odometry_sub = nh.subscribe("ground_truth/odometry", 10, odometryCallback);

  ros::Time previous_time;
  double previous_velocity = 0.0;

  bool should_exit = false;

	while (ros::ok() && !should_exit) {
  	  
	   //passare da velocità ad accelerazione
	   ros::Time current_time = odometry.header.stamp;
  	   double current_velocity = odometry.twist.twist.linear.z;
  	   double current_position = odometry.pose.pose.position.z;
    	   std::cout << "current time = " << current_time << std::endl;
    	   std::cout << "current velocity = " << current_velocity << std::endl;
	   // Calcola l'intervallo di tempo
  	   ros::Duration dt = current_time - previous_time;
    	   //std::cout << "dt = " << dt << std::endl;
	   //std::cout << "dt.Sec = " << dt.toSec() << std::endl;
	   // Assicurati che sia passato un po' di tempo prima di fare la derivata
  	   if (dt.toSec() > 0.001) {
    	   	// Calcola la differenza di velocità
    	   	double delta_velocity = current_velocity - previous_velocity;

    	   	// Calcola l'accelerazione
    	   	double linear_acceleration_z = delta_velocity / dt.toSec();
                std::cout << "current ACC = " << linear_acceleration_z << std::endl;

  		// Verifica se l'accelerazione è vicina all'accele5razione gravitazionale.
  		double gravitational_acceleration = -9.81; // Accelerazione gravitazionale in m/s^2
  		double threshold = 0.1; // Tolleranza di 0.5 m/s^2

  		bool is_near_gravity = (std::abs(linear_acceleration_z - gravitational_acceleration) < threshold && linear_acceleration_z < 0);
                std::cout << "is_near_gravity = " << is_near_gravity << std::endl;
                std::cout << "deltaACC = " << linear_acceleration_z - gravitational_acceleration << std::endl;
                std::cout << "Z = " << current_position << std::endl;

  			if (is_near_gravity) {
    				// Se siamo vicini all'accelerazione gravitazionale, segui la traiettoria.
  				
				should_exit = true;

  			}
			else {
				ROS_INFO("NOT FALLING");
			}
		previous_time = current_time;
    		previous_velocity = current_velocity; 
 	       }
	 // else{ ROS_INFO("THERE IS A PROBLEM WITH TIME");}
  	  ros::spinOnce();
 	  ros::Duration(0.1).sleep();  // Attendere 0.1 secondi tra le iterazioni.
         }

 // Default desired position and yaw.
  	   Eigen::Vector3d desired_position(0.0, 0.0, 200.0);
  	   double desired_yaw = 0.0;
	   mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
	   ROS_INFO("questo è OD Publishing waypoint on namespace %s: [%f, %f, %f].", nh.getNamespace().c_str(), desired_position.x(), 
                    desired_position.y(), desired_position.z());
	  trajectory_pub.publish(trajectory_msg);

  // Overwrite defaults if set as node parameters. 
  /*nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg); */


  ros::spinOnce();
  ros::shutdown();
 ROS_INFO("ROS NOT OK");
  return 0;
}


