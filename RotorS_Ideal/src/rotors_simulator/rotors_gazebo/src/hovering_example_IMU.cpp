/* USING IMU
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
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu imu;
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // Estrarre il messaggio di odometria
   imu = *msg;
  //std::cout << "IMU" << imu << std::endl;
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


  //tentativo di introdurre il sensing del free falling utilizzando IMU
  ros::Subscriber imu_sub = nh.subscribe("ground_truth/imu", 10, imuCallback);

  bool should_exit = false;

	while (ros::ok() && !should_exit) {

	   //usare direttamente l'accelerazione calcolata dall'IMU
	   ros::Time current_time = imu.header.stamp;
  	   double current_acceleration = imu.linear_acceleration.z;
    	   std::cout << "current acceleration = " << current_acceleration << std::endl;
  	   // Verifica se l'accelerazione è vicina all'accele5razione gravitazionale.
  	   double gravitational_acceleration = -9.81; // Accelerazione gravitazionale in m/s^2
		if (std::abs(current_acceleration) < 0.1) {
    				// Se siamo vicini all'accelerazione gravitazionale, segui la traiettoria.
  				//ros::Duration(2.0).sleep();
				should_exit = true;

  			}
			else {
				ROS_INFO("NOT FALLING");
			}
 	
	 // else{ ROS_INFO("THERE IS A PROBLEM WITH TIME");}
  	  ros::spinOnce();
 	  ros::Duration(0.1).sleep();  // Attendere 0.1 secondi tra le iterazioni.
         }

 // Default desired position and yaw.
  	   Eigen::Vector3d desired_position(0.0, 0.0, 200.0);
  	   double desired_yaw = 0.0;
	   mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
	   ROS_INFO("questo è imu Publishing waypoint on namespace %s: [%f, %f, %f].", nh.getNamespace().c_str(), desired_position.x(), 
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


