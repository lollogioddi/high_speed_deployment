/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <ros/ros.h>

// namespace gazebo
// {
//   //////////////////////////////////////////////////
//   /// \brief Sets velocity on a link or joint
//   class SetLinkVelocityPlugin : public ModelPlugin
//   {
//     public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
//       {
//         this->model = _model;
//         this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//         std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));
//       }

//     public: void Update(const common::UpdateInfo &_info)
//       {
//         if (update_num == 0)
//         {
//           // Link velocity instantaneously without applying forces
//           model->GetLink("pelican/base_link")->SetLinearVel({10, 0, 20}); // m/s 5, 5, 2 / ieri sera -5, 10, 15
//           model->GetLink("pelican/base_link")->SetAngularVel({10, -10, 5}); // rad/s 0.2, -0.3, 0.5 / ieri sera 2, -1.5, 1
//           std::cout << "initial velocities imprinted" << std::endl; //debugging
//           ROS_INFO("Velocity applied");


//         }
//         update_num++;
//       }

//     /// \brief a pointer to the model this plugin was loaded by
//     public: physics::ModelPtr model;
//     /// \brief object for callback connection
//     public: event::ConnectionPtr updateConnection;
//     /// \brief number of updates received
//     public: int update_num = 0;
//   };

//   GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
// }

// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h> // For using vector3 for velocities
// #include <std_srvs/SetBool.h>    // For simplicity, let's use this to trigger velocity updates

// namespace gazebo
// {
//   class SetLinkVelocityPlugin : public ModelPlugin
//   {
//     private: physics::ModelPtr model;
//     private: event::ConnectionPtr updateConnection;
//     private: int update_num = 0;
//     private: ros::NodeHandle *nh;
//     private: ros::ServiceServer velocityService;
//     private: double linearX = 10, linearY = 0, linearZ = 20;
//     private: double angularX = 10, angularY = -10, angularZ = 5;

//     public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
//     {
//       this->model = _model;
//       this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//         std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));

//       if (!ros::isInitialized())
//       {
//         int argc = 0;
//         char **argv = NULL;
//         ros::init(argc, argv, "set_link_velocity_plugin");
//       }
//       nh = new ros::NodeHandle();

//       velocityService = nh->advertiseService("update_velocities", &SetLinkVelocityPlugin::UpdateVelocities, this);
//     }

//     public: bool UpdateVelocities(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
//     {
//       // Example of adjusting velocities based on request, could be made more complex
//       if(req.data)
//       {
//         linearX += 3; // Increment velocities for demonstration
//         linearY += 3;
//         linearZ += 3;
//         angularX += 3;
//         angularY += 3;
//         angularZ += 3;
//       }
//       else
//       {
//         linearX -= 3; // Increment velocities for demonstration
//         linearY -= 3;
//         linearZ -= 3;
//         angularX -= 3;
//         angularY -= 3;
//         angularZ -= 3;
//       }
//       res.success = true;
//       res.message = "Velocity updated";
//       return true;
//     }

//     public: void Update(const common::UpdateInfo &_info)
//     {
//       if (update_num == 0)
//       {
//         model->GetLink("pelican/base_link")->SetLinearVel({linearX, linearY, linearZ});
//         model->GetLink("pelican/base_link")->SetAngularVel({angularX, angularY, angularZ});
//         std::cout << "Initial velocities imprinted: " << "Linear X: " << linearX << ", Angular X: " << angularX << std::endl;
//       }
//       update_num++;
//     }
//   };

//   GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
// }


//service version
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <ros/ros.h>
// #include <ros/callback_queue.h>
// #include <ros/subscribe_options.h>
// #include <thread>
// #include "rotors_comm/SetInitialVelocities.h"  // Include your service message

// namespace gazebo
// {
//   class SetLinkVelocityPlugin : public ModelPlugin
//   {
//     private:
//       physics::ModelPtr model;
//       event::ConnectionPtr updateConnection;
//       std::unique_ptr<ros::NodeHandle> rosNode;
//       ros::ServiceServer service;
//       ros::AsyncSpinner spinner; // Async spinner to handle ROS callbacks in a separate thread

//     public:
//       // Constructor
//       SetLinkVelocityPlugin() : spinner(1) {}  // Initialize the spinner with 1 thread

//       // Destructor
//       ~SetLinkVelocityPlugin() {
//         if (ros::isStarted()) {
//           spinner.stop(); // Stop the spinner
//           ros::shutdown();
//           ros::waitForShutdown();
//         }
//       }

//       // Load the plugin
//       void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
//       {
//         this->model = _model;

//         // Initialize ROS
//         if (!ros::isInitialized())
//         {
//           int argc = 0;
//           char **argv = nullptr;
//           ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
//         }

//         // Create a ROS node
//         this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

//         // Create a named service server to set initial velocities
//         this->service = this->rosNode->advertiseService("set_initial_velocities",
//           &SetLinkVelocityPlugin::SetInitialVelocities, this);

//         ROS_INFO("Service set_initial_velocities ready.");

//         // Start the spinner to process incoming service requests asynchronously
//         spinner.start();
//       }

//       // Service callback function
//       bool SetInitialVelocities(rotors_comm::SetInitialVelocities::Request &req,
//                                 rotors_comm::SetInitialVelocities::Response &res)
//       {
//         ROS_INFO("Received request to set velocities: Linear(%f, %f, %f), Angular(%f, %f, %f)",
//                  req.linear_x, req.linear_y, req.linear_z, req.angular_x, req.angular_y, req.angular_z);

//         if (!this->model->GetWorld()->IsPaused()) {
//             this->model->GetLink("pelican/base_link")->SetLinearVel({req.linear_x, req.linear_y, req.linear_z});
//             this->model->GetLink("pelican/base_link")->SetAngularVel({req.angular_x, req.angular_y, req.angular_z});

//             ROS_INFO("Velocities updated successfully.");
//             res.success = true;
//             res.message = "Velocities updated successfully.";
//         } else {
//             ROS_WARN("Failed to update velocities: simulation is paused.");
//             res.success = false;
//             res.message = "Failed to update velocities: simulation is paused.";
//         }
//         return true;
//       }

//       // Called by the world update start event
//       void OnUpdate(const common::UpdateInfo & /*info*/)
//       {
//         // This function can be used for further updates if needed
//       }
//   };

//   // Register this plugin with the simulator
//   GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
// }

//topic version
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
  class SetLinkVelocityPlugin : public ModelPlugin
  {
    private:
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Subscriber velocitySubscriber;
      bool velocitiesReceived = false;  // Tracks if new velocities have been received

    public:
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
      {
        this->model = _model;

        if (!ros::isInitialized())
        {
          int argc = 0;
          char** argv = nullptr;
          ros::init(argc, argv, "set_link_velocity_plugin");
        }

        rosNode.reset(new ros::NodeHandle());
        velocitySubscriber = rosNode->subscribe("velocity_topic", 10,
                                                &SetLinkVelocityPlugin::VelocityCallback, this);

        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));
      }

      void VelocityCallback(const geometry_msgs::Twist& msg)
      {
        model->GetLink("pelican/base_link")->SetLinearVel({msg.linear.x, msg.linear.y, msg.linear.z});
        model->GetLink("pelican/base_link")->SetAngularVel({msg.angular.x, msg.angular.y, msg.angular.z});
        ROS_INFO("Velocity applied 25");
        velocitiesReceived = true;  // Indicate that velocities have been set
      }

      void Update(const common::UpdateInfo & /*info*/)
      {
        if (velocitiesReceived) {
          // Optionally disable further updates if you only want the initial set to take effect
          updateConnection.reset();
          ROS_INFO("Velocities applied once and updates are now disabled.");
        }
      }
  };

  GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
}
