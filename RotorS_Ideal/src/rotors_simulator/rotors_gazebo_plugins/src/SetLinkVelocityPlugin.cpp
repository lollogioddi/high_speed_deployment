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
      std::string robotNamespace;
      std::string linkName;

    public:
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
      {
        this->model = _model;

        // Read the robot namespace and link name from the SDF
        if (_sdf->HasElement("robotNamespace"))
          robotNamespace = _sdf->Get<std::string>("robotNamespace");
        else
          gzerr << "SetLinkVelocityPlugin missing <robotNamespace>, cannot proceed\n";

        if (_sdf->HasElement("linkName"))
          linkName = _sdf->Get<std::string>("linkName");
        else
          gzerr << "SetLinkVelocityPlugin missing <linkName>, cannot proceed\n";

        if (!ros::isInitialized())
        {
          int argc = 0;
          char** argv = nullptr;
          ros::init(argc, argv, "set_link_velocity_plugin");
        }

        rosNode.reset(new ros::NodeHandle(robotNamespace));
        velocitySubscriber = rosNode->subscribe("velocity_topic", 10,
                                                &SetLinkVelocityPlugin::VelocityCallback, this);

        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));
      }

      void VelocityCallback(const geometry_msgs::Twist& msg)
      {
        auto link = model->GetLink(linkName);
        if (link)
        {
          link->SetLinearVel({msg.linear.x, msg.linear.y, msg.linear.z});
          link->SetAngularVel({msg.angular.x, msg.angular.y, msg.angular.z});
          ROS_INFO("Velocity applied");
          velocitiesReceived = true;  // Indicate that velocities have been set
        }
        else
        {
          ROS_WARN("Link %s not found, velocity not applied", linkName.c_str());
        }
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
