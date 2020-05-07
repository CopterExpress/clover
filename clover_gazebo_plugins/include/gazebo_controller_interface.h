/*
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "CommandMotorSpeed.pb.h"
#include "MotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <ignition/math.hh>

#include <stdio.h>

#include "common.h"

namespace gazebo {

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;

// Default values
static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultCommandMotorSpeedSubTopic = "/command/motor_speed";

class GazeboControllerInterface : public ModelPlugin {
 public:
  GazeboControllerInterface()
      : ModelPlugin(),
        received_first_referenc_(false),
        namespace_(kDefaultNamespace),
        motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
        command_motor_speed_sub_topic_(kDefaultCommandMotorSpeedSubTopic) {}
  ~GazeboControllerInterface();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  bool received_first_referenc_;
  Eigen::VectorXd input_reference_;

  std::string namespace_;
  std::string motor_velocity_reference_pub_topic_;
  std::string command_motor_speed_sub_topic_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::SubscriberPtr cmd_motor_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void CommandMotorCallback(CommandMotorSpeedPtr &input_reference_msg);
};
}
