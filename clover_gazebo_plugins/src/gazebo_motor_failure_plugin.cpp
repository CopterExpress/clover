/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
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

#include <gazebo_motor_failure_plugin.h>

namespace gazebo {

GazeboMotorFailure::GazeboMotorFailure() :
    ModelPlugin(),
    ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
    motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic)
{ }

GazeboMotorFailure::~GazeboMotorFailure() {
  this->updateConnection_.reset();

  this->rosQueue.clear();
  this->rosQueue.disable();
  this->rosNode->shutdown();
  this->rosQueueThread.join();

  delete this->rosNode;
}

void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  this->namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);

  if (!_sdf->HasElement("ROSMotorNumSubTopic")) {
    ROS_FATAL_NAMED("MotorFailure", "MotorFailure plugin missing <ROSMotorNumSubTopic>, cannot proceed");
    return;
  }
  else
    this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();

  if (!_sdf->HasElement("MotorFailureNumPubTopic")) {
    ROS_FATAL_NAMED("MotorFailure", "MotorFailure plugin missing <MotorFailureNumPubTopic>, cannot proceed");
    return;
  }
  else
    this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();

  // ROS Topic subscriber
  // Initialize ROS, if it has not already been initialized.
  if (!ros::isInitialized())  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_ros_sub", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosNode = new ros::NodeHandle(this->namespace_);

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int32>(ROS_motor_num_sub_topic_,
        1,
        boost::bind(&GazeboMotorFailure::motorFailNumCallBack, this, _1),
        ros::VoidPtr(),
        &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  this->rosQueueThread = std::thread(std::bind(&GazeboMotorFailure::QueueThread, this));

  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_ << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboMotorFailure::OnUpdate, this, _1));
}

void GazeboMotorFailure::OnUpdate(const common::UpdateInfo &info) {
    this->motor_failure_msg_.set_data(motor_Failure_Number_);
    this->motor_failure_pub_->Publish(motor_failure_msg_);
}

void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::Int32ConstPtr &msg) {
  this->motor_Failure_Number_ = msg->data;
}

void GazeboMotorFailure::QueueThread() {
  static const double timeout = 0.01;

  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
}
