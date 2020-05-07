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



#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"

#include "common.h"


namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

/*
// Protobuf test
typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;
static const std::string kDefaultMotorTestSubTopic = "motors";
*/

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

class GazeboMotorModel : public MotorModel, public ModelPlugin {
 public:
  GazeboMotorModel()
      : ModelPlugin(),
        MotorModel(),
        command_sub_topic_(kDefaultCommandSubTopic),
        motor_failure_sub_topic_(kDefaultMotorFailureNumSubTopic),
        motor_speed_pub_topic_(kDefaultMotorVelocityPubTopic),
        motor_number_(0),
        motor_Failure_Number_(0),
        turning_direction_(turning_direction::CW),
        max_force_(kDefaultMaxForce),
        max_rot_velocity_(kDefaulMaxRotVelocity),
        moment_constant_(kDefaultMomentConstant),
        motor_constant_(kDefaultMotorConstant),
        //motor_test_sub_topic_(kDefaultMotorTestSubTopic),
        ref_motor_rot_vel_(0.0),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        time_constant_down_(kDefaultTimeConstantDown),
        time_constant_up_(kDefaultTimeConstantUp) {
  }

  virtual ~GazeboMotorModel();

  virtual void InitializeParams();
  virtual void Publish();
  //void testProto(MotorSpeedPtr &msg);
 protected:
  virtual void UpdateForcesAndMoments();
  /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
  /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
  virtual void UpdateMotorFail();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_sub_topic_;
  std::string motor_failure_sub_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  int motor_number_;
  int turning_direction_;

  int motor_Failure_Number_; /*!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish accordingly */
  int tmp_motor_num; // A temporary variable used to print msg

  int screen_msg_flag = 1;

  double max_force_;
  double max_rot_velocity_;
  double moment_constant_;
  double motor_constant_;
  double ref_motor_rot_vel_;
  double rolling_moment_coefficient_;
  double rotor_drag_coefficient_;
  double rotor_velocity_slowdown_sim_;
  double time_constant_down_;
  double time_constant_up_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_pub_;
  transport::SubscriberPtr command_sub_;
  transport::SubscriberPtr motor_failure_sub_; /*!< Subscribing to motor_failure_sub_topic_; receiving motor number to fail, as an integer */

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  common::PID pid_;
  bool use_pid_;
  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  std_msgs::msgs::Float turning_velocity_msg_;
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
  void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg);  /*!< Callback for the motor_failure_sub_ subscriber */
  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
/*
  // Protobuf test
  std::string motor_test_sub_topic_;
  transport::SubscriberPtr motor_sub_;
*/
};
}
