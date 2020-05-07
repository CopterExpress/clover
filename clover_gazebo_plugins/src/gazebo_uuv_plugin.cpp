/*
 * Copyright 2016 Austin Buchan, Nils Rottmann TUHH Hamburg, Germany
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
 *
 */

/*
 * This scripts simulates underwater froces and torques for the hippocampus model. The HippoCampus is an autonomous
 * underwater vehicle (AUV) designed by the Technical University Hamburg-Harburg (TUHH).
 * https://www.tuhh.de/mum/forschung/forschungsgebiete-und-projekte/flow-field-estimation-with-a-swarm-of-auvs.html
 */


#include "gazebo_uuv_plugin.h"
#include <ignition/math.hh>

namespace gazebo {

GazeboUUVPlugin::~GazeboUUVPlugin() {
  update_connection_->~Connection();
}

// Load necessary data from the .sdf file
void GazeboUUVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  namespace_.clear();
  getSdfParam<std::string>(
    _sdf, "robotNamespace", namespace_, namespace_, true);

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);

  // get the base link, thus the base hippocampus model
  link_ = _model->GetLink(link_name_);
  // get the child links, these are the links which represents the rotors of the hippocampus
  rotor_links_ = link_->GetChildJointsLinks();
  for(int i = 0; i < rotor_links_.size(); i++) {
    std::cout << "Rotor Link:" << rotor_links_[i]->GetScopedName() << "\n";
    command_[i] = 0.0;
  }

  getSdfParam<std::string>(
    _sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);

  // subscribe to the commands (actuator outputs from the mixer from PX4)
  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
    "~/" + _model->GetName() + command_sub_topic_, &GazeboUUVPlugin::CommandCallback, this);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboUUVPlugin::OnUpdate, this, _1));

  // get force and torque parameters for force and torque calculations of the rotors from motor_speed
  getSdfParam<double>(
    _sdf, "motorForceConstant", motor_force_constant_, motor_force_constant_);
  getSdfParam<double>(
    _sdf, "motorTorqueConstant", motor_torque_constant_, motor_torque_constant_);

  // parameters for added mass and damping
  ignition::math::Vector3d added_mass_linear(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "addedMassLinear", added_mass_linear, added_mass_linear);
  X_udot_ = added_mass_linear[0];
  Y_vdot_ = added_mass_linear[1];
  Z_wdot_ = added_mass_linear[2];

  ignition::math::Vector3d added_mass_angular(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "addedMassAngular", added_mass_angular, added_mass_angular);
  K_pdot_ = added_mass_angular[0];
  M_qdot_ = added_mass_angular[1];
  N_rdot_ = added_mass_angular[2];

  ignition::math::Vector3d damping_linear(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "dampingLinear", damping_linear, damping_linear);
  X_u_ = damping_linear[0];
  Y_v_ = damping_linear[1];
  Z_w_ = damping_linear[2];

  ignition::math::Vector3d damping_angular(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "dampingAngular", damping_angular, damping_angular);
  K_p_ = damping_angular[0];
  M_q_ = damping_angular[1];
  N_r_ = damping_angular[2];

  // variables for debugging
  time_ = 0.0;
  counter_ = 0.0;

}

// function to get the motor speed
void GazeboUUVPlugin::CommandCallback(CommandMotorSpeedPtr &command) {
  for (int i = 0; i < 4; i++) {
    command_[i] = command->motor_speed(i);
  }
  /*std::cout << "UUV Command Callback:"
    << command_[0] << ","
    << command_[1] << ","
    << command_[2] << ","
    << command_[3] << ","
    << "\n"; */

}

// Update function, this runs in every circle
void GazeboUUVPlugin::OnUpdate(const common::UpdateInfo& _info) {
  double now = _info.simTime.Double();
  time_delta_ =  now - last_time_;
  last_time_ = now;
  time_ = time_ + time_delta_;

  //std::cout << "UUV Update at " << now << ", delta " << time_delta_ << "\n";

  double forces[4];
  double torques[4];

  // Apply forces and torques at rotor joints
  for(int i = 0; i < 4; i++) {

    // Currently a rotor index hack to get over IMU link being first, since rotor_links_[0] would be the IMU
    ignition::math::Vector3d rotor_force(motor_force_constant_ * command_[i] * std::abs(command_[i]), 0, 0);
    rotor_links_[i+1]->AddRelativeForce(rotor_force);

    forces[i] = rotor_force[0];
    //std::cout << "Applying force " << rotor_force[2] << " to rotor " << i << "\n";

    // CCW 1, CW 2, CCW 3 and CW 4. Apply drag torque
    // directly to main body X axis
    int propeller_direction = ((i+1)%2==0)?1:-1;            // ternary operator:  (condition) ? (if_true) : (if_false)
    ignition::math::Vector3d rotor_torque(
      propeller_direction * motor_torque_constant_ * command_[i] * std::abs(command_[i]), 0, 0);
    link_->AddRelativeTorque(rotor_torque);

    //std::cout << "Applying torque " << rotor_torque[2] << " to rotor " << i << "\n";
    torques[i] = rotor_torque[0];
  }

  // for debugging
  /*if (counter_ < time_) {

        counter_ = counter_ + 1.0;

        std::cout << "UUV Command Callback:"
        << command_[0] << ","
        << command_[1] << ","
        << command_[2] << ","
        << command_[3] << ","
        << "\n";

        double thrust = 0;
        for(int i = 0; i<4; i++) thrust = thrust + forces[i];
        std::cout << "Thrust:";
        std::cout << thrust;
        std::cout << "\n";

        double L = 0.0481;
        double roll = -torques[0] + torques[1] - torques[2] + torques[3];
        double pitch = (-forces[0] - forces[1] + forces[2] + forces[3])*L;
        double yaw = (forces[0] - forces[1] - forces[2] + forces[3])*L;
        std::cout << "Torques:";
        std::cout << roll << "," << pitch << "," << yaw;
        std::cout << "\n";
  } */

  // Calculate and apply body Coriolis and Drag forces and torques
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d linear_velocity = link_->RelativeLinearVel();
#else
  ignition::math::Vector3d linear_velocity = ignitionFromGazeboMath(link_->GetRelativeLinearVel());
#endif
  double u = linear_velocity[0];
  double v = linear_velocity[1];
  double w = linear_velocity[2];

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d angular_velocity = link_->RelativeAngularVel();
#else
  ignition::math::Vector3d angular_velocity = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
#endif
  double p = angular_velocity[0];
  double q = angular_velocity[1];
  double r = angular_velocity[2];

  //std::cout << "Vels:" << linear_velocity << ":" << angular_velocity << "\n";

  // Linear Damping Matrix, with minus already multiplied
  ignition::math::Matrix3d D_FL(
    -X_u_, 0, 0,
    0, -Y_v_, 0,
    0, 0, -Z_w_
  );

  // Angular Damping Matrix, with minus already multiplied
  ignition::math::Matrix3d D_FA(
    -K_p_, 0, 0,
    0, -M_q_, 0,
    0, 0, -N_r_
  );

  ignition::math::Vector3d damping_force =
        D_FL*linear_velocity;
  ignition::math::Vector3d damping_torque =
        D_FA*angular_velocity;

  // Corriolis Forces and Torques
  // upper right and bottom left matrix, with minus already multiplied
  ignition::math::Matrix3d C_AD_FA(
    0, Z_wdot_ * w, -Y_vdot_ * v,
    -Z_wdot_ * w, 0, X_udot_ * u,
    Y_vdot_ * v, -X_udot_ * u, 0
  );

  // Torques from angular velocity, with minus already multiplied
  ignition::math::Matrix3d C_AD_TA(
    0, N_rdot_ * r,  -M_qdot_ * q,
    -N_rdot_ * r, 0, K_pdot_ * p,
    M_qdot_ * q, -K_pdot_ * p, 0
  );

  ignition::math::Vector3d coriolis_force =
    C_AD_FA*angular_velocity;
  ignition::math::Vector3d coriolis_torque =
    (C_AD_FA*linear_velocity) + (C_AD_TA*angular_velocity);

  //std::cout << C_AD_FA << "\n";
  //std::cout << "Linear:" << coriolis_force << "\n";
  //std::cout << "Angular:" << angular_velocity << "\n";

  link_->AddRelativeForce(damping_force + coriolis_force);
  link_->AddRelativeTorque(damping_torque + coriolis_torque);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboUUVPlugin)
}
