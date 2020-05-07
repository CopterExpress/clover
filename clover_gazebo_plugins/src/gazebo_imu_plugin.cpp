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

#include "gazebo_imu_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      velocity_prev_W_(0,0,0)
{
}

GazeboImuPlugin::~GazeboImuPlugin() {
  updateConnection_->~Connection();
}


void GazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, kDefaultImuTopic);
  getSdfParam<double>(_sdf, "gyroscopeNoiseDensity",
                      imu_parameters_.gyroscope_noise_density,
                      imu_parameters_.gyroscope_noise_density);
  getSdfParam<double>(_sdf, "gyroscopeRandomWalk",
                      imu_parameters_.gyroscope_random_walk,
                      imu_parameters_.gyroscope_random_walk);
  getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
                      imu_parameters_.gyroscope_bias_correlation_time,
                      imu_parameters_.gyroscope_bias_correlation_time);
  assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
                      imu_parameters_.gyroscope_turn_on_bias_sigma,
                      imu_parameters_.gyroscope_turn_on_bias_sigma);
  getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
                      imu_parameters_.accelerometer_noise_density,
                      imu_parameters_.accelerometer_noise_density);
  getSdfParam<double>(_sdf, "accelerometerRandomWalk",
                      imu_parameters_.accelerometer_random_walk,
                      imu_parameters_.accelerometer_random_walk);
  getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
                      imu_parameters_.accelerometer_bias_correlation_time,
                      imu_parameters_.accelerometer_bias_correlation_time);
  assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
                      imu_parameters_.accelerometer_turn_on_bias_sigma,
                      imu_parameters_.accelerometer_turn_on_bias_sigma);

  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Imu>("~/" + model_->GetName() + imu_topic_, 10);

  // Fill imu message.
  // imu_message_.header.frame_id = frame_id_; TODO Add header
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.
  for(int i=0; i< 9; i++){
    switch (i){
    case 0:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    case 1:
    case 2:
    case 3:
      imu_message_.add_angular_velocity_covariance(0.0);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(0.0);
      break;
    case 4:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    case 5:
    case 6:
    case 7:
      imu_message_.add_angular_velocity_covariance(0.0);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(0.0);
      break;
    case 8:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density);
      break;
    }
  }

  gravity_W_ = world_->Gravity();
  imu_parameters_.gravity_magnitude = gravity_W_.Length();

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
      gyroscope_turn_on_bias_[i] =
          sigma_bon_g * standard_normal_distribution_(random_generator_);
      accelerometer_turn_on_bias_[i] =
          sigma_bon_a * standard_normal_distribution_(random_generator_);
  }

  // TODO(nikolicj) incorporate steady-state covariance of bias process
  gyroscope_bias_.setZero();
  accelerometer_bias_.setZero();
}

/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void GazeboImuPlugin::addNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  // CHECK(linear_acceleration);
  // CHECK(angular_velocity);
  assert(dt > 0.0);

  // Gyrosocpe
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d =
      sqrt( - sigma_b_g * sigma_b_g * tau_g / 2.0 *
      (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +
        sigma_b_g_d * standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] = (*angular_velocity)[i] +
        gyroscope_bias_[i] +
        sigma_g_d * standard_normal_distribution_(random_generator_) +
        gyroscope_turn_on_bias_[i];
  }

  // Accelerometer
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d =
      sqrt( - sigma_b_a * sigma_b_a * tau_a / 2.0 *
      (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +
        sigma_b_a_d * standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] = (*linear_acceleration)[i] +
        accelerometer_bias_[i] +
        sigma_a_d * standard_normal_distribution_(random_generator_) +
        accelerometer_turn_on_bias_[i];
  }

}

// This gets called by the world update start event.
void GazeboImuPlugin::OnUpdate(const common::UpdateInfo& _info) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time  = world_->SimTime();
#else
  common::Time current_time  = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose(); //TODO(burrimi): Check tf.
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif

  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

  // Copy ignition::math::Quaterniond to gazebo::msgs::Quaternion
  gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
  orientation->set_x(C_W_I.X());
  orientation->set_y(C_W_I.Y());
  orientation->set_z(C_W_I.Z());
  orientation->set_w(C_W_I.W());

#if GAZEBO_MAJOR_VERSION < 5
  ignition::math::Vector3d velocity_current_W = link_->GetWorldLinearVel();
  // link_->RelativeLinearAccel() does not work sometimes with old gazebo versions.
  // TODO For an accurate simulation, this might have to be fixed. Consider the
  // This issue is solved in gazebo 5.
  ignition::math::Vector3d acceleration = (velocity_current_W - velocity_prev_W_) / dt;
  ignition::math::Vector3d acceleration_I =
      C_W_I.RotateVectorReverse(acceleration - gravity_W_);

  velocity_prev_W_ = velocity_current_W;
#elif GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d acceleration_I = link_->RelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);
#else
  ignition::math::Vector3d acceleration_I = ignitionFromGazeboMath(link_->GetRelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_));
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d angular_vel_I = link_->RelativeAngularVel();
#else
  ignition::math::Vector3d angular_vel_I = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
#endif

  Eigen::Vector3d linear_acceleration_I(acceleration_I.X(),
                                        acceleration_I.Y(),
                                        acceleration_I.Z());
  Eigen::Vector3d angular_velocity_I(angular_vel_I.X(),
                                     angular_vel_I.Y(),
                                     angular_vel_I.Z());

  addNoise(&linear_acceleration_I, &angular_velocity_I, dt);

  // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
  gazebo::msgs::Vector3d* linear_acceleration = new gazebo::msgs::Vector3d();
  linear_acceleration->set_x(linear_acceleration_I[0]);
  linear_acceleration->set_y(linear_acceleration_I[1]);
  linear_acceleration->set_z(linear_acceleration_I[2]);

  // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
  gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
  angular_velocity->set_x(angular_velocity_I[0]);
  angular_velocity->set_y(angular_velocity_I[1]);
  angular_velocity->set_z(angular_velocity_I[2]);

  // Fill IMU message.
  // ADD HEaders
  // imu_message_.header.stamp.sec = current_time.sec;
  // imu_message_.header.stamp.nsec = current_time.nsec;
  imu_message_.set_time_usec(_info.simTime.sec * 1000000 + _info.simTime.nsec / 1000);
  imu_message_.set_seq(seq_++);

  // TODO(burrimi): Add orientation estimator.
  // imu_message_.orientation.w = 1;
  // imu_message_.orientation.x = 0;
  // imu_message_.orientation.y = 0;
  // imu_message_.orientation.z = 0;

  imu_message_.set_allocated_orientation(orientation);
  imu_message_.set_allocated_linear_acceleration(linear_acceleration);
  imu_message_.set_allocated_angular_velocity(angular_velocity);

  imu_pub_->Publish(imu_message_);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}
