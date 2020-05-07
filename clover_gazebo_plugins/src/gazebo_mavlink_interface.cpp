/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Development Team
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

#include <gazebo_mavlink_interface.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  close();
  sigIntConnection_->~Connection();
  updateConnection_->~Connection();
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->GetElement("protocol_version")->Get<float>();
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
      motor_velocity_reference_pub_topic_);
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
  getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
  getSdfParam<std::string>(_sdf, "lidarSubTopic", lidar_sub_topic_, lidar_sub_topic_);
  getSdfParam<std::string>(_sdf, "opticalFlowSubTopic",
      opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  getSdfParam<std::string>(_sdf, "sonarSubTopic", sonar_sub_topic_, sonar_sub_topic_);
  getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);
  getSdfParam<std::string>(_sdf, "magSubTopic", mag_sub_topic_, mag_sub_topic_);
  getSdfParam<std::string>(_sdf, "baroSubTopic", baro_sub_topic_, baro_sub_topic_);
  groundtruth_sub_topic_ = "/groundtruth";

  // set input_reference_ from inputs.control
  input_reference_.resize(n_out_max);
  joints_.resize(n_out_max);
  pids_.resize(n_out_max);
  for (int i = 0; i < n_out_max; ++i)
  {
    pids_[i].Init(0, 0, 0, 0, 0, 0, 0);
    input_reference_[i] = 0;
  }

  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");
    while (channel)
    {
      if (channel->HasElement("input_index"))
      {
        int index = channel->Get<int>("input_index");
        if (index < n_out_max)
        {
          input_offset_[index] = channel->Get<double>("input_offset");
          input_scaling_[index] = channel->Get<double>("input_scaling");
          zero_position_disarmed_[index] = channel->Get<double>("zero_position_disarmed");
          zero_position_armed_[index] = channel->Get<double>("zero_position_armed");
          if (channel->HasElement("joint_control_type"))
          {
            joint_control_type_[index] = channel->Get<std::string>("joint_control_type");
          }
          else
          {
            gzwarn << "joint_control_type[" << index << "] not specified, using velocity.\n";
            joint_control_type_[index] = "velocity";
          }

          // start gz transport node handle
          if (joint_control_type_[index] == "position_gztopic")
          {
            // setup publisher handle to topic
            if (channel->HasElement("gztopic"))
              gztopic_[index] = "~/" + model_->GetName() + channel->Get<std::string>("gztopic");
            else
              gztopic_[index] = "control_position_gztopic_" + std::to_string(index);
#if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
            /// only gazebo 7.4 and above support Any
            joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::Any>(
                gztopic_[index]);
#else
            joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::GzString>(
                gztopic_[index]);
#endif
          }

          if (channel->HasElement("joint_name"))
          {
            std::string joint_name = channel->Get<std::string>("joint_name");
            joints_[index] = model_->GetJoint(joint_name);
            if (joints_[index] == nullptr)
            {
              gzwarn << "joint [" << joint_name << "] not found for channel["
                     << index << "] no joint control for this channel.\n";
            }
            else
            {
              gzdbg << "joint [" << joint_name << "] found for channel["
                    << index << "] joint control active for this channel.\n";
            }
          }
          else
          {
            gzdbg << "<joint_name> not found for channel[" << index
                  << "] no joint control will be performed for this channel.\n";
          }

          // setup joint control pid to control joint
          if (channel->HasElement("joint_control_pid"))
          {
            sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
            double p = 0;
            if (pid->HasElement("p"))
              p = pid->Get<double>("p");
            double i = 0;
            if (pid->HasElement("i"))
              i = pid->Get<double>("i");
            double d = 0;
            if (pid->HasElement("d"))
              d = pid->Get<double>("d");
            double iMax = 0;
            if (pid->HasElement("iMax"))
              iMax = pid->Get<double>("iMax");
            double iMin = 0;
            if (pid->HasElement("iMin"))
              iMin = pid->Get<double>("iMin");
            double cmdMax = 0;
            if (pid->HasElement("cmdMax"))
              cmdMax = pid->Get<double>("cmdMax");
            double cmdMin = 0;
            if (pid->HasElement("cmdMin"))
              cmdMin = pid->Get<double>("cmdMin");
            pids_[index].Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
          }
        }
        else
        {
          gzerr << "input_index[" << index << "] out of range, not parsing.\n";
        }
      }
      else
      {
        gzerr << "no input_index, not parsing.\n";
        break;
      }
      channel = channel->GetNextElement("channel");
    }
  }

  if(_sdf->HasElement("hil_mode"))
  {
    hil_mode_ = _sdf->GetElement("hil_mode")->Get<bool>();
  }

  if(_sdf->HasElement("hil_state_level"))
  {
    hil_state_level_ = _sdf->GetElement("hil_state_level")->Get<bool>();
  }

  if(_sdf->HasElement("serialEnabled"))
  {
    serial_enabled_ = _sdf->GetElement("serialEnabled")->Get<bool>();
  }

  if (!serial_enabled_ && _sdf->HasElement("use_tcp"))
  {
    use_tcp_ = _sdf->GetElement("use_tcp")->Get<bool>();
  }
  gzmsg << "Connecting to PX4 SITL using " << (serial_enabled_ ? "serial" : (use_tcp_ ? "TCP" : "UDP")) << "\n";

  if (!hil_mode_ && _sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->GetElement("enable_lockstep")->Get<bool>();
  }
  gzmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled") << "\n";

  // When running in lockstep, we can run the simulation slower or faster than
  // realtime. The speed can be set using the env variable PX4_SIM_SPEED_FACTOR.
  if (enable_lockstep_)
  {
    const char *speed_factor_str = std::getenv("PX4_SIM_SPEED_FACTOR");
    if (speed_factor_str)
    {
      speed_factor_ = std::atof(speed_factor_str);
      if (!std::isfinite(speed_factor_) || speed_factor_ <= 0.0)
      {
        gzerr << "Invalid speed factor '" << speed_factor_str << "', aborting\n";
        abort();
      }
    }
    gzmsg << "Speed factor set to: " << speed_factor_ << "\n";

    boost::any param;
#if GAZEBO_MAJOR_VERSION >= 8
    physics::PresetManagerPtr presetManager = world_->PresetMgr();
#else
    physics::PresetManagerPtr presetManager = world_->GetPresetManager();
#endif
    presetManager->CurrentProfile("default_physics");

    // We currently need to have the max_step_size pinned at 4 ms and the
    // real_time_update_rate set to 250 Hz for lockstep.
    // Therefore it makes sense to check these params.

    presetManager->GetCurrentProfileParam("real_time_update_rate", param);
    double real_time_update_rate = boost::any_cast<double>(param);
    const double correct_real_time_update_rate = 250.0;
    if (real_time_update_rate != correct_real_time_update_rate)
    {
      gzerr << "real_time_update_rate is set to " << real_time_update_rate
            << " instead of " << correct_real_time_update_rate << ", aborting.\n";
      abort();
    }

    presetManager->GetCurrentProfileParam("max_step_size", param);
    const double max_step_size = boost::any_cast<double>(param);
    const double correct_max_step_size = 0.004;
    if (max_step_size != correct_max_step_size)
    {
      gzerr << "max_step_size is set to " << max_step_size
            << " instead of " << correct_max_step_size << ", aborting.\n";
      abort();
    }

    // Adapt the real_time_update_rate according to the speed
    // that we ask for in the env variable.
    real_time_update_rate *= speed_factor_;
    presetManager->SetCurrentProfileParam("real_time_update_rate", real_time_update_rate);
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Listen to Ctrl+C / SIGINT.
  sigIntConnection_ = event::Events::ConnectSigInt(
      boost::bind(&GazeboMavlinkInterface::onSigInt, this));

  // Subscribe to messages of other plugins.
  imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + imu_sub_topic_, &GazeboMavlinkInterface::ImuCallback, this);
  lidar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/link/" + lidar_sub_topic_, &GazeboMavlinkInterface::LidarCallback, this);
  opticalFlow_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
  sonar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/link/" + sonar_sub_topic_, &GazeboMavlinkInterface::SonarCallback, this);
  irlock_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + irlock_sub_topic_, &GazeboMavlinkInterface::IRLockCallback, this);
  gps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboMavlinkInterface::GpsCallback, this);
  groundtruth_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + groundtruth_sub_topic_, &GazeboMavlinkInterface::GroundtruthCallback, this);
  vision_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + vision_sub_topic_, &GazeboMavlinkInterface::VisionCallback, this);
  mag_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + mag_sub_topic_, &GazeboMavlinkInterface::MagnetometerCallback, this);
  baro_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + baro_sub_topic_, &GazeboMavlinkInterface::BarometerCallback, this);

  // Publish gazebo's motor_speed message
  motor_velocity_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_imu_time_ = world_->SimTime();
  gravity_W_ = world_->Gravity();
#else
  last_time_ = world_->GetSimTime();
  last_imu_time_ = world_->GetSimTime();
  gravity_W_ = ignitionFromGazeboMath(world_->GetPhysicsEngine()->GetGravity());
#endif

  // This doesn't seem to be used anywhere but we leave it here
  // for potential compatibility
  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1 / _sdf->GetElement("imu_rate")->Get<int>();
  }

  mavlink_addr_ = htonl(INADDR_ANY);
  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->GetElement("mavlink_addr")->Get<std::string>();
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_addr_ = inet_addr(mavlink_addr_str.c_str());
      if (mavlink_addr_ == INADDR_NONE) {
        gzerr << "Invalid mavlink_addr: " << mavlink_addr_str << ", aborting\n";
        abort();
      }
    }
  }

#if GAZEBO_MAJOR_VERSION >= 9
  auto worldName = world_->Name();
#else
  auto worldName = world_->GetName();
#endif

  if (_sdf->HasElement("mavlink_udp_port")) {
    mavlink_udp_port_ = _sdf->GetElement("mavlink_udp_port")->Get<int>();
  }
  model_param(worldName, model_->GetName(), "mavlink_udp_port", mavlink_udp_port_);

  if (_sdf->HasElement("mavlink_tcp_port")) {
    mavlink_tcp_port_ = _sdf->GetElement("mavlink_tcp_port")->Get<int>();
  }
  model_param(worldName, model_->GetName(), "mavlink_tcp_port", mavlink_tcp_port_);

  local_qgc_addr_.sin_port = htonl(INADDR_ANY);
  if (_sdf->HasElement("qgc_addr")) {
    std::string qgc_addr = _sdf->GetElement("qgc_addr")->Get<std::string>();
    if (qgc_addr != "INADDR_ANY") {
      local_qgc_addr_.sin_port = inet_addr(qgc_addr.c_str());
      if (local_qgc_addr_.sin_port == INADDR_NONE) {
        gzerr << "Invalid qgc_addr: " << qgc_addr << ", aborting\n";
        abort();
      }
    }
  }
  if (_sdf->HasElement("qgc_udp_port")) {
    qgc_udp_port_ = _sdf->GetElement("qgc_udp_port")->Get<int>();
  }

  local_sdk_addr_.sin_port = htonl(INADDR_ANY);
  if (_sdf->HasElement("sdk_addr")) {
    std::string sdk_addr = _sdf->GetElement("sdk_addr")->Get<std::string>();
    if (sdk_addr != "INADDR_ANY") {
      local_sdk_addr_.sin_port = inet_addr(sdk_addr.c_str());
      if (local_sdk_addr_.sin_port == INADDR_NONE) {
        gzerr << "Invalid sdk_addr: " << sdk_addr << ", aborting\n";
        abort();
      }
    }
  }
  if (_sdf->HasElement("sdk_udp_port")) {
    sdk_udp_port_ = _sdf->GetElement("sdk_udp_port")->Get<int>();
  }

  if (hil_mode_) {

    local_qgc_addr_.sin_family = AF_INET;
    local_qgc_addr_.sin_port = htons(0);
    local_qgc_addr_len_ = sizeof(local_qgc_addr_);

    remote_qgc_addr_.sin_family = AF_INET;
    remote_qgc_addr_.sin_port = htons(qgc_udp_port_);
    remote_qgc_addr_len_ = sizeof(remote_qgc_addr_);

    local_sdk_addr_.sin_family = AF_INET;
    local_sdk_addr_.sin_port = htons(0);
    local_sdk_addr_len_ = sizeof(local_sdk_addr_);

    remote_sdk_addr_.sin_family = AF_INET;
    remote_sdk_addr_.sin_port = htons(sdk_udp_port_);
    remote_sdk_addr_len_ = sizeof(remote_sdk_addr_);

    if ((qgc_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      gzerr << "Creating QGC UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(qgc_socket_fd_, (struct sockaddr *)&local_qgc_addr_, local_qgc_addr_len_) < 0) {
      gzerr << "QGC UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if ((sdk_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      gzerr << "Creating SDK UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(sdk_socket_fd_, (struct sockaddr *)&local_sdk_addr_, local_sdk_addr_len_) < 0) {
      gzerr << "SDK UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

  }

  if (serial_enabled_) {
    // Set up serial interface
    if(_sdf->HasElement("serialDevice"))
    {
      device_ = _sdf->GetElement("serialDevice")->Get<std::string>();
    }

    if (_sdf->HasElement("baudRate")) {
      baudrate_ = _sdf->GetElement("baudRate")->Get<int>();
    }
    io_service.post(std::bind(&GazeboMavlinkInterface::do_read, this));

    // run io_service for async io
    io_thread = std::thread([this] () {
      io_service.run();
    });
    open();

  } else {
    memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
    remote_simulator_addr_.sin_family = AF_INET;
    remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

    memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
    local_simulator_addr_.sin_family = AF_INET;
    local_simulator_addr_len_ = sizeof(local_simulator_addr_);

    if (use_tcp_) {

      local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
      local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        gzerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      int yes = 1;
      int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      if (result != 0) {
        gzerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      struct linger nolinger {};
      nolinger.l_onoff = 1;
      nolinger.l_linger = 0;

      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
      if (result != 0) {
        gzerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        gzerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      errno = 0;
      if (listen(simulator_socket_fd_, 0) < 0) {
        gzerr << "listen failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      simulator_tcp_client_fd_ = accept(simulator_socket_fd_, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

    } else {
      remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
      remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);

      local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
      local_simulator_addr_.sin_port = htons(0);

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        gzerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        gzerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }
    }
  }

  if(_sdf->HasElement("vehicle_is_tailsitter"))
  {
    vehicle_is_tailsitter_ = _sdf->GetElement("vehicle_is_tailsitter")->Get<bool>();
  }

  if(_sdf->HasElement("send_vision_estimation"))
  {
    send_vision_estimation_ = _sdf->GetElement("send_vision_estimation")->Get<bool>();
  }

  if(_sdf->HasElement("send_odometry"))
  {
    send_odometry_ = _sdf->GetElement("send_odometry")->Get<bool>();
  }

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

  // set the Mavlink protocol version to use on the link
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    gzmsg << "Using MAVLink protocol v2.0\n";
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    gzmsg << "Using MAVLink protocol v1.0\n";
  }
  else {
    gzerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
  }

  standard_normal_distribution_ = std::normal_distribution<float>(0.0f, 1.0f);

  // Get the lidar link orientation with respect to the base_link
  const auto lidar_link = model_->GetLink(lidar_sub_topic_ + "::link");
  if (lidar_link != NULL) {
#if GAZEBO_MAJOR_VERSION >= 9
    lidar_orientation_ = lidar_link->RelativePose().Rot();
#else
    lidar_orientation_ = ignitionFromGazeboMath(lidar_link->GetRelativePose()).Rot();
#endif
  }

  // Get the sonar link orientation with respect to the base_link
  const auto sonar_link = model_->GetLink(sonar_sub_topic_ + "::link");
  if (sonar_link != NULL) {
#if GAZEBO_MAJOR_VERSION >= 9
    sonar_orientation_ = sonar_link->RelativePose().Rot();
#else
    sonar_orientation_ = ignitionFromGazeboMath(sonar_link->GetRelativePose()).Rot();
#endif
  }
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo&  /*_info*/) {

  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  if (previous_imu_seq_ > 0) {
    while (previous_imu_seq_ == last_imu_message_.seq() && IsRunning()) {
      last_imu_message_cond_.wait_for(lock, std::chrono::milliseconds(10));
    }
  }

  previous_imu_seq_ = last_imu_message_.seq();

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

  SendSensorMessages();

  if (hil_mode_) {
    pollFromQgcAndSdk();
  } else {
    pollForMAVLinkMessages();
  }

  handle_control(dt);

  if (received_first_actuator_) {
    mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;

    for (int i = 0; i < input_reference_.size(); i++) {
      if (last_actuator_time_ == 0 || (current_time - last_actuator_time_).Double() > 0.2) {
        turning_velocities_msg.add_motor_speed(0);
      } else {
        turning_velocities_msg.add_motor_speed(input_reference_[i]);
      }
    }
    // TODO Add timestamp and Header
    // turning_velocities_msg->header.stamp.sec = current_time.sec;
    // turning_velocities_msg->header.stamp.nsec = current_time.nsec;

    motor_velocity_reference_pub_->Publish(turning_velocities_msg);
  }

  last_time_ = current_time;
}

void GazeboMavlinkInterface::send_mavlink_message(const mavlink_message_t *message)
{
  assert(message != nullptr);

  if (gotSigInt_) {
    return;
  }

  if (serial_enabled_) {

    if (!is_open()) {
      gzerr << "Serial port closed! \n";
      return;
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex);

      if (tx_q.size() >= MAX_TXQ_SIZE) {
        gzwarn << "Tx queue overflow\n";
      }
      tx_q.emplace_back(message);
    }
    io_service.post(std::bind(&GazeboMavlinkInterface::do_write, this, true));

  } else {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    ssize_t len;
    if (use_tcp_) {
      len = send(simulator_tcp_client_fd_, buffer, packetlen, 0);
    } else {
      len = sendto(simulator_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);
    }

    if (len <= 0)
    {
      gzerr << "Failed sending mavlink message: " << strerror(errno) << "\n";
    }
  }
}

template <class T>
void GazeboMavlinkInterface::setMavlinkSensorOrientation(const ignition::math::Vector3d& u_Xs, T& sensor_msg) {
  const ignition::math::Vector3d u_Xb = kForwardRotation; // This is unit vector of X-axis `base_link`

  // Current rotation types are described as https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION
  if(u_Xs.Dot(kDownwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270;
  else if(u_Xs.Dot(kUpwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_90;
  else if(u_Xs.Dot(kBackwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_180;
  else if(u_Xs.Dot(kForwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_NONE;
  else if(u_Xs.Dot(kLeftRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_270;
  else if(u_Xs.Dot(kRightRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_90;
  else {
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_CUSTOM;
  }

}

void GazeboMavlinkInterface::forward_mavlink_message(const mavlink_message_t *message)
{
  if (gotSigInt_) {
    return;
  }

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  ssize_t len;
  if (qgc_socket_fd_ > 0) {
    len = sendto(qgc_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_qgc_addr_, remote_qgc_addr_len_);

    if (len <= 0)
    {
      gzerr << "Failed sending mavlink message to QGC: " << strerror(errno) << "\n";
    }
  }

  if (sdk_socket_fd_ > 0) {
    len = sendto(sdk_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_sdk_addr_, remote_sdk_addr_len_);
    if (len <= 0)
    {
      gzerr << "Failed sending mavlink message to SDK: " << strerror(errno) << "\n";
    }
  }
}

void GazeboMavlinkInterface::ImuCallback(ImuPtr& imu_message)
{
  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  const int64_t diff = imu_message->seq() - last_imu_message_.seq();
  if (diff != 1 && imu_message->seq() != 0)
  {
    gzerr << "Skipped " << (diff - 1) << " IMU samples (presumably CPU usage is too high)\n";
  }

  last_imu_message_ = *imu_message;
  lock.unlock();
  last_imu_message_cond_.notify_one();
}

void GazeboMavlinkInterface::SendSensorMessages()
{
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

  ignition::math::Quaterniond q_gb = q_gr*q_br.Inverse();
  ignition::math::Quaterniond q_nb = q_ng*q_gb;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d pos_g = model_->WorldPose().Pos();
#else
  ignition::math::Vector3d pos_g = ignitionFromGazeboMath(model_->GetWorldPose().pos);
#endif
  ignition::math::Vector3d pos_n = q_ng.RotateVector(pos_g);

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel_b = q_br.RotateVector(model_->RelativeLinearVel());
  ignition::math::Vector3d vel_n = q_ng.RotateVector(model_->WorldLinearVel());
  ignition::math::Vector3d omega_nb_b = q_br.RotateVector(model_->RelativeAngularVel());
#else
  ignition::math::Vector3d vel_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearVel()));
  ignition::math::Vector3d vel_n = q_ng.RotateVector(ignitionFromGazeboMath(model_->GetWorldLinearVel()));
  ignition::math::Vector3d omega_nb_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeAngularVel()));
#endif

  ignition::math::Vector3d accel_b = q_br.RotateVector(ignition::math::Vector3d(
    last_imu_message_.linear_acceleration().x(),
    last_imu_message_.linear_acceleration().y(),
    last_imu_message_.linear_acceleration().z()));
  ignition::math::Vector3d gyro_b = q_br.RotateVector(ignition::math::Vector3d(
    last_imu_message_.angular_velocity().x(),
    last_imu_message_.angular_velocity().y(),
    last_imu_message_.angular_velocity().z()));
  ignition::math::Vector3d mag_b = q_nb.RotateVectorReverse(mag_n_);

  bool should_send_imu = false;
  if (!enable_lockstep_) {
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time current_time = world_->SimTime();
#else
    common::Time current_time = world_->GetSimTime();
#endif
    double dt = (current_time - last_imu_time_).Double();

    if (imu_update_interval_!=0 && dt >= imu_update_interval_) {
      should_send_imu = true;
      last_imu_time_ = current_time;
    }
  }

  if (enable_lockstep_ || should_send_imu) {
    mavlink_hil_sensor_t sensor_msg;
#if GAZEBO_MAJOR_VERSION >= 9
    sensor_msg.time_usec = world_->SimTime().Double() * 1e6;
#else
    sensor_msg.time_usec = world_->GetSimTime().Double() * 1e6;
#endif
    sensor_msg.xacc = accel_b.X();
    sensor_msg.yacc = accel_b.Y();
    sensor_msg.zacc = accel_b.Z();
    sensor_msg.xgyro = gyro_b.X();
    sensor_msg.ygyro = gyro_b.Y();
    sensor_msg.zgyro = gyro_b.Z();
    sensor_msg.xmag = mag_b.X();
    sensor_msg.ymag = mag_b.Y();
    sensor_msg.zmag = mag_b.Z();

    sensor_msg.temperature = temperature_;
    sensor_msg.abs_pressure = abs_pressure_;
    sensor_msg.pressure_alt = pressure_alt_;

    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    float temperature_local = sensor_msg.temperature + 273.0f;
    const float density_ratio = powf((temperature_msl/temperature_local) , 4.256f);
    float rho = 1.225f / density_ratio;

    // Let's use a rough guess of 0.05 hPa as the standard devitiation which roughly yields
    // about +/- 1 m/s noise.
    const float diff_pressure_stddev = 0.05f;
    const float diff_pressure_noise = standard_normal_distribution_(random_generator_) * diff_pressure_stddev;

    // calculate differential pressure in hPa
    // if vehicle is a tailsitter the airspeed axis is different (z points from nose to tail)
    if (vehicle_is_tailsitter_) {
      sensor_msg.diff_pressure = 0.005f*rho*vel_b.Z()*vel_b.Z() + diff_pressure_noise;
    } else {
      sensor_msg.diff_pressure = 0.005f*rho*vel_b.X()*vel_b.X() + diff_pressure_noise;
    }

    sensor_msg.fields_updated = 4095;

    if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
      mavlink_message_t msg;
      mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
      send_mavlink_message(&msg);
    }
  }

  // ground truth
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d accel_true_b = q_br.RotateVector(model_->RelativeLinearAccel());
#else
  ignition::math::Vector3d accel_true_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearAccel()));
#endif

  // send ground truth

  mavlink_hil_state_quaternion_t hil_state_quat;
#if GAZEBO_MAJOR_VERSION >= 9
  hil_state_quat.time_usec = world_->SimTime().Double() * 1e6;
#else
  hil_state_quat.time_usec = world_->GetSimTime().Double() * 1e6;
#endif
  hil_state_quat.attitude_quaternion[0] = q_nb.W();
  hil_state_quat.attitude_quaternion[1] = q_nb.X();
  hil_state_quat.attitude_quaternion[2] = q_nb.Y();
  hil_state_quat.attitude_quaternion[3] = q_nb.Z();

  hil_state_quat.rollspeed = omega_nb_b.X();
  hil_state_quat.pitchspeed = omega_nb_b.Y();
  hil_state_quat.yawspeed = omega_nb_b.Z();

  hil_state_quat.lat = groundtruth_lat_rad * 180 / M_PI * 1e7;
  hil_state_quat.lon = groundtruth_lon_rad * 180 / M_PI * 1e7;
  hil_state_quat.alt = groundtruth_altitude * 1000;

  hil_state_quat.vx = vel_n.X() * 100;
  hil_state_quat.vy = vel_n.Y() * 100;
  hil_state_quat.vz = vel_n.Z() * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = vel_b.X();

#if GAZEBO_MAJOR_VERSION >= 9
  hil_state_quat.true_airspeed = model_->WorldLinearVel().Length() * 100;  //no wind simulated
#else
  hil_state_quat.true_airspeed = model_->GetWorldLinearVel().GetLength() * 100;  //no wind simulated
#endif

  hil_state_quat.xacc = accel_true_b.X() * 1000;
  hil_state_quat.yacc = accel_true_b.Y() * 1000;
  hil_state_quat.zacc = accel_true_b.Z() * 1000;

  if (!hil_mode_ || (hil_mode_ && hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::GpsCallback(GpsPtr& gps_msg) {
  // fill HIL GPS Mavlink msg
  mavlink_hil_gps_t hil_gps_msg;
  hil_gps_msg.time_usec = gps_msg->time_usec();
  hil_gps_msg.fix_type = 3;
  hil_gps_msg.lat = gps_msg->latitude_deg() * 1e7;
  hil_gps_msg.lon = gps_msg->longitude_deg() * 1e7;
  hil_gps_msg.alt = gps_msg->altitude() * 1000.0;
  hil_gps_msg.eph = gps_msg->eph() * 100.0;
  hil_gps_msg.epv = gps_msg->epv() * 100.0;
  hil_gps_msg.vel = gps_msg->velocity() * 100.0;
  hil_gps_msg.vn = gps_msg->velocity_north() * 100.0;
  hil_gps_msg.ve = gps_msg->velocity_east() * 100.0;
  hil_gps_msg.vd = -gps_msg->velocity_up() * 100.0;
  // MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
  ignition::math::Angle cog(atan2(gps_msg->velocity_east(), gps_msg->velocity_north()));
  cog.Normalize();
  hil_gps_msg.cog = static_cast<uint16_t>(GetDegrees360(cog) * 100.0);
  hil_gps_msg.satellites_visible = 10;

  // send HIL_GPS Mavlink msg
  if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::GroundtruthCallback(GtPtr& groundtruth_msg) {
  // update groundtruth lat_rad, lon_rad and altitude
  groundtruth_lat_rad = groundtruth_msg->latitude_rad();
  groundtruth_lon_rad = groundtruth_msg->longitude_rad();
  groundtruth_altitude = groundtruth_msg->altitude();
  // the rest of the data is obtained directly on this interface and sent to
  // the FCU
}

void GazeboMavlinkInterface::LidarCallback(LidarPtr& lidar_message) {
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = lidar_message->time_usec() / 1e3;
  sensor_msg.min_distance = lidar_message->min_distance() * 100.0;
  sensor_msg.max_distance = lidar_message->max_distance() * 100.0;
  sensor_msg.current_distance = lidar_message->current_distance() * 100.0;
  sensor_msg.type = 0;
  sensor_msg.id = 0;
  sensor_msg.covariance = 0;
  sensor_msg.horizontal_fov = lidar_message->h_fov();
  sensor_msg.vertical_fov = lidar_message->v_fov();

  ignition::math::Quaterniond q_ls = ignition::math::Quaterniond(
    lidar_message->orientation().w(),
    lidar_message->orientation().x(),
    lidar_message->orientation().y(),
    lidar_message->orientation().z());

  ignition::math::Quaterniond q_bs = (lidar_orientation_ * q_ls).Inverse();

  sensor_msg.quaternion[0] = q_bs.W();
  sensor_msg.quaternion[1] = q_bs.X();
  sensor_msg.quaternion[2] = q_bs.Y();
  sensor_msg.quaternion[3] = q_bs.Z();

  const ignition::math::Vector3d u_Xb = kForwardRotation; // This is unit vector of X-axis `base_link`
  const ignition::math::Vector3d u_Xs = q_bs.RotateVectorReverse(u_Xb); // This is unit vector of X-axis sensor in `base_link` frame

  setMavlinkSensorOrientation(u_Xs, sensor_msg);

  //distance needed for optical flow message
  optflow_distance = lidar_message->current_distance();  //[m]

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::OpticalFlowCallback(OpticalFlowPtr& opticalFlow_message) {
  mavlink_hil_optical_flow_t sensor_msg;
  sensor_msg.time_usec = opticalFlow_message->time_usec();
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();

  bool no_gyro = (ignition::math::isnan(opticalFlow_message->integrated_xgyro())) ||
                 (ignition::math::isnan(opticalFlow_message->integrated_ygyro())) ||
                 (ignition::math::isnan(opticalFlow_message->integrated_zgyro()));
  if (no_gyro) {
    sensor_msg.integrated_xgyro = NAN;
    sensor_msg.integrated_ygyro = NAN;
    sensor_msg.integrated_zgyro = NAN;
  } else {
    sensor_msg.integrated_xgyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_xgyro() : 0.0f;
    sensor_msg.integrated_ygyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_ygyro() : 0.0f;
    sensor_msg.integrated_zgyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_zgyro() : 0.0f;
  }
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance;

  mavlink_message_t msg;
  mavlink_msg_hil_optical_flow_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::SonarCallback(SonarPtr& sonar_message) {
  mavlink_distance_sensor_t sensor_msg = {};
  sensor_msg.time_boot_ms = sonar_message->time_usec() / 1e3;
  sensor_msg.min_distance = sonar_message->min_distance() * 100.0;
  sensor_msg.max_distance = sonar_message->max_distance() * 100.0;
  sensor_msg.current_distance = sonar_message->current_distance() * 100.0;

  ignition::math::Quaterniond q_ls = sonar_orientation_.Inverse();

  const ignition::math::Vector3d u_Xb = kForwardRotation; // This is unit vector of X-axis `base_link`
  const ignition::math::Vector3d u_Xs = q_ls.RotateVectorReverse(u_Xb); // This is unit vector of X-axis sensor in `base_link` frame

  setMavlinkSensorOrientation(u_Xs, sensor_msg);

  sensor_msg.type = 1;
  sensor_msg.id = 1;
  sensor_msg.covariance = 0;
  sensor_msg.horizontal_fov = sonar_message->h_fov();
  sensor_msg.vertical_fov = sonar_message->v_fov();
  sensor_msg.quaternion[0] = q_ls.W();
  sensor_msg.quaternion[1] = q_ls.X();
  sensor_msg.quaternion[2] = q_ls.Y();
  sensor_msg.quaternion[3] = q_ls.Z();

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::IRLockCallback(IRLockPtr& irlock_message) {
  mavlink_landing_target_t sensor_msg;
  sensor_msg.time_usec = irlock_message->time_usec() / 1e3;
  sensor_msg.target_num = irlock_message->signature();
  sensor_msg.angle_x = irlock_message->pos_x();
  sensor_msg.angle_y = irlock_message->pos_y();
  sensor_msg.size_x = irlock_message->size_x();
  sensor_msg.size_y = irlock_message->size_y();
  sensor_msg.position_valid = false;
  sensor_msg.type = LANDING_TARGET_TYPE_LIGHT_BEACON;

  mavlink_message_t msg;
  mavlink_msg_landing_target_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::VisionCallback(OdomPtr& odom_message) {
  mavlink_message_t msg;

  // transform position from local ENU to local NED frame
  ignition::math::Vector3d position = q_ng.RotateVector(ignition::math::Vector3d(
    odom_message->position().x(),
    odom_message->position().y(),
    odom_message->position().z()));

  // q_gr is the quaternion that represents the orientation of the vehicle
  // the ENU earth/local
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    odom_message->orientation().w(),
    odom_message->orientation().x(),
    odom_message->orientation().y(),
    odom_message->orientation().z());

  // transform the vehicle orientation from the ENU to the NED frame
  // q_nb is the quaternion that represents the orientation of the vehicle
  // the NED earth/local
  ignition::math::Quaterniond q_nb = q_ng * q_gr * q_ng.Inverse();

  // transform linear velocity from local ENU to body FRD frame
  ignition::math::Vector3d linear_velocity = q_ng.RotateVector(
    q_br.RotateVector(ignition::math::Vector3d(
      odom_message->linear_velocity().x(),
      odom_message->linear_velocity().y(),
      odom_message->linear_velocity().z())));

  // transform angular velocity from body FLU to body FRD frame
  ignition::math::Vector3d angular_velocity = q_br.RotateVector(ignition::math::Vector3d(
    odom_message->angular_velocity().x(),
    odom_message->angular_velocity().y(),
    odom_message->angular_velocity().z()));

  // Only sends ODOMETRY msgs if send_odometry is set and the protocol version is 2.0
  if (send_odometry_ && protocol_version_ == 2.0) {
    // send ODOMETRY Mavlink msg
    mavlink_odometry_t odom;

    odom.time_usec = odom_message->time_usec();

    odom.frame_id = MAV_FRAME_VISION_NED;
    odom.child_frame_id = MAV_FRAME_BODY_FRD;

    odom.x = position.X();
    odom.y = position.Y();
    odom.z = position.Z();

    odom.q[0] = q_nb.W();
    odom.q[1] = q_nb.X();
    odom.q[2] = q_nb.Y();
    odom.q[3] = q_nb.Z();

    odom.vx = linear_velocity.X();
    odom.vy = linear_velocity.Y();
    odom.vz = linear_velocity.Z();

    odom.rollspeed= angular_velocity.X();
    odom.pitchspeed = angular_velocity.Y();
    odom.yawspeed = angular_velocity.Z();

    // parse covariance matrices
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrices from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;

        odom.pose_covariance[count++] = odom_message->pose_covariance().data()[index];
        odom.velocity_covariance[count++] = odom_message->velocity_covariance().data()[index];
      }
    }

    mavlink_msg_odometry_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &odom);
    send_mavlink_message(&msg);
  }
  else if (send_vision_estimation_) {
    // send VISION_POSITION_ESTIMATE Mavlink msg
    mavlink_vision_position_estimate_t vision;

    vision.usec = odom_message->time_usec();

    // transform position from local ENU to local NED frame
    vision.x = position.X();
    vision.y = position.Y();
    vision.z = position.Z();

    // q_nb is the quaternion that represents a rotation from NED earth/local
    // frame to XYZ body FRD frame
    ignition::math::Vector3d euler = q_nb.Euler();

    vision.roll = euler.X();
    vision.pitch = euler.Y();
    vision.yaw = euler.Z();

    // parse covariance matrix
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrix from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;

        vision.covariance[count++] = odom_message->pose_covariance().data()[index];
      }
    }

    mavlink_msg_vision_position_estimate_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &vision);
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::MagnetometerCallback(MagnetometerPtr& mag_msg) {
  // update groundtruth magnetometer NED components
  mag_n_ = ignition::math::Vector3d(
    mag_msg->magnetic_field().x(),
    mag_msg->magnetic_field().y(),
    mag_msg->magnetic_field().z());
}

void GazeboMavlinkInterface::BarometerCallback(BarometerPtr& baro_msg) {
  temperature_ = baro_msg->temperature();
  pressure_alt_ = baro_msg->pressure_altitude();
  abs_pressure_ = baro_msg->absolute_pressure();
}

void GazeboMavlinkInterface::pollForMAVLinkMessages()
{
  if (gotSigInt_) {
    return;
  }

  struct pollfd fds[1] = {};

  if (use_tcp_) {
    fds[0].fd = simulator_tcp_client_fd_;
  } else {
    fds[0].fd = simulator_socket_fd_;
  }
  fds[0].events = POLLIN;

  bool received_actuator = false;

  do {
    int timeout_ms = (received_first_actuator_ && enable_lockstep_) ? 1000 : 0;

    int ret = ::poll(&fds[0], 1, timeout_ms);

    if (ret == 0 && timeout_ms > 0) {
      gzerr << "poll timeout\n";
    }

    if (ret < 0) {
      gzerr << "poll error: " << strerror(errno) << "\n";
    }

    if (fds[0].revents & POLLIN) {

      int len = recvfrom(fds[0].fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);
      if (len > 0) {
        mavlink_message_t msg;
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i)
        {
          if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
          {
            if (hil_mode_) {
              send_mavlink_message(&msg);
            }
            handle_message(&msg, received_actuator);
          }
        }
      }
    }
  } while (received_first_actuator_ && !received_actuator && enable_lockstep_ && IsRunning() && !gotSigInt_);
}

void GazeboMavlinkInterface::pollFromQgcAndSdk()
{
  struct pollfd fds[2] = {};
  fds[0].fd = qgc_socket_fd_;
  fds[0].events = POLLIN;
  fds[1].fd = sdk_socket_fd_;
  fds[1].events = POLLIN;

  const int timeout_ms = 0;

  int ret = ::poll(&fds[0], 2, timeout_ms);

  if (ret < 0) {
    gzerr << "poll error: " << strerror(errno) << "\n";
    return;
  }

  if (fds[0].revents & POLLIN) {
    int len = recvfrom(qgc_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_qgc_addr_, &remote_qgc_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, _buf[i], &msg, &status)) {
          // forward message from QGC to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }

  if (fds[1].revents & POLLIN) {
    int len = recvfrom(sdk_socket_fd_, _buf, sizeof(_buf), 0, (struct sockaddr *)&remote_sdk_addr_, &remote_sdk_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, _buf[i], &msg, &status)) {
          // forward message from SDK to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }
}

void GazeboMavlinkInterface::handle_message(mavlink_message_t *msg, bool &received_actuator)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(msg, &controls);

    bool armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);

#if GAZEBO_MAJOR_VERSION >= 9
    last_actuator_time_ = world_->SimTime();
#else
    last_actuator_time_ = world_->GetSimTime();
#endif

    for (unsigned i = 0; i < n_out_max; i++) {
      input_index_[i] = i;
    }

    // set rotor speeds, controller targets
    input_reference_.resize(n_out_max);
    for (int i = 0; i < input_reference_.size(); i++) {
      if (armed) {
        input_reference_[i] = (controls.controls[input_index_[i]] + input_offset_[i])
            * input_scaling_[i] + zero_position_armed_[i];
      } else {
        input_reference_[i] = zero_position_disarmed_[i];
      }
    }

    received_actuator = true;
    received_first_actuator_ = true;
    break;
  }
}

void GazeboMavlinkInterface::handle_control(double _dt)
{
  // set joint positions
  for (int i = 0; i < input_reference_.size(); i++) {
    if (joints_[i]) {
      double target = input_reference_[i];
      if (joint_control_type_[i] == "velocity")
      {
        double current = joints_[i]->GetVelocity(0);
        double err = current - target;
        double force = pids_[i].Update(err, _dt);
        joints_[i]->SetForce(0, force);
      }
      else if (joint_control_type_[i] == "position")
      {

#if GAZEBO_MAJOR_VERSION >= 9
        double current = joints_[i]->Position(0);
#else
        double current = joints_[i]->GetAngle(0).Radian();
#endif

        double err = current - target;
        double force = pids_[i].Update(err, _dt);
        joints_[i]->SetForce(0, force);
      }
      else if (joint_control_type_[i] == "position_gztopic")
      {
     #if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
        /// only gazebo 7.4 and above support Any
        gazebo::msgs::Any m;
        m.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
        m.set_double_value(target);
     #else
        std::stringstream ss;
        gazebo::msgs::GzString m;
        ss << target;
        m.set_data(ss.str());
     #endif
        joint_control_pub_[i]->Publish(m);
      }
      else if (joint_control_type_[i] == "position_kinematic")
      {
        /// really not ideal if your drone is moving at all,
        /// mixing kinematic updates with dynamics calculation is
        /// non-physical.
     #if GAZEBO_MAJOR_VERSION >= 6
        joints_[i]->SetPosition(0, input_reference_[i]);
     #else
        joints_[i]->SetAngle(0, input_reference_[i]);
     #endif
      }
      else
      {
        gzerr << "joint_control_type[" << joint_control_type_[i] << "] undefined.\n";
      }
    }
  }
}

bool GazeboMavlinkInterface::IsRunning()
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world_->Running();
#else
    return world_->GetRunning();
#endif
}

void GazeboMavlinkInterface::open() {
  try{
    serial_dev.open(device_);
    serial_dev.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_dev.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_dev.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    gzdbg << "Opened serial device " << device_ << "\n";
  }
  catch (boost::system::system_error &err) {
    gzerr <<"Error opening serial device: " << err.what() << "\n";
  }
}

void GazeboMavlinkInterface::close()
{
  if(serial_enabled_) {
    ::close(qgc_socket_fd_);
    ::close(sdk_socket_fd_);

    std::lock_guard<std::recursive_mutex> lock(mutex);
    if (!is_open())
      return;

    io_service.stop();
    serial_dev.close();

    if (io_thread.joinable())
      io_thread.join();

  } else {

    if (use_tcp_) {
      ::close(simulator_tcp_client_fd_);
    } else {
      ::close(simulator_socket_fd_);
    }
  }
}

void GazeboMavlinkInterface::do_read(void)
{
  serial_dev.async_read_some(boost::asio::buffer(rx_buf), boost::bind(
      &GazeboMavlinkInterface::parse_buffer, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
      )
  );
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void GazeboMavlinkInterface::parse_buffer(const boost::system::error_code& err, std::size_t bytes_t){
  mavlink_status_t status;
  mavlink_message_t message;
  uint8_t *buf = this->rx_buf.data();

  assert(rx_buf.size() >= bytes_t);

  for(; bytes_t > 0; bytes_t--)
  {
    auto c = *buf++;

    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status);
      m_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        m_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer.len = 0;
        mavlink_start_checksum(&m_buffer);
      }
    }

    if (msg_received != Framing::incomplete) {
      if (hil_mode_) {
        forward_mavlink_message(&message);
      }
      bool not_used;
      handle_message(&message, not_used);
    }
  }
  do_read();
}

void GazeboMavlinkInterface::do_write(bool check_tx_state){
  if (check_tx_state && tx_in_progress)
    return;

  std::lock_guard<std::recursive_mutex> lock(mutex);
  if (tx_q.empty())
    return;

  tx_in_progress = true;
  auto &buf_ref = tx_q.front();

  serial_dev.async_write_some(
    boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()), [this, &buf_ref] (boost::system::error_code error,   size_t bytes_transferred)
    {
      assert(bytes_transferred <= buf_ref.len);
      if(error) {
        gzerr << "Serial error: " << error.message() << "\n";
      return;
      }

    std::lock_guard<std::recursive_mutex> lock(mutex);

    if (tx_q.empty()) {
      tx_in_progress = false;
      return;
    }

    buf_ref.pos += bytes_transferred;
    if (buf_ref.nbytes() == 0) {
      tx_q.pop_front();
    }

    if (!tx_q.empty()) {
      do_write(false);
    }
    else {
      tx_in_progress = false;
    }
  });
}

void GazeboMavlinkInterface::onSigInt() {
  gotSigInt_ = true;
  close();
}

}
