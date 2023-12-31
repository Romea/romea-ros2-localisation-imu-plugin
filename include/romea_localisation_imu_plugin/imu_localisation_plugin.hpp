// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_LOCALISATION_IMU_PLUGIN__IMU_LOCALISATION_PLUGIN_HPP_
#define ROMEA_LOCALISATION_IMU_PLUGIN__IMU_LOCALISATION_PLUGIN_HPP_

// std
#include <string>
#include <memory>

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

// romea
#include "romea_core_localisation_imu/LocalisationIMUPlugin.hpp"
#include "romea_common_utils/conversions/diagnostic_conversions.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"
#include "romea_localisation_utils/conversions/observation_angular_speed_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_attitude_conversions.hpp"
#include "romea_localisation_imu_plugin/visibility_control.h"

namespace romea
{
namespace ros2
{


class IMULocalisationPlugin
{
public:
  using ImuMsg = sensor_msgs::msg::Imu;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using ObservationAttitudeStampedMsg =
    romea_localisation_msgs::msg::ObservationAttitudeStamped;
  using ObservationAngularSpeedStampedMsg =
    romea_localisation_msgs::msg::ObservationAngularSpeedStamped;

public:
  ROMEA_LOCALISATION_IMU_PLUGIN_PUBLIC
  explicit IMULocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_LOCALISATION_IMU_PLUGIN_PUBLIC
  virtual ~IMULocalisationPlugin() = default;

  ROMEA_LOCALISATION_IMU_PLUGIN_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_attitude_publisher_();

  void init_angular_speed_publisher_();

  void init_diagnostic_publisher_();

  void init_imu_subcriber_();

  void init_odometry_subscriber_();

  void init_plugin_();

  void init_debug_();

  void init_timer_();


  void process_imu_(ImuMsg::ConstSharedPtr msg);

  void process_odom_(OdometryMsg::ConstSharedPtr msg);

  void process_angular_speed_(const rclcpp::Time & stamp, const ImuMsg & msg);

  void process_attitude_(const rclcpp::Time & stamp, const ImuMsg & msg);

  void publish_angular_speed_(const rclcpp::Time & stamp, const std::string & frame_id);

  void publish_attitude_(const rclcpp::Time & stamp, const std::string & frame_id);

  void timer_callback_();

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<core::LocalisationIMUPlugin> plugin_;
  core::ObservationAngularSpeed angular_speed_observation_;
  core::ObservationAttitude attitude_observation_;

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<ObservationAttitudeStampedMsg>::SharedPtr attitude_pub_;
  rclcpp::Publisher<ObservationAngularSpeedStampedMsg>::SharedPtr angular_speed_pub_;
  std::shared_ptr<StampedPublisherBase<core::DiagnosticReport>> diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool restamping_;
  bool enable_accelerations_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_IMU_PLUGIN__IMU_LOCALISATION_PLUGIN_HPP_
