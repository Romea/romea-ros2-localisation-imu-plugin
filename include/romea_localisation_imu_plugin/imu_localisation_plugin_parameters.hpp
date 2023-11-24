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

#ifndef ROMEA_LOCALISATION_IMU_PLUGIN__IMU_LOCALISATION_PLUGIN_PARAMETERS_HPP_
#define ROMEA_LOCALISATION_IMU_PLUGIN__IMU_LOCALISATION_PLUGIN_PARAMETERS_HPP_

// eigen
#include <Eigen/Geometry>

// ros
#include <rclcpp/node.hpp>

namespace romea
{
namespace ros2
{

void declare_restamping(rclcpp::Node::SharedPtr node);
void declare_enable_accelerations(rclcpp::Node::SharedPtr node);

bool get_restamping(rclcpp::Node::SharedPtr node);
bool get_enable_accelerations(rclcpp::Node::SharedPtr node);

void declare_imu_rate(rclcpp::Node::SharedPtr node);
void declare_imu_acceleration_noise_density(rclcpp::Node::SharedPtr node);
void declare_imu_acceleration_bias_stability_std(rclcpp::Node::SharedPtr node);
void declare_imu_acceleration_range(rclcpp::Node::SharedPtr node);
void declare_imu_angular_speed_noise_density(rclcpp::Node::SharedPtr node);
void declare_imu_angular_speed_bias_stability_std(rclcpp::Node::SharedPtr node);
void declare_imu_angular_speed_range(rclcpp::Node::SharedPtr node);
void declare_imu_magnetic_noise_density(rclcpp::Node::SharedPtr node);
void declare_imu_magnetic_bias_stability_std(rclcpp::Node::SharedPtr node);
void declare_imu_magnetic_range(rclcpp::Node::SharedPtr node);
void declare_imu_heading_std(rclcpp::Node::SharedPtr node);
void declare_imu_body_pose(rclcpp::Node::SharedPtr node);

double get_imu_rate(rclcpp::Node::SharedPtr node);
double get_imu_acceleration_noise_density(rclcpp::Node::SharedPtr node);
double get_imu_acceleration_bias_stability_std(rclcpp::Node::SharedPtr node);
double get_imu_acceleration_range(rclcpp::Node::SharedPtr node);
double get_imu_angular_speed_noise_density(rclcpp::Node::SharedPtr node);
double get_imu_angular_speed_bias_stability_std(rclcpp::Node::SharedPtr node);
double get_imu_angular_speed_range(rclcpp::Node::SharedPtr node);
double get_imu_magnetic_noise_density(rclcpp::Node::SharedPtr node);
double get_imu_magnetic_bias_stability_std(rclcpp::Node::SharedPtr node);
double get_imu_magnetic_range(rclcpp::Node::SharedPtr node);
double get_imu_heading_std(rclcpp::Node::SharedPtr node);
Eigen::Affine3d get_imu_body_pose(rclcpp::Node::SharedPtr node);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_IMU_PLUGIN__IMU_LOCALISATION_PLUGIN_PARAMETERS_HPP_
