#ifndef __ImuParameters_HPP__
#define __ImuParameters_HPP__

#include <rclcpp/node.hpp>

namespace romea {

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

}

#endif
