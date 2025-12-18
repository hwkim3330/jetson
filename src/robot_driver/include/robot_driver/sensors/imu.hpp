// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__SENSORS__IMU_HPP_
#define KETI_NODE__SENSORS__IMU_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "robot_driver/sensors/sensors.hpp"

namespace keti
{
namespace robot
{
namespace sensors
{
class Imu : public Sensors
{
public:
  explicit Imu(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & imu_topic_name = "imu",
    const std::string & mag_topic_name = "magnetic_field",
    const std::string & frame_id = "imu_link");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  std::string name_space_;
};
}  // namespace sensors
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__SENSORS__IMU_HPP_
