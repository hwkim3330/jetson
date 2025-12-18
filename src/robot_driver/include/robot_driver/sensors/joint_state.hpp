// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__SENSORS__JOINT_STATE_HPP_
#define KETI_NODE__SENSORS__JOINT_STATE_HPP_

#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>

#include "robot_driver/sensors/sensors.hpp"

namespace keti
{
namespace robot
{
namespace sensors
{
constexpr uint8_t JOINT_NUM = 2;

constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

// 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
constexpr double TICK_TO_RAD = 0.001533981;

class JointState : public Sensors
{
public:
  explicit JointState(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & topic_name = "joint_states",
    const std::string & frame_id = "base_link");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

  std::string name_space_;
  std::string wheel_left_joint_ = "wheel_left_joint";
  std::string wheel_right_joint_ = "wheel_right_joint";
};
}  // namespace sensors
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__SENSORS__JOINT_STATE_HPP_
