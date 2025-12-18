// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__SENSORS__SENSORS_HPP_
#define KETI_NODE__SENSORS__SENSORS_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "robot_driver/control_table.hpp"
#include "robot_driver/dynamixel_sdk_wrapper.hpp"

namespace keti
{
namespace robot
{
extern const ControlTable extern_control_table;
namespace sensors
{
class Sensors
{
public:
  explicit Sensors(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & frame_id = "")
  : nh_(nh),
    frame_id_(frame_id)
  {
  }

  virtual void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::string frame_id_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
};
}  // namespace sensors
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__SENSORS__SENSORS_HPP_
