// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__SENSORS__BATTERY_STATE_HPP_
#define KETI_NODE__SENSORS__BATTERY_STATE_HPP_

#include <sensor_msgs/msg/battery_state.hpp>

#include <memory>
#include <string>

#include "robot_driver/sensors/sensors.hpp"

namespace keti
{
namespace robot
{
namespace sensors
{
class BatteryState : public Sensors
{
public:
  explicit BatteryState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "battery_state");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
};
}  // namespace sensors
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__SENSORS__BATTERY_STATE_HPP_
