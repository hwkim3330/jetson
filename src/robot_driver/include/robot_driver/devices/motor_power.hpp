// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__DEVICES__MOTOR_POWER_HPP_
#define KETI_NODE__DEVICES__MOTOR_POWER_HPP_

#include <memory>
#include <string>

#include <std_srvs/srv/set_bool.hpp>

#include "robot_driver/devices/devices.hpp"

namespace keti
{
namespace robot
{
namespace devices
{
class MotorPower : public Devices
{
public:
  static void request(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
    std_srvs::srv::SetBool::Request req);

  explicit MotorPower(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "motor_power");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__DEVICES__MOTOR_POWER_HPP_
