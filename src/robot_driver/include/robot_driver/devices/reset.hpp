// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__DEVICES__RESET_HPP_
#define KETI_NODE__DEVICES__RESET_HPP_

#include <memory>
#include <string>

#include <std_srvs/srv/trigger.hpp>

#include "robot_driver/devices/devices.hpp"

namespace keti
{
namespace robot
{
namespace devices
{
class Reset : public Devices
{
public:
  static void request(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    std_srvs::srv::Trigger::Request req);

  explicit Reset(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "reset");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__DEVICES__RESET_HPP_
