// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__DEVICES__SOUND_HPP_
#define KETI_NODE__DEVICES__SOUND_HPP_

#include <robot_msgs/srv/sound.hpp>
#include <memory>
#include <string>
#include "robot_driver/devices/devices.hpp"

namespace keti
{
namespace robot
{
namespace devices
{
class Sound : public Devices
{
public:
  static void request(
    rclcpp::Client<robot_msgs::srv::Sound>::SharedPtr client,
    robot_msgs::srv::Sound::Request req);

  explicit Sound(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "sound");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<robot_msgs::srv::Sound>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__DEVICES__SOUND_HPP_
