// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define KETI_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_driver/odometry.hpp"

namespace keti
{
namespace robot
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__DIFF_DRIVE_CONTROLLER_HPP_
