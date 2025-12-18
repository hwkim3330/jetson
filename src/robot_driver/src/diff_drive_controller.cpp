// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#include "robot_driver/diff_drive_controller.hpp"

#include <memory>

using keti::robot::DiffDriveController;

DiffDriveController::DiffDriveController(const float wheel_seperation, const float wheel_radius)
: Node("diff_drive_controller", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  odometry_ = std::make_unique<Odometry>(
    nh_,
    wheel_seperation,
    wheel_radius);

  RCLCPP_INFO(this->get_logger(), "Run!");
}
