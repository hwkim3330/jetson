// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#include <rcutils/cmdline_parser.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robot_driver/diff_drive_controller.hpp"
#include "robot_driver/robot.hpp"

void help_print()
{
  printf("For robot node : \n");
  printf("robot_driver [-i usb_port] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-i usb_port: Connected USB port with OpenCR.");
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    help_print();
    return 0;
  }

  rclcpp::init(argc, argv);

  //std::string usb_port = "/dev/ttyACM0";
  std::string usb_port = "/dev/ttyTHS1";
  char * cli_options;
  cli_options = rcutils_cli_get_option(argv, argv + argc, "-i");
  if (nullptr != cli_options) {
    usb_port = std::string(cli_options);
  }

  rclcpp::executors::SingleThreadedExecutor executor;

  auto robot = std::make_shared<keti::robot::Robot>(usb_port);
  auto diff_drive_controller =
    std::make_shared<keti::robot::DiffDriveController>(
    robot->get_wheels()->separation,
    robot->get_wheels()->radius);

  executor.add_node(robot);
  executor.add_node(diff_drive_controller);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
