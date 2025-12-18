// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef KETI_NODE__KETI_HPP_
#define KETI_NODE__KETI_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
//IMU #include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_msgs/msg/sensor_state.hpp>

#include "robot_driver/control_table.hpp"
#include "robot_driver/devices/devices.hpp"
#include "robot_driver/devices/motor_power.hpp"
#include "robot_driver/devices/reset.hpp"
#include "robot_driver/devices/sound.hpp"
#include "robot_driver/dynamixel_sdk_wrapper.hpp"
#include "robot_driver/odometry.hpp"
#include "robot_driver/sensors/battery_state.hpp"
//IMU #include "robot_driver/sensors/imu.hpp"
#include "robot_driver/sensors/joint_state.hpp"
#include "robot_driver/sensors/sensor_state.hpp"
#include "robot_driver/sensors/sensors.hpp"
#include "robot_driver/twist_subscriber.hpp"

namespace keti
{
namespace robot
{
extern const ControlTable extern_control_table;
class Robot : public rclcpp::Node
{
public:
  typedef struct
  {
    float separation;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  explicit Robot(const std::string & usb_port);
  virtual ~Robot() {}

  Wheels * get_wheels();
  Motors * get_motors();

private:
  void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  void check_device_status();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;

  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  std::list<sensors::Sensors *> sensors_;
  std::map<std::string, devices::Devices *> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::unique_ptr<TwistSubscriber> cmd_vel_sub_;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
}  // namespace robot
}  // namespace keti
#endif  // KETI_NODE__KETI_HPP_
