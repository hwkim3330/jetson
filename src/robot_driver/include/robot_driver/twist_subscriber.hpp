// Copyright 2025 KETI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim


#ifndef KETI_NODE__TWIST_SUBSCRIBER_HPP_
#define KETI_NODE__TWIST_SUBSCRIBER_HPP_


#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

/**
 * @class nav2_util::TwistSubscriber

 *
 */

class TwistSubscriber
{
public:
  /**
  * @brief A constructor that supports either Twist and TwistStamped
  * @param node The node to add the Twist subscriber to
  * @param topic The subscriber topic name
  * @param qos The subscriber quality of service
  * @param TwistCallback The subscriber callback for Twist messages
  * @param TwistStampedCallback The subscriber callback for TwistStamped messages
  */
  template<typename TwistCallbackT,
    typename TwistStampedCallbackT
  >
  explicit TwistSubscriber(
    const rclcpp::Node::SharedPtr & node,
    const std::string & topic,
    const rclcpp::QoS & qos,
    TwistCallbackT && TwistCallback,
    TwistStampedCallbackT && TwistStampedCallback
  )
  {
    if (!node->has_parameter("enable_stamped_cmd_vel")) {
      node->declare_parameter("enable_stamped_cmd_vel", false);
    }
    node->get_parameter("enable_stamped_cmd_vel", is_stamped_);
    if (is_stamped_) {
      twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic,
        qos,
        std::forward<TwistStampedCallbackT>(TwistStampedCallback));
    } else {
      twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        topic,
        qos,
        std::forward<TwistCallbackT>(TwistCallback));
    }
  }

  /**
  * @brief A constructor that only supports TwistStamped
  * @param node The node to add the TwistStamped subscriber to
  * @param topic The subscriber topic name
  * @param qos The subscriber quality of service
  * @param TwistStampedCallback The subscriber callback for TwistStamped messages
  * @throw std::invalid_argument When configured with an invalid ROS parameter
  */
  template<typename TwistStampedCallbackT>
  explicit TwistSubscriber(
    const rclcpp::Node::SharedPtr & node,
    const std::string & topic,
    const rclcpp::QoS & qos,
    TwistStampedCallbackT && TwistStampedCallback
  )
  {
    if (!node->has_parameter("enable_stamped_cmd_vel")) {
      node->declare_parameter("enable_stamped_cmd_vel", false);
    }
    node->get_parameter("enable_stamped_cmd_vel", is_stamped_);
    if (is_stamped_) {
      twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic,
        qos,
        std::forward<TwistStampedCallbackT>(TwistStampedCallback));
    } else {
      throw std::invalid_argument(
              "enable_stamped_cmd_vel must be true when using this constructor!");
    }
  }

protected:
  //! @brief The user-configured value for ROS parameter enable_stamped_cmd_vel
  bool is_stamped_{false};
  //! @brief The subscription when using Twist
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_ {nullptr};
  //! @brief The subscription when using TwistStamped
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_ {nullptr};
};

#endif  // KETI_NODE__TWIST_SUBSCRIBER_HPP_
