/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, AI Racing Tech
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_core/parameter.hpp>
#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_models/steering_angle_2d.hpp>
#include <fuse_variables/stamped.hpp>
#include <fuse_variables/steering_angle_2d_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::SteeringAngle2D, fuse_core::SensorModel)

namespace fuse_models
{

SteeringAngle2D::SteeringAngle2D()
: fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  logger_(rclcpp::get_logger("uninitialized"))
{
}

void SteeringAngle2D::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void SteeringAngle2D::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  fuse_core::getParamRequired(
    interfaces_, fuse_core::joinParameterName(name_, "topic"), topic_);
  queue_size_ = fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(name_, "queue_size"), queue_size_);
  covariance_ = fuse_core::getParam(
    interfaces_, fuse_core::joinParameterName(name_, "covariance"), covariance_);
}

void SteeringAngle2D::onStart()
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  sub_ = rclcpp::create_subscription<race_msgs::msg::SteeringReport>(
    interfaces_,
    topic_,
    queue_size_,
    std::bind(&SteeringAngle2D::process, this, std::placeholders::_1),
    sub_options);
}

void SteeringAngle2D::onStop()
{
  sub_.reset();
}

void SteeringAngle2D::process(const race_msgs::msg::SteeringReport & msg)
{
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg.stamp);

  auto steering_var = fuse_variables::SteeringAngle2DStamped::make_shared(
    rclcpp::Time(msg.stamp), device_id_);
  steering_var->data()[fuse_variables::SteeringAngle2DStamped::YAW] =
    static_cast<double>(msg.front_wheel_angle_rad);

  fuse_core::Vector1d mean;
  mean << static_cast<double>(msg.front_wheel_angle_rad);

  fuse_core::Matrix1d covariance;
  covariance << covariance_;

  auto constraint =
    fuse_constraints::AbsoluteConstraint<fuse_variables::SteeringAngle2DStamped>::make_shared(
    name(), *steering_var, mean, covariance);

  transaction->addInvolvedStamp(rclcpp::Time(msg.stamp));
  transaction->addVariable(steering_var);
  transaction->addConstraint(constraint);

  sendTransaction(transaction);
}

}  // namespace fuse_models
