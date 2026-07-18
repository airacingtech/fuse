/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Giacomo Franchini
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
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include <fuse_core/transaction.hpp>
#include <fuse_core/uuid.hpp>
#include <fuse_models/common/sensor_proc.hpp>
#include <fuse_models/odometry_3d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Odometry3D, fuse_core::SensorModel)

namespace fuse_models
{

namespace
{
constexpr double kReacquisitionGapThresholdSec = 0.3;  // publish gap that counts as an outage
constexpr double kReacquisitionMaxStdM = 100.0;        // safety clamp on the initial ramp position std
constexpr double kReacquisitionMaxOrientStdRad = 0.5;  // safety clamp on the initial ramp yaw std
}  // namespace

Odometry3D::Odometry3D()
: fuse_core::AsyncSensorModel(1),
  device_id_(fuse_core::uuid::NIL),
  logger_(rclcpp::get_logger("uninitialized")),
  throttled_callback_(std::bind(&Odometry3D::process, this, std::placeholders::_1))
{
}

void Odometry3D::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void Odometry3D::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
  clock_ = interfaces_.get_node_clock_interface()->get_clock();

  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  params_.loadFromROS(interfaces_, name_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);

  if (!params_.throttle_use_wall_time) {
    throttled_callback_.setClock(clock_);
  }

  if (params_.position_indices.empty() && params_.orientation_indices.empty() &&
    params_.linear_velocity_indices.empty() && params_.angular_velocity_indices.empty())
  {
    RCLCPP_WARN_STREAM(
      logger_,
      "No dimensions were specified. Data from topic " << params_.topic
                                                       << " will be ignored.");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_,
    interfaces_.get_node_base_interface(),
    interfaces_.get_node_logging_interface(),
    interfaces_.get_node_parameters_interface(),
    interfaces_.get_node_topics_interface()
  );
}

void Odometry3D::onStart()
{
  if (!params_.position_indices.empty() || !params_.orientation_indices.empty() ||
    !params_.linear_velocity_indices.empty() || !params_.angular_velocity_indices.empty())
  {
    previous_pose_.reset();

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
      interfaces_,
      params_.topic,
      params_.queue_size,
      std::bind(
        &OdometryThrottledCallback::callback<
          const nav_msgs::msg::Odometry &>,
        &throttled_callback_,
        std::placeholders::_1
      ),
      sub_options
    );
  }
}

void Odometry3D::onStop()
{
  sub_.reset();
}

void Odometry3D::process(const nav_msgs::msg::Odometry & msg)
{
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg.header.stamp);

  // Handle the pose data
  auto pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->header = msg.header;
  pose->pose = msg.pose;

  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header = msg.header;
  twist.header.frame_id = msg.child_frame_id;
  twist.twist = msg.twist;

  // Reacquisition covariance ramp (absolute pose only): after this source drops out
  // and returns, soften its position AND orientation covariance - scaled by the outage
  // length and decayed over reacquisition_tau_s - so the estimate slides back to the
  // fix instead of snapping off the dead-reckoned trajectory in one optimizer step.
  // Orientation is ramped too: during the outage the heading dead-reckons (gyro drift),
  // and a hard yaw correction rotates the whole velocity vector - the harshest part of
  // the snap even when position is already softened.
  if (!params_.differential && params_.reacquisition_tau_s > 0.0) {
    const rclcpp::Time stamp(msg.header.stamp);
    if (last_message_stamp_.nanoseconds() > 0) {
      const double gap = (stamp - last_message_stamp_).seconds();
      if (gap > kReacquisitionGapThresholdSec) {
        const double pos_std = std::min(
          gap * params_.reacquisition_position_softening_mps, kReacquisitionMaxStdM);
        const double yaw_std = std::min(
          gap * params_.reacquisition_heading_softening_radps, kReacquisitionMaxOrientStdRad);
        reacquisition_initial_variance_ = pos_std * pos_std;
        reacquisition_initial_orient_variance_ = yaw_std * yaw_std;
        reacquisition_ramp_start_ = stamp;
        in_reacquisition_ramp_ =
          reacquisition_initial_variance_ > 0.0 || reacquisition_initial_orient_variance_ > 0.0;
      }
    }
    last_message_stamp_ = stamp;
    if (in_reacquisition_ramp_) {
      const double dt = (stamp - reacquisition_ramp_start_).seconds();
      const double decay = std::exp(-2.0 * dt / params_.reacquisition_tau_s);
      const double pos_inflation = reacquisition_initial_variance_ * decay;
      const double orient_inflation = reacquisition_initial_orient_variance_ * decay;
      if (pos_inflation < 1e-4 && orient_inflation < 1e-6) {
        in_reacquisition_ramp_ = false;
      } else {
        pose->pose.covariance[0] += pos_inflation;
        pose->pose.covariance[7] += pos_inflation;
        pose->pose.covariance[14] += pos_inflation;
        pose->pose.covariance[21] += orient_inflation;
        pose->pose.covariance[28] += orient_inflation;
        pose->pose.covariance[35] += orient_inflation;
      }
    }
  }

  const bool validate = !params_.disable_checks;

  if (params_.differential) {
    processDifferential(*pose, twist, validate, *transaction);
  } else {
    common::processAbsolutePose3DWithCovariance(
      name(),
      device_id_,
      *pose,
      params_.pose_loss,
      params_.pose_target_frame,
      params_.position_indices,
      params_.orientation_indices,
      *tf_buffer_,
      validate,
      *transaction,
      params_.tf_timeout);
  }

  // Handle the twist data
  common::processTwist3DWithCovariance(
    name(),
    device_id_,
    twist,
    params_.linear_velocity_loss,
    params_.angular_velocity_loss,
    params_.twist_target_frame,
    params_.linear_velocity_indices,
    params_.angular_velocity_indices,
    *tf_buffer_,
    validate,
    *transaction,
    params_.tf_timeout);

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
}

void Odometry3D::processDifferential(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist,
  const bool validate,
  fuse_core::Transaction & transaction)
{
  auto transformed_pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  transformed_pose->header.frame_id =
    params_.pose_target_frame.empty() ? pose.header.frame_id : params_.pose_target_frame;

  if (!common::transformMessage(*tf_buffer_, pose, *transformed_pose)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 5.0 * 1000,
      "Cannot transform pose message with stamp "
        << rclcpp::Time(
        pose.header.stamp).nanoseconds() << " to pose target frame " << params_.pose_target_frame);
    return;
  }

  if (!previous_pose_) {
    previous_pose_ = std::move(transformed_pose);
    return;
  }

  if (params_.use_twist_covariance) {
    geometry_msgs::msg::TwistWithCovarianceStamped transformed_twist;
    transformed_twist.header.frame_id =
      params_.twist_target_frame.empty() ? twist.header.frame_id : params_.twist_target_frame;

    if (!common::transformMessage(*tf_buffer_, twist, transformed_twist)) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, *clock_, 5.0 * 1000,
        "Cannot transform twist message with stamp " << rclcpp::Time(
          twist.header.stamp).nanoseconds()
                                                     << " to twist target frame " <<
          params_.twist_target_frame);
    } else {
      common::processDifferentialPose3DWithTwistCovariance(
        name(),
        device_id_,
        *previous_pose_,
        *transformed_pose,
        transformed_twist,
        params_.minimum_pose_relative_covariance,
        params_.twist_covariance_offset,
        params_.pose_loss,
        params_.position_indices,
        params_.orientation_indices,
        validate,
        transaction);
    }
  } else {
    common::processDifferentialPose3DWithCovariance(
      name(),
      device_id_,
      *previous_pose_,
      *transformed_pose,
      params_.independent,
      params_.minimum_pose_relative_covariance,
      params_.pose_loss,
      params_.position_indices,
      params_.orientation_indices,
      validate,
      transaction);
  }
  previous_pose_ = std::move(transformed_pose);
}

}  // namespace fuse_models
