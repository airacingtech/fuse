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
#ifndef FUSE_MODELS__STEERING_ANGLE_2D_HPP_
#define FUSE_MODELS__STEERING_ANGLE_2D_HPP_

#include <string>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/uuid.hpp>

#include <race_msgs/msg/steering_report.hpp>
#include <rclcpp/rclcpp.hpp>


namespace fuse_models
{

class SteeringAngle2D : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(SteeringAngle2D)

  SteeringAngle2D();

  virtual ~SteeringAngle2D() = default;

  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  void process(const race_msgs::msg::SteeringReport & msg);

protected:
  void onInit() override;
  void onStart() override;
  void onStop() override;

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  fuse_core::UUID device_id_;

  std::string topic_;
  int queue_size_{10};
  double covariance_{0.01};

  rclcpp::Subscription<race_msgs::msg::SteeringReport>::SharedPtr sub_;
};

}  // namespace fuse_models

#endif  // FUSE_MODELS__STEERING_ANGLE_2D_HPP_
