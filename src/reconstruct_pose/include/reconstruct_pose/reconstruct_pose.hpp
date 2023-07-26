// Copyright 2019 InSoul Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RECONSTRUCT_POSE__RECONSTRUCT_POSE_HPP_
#define RECONSTRUCT_POSE__RECONSTRUCT_POSE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace reconstruct_pose
{

class ReconstructPose : public rclcpp::Node
{
public:
  explicit ReconstructPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ReconstructPose();

private:
  void _Init();
  void _InitializeParameters();
  void _UpdateParameters();
  void _Sub(std_msgs::msg::String::UniquePtr ptr);  // TODO(imp)
  void _Srv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);  // TODO(imp)

private:
  const char * _pubName = "~/pub";  // TODO(imp)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/sub";  // TODO(imp)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;

  const char * _srvName = "~/srv";  // TODO(imp)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv;

  std::thread _init;
};

}  // namespace reconstruct_pose

#endif  // RECONSTRUCT_POSE__RECONSTRUCT_POSE_HPP_
