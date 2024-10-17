// Copyright 2019 Zhushi Tech, Inc.
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

#ifndef CAMERA_MINDVISION__CAMERA_MINDVISION_HPP_
#define CAMERA_MINDVISION__CAMERA_MINDVISION_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_mindvision
{

using std_msgs::msg::Empty;
using sensor_msgs::msg::Image;

class CameraMindvision : public rclcpp::Node
{
public:
  explicit CameraMindvision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CameraMindvision();
  void publish(Image::UniquePtr ptr);

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/image";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<Image>::SharedPtr _pub;

  /**
   * @brief Subscription name.
   *
   */
  const char * _sub_name = "~/grab";

  /**
   * @brief Shared pointer to Subscription.
   *
   */
  rclcpp::Subscription<Empty>::SharedPtr _sub;
};

}  // namespace camera_mindvision

#endif  // CAMERA_MINDVISION__CAMERA_MINDVISION_HPP_
