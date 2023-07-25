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

#undef NDEBUG

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using sensor_msgs::msg::PointCloud2;

int main(int argc, char ** argv)
{
  // Initializes ROS 2 C++ client library.
  rclcpp::init(argc, argv);

  // Creates a node named empty_point_cloud2.
  auto node = rclcpp::Node::make_shared("empty_point_cloud2");

  // A publisher is initialized.
  auto pub = node->create_publisher<PointCloud2>("~/points", rclcpp::SensorDataQoS());

  // An empty PointCloud2 message.
  auto msg = std::make_unique<PointCloud2>();

  // The message is published.
  pub->publish(std::move(msg));

  // Shut down the system.
  rclcpp::shutdown();

  return 0;
}
