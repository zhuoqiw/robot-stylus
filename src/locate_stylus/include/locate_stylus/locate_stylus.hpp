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

#ifndef LOCATE_STYLUS__LOCATE_STYLUS_HPP_
#define LOCATE_STYLUS__LOCATE_STYLUS_HPP_

#include <deque>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace locate_stylus
{

using std_msgs::msg::String;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

/**
 * @brief Construct ROS point cloud message from vector of floats.
 *
 * @param pnts a sequence of floats as points' uv coordinate.
 * @exception std::invalid_argument size is not a multiple of 2.
 * @return PointCloud2::UniquePtr point cloud message to publish.
 */
PointCloud2::UniquePtr to_pc2(const std::vector<float> & pnts);

/**
 * @brief Construct vector of floats from ROS point cloud message.
 *
 * @param ptr PointCloud2::UniquePtr point cloud message to publish.
 * @return std::vector<float> a sequence of floats as points' uv coordinate.
 */
std::vector<float> from_pc2(const PointCloud2::UniquePtr & ptr);

class LocateStylus : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new LocateStylus object.
   *
   * Initialize publisher.
   * Print success if all done.
   * @param options Encapsulation of options for node initialization.
   */
  explicit LocateStylus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the LocateStylus object.
   *
   * Release publisher.
   * Print success if all done.
   * Throw no exception.
   */
  virtual ~LocateStylus();

private:
  /**
   * @brief The worker works in seperate thread to process incoming date parallelly.
   *
   * Create a buffer.
   * Enter infinite loop.
   * Wait for incoming data.
   * Wake up to get a possible data, make a promise and notify the manager.
   * Continue to work on the data and return to sleep if no further data to process.
   */
  void _worker();

  /**
   * @brief The manager works in seperate thread to gather worker's results in order.
   *
   * Spin infinitely until rclcpp:ok() return false.
   * Whenever a future is ready, the manager wake up, get the result from the future and publish.
   */
  void _manager();

  /**
   * @brief Push a image and notity workers.
   *
   * @param ptr Reference to a unique pointer to image to be moved.
   */
  void _push_back_image(Image::UniquePtr ptr);

  /**
   * @brief Promise a future so its future can be sychronized and notify the manager.
   *
   * @param f A future to point cloud msg.
   */
  void _push_back_future(std::future<PointCloud2::UniquePtr> fut);

private:
  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/points";  // TODO(imp)

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<PointCloud2>::SharedPtr _pub;

  /**
   * @brief Subscription name.
   *
   */
  const char * _sub_name = "~/image";  // TODO(imp)

  /**
   * @brief Shared pointer to Subscription.
   *
   */
  rclcpp::Subscription<Image>::SharedPtr _sub;

  /**
   * @brief Number of co-workers.
   *
   */
  int _workers;

  /**
   * @brief Mutex to protect image queue.
   *
   */
  std::mutex _images_mut;

  /**
   * @brief Condition variable for image queue.
   *
   */
  std::condition_variable _images_con;

  /**
   * @brief Double end queue for images.
   *
   */
  std::deque<Image::UniquePtr> _images;

  /**
   * @brief Mutex to protect result queue.
   *
   */
  std::mutex _futures_mut;

  /**
   * @brief Condition variable for result queue.
   *
   */
  std::condition_variable _futures_con;

  /**
   * @brief Double end queue for results.
   *
   */
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  /**
   * @brief Threads for workers and the manager.
   *
   */
  std::vector<std::thread> _threads;
};

}  // namespace locate_stylus

#endif  // LOCATE_STYLUS__LOCATE_STYLUS_HPP_
