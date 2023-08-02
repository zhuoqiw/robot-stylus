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

#include <deque>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "opencv2/opencv.hpp"

namespace reconstruct_pose
{

using sensor_msgs::msg::PointCloud2;

/**
 * @brief Construct vector of floats from ROS point cloud message.
 *
 * @param ptr PointCloud2::UniquePtr point cloud message to publish.
 * @return std::vector<float> a sequence of floats as points' uv coordinate.
 */
std::vector<cv::Point2f> from_pc2(const PointCloud2::UniquePtr & ptr);

class ReconstructPose : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new ReconstructPose object.
   *
   * Initialize broadcaster.
   * Print success if all done.
   * @param options Encapsulation of options for node initialization.
   */
  explicit ReconstructPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the LocateStylus object.
   *
   * Release broadcaster.
   * Print success if all done.
   * Throw no exception.
   */
  virtual ~ReconstructPose();

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
   * @brief Push a points cloud and notity workers.
   *
   * @param ptr Reference to a unique pointer to PointCloud2 to be moved.
   */
  void _push_back_l(PointCloud2::UniquePtr ptr);

  /**
   * @brief Push a points cloud and notity workers.
   *
   * @param ptr Reference to a unique pointer to PointCloud2 to be moved.
   */
  void _push_back_r(PointCloud2::UniquePtr ptr);

private:
  /**
   * @brief The transform broadcaster
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

  /**
   * @brief Subscription name.
   *
   */
  const char * _sub_l_name = "~/points_l";  // TODO(imp)

  /**
   * @brief Shared pointer to Subscription.
   *
   */
  rclcpp::Subscription<PointCloud2>::SharedPtr _sub_l;

  /**
   * @brief Subscription name.
   *
   */
  const char * _sub_r_name = "~/points_r";  // TODO(imp)

  /**
   * @brief Shared pointer to Subscription.
   *
   */
  rclcpp::Subscription<PointCloud2>::SharedPtr _sub_r;

  /**
   * @brief Mutex to protect image queue.
   *
   */
  std::mutex _mutex;

  /**
   * @brief Condition variable for image queue.
   *
   */
  std::condition_variable _con;

  /**
   * @brief Double end queue for PointCloud2.
   *
   */
  std::deque<PointCloud2::UniquePtr> _deq_l;

  /**
   * @brief Double end queue for PointCloud2.
   *
   */
  std::deque<PointCloud2::UniquePtr> _deq_r;

  /**
   * @brief Threads for workers and the manager.
   *
   */
  std::vector<std::thread> _threads;

  cv::Mat _c[2], _d[2], _r[2], _p[2];
};

}  // namespace reconstruct_pose

#endif  // RECONSTRUCT_POSE__RECONSTRUCT_POSE_HPP_
