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

#include "reconstruct_pose/reconstruct_pose.hpp"

#include <memory>

namespace reconstruct_pose
{

ReconstructPose::ReconstructPose(const rclcpp::NodeOptions & options)
: Node("reconstruct_pose_node", options)
{
  _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  _threads.push_back(std::thread(&ReconstructPose::_worker, this));

  _sub_l = this->create_subscription<PointCloud2>(
    _sub_l_name,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr)
    {
      _push_back_l(std::move(ptr));
    }
  );

  _sub_r = this->create_subscription<PointCloud2>(
    _sub_r_name,
    rclcpp::SensorDataQoS(),
    [this](PointCloud2::UniquePtr ptr)
    {
      _push_back_r(std::move(ptr));
    }
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

ReconstructPose::~ReconstructPose()
{
  try {
    _sub_l.reset();
    _sub_r.reset();
    _con.notify_all();
    for (auto & t : _threads) {
      t.join();
    }
    _tf_broadcaster.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

void ReconstructPose::_worker()
{
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_mutex);
    if (_deq_l.empty() || _deq_r.empty()) {
      _con.wait(lk);
    } else {
      auto pL = std::move(_deq_l.front());
      auto pR = std::move(_deq_r.front());
      _deq_l.pop_front();
      _deq_r.pop_front();
      lk.unlock();
      RCLCPP_INFO(this->get_logger(), "id: %s, %s", pL->header.frame_id.c_str(), pR->header.frame_id.c_str());
    }
  }
}

void ReconstructPose::_push_back_l(PointCloud2::UniquePtr ptr)
{
  std::unique_lock<std::mutex> lk(_mutex);
  _deq_l.emplace_back(std::move(ptr));
  lk.unlock();
  _con.notify_all();
}

void ReconstructPose::_push_back_r(PointCloud2::UniquePtr ptr)
{
  std::unique_lock<std::mutex> lk(_mutex);
  _deq_r.emplace_back(std::move(ptr));
  lk.unlock();
  _con.notify_all();
}

}  // namespace reconstruct_pose

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(reconstruct_pose::ReconstructPose)
