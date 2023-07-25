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

#include "locate_stylus/locate_stylus.hpp"

#include <memory>

#include "opencv2/opencv.hpp"

namespace locate_stylus
{

LocateStylus::LocateStylus(const rclcpp::NodeOptions & options)
: Node("locate_stylus_node", options)
{
  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _workers = this->declare_parameter<int>("workers", 1);

  for (int i = 0; i < _workers; ++i) {
    _threads.push_back(std::thread(&LocateStylus::_worker, this));
  }
  _threads.push_back(std::thread(&LocateStylus::_manager, this));

  _sub = this->create_subscription<Image>(
    _sub_name,
    rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr ptr)
    {
      _push_back_image(std::move(ptr));
    }
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

LocateStylus::~LocateStylus()
{
  try {
    _sub.reset();
    _images_con.notify_all();
    _futures_con.notify_one();
    for (auto & t : _threads) {
      t.join();
    }
    _pub.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

void LocateStylus::_worker()
{
  // cv::Mat buf;
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_images_mut);
    if (_images.empty() == false) {
      auto ptr = std::move(_images.front());
      _images.pop_front();
      std::promise<PointCloud2::UniquePtr> prom;
      _push_back_future(prom.get_future());
      lk.unlock();
      auto msg = std::make_unique<PointCloud2>();
      prom.set_value(std::move(msg));
    } else {
      _images_con.wait(lk);
    }
  }
}

void LocateStylus::_manager()
{
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_futures_mut);
    if (_futures.empty() == false) {
      auto f = std::move(_futures.front());
      _futures.pop_front();
      lk.unlock();
      auto ptr = f.get();
      _pub->publish(std::move(ptr));
    } else {
      _futures_con.wait(lk);
    }
  }
}

void LocateStylus::_push_back_image(Image::UniquePtr ptr)
{
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.emplace_back(std::move(ptr));
  auto s = static_cast<int>(_images.size());
  if (s > _workers + 1) {
    _images.pop_front();
    RCLCPP_WARN(this->get_logger(), "Image skipped");
  }
  lk.unlock();
  _images_con.notify_all();
}

void LocateStylus::_push_back_future(std::future<PointCloud2::UniquePtr> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_one();
}

}  // namespace locate_stylus

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(locate_stylus::LocateStylus)
