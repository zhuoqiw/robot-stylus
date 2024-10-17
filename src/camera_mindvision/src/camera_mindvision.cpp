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

#include "camera_mindvision/camera_mindvision.hpp"

#include "CameraApi.h"

namespace camera_mindvision
{

CameraHandle CAMERA = 0;

void check(CameraSdkStatus s)
{
  if (s)
  {
    auto p = CameraGetErrorString(s);
    throw std::runtime_error(p);
  }
}

void fn(CameraHandle hCamera, BYTE * pFrameBuffer, tSdkFrameHead * pFrameHead, PVOID pContext)
{
  auto pNode = reinterpret_cast<CameraMindvision *>(pContext);

  // Get header stamp
  auto t = pNode->now();

  RCLCPP_INFO(rclcpp::get_logger(""), "id: %i, sec: %f", hCamera, t.seconds());

  // Get header id
  static UINT id = 0;
  CameraGetFrameID(hCamera, &id);

  auto msg = std::make_unique<Image>();
  msg->header.stamp = t;
  msg->header.frame_id = std::to_string(id);
  msg->height = pFrameHead->iHeight;
  msg->width = pFrameHead->iWidth;
  msg->encoding = "mono8";
  msg->is_bigendian = false;
  msg->step = pFrameHead->iWidth;
  msg->data.resize(pFrameHead->uBytes);
  memcpy(msg->data.data(), pFrameBuffer, pFrameHead->uBytes);

  pNode->publish(std::move(msg));
}

CameraMindvision::CameraMindvision(const rclcpp::NodeOptions & options)
: Node("camera_mindvision_node", options)
{
  try {
    CameraSdkStatus s = CAMERA_STATUS_SUCCESS;

    auto name = this->declare_parameter<std::string>("name", "");
    s = CameraInitEx2((char *) name.c_str(), &CAMERA);
    check(s);

    auto config = this->declare_parameter<std::string>("config", "");
    s = CameraReadParameterFromFile(CAMERA, (char *) config.c_str());
    check(s);

    s = CameraSetCallbackFunction(CAMERA, fn, this, NULL);
    check(s);

    s = CameraPlay(CAMERA);
    check(s);

    _pub = this->create_publisher<Image>(_pub_name, rclcpp::SensorDataQoS());

    _sub = this->create_subscription<Empty>(
      _sub_name,
      1,
      [](Empty::UniquePtr /*ptr*/) {
        CameraSoftTrigger(CAMERA);
      }
    );

    RCLCPP_INFO(this->get_logger(), "Initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
    rclcpp::shutdown();
  }
}

CameraMindvision::~CameraMindvision()
{
  _sub.reset();
  _pub.reset();

  CameraUnInit(CAMERA);
  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

void CameraMindvision::publish(Image::UniquePtr ptr)
{
  _pub->publish(std::move(ptr));
}

}  // namespace camera_mindvision

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_mindvision::CameraMindvision)
