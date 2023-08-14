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

#include "camera_pylon/camera_pylon.hpp"

#include <map>
#include <string>
#include <utility>

namespace camera_pylon
{

using namespace Pylon;  // NOLINT

Image::UniquePtr execute(const Pylon::CGrabResultPtr & ptr)
{
  if (ptr->GetPixelType() != EPixelType::PixelType_Mono8) {
    throw std::invalid_argument("Only mono 8 is supported.");
  }
  if (ptr->GetBufferSize() != ptr->GetHeight() * ptr->GetWidth()) {
    throw std::invalid_argument("Buffer is not continuous.");
  }

  auto msg = std::make_unique<Image>();
  msg->height = ptr->GetHeight();
  msg->width = ptr->GetWidth();
  msg->encoding = "mono8";
  msg->is_bigendian = false;
  msg->step = ptr->GetWidth();
  msg->data.resize(ptr->GetBufferSize());
  memcpy(msg->data.data(), ptr->GetBuffer(), ptr->GetBufferSize());
  return msg;
}

class _CImageEventHandler : public CImageEventHandler
{
public:
  explicit _CImageEventHandler(CameraPylon * ptr)
  : _ptr(ptr) {}

  virtual void OnImageGrabbed(CInstantCamera & /*camera*/, const CGrabResultPtr & ptrGrabResult)
  {
    // Image grabbed successfully?
    if (ptrGrabResult->GrabSucceeded()) {
      _ptr->_push_back_image(ptrGrabResult);
    } else {
      RCLCPP_WARN(
        _ptr->get_logger(),
        "Camera error: %s",
        ptrGrabResult->GetErrorDescription().c_str());
    }
  }

private:
  CameraPylon * _ptr;
};

CameraPylon::CameraPylon(const rclcpp::NodeOptions & options)
: Node("camera_pylon_node", options)
{
  _workers = this->declare_parameter<int>("workers", 1);
  auto sn = this->declare_parameter<std::string>("serial", "");

  for (int i = 0; i < _workers; ++i) {
    _threads.push_back(std::thread(&CameraPylon::_worker, this));
  }
  _threads.push_back(std::thread(&CameraPylon::_manager, this));

  // Initialize cameras
  PylonInitialize();

  // Attach an instant camera object for the camera device found serial.
  CTlFactory & TlFactory = CTlFactory::GetInstance();
  CDeviceInfo di;
  di.SetSerialNumber(sn.c_str());
  di.SetDeviceClass(BaslerUsbDeviceClass);
  cam.Attach(TlFactory.CreateDevice(di));

  // Register the standard configuration event handler for enabling software triggering.
  // The software trigger configuration handler replaces the default configuration
  // as all currently registered configuration handlers are removed
  // by setting the registration mode to RegistrationMode_ReplaceAll.
  cam.RegisterConfiguration(
    new CSoftwareTriggerConfiguration,
    RegistrationMode_ReplaceAll,
    Cleanup_Delete);

  // The image event printer serves as sample image processing.
  // When using the grab loop thread provided by the Instant Camera object,
  // an image event handler processing the grab
  // results must be created and registered.
  cam.RegisterImageEventHandler(
    new _CImageEventHandler(this),
    RegistrationMode_Append,
    Cleanup_Delete);

  cam.Open();

  cam.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);

  _pub = this->create_publisher<Image>(_pub_name, rclcpp::SensorDataQoS());

  _sub = this->create_subscription<Empty>(
    _sub_name,
    10,
    [this](Empty::UniquePtr)
    {
      try {
        cam.ExecuteSoftwareTrigger();
      } catch (const Pylon::GenericException & e) {
        RCLCPP_WARN(this->get_logger(), "Pylon exception: %s", e.what());
      }
    }
  );

  _srv = this->create_service<Trigger>(
    _srv_name,
    [this](
      const std::shared_ptr<Trigger::Request>/*request*/,
      std::shared_ptr<Trigger::Response> response)
    {
      try {
        cam.ExecuteSoftwareTrigger();
        response->success = true;
      } catch (const Pylon::GenericException & e) {
        response->message = std::string(e.what());
        response->success = false;
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

CameraPylon::~CameraPylon()
{
  cam.Attach(NULL);
  PylonTerminate();
  try {
    _srv.reset();
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

void CameraPylon::_worker()
{
  // cv::Mat buf;
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_images_mut);
    if (_images.empty() == false) {
      auto ptr = _images.front();
      _images.pop_front();
      std::promise<Image::UniquePtr> prom;
      _push_back_future(prom.get_future());
      lk.unlock();
      auto msg = execute(ptr);
      msg->header.stamp = this->now();
      msg->header.frame_id = std::to_string(ptr->GetImageNumber());
      prom.set_value(std::move(msg));
    } else {
      _images_con.wait(lk);
    }
  }
}

void CameraPylon::_manager()
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

void CameraPylon::_push_back_image(const CGrabResultPtr & rhs)
{
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.push_back(rhs);
  auto s = static_cast<int>(_images.size());
  if (s > _workers + 1) {
    _images.pop_front();
    RCLCPP_WARN(this->get_logger(), "Image skipped");
  }
  lk.unlock();
  _images_con.notify_all();
}

void CameraPylon::_push_back_future(std::future<Image::UniquePtr> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_all();
}

}  // namespace camera_pylon

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_pylon::CameraPylon)
