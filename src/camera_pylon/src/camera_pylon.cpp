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
int COUNT = 0;
int ID = 0;

using namespace Pylon;  // NOLINT

/**
 * @brief Basler time stamp, ticks per second.
 *
 */
constexpr int TICKS_PER_SEC = 1000000000;

/**
 * @brief Basler image buffer number.
 *
 */
constexpr int BUFFER_NUMBER = 10;

class CImageEventPrinter : public CImageEventHandler
{
public:
  explicit CImageEventPrinter(CameraPylon * ptr)
  : _ptr(ptr) {}

  virtual void OnImageGrabbed(CInstantCamera & /*camera*/, const CGrabResultPtr & ptrGrabResult)
  {
    // Image grabbed successfully?
    if (ptrGrabResult->GrabSucceeded()) {
      _ptr->_push_back_image(ptrGrabResult);
    } else {
      RCLCPP_WARN(_ptr->get_logger(), "Image broken");
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
    new CImageEventPrinter(this),
    RegistrationMode_Append,
    Cleanup_Delete);

  cam.Open();

  cam.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);

  _pub = this->create_publisher<Image>(_pub_name, rclcpp::SensorDataQoS());

  _srv_trigger = this->create_service<Trigger>(
    _srv_trigger_name,
    [this](
      const std::shared_ptr<Trigger::Request>/*request*/,
      std::shared_ptr<Trigger::Response> response)
    {
      response->success = true;
      cam.ExecuteSoftwareTrigger();
    }
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

CameraPylon::~CameraPylon()
{
  cam.Attach(NULL);
  PylonTerminate();
  RCLCPP_INFO(this->get_logger(), "count: %i, id: %i", COUNT, ID);
  // _init.join();

  // _srv.reset();
  // _sub.reset();
  // _impl.reset();
  // _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
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
      auto msg = std::make_unique<Image>();
      msg->header.stamp = this->now();
      msg->header.frame_id = std::to_string(ptr->GetImageNumber());
      msg->height = ptr->GetHeight();
      msg->width = ptr->GetWidth();
      msg->encoding = "mono8";
      msg->is_bigendian = false;
      msg->step = msg->width;
      msg->data.resize(msg->height * msg->width);
      memcpy(msg->data.data(), ptr->GetBuffer(), msg->height * msg->width);
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
  COUNT++;
  ID = rhs->GetImageNumber();
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.push_back(rhs);
  auto s = static_cast<int>(_images.size());
  if (s >= BUFFER_NUMBER - 1) {
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
