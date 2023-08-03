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

#include "tf2/LinearMath/Quaternion.h"

namespace reconstruct_pose
{
std::vector<cv::Point3f> DST{
  {279, 60, 966},
  {281, 29, 963},
  {283, -8, 930},
  {280, -65, 924},
  {265, -139, 942},
  {266, -180, 939}};

std::vector<cv::Point2f> from_pc2(const PointCloud2::UniquePtr & ptr)
{
  auto num = ptr->width;
  std::vector<cv::Point2f> pnts(num);
  memcpy(pnts.data(), ptr->data.data(), num * 4 * 2);
  return pnts;
}

void getQuaternion(const cv::Mat & R, double Q[])
{
  double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

  if (trace > 0.0) {
    double s = sqrt(trace + 1.0);
    Q[3] = (s * 0.5);
    s = 0.5 / s;
    Q[0] = ((R.at<double>(2, 1) - R.at<double>(1, 2)) * s);
    Q[1] = ((R.at<double>(0, 2) - R.at<double>(2, 0)) * s);
    Q[2] = ((R.at<double>(1, 0) - R.at<double>(0, 1)) * s);
  } else {
    int i = R.at<double>(0, 0) < R.at<double>(1, 1) ? (R.at<double>(1, 1) < R.at<double>(2, 2) ? 2 : 1) : (R.at<double>(0, 0) < R.at<double>(2, 2) ? 2 : 0);
    int j = (i + 1) % 3;
    int k = (i + 2) % 3;

    double s = sqrt(R.at<double>(i, i) - R.at<double>(j, j) - R.at<double>(k, k) + 1.0);
    Q[i] = s * 0.5;
    s = 0.5 / s;

    Q[3] = (R.at<double>(k, j) - R.at<double>(j, k)) * s;
    Q[j] = (R.at<double>(j, i) + R.at<double>(i, j)) * s;
    Q[k] = (R.at<double>(k, i) + R.at<double>(i, k)) * s;
  }
}

ReconstructPose::ReconstructPose(const rclcpp::NodeOptions & options)
: Node("reconstruct_pose_node", options)
{
  std::vector<double> temp, vd;

  vd = this->declare_parameter<std::vector<double>>("camera_matrix_1", temp);
  _c[0] = cv::Mat(3, 3, CV_64F, vd.data()).clone();

  vd = this->declare_parameter<std::vector<double>>("camera_matrix_2", temp);
  _c[1] = cv::Mat(3, 3, CV_64F, vd.data()).clone();

  vd = this->declare_parameter<std::vector<double>>("dist_coeffs_1", temp);
  _d[0] = cv::Mat(1, 5, CV_64F, vd.data()).clone();

  vd = this->declare_parameter<std::vector<double>>("dist_coeffs_2", temp);
  _d[1] = cv::Mat(1, 5, CV_64F, vd.data()).clone();

  vd = this->declare_parameter<std::vector<double>>("r_1", temp);
  _r[0] = cv::Mat(3, 3, CV_64F, vd.data()).clone();

  vd = this->declare_parameter<std::vector<double>>("r_2", temp);
  _r[1] = cv::Mat(3, 3, CV_64F, vd.data()).clone();

  vd = this->declare_parameter<std::vector<double>>("p_1", temp);
  _p[0] = cv::Mat(3, 4, CV_64F, vd.data()).clone();

  vd = this->declare_parameter<std::vector<double>>("p_2", temp);
  _p[1] = cv::Mat(3, 4, CV_64F, vd.data()).clone();

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
      auto v1 = from_pc2(pL);
      auto v2 = from_pc2(pR);
      if (v1.size() != 6 || v2.size() != 6) {
        RCLCPP_INFO(this->get_logger(), "Not pair by number, %ld, %ld", v1.size(), v2.size());
        continue;   // Not pair by number
      } else {
        std::vector<cv::Point2f> uv1, uv2;
        cv::undistortPoints(v1, uv1, _c[0], _d[0], _r[0], _p[0]);
        cv::undistortPoints(v2, uv2, _c[1], _d[1], _r[1], _p[1]);

        for (size_t count = 0; count < 6; ++count) {
          if (abs(uv1[count].y - uv2[count].y) > 3.) {
            RCLCPP_INFO(this->get_logger(), "Not pair by epipolar line constrain");
            continue; // Not pair by epipolar line constrain
          }
        }
        // RCLCPP_INFO(this->get_logger(), "Paired!");
        cv::Mat pnts, src;
        cv::triangulatePoints(_p[0], _p[1], uv1, uv2, pnts);
        cv::convertPointsFromHomogeneous(pnts.t(), src);
        auto ret = cv::estimateAffine3D(src, DST);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "vision";
        t.child_frame_id = "stylus";

        t.transform.translation.x = ret.at<double>(0, 3) / 1000.;
        t.transform.translation.y = ret.at<double>(1, 3) / 1000.;
        t.transform.translation.z = ret.at<double>(2, 3) / 1000.;

        double q[4];
        getQuaternion(ret, q);
        t.transform.rotation.x = q[0];
        t.transform.rotation.y = q[1];
        t.transform.rotation.z = q[2];
        t.transform.rotation.w = q[3];

        _tf_broadcaster->sendTransform(t);

        // RCLCPP_INFO(this->get_logger(), "Paired!");
      }
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
