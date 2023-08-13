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

PointCloud2::UniquePtr to_pc2(const cv::Mat & src)
{
  auto pnts = src.reshape(1);

  if (pnts.type() != CV_32F && pnts.type() != CV_64F) {
    throw std::invalid_argument("Type is neither float nor double.");
  }

  if (pnts.cols < 3) {
    throw std::invalid_argument("Mat should be at least 3 columns.");
  }

  auto ptr = std::make_unique<PointCloud2>();

  auto num = pnts.rows;

  auto offset = pnts.type() == CV_64F ? 8 : 4;
  auto datatype = pnts.type() == CV_64F ? 8 : 7;

  ptr->height = 1;
  ptr->width = num;

  ptr->fields.resize(3);

  ptr->fields[0].name = "x";
  ptr->fields[0].offset = offset * 0;
  ptr->fields[0].datatype = datatype;
  ptr->fields[0].count = 1;

  ptr->fields[1].name = "y";
  ptr->fields[1].offset = offset * 1;
  ptr->fields[1].datatype = datatype;
  ptr->fields[1].count = 1;

  ptr->fields[2].name = "z";
  ptr->fields[2].offset = offset * 2;
  ptr->fields[2].datatype = datatype;
  ptr->fields[2].count = 1;

  ptr->is_bigendian = false;
  ptr->point_step = offset * 3;
  ptr->row_step = num * offset * 3;

  ptr->data.resize(num * offset * 3);

  ptr->is_dense = true;

  if (pnts.type() == CV_64F) {
    auto m = cv::Mat_<double>(num, 3, reinterpret_cast<double *>(ptr->data.data()));
    for (auto i = 0; i < num; ++i) {
      m(i, 0) = pnts.at<double>(i, 0);
      m(i, 1) = pnts.at<double>(i, 1);
      m(i, 2) = pnts.at<double>(i, 2);
    }
  } else {
    auto m = cv::Mat_<float>(num, 3, reinterpret_cast<float *>(ptr->data.data()));
    for (auto i = 0; i < num; ++i) {
      m(i, 0) = pnts.at<float>(i, 0);
      m(i, 1) = pnts.at<float>(i, 1);
      m(i, 2) = pnts.at<float>(i, 2);
    }
  }

  return ptr;
}

cv::Mat from_pc2(const PointCloud2::UniquePtr & ptr)
{
  if (ptr->fields.empty()) {
    throw std::invalid_argument("Empty fields.");
  }

  for (size_t i = 1; i < ptr->fields.size(); ++i) {
    if (ptr->fields[i].datatype != ptr->fields[i - 1].datatype) {
      throw std::invalid_argument("Fields are not homogeneous.");
    }
  }

  if (ptr->fields[0].datatype != 7 && ptr->fields[0].datatype != 8) {
    throw std::invalid_argument("Type is neither float nor double.");
  }

  auto num = ptr->width;
  auto dim = ptr->fields.size();

  if (ptr->fields[0].datatype == 7) {
    auto ret = cv::Mat_<float>(num, dim, reinterpret_cast<float *>(ptr->data.data()));
    return ret.clone();
  } else {
    auto ret = cv::Mat_<double>(num, dim, reinterpret_cast<double *>(ptr->data.data()));
    return ret.clone();
  }
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
    int i = R.at<double>(0, 0) < R.at<double>(1, 1) ?
      (R.at<double>(1, 1) < R.at<double>(2, 2) ? 2 : 1) :
      (R.at<double>(0, 0) < R.at<double>(2, 2) ? 2 : 0);
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

  vd = this->declare_parameter<std::vector<double>>("R", temp);
  auto R = cv::Mat(3, 3, CV_64F, vd.data());

  vd = this->declare_parameter<std::vector<double>>("T", temp);
  auto T = cv::Mat(3, 1, CV_64F, vd.data());

  cv::Mat Q;
  cv::stereoRectify(_c[0], _d[0], _c[1], _d[1], cv::Size(2048, 1536), R, T, _r[0], _r[1], _p[0], _p[1], Q);

  // _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

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
      auto p0 = from_pc2(pL);
      auto p1 = from_pc2(pR);
      if (p0.rows != 6 || p1.rows != 6) {
        RCLCPP_INFO(this->get_logger(), "Not pair by number, %d, %d", p0.rows, p1.rows);
        continue;   // Not pair by number
      } else {
        cv::Mat up0, up1;
        cv::undistortPoints(p0, up0, _c[0], _d[0], _r[0], _p[0]);
        cv::undistortPoints(p1, up1, _c[1], _d[1], _r[1], _p[1]);

        // auto dif = cv::Mat_<double>(up0.col(1) - up1.col(1));

        // for (auto iter = dif.begin(); iter != dif.end(); ++iter) {
        //   if (abs(*iter) > 3.) {
        //     RCLCPP_INFO(this->get_logger(), "Not pair by epipolar line constrain");
        //     continue;   // Not pair by epipolar line constrain
        //   }
        // }
        // RCLCPP_INFO(this->get_logger(), "Paired!");
        cv::Mat pnts4D, pnts;
        cv::triangulatePoints(_p[0], _p[1], up0, up1, pnts4D);
        cv::convertPointsFromHomogeneous(pnts4D.t(), pnts);
        auto msg = to_pc2(pnts);
        msg->header.stamp = this->now();
        msg->header.frame_id = "stylus";
        _pub->publish(std::move(msg));
        // auto ret = cv::estimateAffine3D(src, DST);
        // geometry_msgs::msg::TransformStamped t;
        // t.header.stamp = this->now();
        // t.header.frame_id = "vision";
        // t.child_frame_id = "stylus";

        // t.transform.translation.x = ret.at<double>(0, 3) / 1000.;
        // t.transform.translation.y = ret.at<double>(1, 3) / 1000.;
        // t.transform.translation.z = ret.at<double>(2, 3) / 1000.;

        // double q[4];
        // getQuaternion(ret, q);
        // t.transform.rotation.x = q[0];
        // t.transform.rotation.y = q[1];
        // t.transform.rotation.z = q[2];
        // t.transform.rotation.w = q[3];

        // _tf_broadcaster->sendTransform(t);

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
