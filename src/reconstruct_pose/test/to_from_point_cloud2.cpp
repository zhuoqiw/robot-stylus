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

#include "reconstruct_pose/reconstruct_pose.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
  // Test case 1
  // Float matrix(30, 3) to point cloud2
  {
    auto src = cv::Mat(30, 3, CV_32F);
    cv::randn(src, 0., 2.);
    auto pc2 = reconstruct_pose::to_pc2(src);
    auto dst = reconstruct_pose::from_pc2(pc2);

    assert(src.type() == dst.type() && src.rows == dst.rows && src.cols == dst.cols);

    auto dif = cv::Mat(src - dst);
    for (auto iter = dif.begin<float>(); iter != dif.end<float>(); ++iter) {
      assert(*iter == 0);
    }
  }

  // Test case 2
  // Double matrix(30, 3) to point cloud2
  // {
  //   auto src = cv::Mat(30, 3, CV_64F, 0.);
  //   cv::randn(src, 0., 2.);
  //   auto pc2 = reconstruct_pose::to_pc2(src);
  //   auto dst = reconstruct_pose::from_pc2(pc2);

  //   assert(src.type() == dst.type() && src.rows == dst.rows && src.cols == dst.cols);
  //   auto dif = cv::Mat(src - dst);
  //   for (auto iter = dif.begin<float>(); iter != dif.end<float>(); ++iter) {
  //     assert(*iter == 0);
  //   }
  // }

  // Test case 3
  // Double matrix(30, 4) to point cloud2
  // {
  //   auto src = cv::Mat(30, 4, CV_64F, 0.);
  //   cv::randn(src, 0., 2.);
  //   auto pc2 = reconstruct_pose::to_pc2(src);
  //   auto dst = reconstruct_pose::from_pc2(pc2);

  //   auto abc = src.colRange(0, 3);

  //   assert(abc.type() == dst.type() && abc.rows == dst.rows && abc.cols == dst.cols);
  //   auto dif = cv::Mat(abc - dst);
  //   for (auto iter = dif.begin<float>(); iter != dif.end<float>(); ++iter) {
  //     assert(*iter == 0);
  //   }
  // }

  return 0;
}
