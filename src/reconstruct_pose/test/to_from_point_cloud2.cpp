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
  auto m = cv::Mat(3, 3, CV_64F, 1.);

  m.row(2) = m.row(0) + m.row(1) + m.row(2);
  m.row(1) = m.row(0) + m.row(1);

  m.col(2) = m.col(0) + m.col(1) + m.col(2);
  m.col(1) = m.col(0) + m.col(1);

  const cv::Mat_<float> & dm = m;

  std::cout << dm << std::endl;
  return 0;
}
