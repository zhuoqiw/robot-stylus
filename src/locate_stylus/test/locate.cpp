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

#include <chrono>

#include "locate_stylus/locate_stylus.hpp"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

int main(int /*argc*/, char ** /*argv*/)
{
  // Test case 1
  {
    cv::Mat img;
    auto ret = locate_stylus::locate(img);
    assert(ret.empty());
  }

  // Test case 2
  {
    cv::Mat img(768, 1024, CV_8UC1);

    auto start = high_resolution_clock::now();

    for (auto i = 0; i < 1000; ++i) {
      auto ret = locate_stylus::locate(img);
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    std::cout << "fps: " << 1000 * 1000. / duration.count() << "\n";
  }

  return 0;
}
