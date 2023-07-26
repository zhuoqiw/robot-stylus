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

#include <random>

#include "locate_stylus/locate_stylus.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
  using locate_stylus::to_pc2;
  using locate_stylus::from_pc2;

  // Test case 1
  {
    // Prepare random
    std::default_random_engine e;
    std::normal_distribution<float> nd(0, 1);

    // Create a random vector of float
    std::vector<float> src(1000);
    for (auto & p : src) {
      p = nd(e);
    }

    // From vector to point cloud2
    auto ptr = to_pc2(src);

    // Vice verse
    auto dst = from_pc2(ptr);

    // Compare two vector
    assert(src == dst);
  }

  // Test case 2
  {
    std::vector<float> src(999);
    try {
      auto ptr = to_pc2(src);
    } catch (const std::invalid_argument & e) {
    }
  }

  return 0;
}
