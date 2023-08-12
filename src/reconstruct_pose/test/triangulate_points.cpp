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

#include "reconstruct_pose/reconstruct_pose.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
  double camL[] = {
    2.3963365682379472e+03, 0., 1.0381244825685860e+03,
    0., 2.3963365682379472e+03, 7.3522354517671818e+02,
    0., 0., 1.};
  double camR[] = {
    2.3979202166717037e+03, 0., 1.0246038481245218e+03,
    0., 2.3979202166717037e+03, 7.5583322007652328e+02,
    0., 0., 1.};
  double disL[] = {
    -1.6609743480630104e-01, 1.8059350130377208e-01, 8.6264119226421636e-05,
    6.3388807558157061e-04, -5.0579492985700608e-02};
  double disR[] = {
    -1.5868913120695460e-01, 1.1985018123339684e-01, 3.3259328128667386e-05,
    1.4906857068374316e-04, 1.0078485762652550e-01};
  double recL[] = {
    9.8653748817720632e-01, -1.4936211295192368e-03, 1.6352844864710853e-01,
    1.7558018938361965e-04, 9.9996738796879669e-01, 8.0741668585229548e-03,
    -1.6353517539846177e-01, -7.9367559357478110e-03, 9.8650557743613942e-01};
  double recR[] = {
    9.8330698914932346e-01, -1.8548295194217668e-03, -1.8194483971123246e-01,
    3.8715213292001542e-04, 9.9996710500500829e-01, -8.1017912296343555e-03,
    1.8195388207817434e-01, 7.8961076079601608e-03, 9.8327536137204485e-01};
  double proL[] = {
    2.3971283924548252e+03, 0., 1.0609726867675781e+03, 0.,
    0., 2.3971283924548252e+03, 7.4409719085693359e+02, 0.,
    0., 0., 1., 0.};
  double proR[] = {
    2.3971283924548252e+03, 0., 1.0609726867675781e+03, -1.2555569131423391e+06,
    0., 2.3971283924548252e+03, 7.4409719085693359e+02, 0.,
    0., 0., 1., 0.};
  auto c0 = cv::Mat_<double>(3, 3, camL);
  auto c1 = cv::Mat_<double>(3, 3, camR);
  auto d0 = cv::Mat_<double>(1, 5, disL);
  auto d1 = cv::Mat_<double>(1, 5, disR);
  auto r0 = cv::Mat_<double>(3, 3, recL);
  auto r1 = cv::Mat_<double>(3, 3, recR);
  auto p0 = cv::Mat_<double>(3, 4, proL);
  auto p1 = cv::Mat_<double>(3, 4, proR);

  float v0[12] = {
    1320.0, 860.0,
    1326.0, 786.0,
    1354.0, 696.0,
    1351.0, 554.0,
    1302.0, 375.0,
    1306.0, 275.0};
  float v1[12] = {
    869.0, 921.0,
    872.0, 847.0,
    857.0, 755.0,
    845.0, 611.0,
    821.0, 435.0,
    823.0, 335.0};

  auto pnts0 = cv::Mat_<float>(6, 2, v0);
  auto pnts1 = cv::Mat_<float>(6, 2, v1);
  cv::Mat upnts0, upnts1, pnts4D;
  cv::undistortPoints(pnts0, upnts0, c0, d0, r0, p0);
  cv::undistortPoints(pnts1, upnts1, c1, d1, r1, p1);

  std::cout << upnts0 - upnts1 << std::endl;
  // std::cout << upnts1 << std::endl;

  cv::triangulatePoints(p0, p1, upnts0, upnts1, pnts4D);
  std::cout << pnts4D << std::endl;
}
