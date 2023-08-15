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
  double R[] = {
    9.4031341513862454e-01, -2.5256721423111048e-03, 3.4030031191664573e-01,
    -2.9455757840460097e-03, 9.9987459498021269e-01, 1.5560138060317327e-02,
    -3.4029693635652564e-01, -1.5633786917610453e-02, 9.4018805555749196e-01};
  double T[] = {-5.1503202408916297e-01, 9.7151409709307290e-04, 9.5298233515178865e-02};

  // double R0[] = {
  //   0.9865374881772063, -0.0014936211295192342, 0.16352844864710853,
  //   0.00017558018938361705, 0.9999673879687967, 0.008074166858522955,
  //   -0.16353517539846177, -0.007936755935747811, 0.9865055774361394};

  // double R1[] = {
  //   0.9833069891493235, -0.0018548295194217642, -0.18194483971123246,
  //   0.0003871521329200128, 0.9999671050050083, -0.008101791229634355,
  //   0.18195388207817434, 0.00789610760796016, 0.9832753613720449};

  // double P0[] = {
  //   2397.128392454825, 0.0, 1060.9726867675781, 0.0,
  //   0.0, 2397.128392454825, 744.0971908569336, 0.0,
  //   0.0, 0.0, 1.0, 0.0};

  // double P1[] = {
  //   2397.128392454825, 0.0, 1060.9726867675781, -1255.556913142339,
  //   0.0, 2397.128392454825, 744.0971908569336, 0.0,
  //   0.0, 0.0, 1.0, 0.0};

  double P[] = {
    0.2794257888387648, 0.06033236020688468, 0.965578877363138,
    0.2813044624014014, 0.02944443632489939, 0.9633302159173989,
    0.2834064713081882, -0.008072515285469335, 0.9304445205122588,
    0.2801647345230634, -0.06512649960154904, 0.9237126100815688,
    0.2651154022300604, -0.1388610695871446, 0.9417144820023857,
    0.2663857287273995, -0.1796075476032378, 0.9394877529170468};

  auto dst = cv::Mat(6, 3, CV_64F, P);
  auto c0 = cv::Mat_<double>(3, 3, camL);
  auto c1 = cv::Mat_<double>(3, 3, camR);
  auto d0 = cv::Mat_<double>(1, 5, disL);
  auto d1 = cv::Mat_<double>(1, 5, disR);
  auto rr = cv::Mat_<double>(3, 3, R);
  auto rt = cv::Mat_<double>(3, 1, T);
  // auto r0 = cv::Mat_<double>(3, 3, R0);
  // auto r1 = cv::Mat_<double>(3, 3, R1);
  // auto p0 = cv::Mat_<double>(3, 4, P0);
  // auto p1 = cv::Mat_<double>(3, 4, P1);

  cv::Mat r0, r1, p0, p1, Q;
  cv::stereoRectify(c0, d0, c1, d1, cv::Size(2048, 1536), rr, rt, r0, r1, p0, p1, Q);

  double v0[12] = {
    1320.0, 860.0,
    1326.0, 786.0,
    1354.0, 696.0,
    1351.0, 554.0,
    1302.0, 375.0,
    1306.0, 275.0};
  double v1[12] = {
    869.0, 921.0,
    872.0, 847.0,
    857.0, 755.0,
    845.0, 611.0,
    821.0, 435.0,
    823.0, 335.0};

  auto pnts0 = cv::Mat_<double>(6, 2, v0);
  auto pnts1 = cv::Mat_<double>(6, 2, v1);
  cv::Mat upnts0, upnts1, pnts4D, pnts;
  cv::undistortPoints(pnts0, upnts0, c0, d0, r0, p0);
  cv::undistortPoints(pnts1, upnts1, c1, d1, r1, p1);

  // std::cout << r0 << std::endl;
  // std::cout << r1 << std::endl;
  // std::cout << p0 << std::endl;
  // std::cout << p1 << std::endl;

  // std::cout << upnts1 << std::endl;
  auto dif = cv::Mat_<double>(upnts0.reshape(1).col(1) - upnts1.reshape(1).col(1));

  for (auto it = dif.begin(); it != dif.end(); ++it) {
    assert(*it > -2. && *it < 2.);
  }

  cv::triangulatePoints(p0, p1, upnts0, upnts1, pnts4D);
  cv::convertPointsFromHomogeneous(pnts4D.t(), pnts);

  dif = cv::Mat_<double>(dst - pnts.reshape(1));

  for (auto it = dif.begin(); it != dif.end(); ++it) {
    assert(*it > -0.001 && *it < 0.001);
  }

  return 0;
}
