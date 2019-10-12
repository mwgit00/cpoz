// MIT License
//
// Copyright(c) 2019 Mark Whitney
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ROOM_TEST_H_
#define ROOM_TEST_H_

#include <map>
#include <string>
#include <opencv2/imgproc.hpp>
#include "XYZLandmark.h"

// consider camera looking straight ahead
// then world location (on floor) is in X,Z plane
// view in room diagrams (see comments) is looking down at the room
//
// +X "cross" +Y points in +Z direction, so +Y points down into floor
// a positive rotation about Y will be counterclockwise

typedef std::map<std::string, cv::Point3d> tMapStrToXYZ;
typedef std::map<std::string, cv::Vec2d> tMapStrToAZEL;
typedef std::map<std::string, tMapStrToXYZ> tMapStrToMapStrToXYZ;

bool landmark_test(
    cpoz::XYZLandmark& lm1,
    cpoz::XYZLandmark& lm2,
    const cv::Point3d& cam_xyz,
    const cv::Vec2d& cam_angs_rad,
    cv::Point3d& pos_xyz,
    double& world_azim);

void room_test(
    const tMapStrToMapStrToXYZ& lm_maps,
    const tMapStrToAZEL& lm_vis,
    const cv::Point3d& known_cam_xyz,
    const std::string& lm_map_name_A,
    const std::string& lm_map_name_B,
    const double elev_offset = 0.0);

void reset_counts();
void show_tally();

#endif // ROOM_TEST_H_
