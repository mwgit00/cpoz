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

#ifndef ROOM_1_H_
#define ROOM_1_H_

#include <map>
#include <string>
#include <opencv2/imgproc.hpp>

typedef struct _T_xyz_RL_struct
{
    cv::Point3f xyz;
    double adj_R;   // always negative
    double adj_L;   // always positive
} T_xyz_RL;

typedef std::map<std::string, T_xyz_RL> tMapStrToXYZRL;
typedef std::map<std::string, cv::Vec2d> tMapStrToAZEL;
typedef std::map<std::string, tMapStrToXYZRL> tMapStrToMapStrToXYZRL;

extern tMapStrToXYZRL mark1;
extern tMapStrToXYZRL mark2;
extern tMapStrToXYZRL mark3;
extern tMapStrToXYZRL markb;

extern tMapStrToMapStrToXYZRL all_landmark_maps;

extern tMapStrToAZEL lm_vis_1_1;
extern tMapStrToAZEL lm_vis_7_6;

#endif // ROOM_1_H_
