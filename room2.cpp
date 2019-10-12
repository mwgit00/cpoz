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

#include <string>
#include <map>

#include "room1.h"

// this file contains data that defines landmarks in an imaginary room
// it is simply a rectangular 10x16 area with landmarks near corners
// the units are feet but have been converted to inches for testing
//
// +Z
// 0,16 -*-----*- 10,16
//  | B          C |
//  |              |
//  |              |
//  |              |
//  |              |
//  |              |
//  |              |
//  |              |
//  |              |
//  |              |
//  |              |
//  |              |
//  | A          D |
//  0,0 -*-----*- 10,0 +X
//

namespace room2
{
    const double y = -120.0;  // 10 foot ceiling

    // the so-called "fixed" landmarks in corners
    tMapStrToXYZ mark1 =
    { {"A", {0.0, y, 0.0}},
    {"B", {0.0, y, 192.0}},
    {"C", {120.0, y, 192.0}},
    {"D", {120.0, y, 0.0}} };

    // companion landmarks
    tMapStrToXYZ mark2 =
    { {"A", {24.0, y, 0.0}},
    {"B", {24.0, y, 192.0}},
    {"C", {96.0, y, 192.0}},
    {"D", {96.0, y, 0.0}} };

    tMapStrToMapStrToXYZ landmark_maps =
    { {"mark1", mark1},
    {"mark2", mark2} };

    // azimuth and elevation of camera so that landmarks are visible from (5, 8)
    tMapStrToAZEL lm_vis_5_8 =
    { {"A", {210.0, 30.0}},
    {"B", {330.0, 30.0}},
    {"C", {30.0, 30.0}},
    {"D", {150.0, 30.0}} };

    // azimuth and elevation of camera so that landmarks are visible from (1, 8)
    tMapStrToAZEL lm_vis_1_8 =
    { { "A",{ 180.0, 30.0 } },
    { "B",{ 00.0, 30.0 } },
    { "C",{ 45.0, 30.0 } },
    { "D",{ 135.0, 30.0 } } };
}
