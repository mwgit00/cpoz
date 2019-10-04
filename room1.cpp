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
// it is a 10x16 "room" with 3x9 "alcove" with landmarks at some corners
// the asterisks show the locations of secondary landmarks
// landmark C is in a 135 degree corner (instead of typical 90 degrees)
//
// +Z
// 0,16 -*--*- 8,16
//  | B      C  \
//  *            *
//  |            |
//  |            |
//  |            *
//  |            |  E (10,9)
//  |            @--*
//  |               |
//  |         .     *
//  |               @ F (13,6)
//  |               *
//  |               |
//  *   .           *
//  | A           D |
//  0,0 -*-----*- 13,0 +X
//
// world location is in X,Z plane
// view is looking down at the room 
// positive rotation about Y is clockwise
// the angle from A to B is 0 degrees
//
// landmarks are higher on the AB side
// ceiling slopes down to E then is flat again
// so heights of some landmarks had to be extrapolated
// height is negative to be consistent with right-hand coordinate system
// (+X "cross" +Y points in +Z direction, so +Y points down into floor)


const double y_ab = -10.0;
const double y_def = -8.0;
const double y_offs = -2.0;


// the fixed landmarks
tMapStrToXYZRL mark1 =
    {{"A", {{0.0, y_ab, 0.0}, 0.0, 270.0}},
    {"B", {{0.0, y_ab, 16.0}, -270.0, 0.0}},
    {"C", {{8.0, y_ab + 1.6, 16.0}, -180.0, 45.0}},
    {"D", {{13.0, y_def, 0.0}, -90.0, 180.0}},
    {"E", {{10.0, y_def, 9.0}, -90.0, 0.0}},
    {"F", {{13.0, y_def, 6.0}, -90.0, 90.}}};

// landmarks that appear to left of the fixed landmarks
// (u1 of fixed LM is MAX, or greater than u2 of this LM)
tMapStrToXYZRL mark2 =
    {{"A", {{2.0, y_ab + 0.4, 0.0}, 0.0, 0.0}},
    {"B", {{0.0, y_ab, 14.0}, 0.0, 0.0}},
    {"C", {{6.0, y_ab + 1.2, 16.0}, 0.0, 0.0}},
    {"D", {{13.0, y_def, 2.0}, 0.0, 0.0}},
    {"E", {{10.0, y_def, 11.0}, 0.0, 0.0}},
    {"F", {{13.0, y_def + 1.0, 7.0}, 0.0, 0.0}}};

// landmarks that appear to right of the fixed landmarks
// (u1 of fixed LM is MIN, or less than u2 of this LM)
tMapStrToXYZRL mark3 =
    {{"A", {{0.0, y_ab, 2.0}, 0.0, 0.0}},
    {"B", {{2.0, y_ab + 0.4, 16.0}, 0.0, 0.0}},
    {"C", {{10.0, y_def, 14.0}, 0.0, 0.0}},
    {"D", {{11.0, y_def, 0.0}, 0.0, 0.0}},
    {"E", {{12.0, y_def, 9.0}, 0.0, 0.0}},
    {"F", {{13.0, y_def + 1.0, 5.0}, 0.0, 0.0}}};

// landmarks that appear below the fixed landmarks
tMapStrToXYZRL markb =
    {{"A", {{0.0, y_ab - y_offs, 0.0}, 0.0, 270.0}},
    {"B", {{0.0, y_ab - y_offs, 16.0}, -270.0, 0.0}},
    {"C", {{8.0, y_ab + 1.6 - y_offs, 16.0}, -180.0, 45.0}},
    {"D", {{13.0, y_def - y_offs, 0.0}, -90.0, 180.0}},
    {"E", {{10.0, y_def - y_offs, 9.0}, -90.0, 0.0}},
    {"F", {{13.0, y_def - y_offs, 6.0}, -90.0, 90.}}};

tMapStrToMapStrToXYZRL all_landmark_maps =
    { {"mark1", mark1},
    {"mark2", mark2},
    {"mark3", mark3},
    {"markb", markb} };

// azimuth and elevation of camera so that landmarks
// are visible from (1, 1) at height -3
tMapStrToAZEL lm_vis_1_1 =
    {{"A", {225.0, 70.0}},
    {"B", {0.0, 30.0}},
    {"C", {30.0, 0.0}},
    {"D", {90.0, 30.0}},
    {"E", {45.0, 15.0}},
    {"F", {60.0, 15.}}};

// azimuth and elevation of camera so that landmarks
// are visible from (7, 6) at height -2
tMapStrToAZEL lm_vis_7_6 =
    {{"A", {225.0, 30.0}},
    {"B", {315.0, 30.0}},
    {"C", {0.0, 20.0}},
    {"D", {135.0, 30.0}},
    {"E", {45.0, 60.0}},
    {"F", {90.0, 60.}}};
