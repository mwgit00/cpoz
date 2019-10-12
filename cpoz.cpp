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

#include <iostream>

#include "room_test.h"
#include "room1.h"
#include "room2.h"
//#include "CameraHelper.h"
//#include "XYZLandmark.h"


void test_room1()
{
    std::cout << "==========================================" << std::endl;
    std::cout << "ROOM 1 TEST" << std::endl;

    reset_counts();

    {
        // LM name mapped to [world_azim, elev] for visibility at world (1, 1)
        cv::Point3d world_xyz = { 1.0, -2.0, 1.0 };
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark2");  // one case with one landmark not visible
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark3");
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark2", 10.0);
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark3", 10.0);
    }

    {
        // LM name mapped to[world_azim, elev] for visibility at world (1, 1)
        cv::Point3d world_xyz = { 1.0, -3.0, 1.0 };
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark2");
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark3");
    }

    {
        // LM name mapped to[world_azim, elev] for visibility at world(7, 6)
        cv::Point3d world_xyz = { 7.0, -2.0, 6.0 };
        room_test(room1::landmark_maps, room1::lm_vis_7_6, world_xyz, "mark1", "mark2");
        room_test(room1::landmark_maps, room1::lm_vis_7_6, world_xyz, "mark1", "mark3");
    }

    show_tally();
}

void test_room2()
{
    std::cout << "==========================================" << std::endl;
    std::cout << "ROOM 2 TEST" << std::endl;

    reset_counts();

    {
        // LM name mapped to [world_azim, elev] for visibility at world (5, 8)
        // but convert to inches to test scaling
        cv::Point3d world_xyz = { 60.0, -24.0, 96.0 };
        room_test(room2::landmark_maps, room2::lm_vis_5_8, world_xyz, "mark1", "mark2");
        room_test(room2::landmark_maps, room2::lm_vis_5_8, world_xyz, "mark1", "mark2", 10.0);
    }

    {
        // LM name mapped to [world_azim, elev] for visibility at world (1, 8)
        // but convert to inches to test scaling
        cv::Point3d world_xyz = { 12.0, -24.0, 96.0 };
        room_test(room2::landmark_maps, room2::lm_vis_1_8, world_xyz, "mark1", "mark2");
        room_test(room2::landmark_maps, room2::lm_vis_1_8, world_xyz, "mark1", "mark2", 10.0);
    }

    show_tally();
}


int main()
{
    std::cout << "CPOZ Test Application" << std::endl;
    //cpoz::CameraHelper cam;
    //cam.cal(".\\cal_set_2");
    test_room1();
    test_room2();
}
