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

#include "room1.h"
#include "CameraHelper.h"
#include "XYZLandmark.h"


const double EPS = 0.01;

cpoz::CameraHelper cam;


bool landmark_test(
    cpoz::XYZLandmark& lm1,
    cpoz::XYZLandmark& lm2,
    const cv::Point3d& cam_xyz,
    const cv::Vec2d& cam_angs_rad,
    cv::Point3d& pos_xyz,
    double& world_azim)
{
    // for the two landmarks:
    // - translate landmark by camera offset
    // - rotate by azimuth and elevation
    // - project into image
    
    double azim = cam_angs_rad[0];
    double elev = cam_angs_rad[1];
    
    // determine pixel location of LM 1
    cv::Point3d xyz1 = lm1.world_xyz - cam_xyz;
    cv::Point3d xyz1_rot = cam.calc_xyz_after_rotation(xyz1, elev, azim, 0);
    cv::Point2d img_xy1 = cam.project_world_xyz_to_img_xy(xyz1_rot);

    // determine pixel location of LM 2
    cv::Point3d xyz2 = lm2.world_xyz - cam_xyz;
    cv::Point3d xyz2_rot = cam.calc_xyz_after_rotation(xyz2, elev, azim, 0);
    cv::Point2d img_xy2 = cam.project_world_xyz_to_img_xy(xyz2_rot);

    // dump the image X,Y points and perform visibility check
    std::cout << std::endl;
    std::cout << "Image Landmark 1:  " << img_xy1 << std::endl;
    std::cout << "Image Landmark 2:  " << img_xy2 << std::endl;
    if (cam.is_visible(img_xy1) && cam.is_visible(img_xy2))
    {
        std::cout << "Both landmarks are visible." << std::endl;
    }
    else
    {
        std::cout << "********** At least one landmark is NOT visible!!!" << std::endl;
        return false;
    }
    
    // all is well so proceed with test...

    // landmarks have been acquired
    // camera elevation and world Y also need updating
    cam.cam_elev = elev;
    cam.cam_y = cam_xyz.y;

    lm1.set_img_xy(img_xy1);
    lm2.set_img_xy(img_xy2);

    double r;
    double ang;
    double rel_azim;
    cam.triangulate(lm1.world_xyz.y, lm2.world_xyz.y, img_xy1, img_xy2, r, ang, rel_azim);

    // landmark has angle offset info
    // which is used to calculate world coords and azim
    double u2 = lm2.img_xy.x;
    world_azim = lm1.calc_world_azim(u2, ang, rel_azim);
    pos_xyz = lm1.calc_world_xyz(u2, ang, r);

    std::cout << "Robot is at: [" << pos_xyz.x << ", " << pos_xyz.z << "] @ " << world_azim * cpoz::RAD2DEG << std::endl;
    //
    //// this integer coordinate stuff is disabled for now...
    //if False:
    //print "Now try with integer pixel coords and known Y coords..."
    //lm1.set_current_uv((int(u1 + 0.5), int(v1 + 0.5)))
    //lm2.set_current_uv((int(u2 + 0.5), int(v2 + 0.5)))
    //print lm1.uv
    //print lm2.uv
    //
    return true;
}



void room_test(
    const tMapStrToAZEL& lm_vis,
    const cv::Point3d& known_cam_xyz,
    const std::string& lm_map_name,
    const double elev_offset = 0.0)
{
    std::cout << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "ROOM TEST ";
    std::cout << lm_map_name << ", " << known_cam_xyz;
    std::cout << ", " << elev_offset << std::endl;
    
    for (const auto& r : lm_vis)
    {
        double cam_azim_deg = r.second[0];
        double cam_elev_deg = r.second[1] + elev_offset;
        cv::Vec2d cam_angs_rad = { cam_azim_deg * cpoz::DEG2RAD, cam_elev_deg * cpoz::DEG2RAD };

        tMapStrToXYZRL& markx = all_landmark_maps[lm_map_name];

        cv::Point3d pos_xyz;
        double world_azim;

        const std::string& rkey = r.first;
        cpoz::XYZLandmark lm1(mark1[rkey].world_xyz, mark1[rkey].adj_R, mark1[rkey].adj_L);
        cpoz::XYZLandmark lm2(markx[rkey].world_xyz, markx[rkey].adj_R, markx[rkey].adj_L);

        bool result = true;
        bool flag = landmark_test(lm1, lm2, known_cam_xyz, cam_angs_rad, pos_xyz, world_azim);

        if (!flag)
        {
            result = false;
        }

        if (abs(pos_xyz.x - known_cam_xyz.x) >= EPS)
        {
            result = false;
        }

        if (abs(pos_xyz.z - known_cam_xyz.z) >= EPS)
        {
            result = false;
        }

        world_azim *= cpoz::RAD2DEG;
        if ((abs(world_azim - cam_azim_deg) >= EPS) && (abs(world_azim - 360.0 - cam_azim_deg) >= EPS))
        {
            result = false;
        }

        if (result)
        {
            std::cout << "PASS!!!" << std::endl;
        }
    }
}


void test_room1()
{
    {
        // LM name mapped to [world_azim, elev] for visibility at world (1, 1)
        cv::Point3d world_xyz = { 1.0, -2.0, 1.0 };
        room_test(lm_vis_1_1, world_xyz, "mark2");  // one case with one landmark not visible
        room_test(lm_vis_1_1, world_xyz, "mark3");
        room_test(lm_vis_1_1, world_xyz, "mark2", 10.0);
        room_test(lm_vis_1_1, world_xyz, "mark3", 10.0);
    }

    {
        // LM name mapped to[world_azim, elev] for visibility at world (1, 1)
        cv::Point3d world_xyz = { 1.0, -3.0, 1.0 };
        room_test(lm_vis_1_1, world_xyz, "mark2");
        room_test(lm_vis_1_1, world_xyz, "mark3");
    }

    {
        // LM name mapped to[world_azim, elev] for visibility at world(7, 6)
        cv::Point3d world_xyz = { 7.0, -2.0, 6.0 };
        room_test(lm_vis_7_6, world_xyz, "mark2");
        room_test(lm_vis_7_6, world_xyz, "mark3");
    }
}



int main()
{
    //std::cout << "CPOZ Test Application" << std::endl;
    //cpoz::CameraHelper cam;
    //cam.cal(".\\cal_set_2");
    test_room1();
}
