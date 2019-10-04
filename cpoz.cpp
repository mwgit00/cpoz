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
    const cv::Vec3d& cam_xyz,
    const cv::Vec2d& cam_angs_rad,
    cv::Vec2d& pos_xz,
    double& world_azim)
{
    // for the two landmarks:
    // - translate landmark by camera offset
    // - rotate by azimuth and elevation
    // - project into image
    
    double azim = cam_angs_rad[0];
    double elev = cam_angs_rad[1];
    
    // determine pixel location of fixed LM 1
    cv::Vec3d xyz1 = lm1.xyz - cam_xyz;
    cv::Vec3d xyz1_rot = cam.calc_xyz_after_rotation(xyz1, elev, azim, 0);
    cv::Vec2d uv1 = cam.project_xyz_to_uv(xyz1_rot);

    // determine pixel location of left/right LM
    cv::Vec3d xyz2 = lm2.xyz - cam_xyz;
    cv::Vec3d xyz2_rot = cam.calc_xyz_after_rotation(xyz2, elev, azim, 0);
    cv::Vec2d uv2 = cam.project_xyz_to_uv(xyz2_rot);

    // dump the U,V points and perform visibility check
    std::cout << std::endl;
    std::cout << "Image Landmark 1:  " << uv1 << std::endl;
    std::cout << "Image Landmark 2:  " << uv2 << std::endl;
    if (cam.is_visible(uv1) && cam.is_visible(uv2))
    {
        std::cout << "Both landmarks are visible." << std::endl;
    }
    else
    {
        std::cout << "At least one landmark is NOT visible!" << std::endl;
        return false;
        //return False, 0., 0., 0.
    }
    
    // all is well so proceed with test...

    // landmarks have been acquired
    // camera elevation and world Y also need updating
    cam.elev = elev;
    cam.world_y = cam_xyz[1];

    lm1.set_current_uv(uv1);
    lm2.set_current_uv(uv2);

    double r;
    double ang;
    double rel_azim;
    cam.triangulate(lm1.xyz, lm2.xyz, uv1, uv2, r, ang, rel_azim);

    // landmark has angle offset info
    // which is used to calculate world coords and azim
    double u2 = lm2.uv[0];
    world_azim = lm1.calc_world_azim(u2, ang, rel_azim);
    pos_xz = lm1.calc_world_xz(u2, ang, r);

    std::cout << "Robot is at: [" << pos_xz[0] << ", " << pos_xz[1] << "] @ " << world_azim * cpoz::RAD2DEG << std::endl;
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
    const cv::Vec3d& known_cam_xyz,
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

        cv::Vec2d pos_xz;
        double world_azim;

        const std::string& rkey = r.first;
        cpoz::XYZLandmark lm1(mark1[rkey].xyz, mark1[rkey].adj_R, mark1[rkey].adj_L);
        cpoz::XYZLandmark lm2(markx[rkey].xyz, markx[rkey].adj_R, markx[rkey].adj_L);

        bool result = true;
        bool flag = landmark_test(lm1, lm2, known_cam_xyz, cam_angs_rad, pos_xz, world_azim);

        if (!flag)
        {
            result = false;
        }

        if (abs(pos_xz[0] - known_cam_xyz[0]) >= EPS)
        {
            result = false;
        }

        if (abs(pos_xz[1] - known_cam_xyz[2]) >= EPS)
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
        cv::Vec3d xyz = { 1.0, -2.0, 1.0 };
        room_test(lm_vis_1_1, xyz, "mark2");  // one case with one landmark not visible
        room_test(lm_vis_1_1, xyz, "mark3");
        room_test(lm_vis_1_1, xyz, "mark2", 10.0);
        room_test(lm_vis_1_1, xyz, "mark3", 10.0);
    }

    {
        // LM name mapped to[world_azim, elev] for visibility at world (1, 1)
        cv::Vec3d xyz = { 1.0, -3.0, 1.0 };
        room_test(lm_vis_1_1, xyz, "mark2");
        room_test(lm_vis_1_1, xyz, "mark3");
    }

    {
        // LM name mapped to[world_azim, elev] for visibility at world(7, 6)
        cv::Vec3d xyz = { 7.0, -2.0, 6.0 };
        room_test(lm_vis_7_6, xyz, "mark2");
        room_test(lm_vis_7_6, xyz, "mark3");
    }
}



int main()
{
    std::cout << "CPOZ Test Application" << std::endl;
    cpoz::CameraHelper cam;
    cam.cal(".\\cal_set_2");
    test_room1();
}
