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


const double EPS = 0.0001;
static int pass_ct = 0;
static int test_ct = 0;
static cpoz::CameraHelper cam;


bool landmark_test(
    cpoz::XYZLandmark& lm1,
    cpoz::XYZLandmark& lm2,
    const cv::Point3d& cam_xyz,
    const cv::Vec2d& cam_angs_rad,
    cv::Point3d& pos_xyz,
    double& world_azim)
{
    test_ct++;
    
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
        std::cout << "Both landmarks are visible.  ";
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

    // update landmark image coords
    lm1.set_img_xy(img_xy1);
    lm2.set_img_xy(img_xy2);

    // landmark with smallest img X is always first parameter
    if (img_xy1.x < img_xy2.x)
    {
        cam.triangulate_landmarks(lm1, lm2, pos_xyz, world_azim);
    }
    else
    {
        cam.triangulate_landmarks(lm2, lm1, pos_xyz, world_azim);
    }

    std::cout << "Robot is at: [" << pos_xyz.x << ", " << pos_xyz.z << "] @ " << world_azim * cpoz::RAD2DEG << std::endl;
    return true;
}



void room_test(
    const tMapStrToMapStrToXYZ& lm_maps,
    const tMapStrToAZEL& lm_vis,
    const cv::Point3d& known_cam_xyz,
    const std::string& lm_map_name_A,
    const std::string& lm_map_name_B,
    const double elev_offset)
{
    std::cout << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "ROOM TEST ";
    std::cout << lm_map_name_A << ", " << lm_map_name_B << ", " << known_cam_xyz;
    std::cout << ", " << elev_offset << std::endl;
    
    for (const auto& r : lm_vis)
    {
        double cam_azim_deg = r.second[0];
        double cam_elev_deg = r.second[1] + elev_offset;
        cv::Vec2d cam_angs_rad = { cam_azim_deg * cpoz::DEG2RAD, cam_elev_deg * cpoz::DEG2RAD };

        const tMapStrToXYZ& marka = lm_maps.find(lm_map_name_A)->second;
        const tMapStrToXYZ& markb = lm_maps.find(lm_map_name_B)->second;

        cv::Point3d pos_xyz;
        double world_azim;

        const std::string& rkey = r.first;
        cpoz::XYZLandmark lm1(marka.find(rkey)->second);
        cpoz::XYZLandmark lm2(markb.find(rkey)->second);

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

        // need to handle occasional ~360 angle in azimuth test
        world_azim *= cpoz::RAD2DEG;
        if ((abs(world_azim - cam_azim_deg) >= EPS) && (abs(world_azim - 360.0 - cam_azim_deg) >= EPS))
        {
            result = false;
        }

        if (result)
        {
            std::cout << "PASS!!!" << std::endl;
            pass_ct++;
        }
    }
}


void reset_counts()
{
    pass_ct = 0;
    test_ct = 0;
}


void show_tally()
{
    std::cout << std::endl;
    std::cout << "TALLY: " << pass_ct << "/" << test_ct << std::endl;
}
