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



bool assign_landmarks(
    cpoz::XYZLandmark& rLM1,
    cpoz::XYZLandmark& rLM2,
    const cv::Point3d& rCamXYZ,
    const double azim,
    const double elev)
{
    bool result = true;

    // for the two landmarks:
    // - translate landmark by camera offset
    // - rotate by azimuth and elevation
    // - project into image
    
    // determine pixel location of LM 1
    cv::Point3d xyz1 = rLM1.world_xyz - rCamXYZ;
    cv::Point3d xyz1_rot = cam.calc_xyz_after_rotation(xyz1, elev, azim, 0);
    cv::Point2d img_xy1 = cam.project_xyz_to_img_xy(xyz1_rot);

    // determine pixel location of LM 2
    cv::Point3d xyz2 = rLM2.world_xyz - rCamXYZ;
    cv::Point3d xyz2_rot = cam.calc_xyz_after_rotation(xyz2, elev, azim, 0);
    cv::Point2d img_xy2 = cam.project_xyz_to_img_xy(xyz2_rot);

    // dump the image X,Y points and perform visibility check
    std::cout << std::endl;
    std::cout << "Image Landmark 1:  " << img_xy1 << std::endl;
    std::cout << "Image Landmark 2:  " << img_xy2 << std::endl;
    if (cam.is_visible(img_xy1) && cam.is_visible(img_xy2))
    {
        std::cout << "Both landmarks are visible.  ";
        rLM1.set_img_xy(img_xy1);
        rLM2.set_img_xy(img_xy2);
    }
    else
    {
        std::cout << "********** At least one landmark is NOT visible!!!" << std::endl;
        result = false;
    }

    // update landmark image coords
    return result;
}



void foo()
{
    // X,Z and azim are unknowns that must be solved
    cv::Point3d cam_xyz({ 24, -48, 120 });
    double azim = 220 * cpoz::DEG2RAD;
    double elev = 20 * cpoz::DEG2RAD;
    //double elev = atan(48.0 / 120.0);// *cpoz::DEG2RAD;

    // let landmarks ALWAYS have same Y and be above camera
    cpoz::XYZLandmark lm1({ 0, -84, 0 });
    cpoz::XYZLandmark lm2({ 0, -96, 0 });

    (void) assign_landmarks(lm1, lm2, cam_xyz, azim, elev);

    //double a1 = sqrt(24 * 24 + 120 * 120 + 48 * 48);
    //double a2 = sqrt(36 * 36 + 120 * 120 + 48 * 48);
    //double aa1 = sqrt(24 * 24 + 120 * 120);
    //double aa2 = sqrt(36 * 36 + 120 * 120);
    //double cos_C = ((a1 * a1) + (a2 * a2) - (12 * 12)) / (2 * a1 * a2);
    //double ang_C = acos(cos_C) * cpoz::RAD2DEG;
    double a1 = sqrt(24 * 24 + 120 * 120 + 48 * 48);
    double a2 = sqrt(24 * 24 + 120 * 120 + 36 * 36);
    double aa1 = sqrt(24 * 24 + 120 * 120);
    double aa2 = sqrt(24 * 24 + 120 * 120);
    double cos_C = ((a1 * a1) + (a2 * a2) - (12 * 12)) / (2 * a1 * a2);
    double ang_C = acos(cos_C) * cpoz::RAD2DEG;

    // guesses
    cpoz::CameraHelper cam;
    double pos_elev = elev;// 10 * cpoz::DEG2RAD;
    cam.cam_elev = pos_elev;
    cam.cam_y = cam_xyz.y;// -48;

    cpoz::CameraHelper::T_TRIANG_SOL sol;
    if (lm1.img_xy.x < lm2.img_xy.x)
    {
        cam.triangulate_landmarks(lm1, lm2, sol);
    }
    else
    {
        cam.triangulate_landmarks(lm2, lm1, sol);
    }

    std::cout << (sol.ang_180_err) * cpoz::RAD2DEG << std::endl;
    std::cout << sol.ang_ABC * cpoz::RAD2DEG << ", ";
    std::cout << sol.len_abc << std::endl;
    std::cout << sol.ang0_ABC * cpoz::RAD2DEG << ", ";
    std::cout << sol.len0_abc << std::endl;
    std::cout << sol.ang1_ABC * cpoz::RAD2DEG << ", ";
    std::cout << sol.len1_abc << std::endl;

    std::cout << sol.gnd_rng_to_LM1 << std::endl;
    std::cout << sol.gnd_rng_to_LM2 << std::endl;
    std::cout << sol.a1 * cpoz::RAD2DEG << std::endl;
    std::cout << sol.a2 * cpoz::RAD2DEG << std::endl;
    std::cout << sol.a3 * cpoz::RAD2DEG << std::endl;
    std::cout << sol.cam_xyz << std::endl;

    cv::Point3d pos_xyz;
    double pos_azim;
    cam.triangulate_landmarks_ideal(lm1, lm2, pos_xyz, pos_azim);
    std::cout << "Robot is at: [" << pos_xyz.x << ", " << pos_xyz.z << "] @ " << pos_azim * cpoz::RAD2DEG << std::endl;

    cv::Point2d foopt;

    std::cout << "foo" << std::endl;
}


bool landmark_test(
    cpoz::XYZLandmark& lm1,
    cpoz::XYZLandmark& lm2,
    const cv::Point3d& cam_xyz,
    const cv::Vec2d& cam_angs_rad,
    cv::Point3d& pos_xyz,
    double& world_azim)
{
    test_ct++;
    if (!assign_landmarks(lm1, lm2, cam_xyz, cam_angs_rad[0], cam_angs_rad[1]))
    {
        return false;
    }
    
    // all is well so assign camera elev and known Y
    cam.cam_elev = cam_angs_rad[1];
    cam.cam_y = cam_xyz.y;

    // landmark with smallest img X is always first parameter
    if (lm1.img_xy.x < lm2.img_xy.x)
    {
        cam.triangulate_landmarks_ideal(lm1, lm2, pos_xyz, world_azim);
    }
    else
    {
        cam.triangulate_landmarks_ideal(lm2, lm1, pos_xyz, world_azim);
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
