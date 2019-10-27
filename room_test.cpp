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
#include <opencv2/calib3d.hpp>
#include "room1.h"
#include "CameraHelper.h"
#include "XYZLandmark.h"


const double EPS = 0.0001;
static int pass_ct = 0;
static int test_ct = 0;
static cpoz::CameraHelper cam;



bool assign_landmarks(
    cpoz::CameraHelper& rcam,
    std::vector<cpoz::XYZLandmark>& rvLM,
    const cv::Point3d& rCamXYZ,
    const double azim,
    const double elev)
{
    // - translate landmark by camera offset
    // - rotate by azimuth and elevation
    // - project into image
    bool result = true;
    for (size_t i = 0; i < rvLM.size(); i++)
    {
        cv::Point3d xyz = rvLM[i].world_xyz - rCamXYZ;
        cv::Point3d xyz_rot = rcam.calc_xyz_after_rotation(xyz, elev, azim, 0);
        rvLM[i].img_xy = rcam.project_xyz_to_img_xy(xyz_rot);
        result = result && cam.is_visible(rvLM[i].img_xy);
        std::cout << "Image Landmark " << i << ":  " << rvLM[i].img_xy << std::endl;
    }

    if (result)
    {
        std::cout << "All landmarks are visible.  ";
    }
    else
    {
        std::cout << "********** At least one landmark is NOT visible!!!" << std::endl;
    }

    return result;
}



void foo()
{
    // X,Z and azim are unknowns that must be solved
//    cv::Point3d cam_xyz({ 24, -48, 120 });
    cv::Point3d cam_xyz({ 24, -48, -120 });

    double r = sqrt(24 * 24 + 120 * 120);
    double raz = -(atan2(-120, -24) - (CV_PI / 2)) * cpoz::RAD2DEG;
    double rel = atan(48.0 / r) * cpoz::RAD2DEG;
    double azim = 0 * cpoz::DEG2RAD;
    double elev = 5 * cpoz::DEG2RAD;

    // let landmarks ALWAYS have same Y and be above camera ???
    cpoz::XYZLandmark lm0({ 12, -96, 0 });
    cpoz::XYZLandmark lm1({ 0, -96, 0 });
    cpoz::XYZLandmark lm2({ 0, -84, 0 });
    cpoz::XYZLandmark lm3({ 12, -84, 0 });
    //cpoz::XYZLandmark lm0({ 6, 6, 0 });
    //cpoz::XYZLandmark lm1({ -6, 6, 0 });
    //cpoz::XYZLandmark lm2({ -6, -6, 0 });
    //cpoz::XYZLandmark lm3({ 6, -6, 0 });

    //for (int i = 0; i < 5; i++)
    //{
        std::cout << "---------------------\n";
        std::vector<cpoz::XYZLandmark> vlm;
        vlm.push_back(lm0);
        vlm.push_back(lm1);
        vlm.push_back(lm2);
        vlm.push_back(lm3);
        (void)assign_landmarks(cam, vlm, cam_xyz, azim, elev);

        //lm0.world_xyz.y += (90);
        //lm1.world_xyz.y += (90);
        //lm2.world_xyz.y += (90);
        //lm3.world_xyz.y += (90);
        //lm0.world_xyz.x -= 6;
        //lm1.world_xyz.x -= 6;
        //lm2.world_xyz.x -= 6;
        //lm3.world_xyz.x -= 6;

        std::vector<cv::Point3f> obj_pts;
        std::vector<cv::Point2f> img_pts;
        obj_pts.push_back({ (float)lm0.world_xyz.x, (float)lm0.world_xyz.y, (float)lm0.world_xyz.z });
        obj_pts.push_back({ (float)lm1.world_xyz.x, (float)lm1.world_xyz.y, (float)lm1.world_xyz.z });
        obj_pts.push_back({ (float)lm2.world_xyz.x, (float)lm2.world_xyz.y, (float)lm2.world_xyz.z });
        img_pts.push_back({ (float)lm0.img_xy.x, (float)lm0.img_xy.y });
        img_pts.push_back({ (float)lm1.img_xy.x, (float)lm1.img_xy.y });
        img_pts.push_back({ (float)lm2.img_xy.x, (float)lm2.img_xy.y });

        {
            std::cout << "####\n";
            cpoz::CameraHelper cam;
            std::vector<cv::Mat> rvecs;
            std::vector<cv::Mat> tvecs;
            cv::solveP3P(obj_pts, img_pts, cam.cam_matrix, cam.dist_coeffs, rvecs, tvecs, cv::SOLVEPNP_P3P);
            for (auto& each : rvecs)
                std::cout << each << std::endl;
            std::cout << "----\n";
            for (auto& each : tvecs)
            {
                std::cout << each << std::endl;
                std::cout << cv::norm(each, cv::NORM_L2) << std::endl;
            }
            std::vector<cv::Point2f> fug; 
            cv::projectPoints(obj_pts, rvecs[0], tvecs[0], cam.cam_matrix, cam.dist_coeffs, fug);
            for (auto& each : fug)
                std::cout << each << std::endl;
            cv::projectPoints(obj_pts, rvecs[1], tvecs[1], cam.cam_matrix, cam.dist_coeffs, fug);
            for (auto& each : fug)
                std::cout << each << std::endl;
        }

        obj_pts.push_back({ (float)lm3.world_xyz.x, (float)lm3.world_xyz.y, (float)lm3.world_xyz.z });
        img_pts.push_back({ (float)lm3.img_xy.x, (float)lm3.img_xy.y });

        {
            std::cout << "####\n";
            cpoz::CameraHelper cam;
            cv::Mat rvecs;
            cv::Mat tvecs;
            cv::solvePnP(obj_pts, img_pts, cam.cam_matrix, cam.dist_coeffs, rvecs, tvecs);
            std::cout << rvecs << std::endl;
            std::cout << tvecs << std::endl;
            std::cout << cv::norm(tvecs, cv::NORM_L2) << std::endl;
            std::vector<cv::Point2f> fug;
            cv::projectPoints(obj_pts, rvecs, tvecs, cam.cam_matrix, cam.dist_coeffs, fug);
            for (auto& each : fug)
                std::cout << each << std::endl;
        }

        //elev += 5 * cpoz::DEG2RAD;
        //cam_xyz.x += 4;
    //}


    //{
    //    std::cout << "####\n";
    //    cpoz::CameraHelper cam;
    //    std::vector<cv::Mat> rvecs;
    //    std::vector<cv::Mat> tvecs;
    //    cv::solveP3P(obj_pts, img_pts, cam.cam_matrix, cam.dist_coeffs, rvecs, tvecs, cv::SOLVEPNP_AP3P);
    //    for (auto& each : rvecs)
    //        std::cout << each << std::endl;
    //    std::cout << "----\n";
    //    for (auto& each : tvecs)
    //        std::cout << each << std::endl;
    //}

    //double a1 = sqrt(24 * 24 + 120 * 120 + 48 * 48);
    //double a2 = sqrt(36 * 36 + 120 * 120 + 48 * 48);P
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
    double pos_azim = azim;
    cam.cam_elev = pos_elev;
    cam.cam_y = cam_xyz.y;// -48;


    cv::Point3d pt00 = { 0,0,100 };
    cv::Point3d pt01 = cam.calc_xyz_after_rotation(pt00, elev, azim, 0.0);
    cv::Point3d pt02 = cam.calc_xyz_after_rotation(pt01, elev, azim, 0.0, true);
    cv::Point3d pt11 = cam.calc_xyz_after_rotation(pt00, 0, azim, 0.0);
    cv::Point3d pt12 = cam.calc_xyz_after_rotation(pt11, elev, 0, 0.0);


    cpoz::CameraHelper::T_TRIANG_SOL sol;
    sol.ang0_ABC[0] = elev;
    sol.ang0_ABC[1] = azim;
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
//    double pos_azim;
    cam.triangulate_landmarks_ideal(lm1, lm2, pos_xyz, pos_azim);
    std::cout << "Robot is at: [" << pos_xyz.x << ", " << pos_xyz.z << "] @ " << pos_azim * cpoz::RAD2DEG << std::endl;

    cv::Point2d foopt;

    std::cout << "foo" << std::endl;
}



bool landmark_test(
    std::vector<cpoz::XYZLandmark> vlm,
    const cv::Point3d& cam_xyz,
    const cv::Vec2d& cam_angs_rad,
    cv::Point3d& pos_xyz,
    double& world_azim)
{
    test_ct++;
    if (!assign_landmarks(cam, vlm, cam_xyz, cam_angs_rad[0], cam_angs_rad[1]))
    {
        return false;
    }
    
    // all is well so assign camera elev and known Y
    cam.cam_elev = cam_angs_rad[1];
    cam.cam_y = cam_xyz.y;

    // landmark with smallest img X is always first parameter
    if (vlm[0].img_xy.x < vlm[1].img_xy.x)
    {
        cam.triangulate_landmarks_ideal(vlm[0], vlm[1], pos_xyz, world_azim);
    }
    else
    {
        cam.triangulate_landmarks_ideal(vlm[1], vlm[0], pos_xyz, world_azim);
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
    const std::string& lm_map_name_C,
    const double elev_offset)
{
    std::cout << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "ROOM TEST ";
    std::cout << lm_map_name_A << ", " << lm_map_name_B << ", " << known_cam_xyz;
    std::cout << ", " << elev_offset << std::endl;
    
    for (const auto& r : lm_vis)
    {
        const std::string& rkey = r.first;

        double cam_azim_deg = r.second[0];
        double cam_elev_deg = r.second[1] + elev_offset;
        cv::Vec2d cam_angs_rad = { cam_azim_deg * cpoz::DEG2RAD, cam_elev_deg * cpoz::DEG2RAD };

        std::vector<cpoz::XYZLandmark> vlm;
        const tMapStrToXYZ& marka = lm_maps.find(lm_map_name_A)->second;
        const tMapStrToXYZ& markb = lm_maps.find(lm_map_name_B)->second;
        cpoz::XYZLandmark lm1(marka.find(rkey)->second);
        cpoz::XYZLandmark lm2(markb.find(rkey)->second);
        vlm.push_back(lm1);
        vlm.push_back(lm2);
        if (lm_map_name_C.size())
        {
            const tMapStrToXYZ& markc = lm_maps.find(lm_map_name_C)->second;
            cpoz::XYZLandmark lm3(markc.find(rkey)->second);
            vlm.push_back(lm3);
        }

        bool result = true;
        cv::Point3d pos_xyz;
        double world_azim;
        bool flag = landmark_test(vlm, known_cam_xyz, cam_angs_rad, pos_xyz, world_azim);

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
