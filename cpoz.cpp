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
#include <fstream>
#include <sstream>
#include <iomanip>
#include <set>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/calib3d.hpp>

#include "room_test.h"
#include "room1.h"
#include "room2.h"
#include "BGRLandmark.h"
#include "CameraHelper.h"
#include "XYZLandmark.h"

using namespace cv;

#define CALIB_PATH  ".\\calib\\"    // user may need to create or change this
#define MOVIE_PATH  ".\\movie\\"    // user may need to create or change this
#define DATA_PATH   ".\\data\\"     // user may need to change this

#define SCA_BLACK   (cv::Scalar(0,0,0))
#define SCA_RED     (cv::Scalar(0,0,255))
#define SCA_GREEN   (cv::Scalar(0,255,0))
#define SCA_YELLOW  (cv::Scalar(0,255,255))
#define SCA_BLUE    (cv::Scalar(255,0,0))
#define SCA_WHITE   (cv::Scalar(255,255,255))


static bool is_rec_enabled = false;
static bool is_loc_enabled = false;
const char * stitle = "CPOZ Test Application";
int n_record_ctr = 0;


void test_room1()
{
    std::cout << "==========================================" << std::endl;
    std::cout << "ROOM 1 TEST" << std::endl;

    reset_counts();

    {
        // LM name mapped to [world_azim, elev] for visibility at world (1, 1)
        cv::Point3d world_xyz = { 1.0, -2.0, 1.0 };
        
        // this test has one case with one landmark not visible
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark2");
        
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark3");
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark2", "", 10.0);
        room_test(room1::landmark_maps, room1::lm_vis_1_1, world_xyz, "mark1", "mark3", "", 10.0);
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
        room_test(room2::landmark_maps, room2::lm_vis_5_8, world_xyz, "mark1", "mark2", "markv");
        room_test(room2::landmark_maps, room2::lm_vis_5_8, world_xyz, "mark1", "mark2", "markv", 10.0);
    }

    {
        // LM name mapped to [world_azim, elev] for visibility at world (1, 8)
        // but convert to inches to test scaling
        cv::Point3d world_xyz = { 12.0, -24.0, 96.0 };
        room_test(room2::landmark_maps, room2::lm_vis_1_8, world_xyz, "mark1", "mark2", "markv");
        room_test(room2::landmark_maps, room2::lm_vis_1_8, world_xyz, "mark1", "mark2", "markv", 10.0);
    }

    show_tally();
}


bool wait_and_check_keys()
{
    bool result = true;

    int nkey = waitKey(1);
    char ckey = static_cast<char>(nkey);

    // check that a keypress has been returned
    if (nkey >= 0)
    {
        if (ckey == 27)
        {
            // done if ESC has been pressed
            result = false;
        }
        else
        {
            switch (ckey)
            {
            case 'r': is_rec_enabled = !is_rec_enabled; std::cout << "REC\n"; break;
            case 'l': is_loc_enabled = !is_loc_enabled; std::cout << "LOC\n"; break;
            default: break;
            }
        }
    }

    return result;
}


void image_output(cv::Mat& rimg)
{
    // save each frame to a file if recording
    if (is_rec_enabled)
    {
        std::ostringstream osx;
        osx << MOVIE_PATH << "img_" << std::setfill('0') << std::setw(5) << n_record_ctr << ".png";
        imwrite(osx.str(), rimg);
        n_record_ctr++;

        // red box in upper corner if recording
        rectangle(rimg, { 0, 0, 4, 4 }, SCA_RED, -1);
    }

    imshow(stitle, rimg);
}


void loop(void)
{
    cpoz::XYZLandmark lm0({-6, -72.0, 0 });
    cpoz::XYZLandmark lm1({6, -71.5, 0});

    cpoz::CameraHelper cam;
    cam.load(".\\calib_03\\cal_final.yaml");
    std::map<int, cpoz::BGRLandmark::landmark_info_t> map_pts;

#if 0
    // L-shaped landmark pattern:
    //
    // B F C
    // D
    // G
    std::vector<cv::Vec3f> vaxis = { {154, 0, 0}, {77, 77, 0}, {77, 0, -77} }; // L-corner
    std::vector<cv::Vec3f> vpattpts = { {77,0,0}, {154,0,0}, {0,77,0}, {0,154,0} }; // F, C, D, G
    const int mkr[4] = { 5,2,3,6 };
    const int cix = 5;
    const int numx = 5;
#else
    // Square landmark pattern
    // 
    // A G
    // K E
    std::vector<cv::Vec3f> vaxis = { {77, 0, 0}, {0, 77, 0}, {0, 0, -77} }; // L-corner
    std::vector<cv::Vec3f> vpattpts = { {0,0,0}, {154,0,0}, {154,154,0}, {0,154,0} }; // A, G, E, K
    const int mkr[4] = { 0,6,4,10 };
    const int cix = 0;
    const int numx = 4;
#endif

    cam.cam_y = -44.0;
    cam.cam_elev = 0.0;

    Size capture_size;
    Point ptmax;

    Mat img_viewer_0;
    Mat img_viewer;
    Mat img_gray;
    Mat tmatch;

    cpoz::BGRLandmark bgrm;
    bgrm.init(9, 1.5, 30);

    // need a 0 as argument
    VideoCapture vcap(0);
    if (!vcap.isOpened())
    {
        std::cout << "Failed to open VideoCapture device!" << std::endl;
        ///////
        return;
        ///////
    }

    // camera is ready so grab a first image to determine its full size
    vcap >> img_viewer;
    capture_size = img_viewer.size();

    Mat map1;
    Mat map2;
    initUndistortRectifyMap(
        cam.cam_matrix, cam.dist_coeffs,
        cv::Mat(), cam.cam_matrix, capture_size, CV_32FC1, map1, map2);

    // and the image processing loop is running...
    bool is_running = true;

    while (is_running)
    {
        // grab image
        vcap >> img_viewer_0;
        remap(img_viewer_0, img_viewer, map1, map2, INTER_LINEAR);

        // combine all channels into grayscale
        cvtColor(img_viewer, img_gray, COLOR_BGR2GRAY);

        // look for landmarks
        std::vector<cpoz::BGRLandmark::landmark_info_t> qinfo;
        bgrm.perform_match(img_viewer, img_gray, tmatch, qinfo);

        map_pts.clear();
        for (const auto& r : qinfo)
        {
            map_pts[r.code] = r;
            char x[2] = { 0 };
            x[0] = static_cast<char>(r.code) + 'A';
            circle(img_viewer, r.ctr, 4, (r.diff > 0.0) ? SCA_RED : SCA_BLUE, -1);
            circle(img_viewer, r.ctr, 2, SCA_WHITE, -1);
            putText(img_viewer, std::string(x), r.ctr, FONT_HERSHEY_PLAIN, 2.0, SCA_GREEN, 2);
        }

        if (is_loc_enabled)
        {
            if (qinfo.size() == numx)
            {
                // refine corner positions
                std::vector<cv::Point2f> vpt;
                vpt.push_back(map_pts[mkr[0]].ctr);
                vpt.push_back(map_pts[mkr[1]].ctr);
                vpt.push_back(map_pts[mkr[2]].ctr);
                vpt.push_back(map_pts[mkr[3]].ctr);
                cv::TermCriteria tc_corners(
                    cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                    50, // max number of iterations
                    0.0001);
                cornerSubPix(img_gray, vpt, { 5,5 }, { -1,-1 }, tc_corners);

                Mat rvec;
                Mat tvec;
                bool is_ok = solvePnP(
                    vpattpts, vpt, cam.cam_matrix, cam.dist_coeffs,
                    rvec, tvec, false, SOLVEPNP_P3P);

                if (is_ok)
                {
                    // draw axes just like in Python example found online
                    std::vector<Point2f> ifoo;
                    projectPoints(vaxis, rvec, tvec, cam.cam_matrix, cam.dist_coeffs, ifoo);
                    line(img_viewer, map_pts[cix].ctr, ifoo[0], SCA_GREEN, 5);
                    line(img_viewer, map_pts[cix].ctr, ifoo[1], SCA_BLUE, 5);
                    line(img_viewer, map_pts[cix].ctr, ifoo[2], SCA_RED, 5);
                    std::cout << cv::norm(cv::Mat(tvec), cv::NORM_L2) << ":  ";

                    cv::Mat rot;
                    cv::Rodrigues(rvec, rot);
                    cv::Mat pos = -rot.t() * cv::Mat(tvec);
                    std::cout << "pos = " << pos.t() << std::endl;
                }

                //lm0.set_img_xy(vpt[0]);
                //lm1.set_img_xy(vpt[1]);
                //    
                //cv::Point3d xyz;
                //double azim;
                //if (lm0.img_xy.x < lm1.img_xy.x)
                //{
                //    cam.triangulate_landmarks_ideal(lm0, lm1, xyz, azim);
                //}
                //else
                //{
                //    cam.triangulate_landmarks_ideal(lm1, lm0, xyz, azim);
                //}
                //std::cout << xyz.x << ", " << xyz.z << " " << azim * cpoz::RAD2DEG << std::endl;
            }
        }

        // always show best match contour and target dot on BGR image
        image_output(img_viewer);

        // handle keyboard events and end when ESC is pressed
        is_running = wait_and_check_keys();
    }

    // when everything is done, release the capture device and windows
    vcap.release();
    cv::destroyAllWindows();
}



int main()
{
    std::cout << stitle << std::endl;
    cpoz::CameraHelper cam;
    //cam.cal(".\\calib_02");
    //test_room1();
    //test_room2();
    loop();
}
