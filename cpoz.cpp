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


static bool is_rec_enabled = false;
static bool is_cal_enabled = false;
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
            case 'c': is_cal_enabled = !is_cal_enabled; break;
            case 'r': is_rec_enabled = !is_rec_enabled; break;
            case 'l': is_loc_enabled = !is_loc_enabled; break;
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


void loop_cal(void)
{
    const int max_good_ct = 20;

    cv::FileStorage cvfs;
    std::vector<std::vector<cv::Vec2f>> vvcal;
    std::vector<std::string> vcalfiles;
    std::set<int> cal_label_set;
    int cal_good_ct = 0;
    int cal_ct = 0;

    cpoz::XYZLandmark lm0({-6, -72.0, 0 });
    cpoz::XYZLandmark lm1({6, -71.5, 0});

    cpoz::CameraHelper cam;
    //cam.load("cal_final.yaml");
    cam.cam_y = -44.0;
    cam.cam_elev = 0.0;

    Size capture_size;
    Point ptmax;

    Mat img_viewer;
    Mat img_gray;
    Mat tmatch;

    cpoz::BGRLandmark bgrm;

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

    cvfs.open("cal_meta.yaml", cv::FileStorage::WRITE);

    // and the image processing loop is running...
    bool is_running = true;

    while (is_running)
    {
        // grab image
        vcap >> img_viewer;

        // combine all channels into grayscale
        cvtColor(img_viewer, img_gray, COLOR_BGR2GRAY);

        // look for landmarks
        std::vector<cpoz::BGRLandmark::landmark_info_t> qinfo;
        bgrm.perform_match(img_viewer, img_gray, tmatch, qinfo);

#ifdef _COLLECT_SAMPLES
        std::cout << bgrm.samp_ct << std::endl;
#endif

        if (true)
        {
            // draw circles around all BGR landmarks and put labels by each one
            // unless about to snap a calibration image which can't have the circles
            cal_label_set.clear();
            for (const auto& r : qinfo)
            {
                cal_label_set.insert(r.code);
                if ((cal_good_ct < (max_good_ct - 3)) || (cal_good_ct >= max_good_ct))
                {
                    char x[2] = { 0 };
                    x[0] = static_cast<char>(r.code) + 'A';
                    circle(img_viewer, r.ctr, 7, (r.diff > 0.0) ? SCA_RED : SCA_BLUE, 3);
                    putText(img_viewer, std::string(x), r.ctr, FONT_HERSHEY_PLAIN, 2.0, SCA_GREEN, 2);
                }
            }

            if (is_loc_enabled)
            {
                if (qinfo.size() == 2)
                {
                    // refine corner positions
                    std::vector<cv::Point2f> vpt;
                    vpt.push_back(qinfo[0].ctr);
                    vpt.push_back(qinfo[1].ctr);
                    cv::TermCriteria tc_corners(
                        cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                        50, // max number of iterations
                        0.0001);
                    cv::cornerSubPix(img_gray, vpt, { 5,5 }, { -1,-1 }, tc_corners);
                    lm0.set_img_xy(vpt[0]);
                    lm1.set_img_xy(vpt[1]);
                    
                    cv::Point3d xyz;
                    double azim;
                    if (lm0.img_xy.x < lm1.img_xy.x)
                    {
                        cam.triangulate_landmarks_ideal(lm0, lm1, xyz, azim);
                    }
                    else
                    {
                        cam.triangulate_landmarks_ideal(lm1, lm0, xyz, azim);
                    }
                    std::cout << xyz.x << ", " << xyz.z << " " << azim * cpoz::RAD2DEG << std::endl;
                }
            }

            // in this loop, use mask flag as calibration grab mode
            // the image is dumped to file if 12 unique landmarks found (3x4 pattern)
            // then user must "hide" some landmarks to trigger another grab
            if (is_cal_enabled)
            {
                if (cal_label_set.size() == 12)
                {
                    if (cal_good_ct < max_good_ct)
                    {
                        cal_good_ct++;
                        if (cal_good_ct == max_good_ct)
                        {
                            // save snapshot of image
                            std::ostringstream osx;
                            osx << CALIB_PATH << "img_" << std::setfill('0') << std::setw(5) << cal_ct << ".png";
                            std::string sfile = osx.str();
                            imwrite(sfile, img_viewer);
                            vcalfiles.push_back(sfile);
                            std::cout << "CALIB. SNAP " << sfile << std::endl;

                            // sort image points by label code
                            std::vector<cv::Vec2f> vimgpts;
                            std::sort(qinfo.begin(), qinfo.end(), cpoz::BGRLandmark::compare_by_code);
                            for (const auto& r : qinfo)
                            {
                                vimgpts.push_back(cv::Vec2f(
                                    static_cast<float>(r.ctr.x),
                                    static_cast<float>(r.ctr.y)));
                            }
                            vvcal.push_back(vimgpts);
                            cal_ct++;
                        }
                    }
                }
                else
                {
                    cal_good_ct = 0;
                }
            }
            else
            {
                // calibration mode turned off
                cal_good_ct = 0;
                cal_ct = 0;
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

    // dump cal data if still in cal mode
    if (is_cal_enabled)
    {
        // the BGRLandmark calibration pattern has 12 corners A-L in ordering shown below
        // so the corners array must be initialized in same order
        // A D G J
        // B E H K
        // C F I L
        std::vector<Point3f> vgridpts;
        double grid_square = 2.25;
        Size board_size(4, 3);
        for (int j = 0; j < board_size.width; j++)
        {
            for (int i = 0; i < board_size.height; i++)
            {
                vgridpts.push_back(cv::Point3f(float(j * grid_square), float(i * grid_square), 0));
            }
        }
        cvfs << "image_size" << capture_size;
        cvfs << "grid_size" << board_size;
        cvfs << "grid_square" << grid_square;
        cvfs << "grid_pts" << vgridpts;
        cvfs << "files" << vcalfiles;
        cvfs << "points" << vvcal;
    }
}



int main()
{
    std::cout << stitle << std::endl;
    //foo();
    //return 0;
    //cpoz::CameraHelper cam;
    //cam.cal(".\\calib");
    test_room1();
    test_room2();
    loop_cal();
}
