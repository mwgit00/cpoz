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

#include "FakeLidar.h"
#include "GHSLAM.h"

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
#define SCA_DKGRAY  (cv::Scalar(64,64,64))


// rotation velocity settings corresponding to keypresses (degrees per frame)
static const std::vector<double> g_velcomp = {
-2.0, -1.5, -0.7, -0.2,
 0.0,
 0.2,  0.7,  1.5,  2.0,
 0.0, };

#define RO_VEL_FORWARD  (4)
#define RO_VEL_STOP     (9)



static bool is_rec_enabled = false;
static bool is_loc_enabled = false;
static int iivel = RO_VEL_STOP;
static bool is_resync = true;   // resync map (do on first iteration)
static bool is_rehome = true;   // new position (do on first iteration)
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


bool wait_and_check_keys(const int delay_ms = 1)
{
    bool result = true;

    int nkey = waitKey(delay_ms);
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
            //case 'l': is_loc_enabled = !is_loc_enabled; std::cout << "LOC\n"; break;
            case 'a': iivel = 0; break;
            case 's': iivel = 1; break; 
            case 'd': iivel = 2; break;
            case 'f': iivel = 3; break;
            case 'g': iivel = 4; break;
            case 'h': iivel = 5; break;
            case 'j': iivel = 6; break;
            case 'k': iivel = 7; break;
            case 'l': iivel = 8; break;
            case 'b': iivel = 9; break;
            case '=': is_resync = true; break;
            case '0': is_rehome = true; break;
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


static cv::Point2d get_cos_sin(const double ang_deg)
{
    double c = cos(ang_deg * CV_PI / 180.0);
    double s = sin(ang_deg * CV_PI / 180.0);
    return { c, s };
}


void vroom(void)
{
    Mat img_orig;
    Mat img_viewer;
    Mat img_viewer_bgr;
    int ticker = 0;

    cpoz::GHSLAM ghslam;
    cpoz::FakeLidar lidar;

    Point slam_offset = { 0,0 };
    double slam_angle = 0.0;

    Point match_offset = { 0, 0 };
    double match_angle = 0.0;

    // various starting positions in default floorplan
    std::vector<Point2d> vhomepos;
    //vhomepos.push_back({ 870.0, 360.0 });
    //vhomepos.push_back({ 930.0, 630.0 });
    //vhomepos.push_back({ 1140.0, 110.0 });
    //vhomepos.push_back({ 1180.0, 440.0 });
    vhomepos.push_back({ 560.0, 360.0 });
    vhomepos.push_back({ 560.0, 540.0 });
    vhomepos.push_back({ 650.0, 140.0 });
    size_t iivhomepos = 0;

    // robot state
    Point2d botpos = { 0.0, 0.0 };
    double botang = 0.0;
    Point2d botvel = { 0.0, 0.0 };
    double botvelmag = 0.0;
    double botangvel = 0.0;

    Point pt_drawing_offset;

#if 0
    // really noisy LIDAR
    lidar.jitter_angle_deg_u = 0.5;
    lidar.jitter_range_cm_u = 4.0;
    lidar.jitter_sync_deg_u = 0.5;
#endif

    lidar.set_scan_angs(ghslam.get_scan_angs());
    lidar.load_floorplan(".\\docs\\apt_1cmpp_720p.png");

    img_orig = imread(".\\docs\\apt_1cmpp_720p.png", IMREAD_GRAYSCALE);

    // and the image processing loop is running...
    bool is_running = true;

    while (is_running)
    {
        // handle keyboard events and end when ESC is pressed
        is_running = wait_and_check_keys(25);

        if ((iivel == RO_VEL_FORWARD) || (iivel == RO_VEL_STOP))
        {
            botvelmag = (iivel == RO_VEL_FORWARD) ? 1.5 : 0.0;
            botangvel = 0.0;
        }
        else
        {
            // spin in place
            botvelmag = 0.0;
            botangvel = g_velcomp[iivel];
        }

        Point2d dpos = get_cos_sin(botang) * botvelmag;
        botpos += dpos;
        botang += botangvel;
        if (botang >= 360.0) botang -= 360.0;
        if (botang < 0.0) botang += 360.0;

        if (is_rehome)
        {
            // one-shot keypress
            is_rehome = false;

            // reset offset for drawing map info
            pt_drawing_offset = {
                static_cast<int>(vhomepos[iivhomepos].x),
                static_cast<int>(vhomepos[iivhomepos].y) };
            
            // clear map and start at 0
            // and set flag for a resync to add first waypoint to new map
            ghslam.m_waypoints.clear();
            slam_offset = { 0,0 };
            slam_angle = 0.0;
            match_offset = { 0, 0 };
            match_angle = 0.0;
            is_resync = true;

            // stop robot
            iivel = RO_VEL_STOP;
            botpos = vhomepos[iivhomepos];
            botang = 0.0;
            botvel = { 0.0, 0.0 };
            botvelmag = 0.0;
            botangvel = 0.0;

            iivhomepos++;
            if (iivhomepos == vhomepos.size()) iivhomepos = 0;
        }

        lidar.set_pos(botpos);
        lidar.set_ang(botang);
        lidar.run_scan();

        ghslam.preprocess_scan(61 / 2, lidar.get_last_scan());
        
        std::vector<Point> vpt;
        std::vector<cv::Point> last_scan_xy;        ///< last result from run_scan (relative x,y)

        //if ((ticker % 10) == 0)
        if (is_resync)
        {
            // apply latest scan as new waypoint
            ghslam.update_scan_templates(lidar.get_last_scan());

            Point p0 = match_offset;
            Point roffset;
            double rang_rad = (slam_angle) * CV_PI / 180.0;
            double cos0 = cos(rang_rad);
            double sin0 = sin(rang_rad);
#if 0
            roffset.x = static_cast<int>(p0.x * cos0 - p0.y * sin0);
            roffset.y = static_cast<int>(p0.x * sin0 + p0.y * cos0);
#else
            roffset.x = static_cast<int>(p0.x * cos0 + p0.y * sin0);
            roffset.y = static_cast<int>(-p0.x * sin0 + p0.y * cos0);
#endif

            slam_offset += roffset;// match_offset;
            slam_angle += match_angle;

            ghslam.m_waypoints.push_back({ slam_offset, slam_angle, lidar.get_last_scan() });
            is_resync = false;
        }

        //ghslam.perform_match(lidar.get_last_scan(), match_offset, match_angle);

#if 0
        if ((abs(slam_offset.x) > 3) || (abs(slam_offset.y) > 3) || (abs(slam_angle) > 3.0))
        {
            is_resync = true;
        }
#endif

        // init image output with source floorplan
        img_orig.copyTo(img_viewer);
#if 1
        Mat foo;
        Point foopt;
        ghslam.draw_preprocessed_scan(foo, foopt);

        // show latest LIDAR scan in upper left
        Rect mroi = { {0,0}, foo.size() };
        foo.copyTo(img_viewer(mroi));

        // show latest 0 degree template in middle left
        Rect mroi0 = { {0,410}, ghslam.m_img_template_ang_0.size() };
        //ghslam.m_img_template_ang_0.copyTo(img_viewer(mroi0));
#endif

        // switch to BGR...
        cvtColor(img_viewer, img_viewer_bgr, COLOR_GRAY2BGR);
        
        std::vector<double> vang;
        vang.resize(lidar.get_last_scan().size());
        for (size_t nn = 0; nn < vang.size(); nn++)
        {
            vang[nn] = ghslam.m_preproc[nn].ang;
            if (ghslam.m_preproc[nn].flags == 0)
            {
                vang[nn] = -1.0;
            }
        }
        
        // draw LIDAR scan lines over floorplan
        lidar.draw_last_scan(img_viewer_bgr, vang, SCA_DKGRAY);
#if 1
        // draw robot position and direction in LIDAR scan in upper left
        circle(img_viewer_bgr, foopt, 3, SCA_GREEN, -1);
        line(img_viewer_bgr, foopt, foopt + Point{ 10, 0 }, SCA_GREEN, 1);

        // draw robot position and direction in template in middle left
        Point t0 = ghslam.m_pt0_template_ang_0 + Point{ 0, 410 };
        circle(img_viewer_bgr, t0, 3, SCA_GREEN, -1);
        line(img_viewer_bgr, t0, t0 + Point{ 10, 0 }, SCA_GREEN, 1);
#endif
        // now draw robot in floorplan
        const int r = 20;
        Point ibotpos = botpos;
        circle(img_viewer_bgr, ibotpos, r, SCA_WHITE, -1);
        circle(img_viewer_bgr, ibotpos, r, SCA_BLACK, 1);
        circle(img_viewer_bgr, ibotpos, 7, SCA_GREEN, -1);
        Point2d angd = get_cos_sin(botang);
        int angx = static_cast<int>(angd.x * r * 0.8);
        int angy = static_cast<int>(angd.y * r * 0.8);
        line(img_viewer_bgr, ibotpos, ibotpos + Point{ angx, angy }, SCA_GREEN, 3);

        // lastly draw map stuff
        double rescale = 1.0 / ghslam.get_match_scale();
        for (const auto& r : ghslam.get_waypoints())
        {
            Point qpt = r.pt;
            qpt.x = static_cast<int>(rescale * qpt.x);
            qpt.y = static_cast<int>(rescale * qpt.y);
            circle(img_viewer_bgr, qpt + pt_drawing_offset, 4, SCA_BLUE, -1);
        }

        {
            std::ostringstream oss;
            oss << " IMG:XY = " << std::setw(4) << ibotpos.x << ", " << ibotpos.y;
            oss << "  " << std::fixed << std::setprecision(1) << botang;
            putText(img_viewer_bgr, oss.str(), { 0, 360 }, FONT_HERSHEY_PLAIN, 2.0, SCA_BLACK, 2);
        }

        {
            std::ostringstream oss;
            oss << " MATCH = " << std::setw(4) << match_offset.x << ", " << match_offset.y;
            oss << "  " << std::fixed << std::setprecision(1) << match_angle;
            putText(img_viewer_bgr, oss.str(), { 0, 385 }, FONT_HERSHEY_PLAIN, 2.0, SCA_BLUE, 2);
        }

        // show the BGR image
        image_output(img_viewer_bgr);
    }

    // when everything is done, release the capture device and windows
    cv::destroyAllWindows();
}




int main()
{
    std::cout << stitle << std::endl;
    //cpoz::CameraHelper cam;
    //cam.cal(".\\calib_02");
    //test_room1();
    //test_room2();
    //loop();

    vroom();
    return 0;
}
