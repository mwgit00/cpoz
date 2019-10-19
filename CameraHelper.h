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

#ifndef CAMERA_HELPER_H_
#define CAMERA_HELPER_H_

#include <opencv2/imgproc.hpp>
#include "XYZLandmark.h"


namespace cpoz
{
    // camera convention
    //
    // 0 -------- - +X -->
    // |           |
    // |  (cx,cy)  |
    // |           |
    // +Y --------- (w,h)
    // |
    // V
    //
    // right-hand rule for Z
    // -Z is pointing into camera, +Z is pointing away from camera
    // +X (fingers) cross +Y (palm) will make +Z (thumb) point away from camera
    //
    // positive elevation is clockwise rotation around X (axis pointing "out")
    // positive azimuth is clockwise rotation around Y (axis pointing "out")
    // +elevation TO point (U,V) is UP
    // +azimuth TO point (U,V) is RIGHT
    //
    // robot camera is always "looking" in its +Z direction
    // so its world azimuth is 0 when robot is pointing in +Z direction
    // since that is when the two coordinate systems line up
    //
    // world location is in X,Z plane
    // normally the +X axis in X,Z plane would be an angle of 0
    // but there is a 90 degree rotation between X,Z and world azimuth
    //
    // roll,pitch,yaw is also phi,theta,psi
    //
    // https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    // "The industry standard is Z-Y-X because that corresponds to yaw, pitch and roll."

    class CameraHelper
    {
    public:

        typedef struct _T_TRIANG_SOL_struct
        {
            double ang_180_err;
            cv::Vec3d ang_ABC;
            cv::Vec3d len_abc;
            cv::Vec3d ang0_ABC;
            cv::Vec3d len0_abc;
            cv::Vec3d ang1_ABC;
            cv::Vec3d len1_abc;
            cv::Vec3d gnd_rng_to_LM1;
            cv::Vec3d gnd_rng_to_LM2;
        } T_TRIANG_SOL;

        CameraHelper();
        virtual ~CameraHelper();

        static void cal(const std::string& rs);

        static cv::Mat calc_axes_rotation_mat(const double roll, const double pitch, const double yaw);
        static cv::Point3d calc_xyz_after_rotation(const cv::Point3d& xyz_pos, const double roll, const double pitch, const double yaw);

        // load calibration data
        bool load(const std::string& rscal);
        
        // test if pixel at (X,Y) is within FOV
        bool is_visible(const cv::Point2d& rImgXY) const;

        // project 3D world point (X,Y,Z) to image plane (X,Y)
        // this is mainly just for testing the triangulation
        cv::Point2d project_xyz_to_img_xy(const cv::Point3d& rXYZ) const;


        // determine normalized 3D coordinates (X,Y,1) based on image coordinates
        cv::Point3d undistort_img_xy_to_xyz(const cv::Point2d& rImgXY) const;

        // Use known XYZ of two landmarks and their image coordinates to perform triangulation.
        // param: rLM1 Landmark 1 info
        // param: rLM2 Landmark 2 info
        // param: rsol Solution
        void CameraHelper::triangulate_landmarks(
            const XYZLandmark& rLM1,
            const XYZLandmark& rLM2,
            T_TRIANG_SOL& rsol) const;


        // calculate vector from camera to world point (X,Y,Z)
        // given its known world Y, image coords (X,Y), and camera elevation
        cv::Point3d calc_cam_to_xyz(
            const double known_Y,
            const cv::Point2d& rImgXY,
            const double cam_elev_rad);

        // Use known Y of landmarks A and B and their image coordinates to perform triangulation.
        // param: lmA Landmark A info
        // param: lmB Landmark B info
        // param: cam_azim azimuth angle of camera in world
        // param: rCamXYZ position of camera in world
        void triangulate_landmarks_old(
            const XYZLandmark& lmA,
            const XYZLandmark& lmB,
            cv::Point3d& rCamXYZ,
            double& cam_azim);

    public:

        // these must be updated prior to triangulation
        double cam_y;       // known camera height
        double cam_elev;    // known camera elevation (radians)

        // camera calibration data
        cv::Size img_sz;
        double cx;
        double cy;
        double fx;
        double fy;

        cv::Mat cam_matrix;
        cv::Mat dist_coeffs;
    };
}

#endif // CAMERA_HELPER_H_
