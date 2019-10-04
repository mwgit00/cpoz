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

        CameraHelper();
        virtual ~CameraHelper();

        static void cal(const std::string& rs);

        static cv::Mat calc_axes_rotation_mat(const double roll, const double pitch, const double yaw);
        static cv::Vec3d calc_xyz_after_rotation(const cv::Vec3d& xyz_pos, const double roll, const double pitch, const double yaw);

        // test if pixel at (u, v) is within valid range.
        bool is_visible(const cv::Vec2d& rUV) const;

        // project 3D world point to image plane
        cv::Vec2d project_xyz_to_uv(const cv::Vec3d& rXYZ);

        // calculate azimuth (radians) and elevation (radians) to image point
        cv::Vec2d calc_azim_elev(const cv::Vec2d& rUV);

        // calculate camera-relative X,Y,Z vector to point in image
        cv::Vec3d calc_rel_xyz_to_pixel(
            const double known_Y,
            const cv::Vec2d& rUV,
            const double cam_elev_rad);

        // Use sightings of real world X,Y,Z and corresponding U,V to perform triangulation.
        // param: rXYZ1 reference to real-world location 1
        // param: rXYZ2 reference to real-world location 2
        // param: rUV1 reference to pixel coords for XYZ1
        // param: rUV2 reference to pixel coords for XYZ2
        // param: range X,Z plane ground range from camera to XYZ1
        // param: loc_angle angle about XYZ1 from XYZ2 to camera XYZ from Law-of-Cosines
        // param: rel_azim azimuth angle of XYZ1 relative to camera
        void triangulate(
            const cv::Vec3d& rXYZ1,
            const cv::Vec3d& rXYZ2,
            const cv::Vec2d& rUV1,
            const cv::Vec2d& rUV2,
            double& range,
            double& loc_angle,
            double& rel_azim);

    public:

        // TODO -- make it accept OpenCV intrinsic camera calib matrix
        // these must be updated prior to triangulation
        double world_y;     // known camera height
        double elev;        // known camera elevation (radians)

        // arbitrary test params
        cv::Size img_sz;
        double cx;
        double cy;
        double fx;
        double fy;

        cv::Mat calib;
    };
}

#endif // CAMERA_HELPER_H_
