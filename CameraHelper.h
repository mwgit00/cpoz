// MIT License
// 
// Copyright(c) 2019 Mark Whitney
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this softwareand associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
// 
// The above copyright noticeand this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CAMERA_HELPER_H_
#define CAMERA_HELPER_H_

#include "cpoz_def.h"

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
// https://www.learnopencv.com/rotation-matrix-to-euler-angles/
// rotation about Z,Y,X is yaw,pitch,roll respectively


class CameraHelper
{
public:
    
    CameraHelper();
    virtual ~CameraHelper();

    // test if pixel at (u, v) is within valid range.
    bool is_visible(const cv::Vec2d& rUV) const;

    // project 3D world point to image plane
    cv::Point2d project_xyz_to_uv(const cv::Vec3d& rXYZ);

    // calculate azimuth (radians) and elevation (radians) to image point
    cv::Vec2d calc_azim_elev(const cv::Vec2d& rUV);

    // calculate camera-relative X,Y,Z vector to point in image
    cv::Vec3d calc_rel_xyz_to_pixel(const double known_y, const cv::Vec2d& rUV, const double cam_elev_rad);

    // Use sightings of real world X,Y,Z and corresponding U,V
    // to perform triangulation.  Convert angle and range
    // from triangulation into world coordinates based
    // on fixed landmark's known orientation in world.
    void triangulate(
        const cv::Vec3d& rXYZ1,
        const cv::Vec3d& rXYZ2,
        const cv::Point2d& rUV1,
        const cv::Point2d& rUV2);

public:

    // TODO -- make it accept OpenCV intrinsic camera calib matrix
    // these must be updated prior to triangulation
    double world_y;
    double elev;

    // arbitrary test params
    cv::Size img_sz;
    double cx;
    double cy;
    double fx;
    double fy;

    cv::Mat calib;

        //self.distCoeff = None
        //self.camA = np.float32([[self.fx, 0., self.cx],
        //    [0., self.fy, self.cy],
        //    [0., 0., 1.]] )

};

#endif // CAMERA_HELPER_H_
