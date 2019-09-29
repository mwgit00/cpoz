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

#include "CameraHelper.h"


CameraHelper::CameraHelper()
{
    img_sz = { 640, 480 };
    cx = 320;
    cy = 240;
    fx = 554; // 60 deg hfov(30.0)
    fy = 554; // 46 deg vfov(23.0)

        //distCoeff = None
        //camA = np.float32([[fx, 0., cx],
        //    [0., fy, cy],
        //    [0., 0., 1.]] )

}


CameraHelper::~CameraHelper()
{

}


bool CameraHelper::is_visible(const cv::Vec2d& rUV) const
{
    bool result = true;

    if ((rUV[0] < 0) || (rUV[0] >= img_sz.width))
    {
        result = false;
    }

    if ((rUV[1] < 0) || (rUV[1] >= img_sz.height))
    {
        result = false;
    }

    return false;
}


cv::Point2d CameraHelper::project_xyz_to_uv(const cv::Vec3d& rXYZ)
{
    double pixel_u = fx * (rXYZ[0] / rXYZ[2]) + cx;
    double pixel_v = fy * (rXYZ[1] / rXYZ[2]) + cy;
    return cv::Point2d(pixel_u, pixel_v);
}


cv::Vec2d CameraHelper::calc_azim_elev(const cv::Vec2d& rUV)
{
    double ang_azimuth = atan((rUV[0] - cx) / fx);

    // need negation here so elevation matches camera convention
    double ang_elevation = atan((cy - rUV[1]) / fy);

    return { ang_azimuth, ang_elevation };
}


cv::Vec3d CameraHelper::calc_rel_xyz_to_pixel(const double known_y, const cv::Vec2d& rUV, const double cam_elev_rad)
{
    // use camera params to convert(u, v) to ray
    // Z coordinate is 1
    double ray_x = (rUV[0] - cx) / fx;
    double ray_y = (rUV[1] - cy) / fy;
    cv::Vec3d ray_cam = { ray_x, ray_y, 1.0 };

    // rotate ray to undo known camera elevation
    cv::Mat ro_mat_undo_ele = calc_axes_rotation_mat(-cam_elev_rad, 0, 0);
    cv::Vec3d ray_cam_unrot = calc_xyz_after_rotation_deg(ro_mat_undo_ele, 1.0, 0.0, 0.0);

    // scale ray based on known height (Y)
    // this has [X, Y, Z] relative to camera body
    // (can derive angles and ranges from that vector)
//    rescale = known_y / ray_cam_unrot[1][0]
//    ray_cam_unrot_rescale = np.multiply(ray_cam_unrot, rescale)
//    return ray_cam_unrot_rescale.reshape(3, )
    return cv::Vec3d();
}


void CameraHelper::triangulate(
    const cv::Vec3d& rXYZa,
    const cv::Vec3d& rXYZb,
    const cv::Point2d& rUVa,
    const cv::Point2d& rUVb)
{
    // camera is at known Y but sightings can be at different heights
    double known_Y_a = rXYZa[1] - world_y;
    double known_Y_b = rXYZb[1] - world_y;

    // find relative vector to known real-world location A
    // then calculate ground range to A
    cv::Vec3d xyz_a = calc_rel_xyz_to_pixel(known_Y_a, rUVa, elev);
    double x_a = xyz_a[0];
    double z_a = xyz_a[2];
    double r_a = sqrt(x_a * x_a + z_a * z_a);

    // calculate relative azim to real-world location A
    double rel_azim = atan(x_a / z_a);

    // find relative vector to known real-world location B
    // this landmark could be point along an edge at unknown position
    // then calculate ground range to B
    cv::Vec3d xyz_b = calc_rel_xyz_to_pixel(known_Y_b, rUVb, elev);
    double x_b = xyz_b[0];
    double z_b = xyz_b[2];
    double r_b = sqrt(x_b * x_b + z_b * z_b);

    // find vector between real-world locations A and B
    // then calculate the ground range between them
    cv::Vec3d xyz_c = xyz_b - xyz_a;
    double x_c = xyz_c[0];
    double z_c = xyz_c[2];
    double r_c = sqrt(x_c * x_c + z_c * z_c);

    // now all three sides of triangle have been found
    // so use Law of Cosines to calculate angle between the
    // vector to landmark 1 and vector between landmarks
    double gamma_cos = ((r_a * r_a) + (r_c * r_c) - (r_b * r_b)) / (2 * r_a * r_c);
    double angle = acos(gamma_cos);

        //// landmark has angle offset info
        //// which is used to calculate world coords and azim
        //u2 = lm_var.uv[0]
        //world_azim = lm_fix.calc_world_azim(u2, angle, rel_azim)
        //x, z = lm_fix.calc_world_xz(u2, angle, r1)
        //return x, z, world_azim
}