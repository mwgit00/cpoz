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
// The above copyright noticeand this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "CameraHelper.h"

namespace cpoz
{
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


    cv::Mat CameraHelper::calc_axes_rotation_mat(const double roll, const double pitch, const double yaw)
    {
        // Calculate 3D Euler angle rotation matrix.
        //
        // Creates matrix for rotating AXES.
        // With axis pointing out, positive rotation is clockwise.
        // Uses right-handed "airplane" conventions:
        //   - x, forward, roll, phi
        //   - y, right, pitch, theta
        //   - z, down, yaw, psi
        //
        // param roll : roll angle (radians)
        // param pitch : pitch angle (radians)
        // param yaw : yaw angle (radians)

        cv::Mat rpy = cv::Mat::eye(3, 3, CV_64F);

        double c_r = cos(roll);
        double s_r = sin(roll);
        double c_p = cos(pitch);
        double s_p = sin(pitch);
        double c_y = cos(yaw);
        double s_y = sin(yaw);

        rpy.at<double>(0, 0) = c_p * c_y;
        rpy.at<double>(0, 1) = c_p * s_y;
        rpy.at<double>(0, 2) = -s_p;

        rpy.at<double>(1, 0) = (-c_r) * s_y + s_r * s_p * c_y;
        rpy.at<double>(1, 1) = c_r * c_y + s_r * s_p * s_y;
        rpy.at<double>(1, 2) = s_r * c_p;

        rpy.at<double>(2, 0) = s_r * s_y + c_r * s_p * c_y;
        rpy.at<double>(2, 1) = (-s_r) * c_y + c_r * s_p * s_y;
        rpy.at<double>(2, 2) = c_r * c_p;

        return rpy;
    }


    cv::Vec3d CameraHelper::calc_xyz_after_rotation(const cv::Vec3d& xyz_pos, const double roll, const double pitch, const double yaw)
    {
        // Rotates axes by roll, pitch, yaw angles
        // and returns new position with respect to rotated axes.
        // Rotate along X, Y, Z in that order to visualize.
        cv::Mat ro_mat = calc_axes_rotation_mat(roll, pitch, yaw);
        cv::Mat r = ro_mat * cv::Mat(xyz_pos);
        return cv::Vec3d(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0));
    }


    bool CameraHelper::is_visible(const cv::Vec2d& rUV) const
    {
        bool result = true;

        if ((rUV[0] < 0.0) || (rUV[0] >= img_sz.width))
        {
            result = false;
        }

        if ((rUV[1] < 0.0) || (rUV[1] >= img_sz.height))
        {
            result = false;
        }

        return result;
    }


    cv::Vec2d CameraHelper::project_xyz_to_uv(const cv::Vec3d& rXYZ)
    {
        double pixel_u = (fx * (rXYZ[0] / rXYZ[2])) + cx;
        double pixel_v = (fy * (rXYZ[1] / rXYZ[2])) + cy;
        return cv::Vec2d(pixel_u, pixel_v);
    }


    cv::Vec2d CameraHelper::calc_azim_elev(const cv::Vec2d& rUV)
    {
        // need negation for elevation so it matches camera convention
        double ang_azimuth = atan((rUV[0] - cx) / fx);
        double ang_elevation = atan((cy - rUV[1]) / fy);
        return { ang_azimuth, ang_elevation };
    }


    cv::Vec3d CameraHelper::calc_rel_xyz_to_pixel(
        const double known_Y,
        const cv::Vec2d& rUV,
        const double cam_elev_rad)
    {
        // use camera params to convert(u, v) to ray
        // Z coordinate is 1
        double ray_x = (rUV[0] - cx) / fx;
        double ray_y = (rUV[1] - cy) / fy;
        cv::Vec3d ray_cam = { ray_x, ray_y, 1.0 };

        // rotate ray to undo known camera elevation
        cv::Vec3d ray_cam_unrot = calc_xyz_after_rotation(ray_cam, -cam_elev_rad, 0.0, 0.0);

        // scale ray based on known height (Y)
        // this has X,Y,Z relative to camera body
        // angles and ranges can be derived from this vector
        double rescale = known_Y / ray_cam_unrot[1];
        return ray_cam_unrot * rescale;
    }


    void CameraHelper::triangulate(
        const cv::Vec3d& rXYZa,
        const cv::Vec3d& rXYZb,
        const cv::Vec2d& rUVa,
        const cv::Vec2d& rUVb,
        double& range,
        double& loc_angle,
        double& rel_azim)
    {
        // camera is at known Y but sightings can be at different heights
        double known_Y_a = rXYZa[1] - world_y;
        double known_Y_b = rXYZb[1] - world_y;

        // find relative vector to known real-world location A given its pixel coords
        // then calculate ground range to A
        cv::Vec3d xyz_a = calc_rel_xyz_to_pixel(known_Y_a, rUVa, elev);
        double x_a = xyz_a[0];
        double z_a = xyz_a[2];
        double r_a = sqrt((x_a * x_a) + (z_a * z_a));

        // calculate relative azim to real-world location A
        rel_azim = atan(x_a / z_a);

        // find relative vector to known real-world location B given its pixel coords
        // FIXME -- this landmark could be point along an edge at unknown position
        // then calculate ground range to B
        cv::Vec3d xyz_b = calc_rel_xyz_to_pixel(known_Y_b, rUVb, elev);
        double x_b = xyz_b[0];
        double z_b = xyz_b[2];
        double r_b = sqrt((x_b * x_b) + (z_b * z_b));

        // find vector between real-world locations A and B
        // then calculate the ground range between them
        cv::Vec3d xyz_c = xyz_b - xyz_a;
        double x_c = xyz_c[0];
        double z_c = xyz_c[2];
        double r_c = sqrt((x_c * x_c) + (z_c * z_c));

        // now all three sides of triangle have been found
        // so use Law of Cosines to calculate angle between the
        // vector to real-world location A and the vector between A and B
        double gamma_cos = ((r_a * r_a) + (r_c * r_c) - (r_b * r_b)) / (2 * r_a * r_c);
        loc_angle = acos(gamma_cos);

        // finally stuff ground range to real-world location A
        range = r_a;
    }
}