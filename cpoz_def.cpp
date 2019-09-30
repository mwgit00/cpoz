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

#include "cpoz_def.h"


cv::Mat calc_axes_rotation_mat(const double roll, const double pitch, const double yaw)
{
    //    Calculate 3D Euler angle rotation matrix.
    //
    //    Creates matrix for rotating AXES.
    //    With axis pointing out, positive rotation is clockwise.
    //    Uses right - handed "airplane" conventions:
    //     - x, forward, roll, phi
    //     - y, right, pitch, theta
    //     - z, down, yaw, psi
    //
    //    : param roll_phi : roll angle (radians)
    //    : param pitch_theta : pitch angle (radians)
    //    : param yaw_psi : yaw angle (radians)
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


cv::Vec3d calc_xyz_after_rotation(const cv::Vec3d& xyz_pos, const double roll, const double pitch, const double yaw)
{
    // Rotates axes by roll, pitch, yaw angles
    // and returns new position with respect to rotated axes.
    // Rotate along X, Y, Z in that order to visualize.
    cv::Mat ro_mat = calc_axes_rotation_mat(roll, pitch, yaw);
    CV_Assert(isRotationMatrix(ro_mat));
    cv::Mat r = ro_mat * cv::Mat(xyz_pos);
    return cv::Vec3d(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0));
}


// everything below is from:
// https://www.learnopencv.com/rotation-matrix-to-euler-angles/


// Calculates rotation matrix given euler angles.
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f& theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(theta[0]), -sin(theta[0]),
        0, sin(theta[0]), cos(theta[0])
        );

    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
        cos(theta[1]), 0, sin(theta[1]),
        0, 1, 0,
        -sin(theta[1]), 0, cos(theta[1])
        );

    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
        cos(theta[2]), -sin(theta[2]), 0,
        sin(theta[2]), cos(theta[2]), 0,
        0, 0, 1);

    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;

    return R;
}


// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(const cv::Mat& R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
    return (norm(I, shouldBeIdentity) < 1.0e-6);
}


// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R)
{
    assert(isRotationMatrix(R));

    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1.0e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}
