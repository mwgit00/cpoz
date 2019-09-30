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

#ifndef CPOZ_DEF_H_
#define CPOZ_DEF_H_

#include <map>
#include <opencv2/imgproc.hpp>


const double DEG2RAD = CV_PI / 180.0;
const double RAD2DEG = 180.0 / CV_PI;

typedef struct _T_xyz_RL_struct
{
    cv::Vec3d xyz;
    double adj_R;   // always negative
    double adj_L;   // always positive
} T_xyz_RL;

typedef std::map<std::string, T_xyz_RL> tMapStrToXYZRL;
typedef std::map<std::string, cv::Vec2d> tMapStrToAZEL;
typedef std::map<std::string, tMapStrToXYZRL> tMapStrToMapStrToXYZRL;

cv::Mat calc_axes_rotation_mat(const double roll, const double pitch, const double yaw);
cv::Vec3d calc_xyz_after_rotation(const cv::Vec3d& xyz_pos, const double roll, const double pitch, const double yaw);

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f& theta);
bool isRotationMatrix(const cv::Mat& R);
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);


#endif // CPOZ_DEF_H_
