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
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "CameraHelper.h"

namespace cpoz
{
    CameraHelper::CameraHelper()
    {
        // create ideal camera calibration data for testing
        // real calibration data must be applied with load method
        
        img_sz = { 640, 480 };
        
        cx = 320;
        cy = 240;
        fx = 554; // 60 deg hfov(30.0)
        fy = 554; // 46 deg vfov(23.0)

        cam_matrix = cv::Mat::eye({ 3, 3 }, CV_64F);
        cam_matrix.at<double>(0, 0) = fx;
        cam_matrix.at<double>(1, 1) = fy;
        cam_matrix.at<double>(0, 2) = cx;
        cam_matrix.at<double>(1, 2) = cy;
        
        std::vector<double> dc = { 0,0,0,0,0 };
        dist_coeffs = cv::Mat(dc).t();
    }


    CameraHelper::~CameraHelper()
    {

    }


    void CameraHelper::cal(const std::string& rs)
    {
        std::string spath = rs + "\\cal_meta.yaml";
        cv::FileStorage fsin;
        fsin.open(spath, cv::FileStorage::READ);
        if (fsin.isOpened())
        {
            cv::Size img_sz;
            cv::Size grid_sz;
            std::vector<std::string> vcalfiles;
            std::vector<std::vector<cv::Point2f>> vvcalpts;
            std::vector<cv::Point3f> vgridpts;
            double grid_square;

            fsin["image_size"] >> img_sz;
            fsin["grid_size"] >> grid_sz;
            fsin["grid_square"] >> grid_square;
            fsin["grid_pts"] >> vgridpts;
            fsin["files"] >> vcalfiles;
            fsin["points"] >> vvcalpts;

            cv::TermCriteria tc_corners(
                cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                50, // max number of iterations
                0.0001);

            // input only has corners to nearest pixel
            // so get sub-pixel corner locations
            for (size_t i = 0; i < vcalfiles.size(); i++)
            {
                std::string sfile = rs + "\\" + vcalfiles[i];
                cv::Mat img = cv::imread(sfile.c_str(), cv::IMREAD_GRAYSCALE);
                std::vector<cv::Point2f>& rv = vvcalpts[i];
                cv::cornerSubPix(img, rv, { 5,5 }, { -1,-1 }, tc_corners);
            }

            std::vector<std::vector<cv::Point3f>> object_pts(vvcalpts.size(), vgridpts);
            cv::Mat cameraMatrix;
            cv::Mat distCoeffs;
            std::vector<cv::Mat> rvecs;
            std::vector<cv::Mat> tvecs;
            int flags = cv::CALIB_USE_LU; // cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_K3

            // use the old-school calibration command
            double rms = calibrateCamera(object_pts, vvcalpts, img_sz,
                cameraMatrix, distCoeffs, rvecs, tvecs, flags);

            bool is_range_ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

            if (is_range_ok)
            {
#if 0
                // reprojection errors if desired someday
                std::vector<double> perViewErrors(object_pts.size(), 0.0);
                std::vector<cv::Point2f> reproj_calpts;
                int total_pts = 0;
                double total_err = 0.0;

                for (size_t i = 0; i < object_pts.size(); i++)
                {
                    projectPoints(cv::Mat(object_pts[i]), rvecs[i], tvecs[i],
                        cameraMatrix, distCoeffs, reproj_calpts);
                    double err = norm(cv::Mat(vvcalpts[i]), cv::Mat(reproj_calpts), cv::NORM_L2);
                    int n = (int)object_pts[i].size();
                    perViewErrors[i] = (float)std::sqrt((err * err) / n);
                    total_err += (err * err);
                    total_pts += n;
                }

                double reproj_err = std::sqrt(total_err / total_pts);
#endif
                std::cout << "Calibration successful!!!" << std::endl;
                cv::FileStorage fsout;
                fsout.open("foo.yaml", cv::FileStorage::WRITE);
                if (fsout.isOpened())
                {
                    fsout << "image_size" << img_sz;
                    fsout << "flags" << flags;
                    fsout << "rms" << rms;
                    fsout << "camera_matrix" << cameraMatrix;
                    fsout << "distortion_coefficients" << distCoeffs;
                }
            }
            else
            {
                std::cout << "Calibration failed!!!" << std::endl;
            }
        }
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


    cv::Point3d CameraHelper::calc_xyz_after_rotation(const cv::Point3d& xyz_pos, const double roll, const double pitch, const double yaw)
    {
        // Rotates axes by roll, pitch, yaw angles
        // and returns new position with respect to rotated axes.
        // Rotate along X, Y, Z in that order to visualize.
        cv::Mat ro_mat = calc_axes_rotation_mat(roll, pitch, yaw);
        cv::Mat r = ro_mat * cv::Mat(xyz_pos);
        return cv::Point3d(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0));
    }


    bool CameraHelper::load(const std::string& rscal)
    {
        bool result = false;
        cv::FileStorage fsin;
        fsin.open(rscal, cv::FileStorage::READ);
        if (fsin.isOpened())
        {
            fsin["image_size"] >> img_sz;
            fsin["cam_matrix"] >> cam_matrix;
            fsin["distortion_coefficients"] >> dist_coeffs;
        }
        return result;
    }


    bool CameraHelper::is_visible(const cv::Point2d& rImgXY) const
    {
        bool result = true;

        if ((rImgXY.x < 0.0) || (rImgXY.x >= img_sz.width))
        {
            result = false;
        }

        if ((rImgXY.y < 0.0) || (rImgXY.y >= img_sz.height))
        {
            result = false;
        }

        return result;
    }


    cv::Point2d CameraHelper::project_world_xyz_to_img_xy(const cv::Point3d& rXYZ) const
    {
        // zero rotation and zero translation
        const cv::Mat v0 = cv::Mat(std::vector<double>{ 0, 0, 0 }).t();
        std::vector<cv::Point3d> world_xyz(1, rXYZ);
        std::vector<cv::Point2d> img_xy;
        cv::projectPoints(world_xyz, v0, v0, cam_matrix, dist_coeffs, img_xy);
        return img_xy[0];
    }


    cv::Point3d CameraHelper::calc_cam_to_xyz(
        const double known_Y,
        const cv::Point2d& rImgXY,
        const double cam_elev_rad)
    {
        // use camera params to convert (X,Y) to normalized world coordinates (X,Y,1)
        std::vector<cv::Point2d> input_xy(1, rImgXY);
        std::vector<cv::Point2d> output_xy;
        cv::undistortPoints(input_xy, output_xy, cam_matrix, dist_coeffs);
        cv::Point3d world_xy1 = { output_xy[0].x, output_xy[0].y, 1.0 };

        // rotate to undo known camera elevation
        cv::Point3d world_xy1_unrot = calc_xyz_after_rotation(world_xy1, -cam_elev_rad, 0.0, 0.0);

        // scale normalized world coordinates based on known height Y
        // this has X,Y,Z relative to camera body
        // angles and ranges can be derived from this vector
        double rescale = known_Y / world_xy1_unrot.y;
        return world_xy1_unrot * rescale;
    }


    void CameraHelper::triangulate(
        const cv::Point3d& rXYZa,
        const cv::Point3d& rXYZb,
        const cv::Point2d& rImgXYa,
        const cv::Point2d& rImgXYb,
        double& cam_azim,
        cv::Point3d& rCamXYZ)
    {
        // camera is at known Y but landmark sightings can be at different heights
        double known_Y_a = rXYZa.y - cam_y;
        double known_Y_b = rXYZb.y - cam_y;

        // find relative ground vector AC from landmark A to camera in X,Z plane
        // negation is required to go from A to camera
        cv::Point3d xyz_a = calc_cam_to_xyz(known_Y_a, rImgXYa, cam_elev);
        double x_ac = -xyz_a.x;
        double z_ac = -xyz_a.z;

        // find relative ground vector AB from landmark A to landmark B in X,Z plane
        cv::Point3d xyz_b = calc_cam_to_xyz(known_Y_b, rImgXYb, cam_elev) - xyz_a;
        double x_ab = xyz_b.x;
        double z_ab = xyz_b.z;

        // calculate cosine between vectors AC and AB (dot product divided by magnitudes)
        double mag_ac = sqrt((x_ac * x_ac) + (z_ac * z_ac));
        double mag_ab = sqrt((x_ab * x_ab) + (z_ab * z_ab));
        double cos_ac_ab = (x_ac * x_ab + z_ac * z_ab) / (mag_ac * mag_ab);

        // determine angle from AC to AB
        double rel_world_ang = acos(cos_ac_ab);

        // calculate relative azim between location A and camera
        double rel_cam_azim = atan2(z_ac, x_ac);

        // calculate angle formed by vector in X,Z plane
        // from world XYZ for landmark A to world XYZ for landmark B 
        double dz = rXYZb.z - rXYZa.z;
        double dx = rXYZb.x - rXYZa.x;
        double landmark_world_ang = atan2(dz, dx);

        // then calculate world angle from A to camera
        // use world angle and AC ground range to calulate X,Z offsets to find camera XYZ
        double world_ang = landmark_world_ang - rel_world_ang;
        double world_x = rXYZa.x + cos(world_ang) * mag_ac;
        double world_z = rXYZa.z + sin(world_ang) * mag_ac;
        rCamXYZ = { world_x, cam_y, world_z };

        // TODO -- fix this
        cam_azim = rel_cam_azim;
    }
}