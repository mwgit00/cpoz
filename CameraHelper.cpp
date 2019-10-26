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
                std::string sfile = vcalfiles[i];
                cv::Mat img = cv::imread(sfile.c_str(), cv::IMREAD_GRAYSCALE);
                std::vector<cv::Point2f>& rv = vvcalpts[i];
                cv::cornerSubPix(img, rv, { 5,5 }, { -1,-1 }, tc_corners);
#if 0
                cv::Point prev(0, 0);
                for (size_t j = 0; j < rv.size(); j++)
                {
                    cv::Point pt(rv[j]);
                    cv::circle(img, pt, 9, 255);
                    cv::line(img, prev, pt, 255, 1);
                    prev = pt;
                }
                std::ostringstream oss;
                oss << "foo" << i << ".png";
                cv::imwrite(oss.str(), img);
#endif
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
        //   - x, forward, roll, phi (elevation)
        //   - y, right, pitch, theta (azimuth)
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
            fsin["camera_matrix"] >> cam_matrix;
            fsin["distortion_coefficients"] >> dist_coeffs;
            result = true;
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


    cv::Point2d CameraHelper::project_xyz_to_img_xy(const cv::Point3d& rXYZ) const
    {
        // zero rotation and zero translation
        const cv::Mat v0 = cv::Mat(std::vector<double>{ 0, 0, 0 }).t();
        std::vector<cv::Point3d> world_xyz(1, rXYZ);
        std::vector<cv::Point2d> img_xy;
        cv::projectPoints(world_xyz, v0, v0, cam_matrix, dist_coeffs, img_xy);
        return img_xy[0];
    }


    cv::Point3d CameraHelper::undistort_img_xy_to_xyz(const cv::Point2d& rImgXY) const
    {
        // use camera params to convert (X,Y) to normalized world coordinates (X,Y,1)
        std::vector<cv::Point2d> input_xy(1, rImgXY);
        std::vector<cv::Point2d> output_xy;
        cv::undistortPoints(input_xy, output_xy, cam_matrix, dist_coeffs);
        return{ output_xy[0].x, output_xy[0].y, 1.0 };
    }


    void CameraHelper::triangulate_landmarks(
        const XYZLandmark& rLM1,
        const XYZLandmark& rLM2,
        T_TRIANG_SOL& rsol) const
    {
        // LM1 is left landmark in image and LM2 is right landmark in image
        // determine vectors from camera to left and right landmarks based on image coordinates
        cv::Point3d cam_to_1 = undistort_img_xy_to_xyz(rLM1.img_xy);
        cv::Point3d cam_to_2 = undistort_img_xy_to_xyz(rLM2.img_xy);

        // calculate angle C between the left and right landmark vectors
        // (cosine is dot product divided by product of magnitudes)
        double mag_cam_to_1 = cv::norm(cv::Mat(cam_to_1), cv::NORM_L2);
        double mag_cam_to_2 = cv::norm(cv::Mat(cam_to_2), cv::NORM_L2);
        double cos_C = cam_to_1.dot(cam_to_2) / (mag_cam_to_1 * mag_cam_to_2);
        rsol.ang_ABC[2] = acos(cos_C);

        cv::Point3d v_dn = { 0, 1, 0 };
        cv::Point3d v_up = { 0, -1, 0 };
        rsol.ang_ABC[0] = acos(v_up.dot(-cam_to_1) / (mag_cam_to_1));
        rsol.ang_ABC[1] = acos(v_dn.dot(-cam_to_2) / (mag_cam_to_2));

        rsol.ang_180_err = rsol.ang_ABC[0] + rsol.ang_ABC[1] + rsol.ang_ABC[2] - CV_PI;

        rsol.len_abc = solve_law_of_sines(rsol.ang_ABC, 12);

        //double f0 = 1;// cam_to_L.y;
        //cv::Point3d fpt0 = cam_to_L * (-10 / f0);
        //double f1 = 1;// cam_to_R.y;
        //cv::Point3d fpt1 = cam_to_R * (-10 / f1);
        //cv::Point3d fud = fpt0 - fpt1;
        //double q = cv::norm(cv::Mat(fud), cv::NORM_L2);
        //double fudge = 12 / q;

        //fpt0 = fpt0 * fudge;
        //fpt1 = fpt1 * fudge;
        //fud = fpt0 - fpt1;
        //q = cv::norm(cv::Mat(fud), cv::NORM_L2);

        //double aaa = cv::norm(cv::Mat(fpt0), cv::NORM_L2);
        //double bbb = cv::norm(cv::Mat(fpt1), cv::NORM_L2);

        // and we are stuck...

        // ???
        rsol.a1 = 0;
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

    void CameraHelper::triangulate_landmarks_ideal(
        const XYZLandmark& lmA,
        const XYZLandmark& lmB,
        cv::Point3d& rCamXYZ,
        double& cam_azim)
    {
        // let camera be at 0,0 in X,Z plane
        // landmark sightings will be offset by the known camera Y
        rCamXYZ = { 0.0, cam_y, 0.0 };

        // find relative ground vector AC from landmark A to camera in X,Z plane
        cv::Point3d xyz_a = calc_cam_to_xyz(lmA.world_xyz.y - cam_y, lmA.img_xy, cam_elev);
        double x_ac = rCamXYZ.x - xyz_a.x;
        double z_ac = rCamXYZ.z - xyz_a.z;

        // find relative ground vector AB from landmark A to landmark B in X,Z plane
        cv::Point3d xyz_b = calc_cam_to_xyz(lmB.world_xyz.y - cam_y, lmB.img_xy, cam_elev);
        double x_ab = xyz_b.x - xyz_a.x;
        double z_ab = xyz_b.z - xyz_a.z;

        // calculate cosine between vectors AC and AB
        // this is the dot product divided by magnitudes
        double mag_ac = sqrt((x_ac * x_ac) + (z_ac * z_ac));
        double mag_ab = sqrt((x_ab * x_ab) + (z_ab * z_ab));
        double cos_ac_ab = (x_ac * x_ab + z_ac * z_ab) / (mag_ac * mag_ab);

        // determine angle from AC to AB
        double rel_world_ang = acos(cos_ac_ab);

        // calculate angle that camera is "rotated away from" landmark A
        // if it was pointed directly towards the landmark then the angle would be 0
        double rel_cam_azim = atan2(z_ac, x_ac);

        // calculate angle formed by vector in X,Z plane
        // from world XYZ for landmark A to world XYZ for landmark B 
        double dz = lmB.world_xyz.z - lmA.world_xyz.z;
        double dx = lmB.world_xyz.x - lmA.world_xyz.x;
        double landmark_world_ang = atan2(dz, dx);

        // then calculate world angle from A to camera
        // use world angle and AC ground range to calulate X,Z offsets
        // use offsets to find camera real-world XYZ
        double world_ang = landmark_world_ang - rel_world_ang;
        double world_x = lmA.world_xyz.x + cos(world_ang) * mag_ac;
        double world_z = lmA.world_xyz.z + sin(world_ang) * mag_ac;
        rCamXYZ = { world_x, cam_y, world_z };

        // determine camera's real-world azimuth in X,Z plane
        cam_azim = rel_cam_azim - world_ang;
        if (cam_azim < 0.0) cam_azim += (CV_PI * 2.0);
        if (cam_azim >= (CV_PI * 2.0)) cam_azim -= (CV_PI * 2.0);
    }


    cv::Vec3d CameraHelper::solve_law_of_sines(const cv::Vec3d& rAngABC, const double c)
    {
        // Law of Sines:  sinA/a = sinB/b = sinC/c
        // we know length of side c and all three angles so solve for sides a and b
        cv::Vec3d result;
        double sinC_over_c = sin(rAngABC[2]) / c;
        result[0] = sin(rAngABC[0]) / sinC_over_c;  // side a (opposite of LM1, cam to LM2)
        result[1] = sin(rAngABC[1]) / sinC_over_c;  // side b (opposite of LM2, cam to LM1)
        result[2] = c;
        return result;
    }
}
