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

#ifndef XYZ_LANDMARK_H_
#define XYZ_LANDMARK_H_

#include <string>
#include <opencv2/imgproc.hpp>


namespace cpoz
{
    const double DEG2RAD = CV_PI / 180.0;
    const double RAD2DEG = 180.0 / CV_PI;


    class XYZLandmark
    {
    public:

        XYZLandmark();
        virtual ~XYZLandmark();

        XYZLandmark(
            const cv::Point3d& xyz,
            const double u1max = 0.0,
            const double u1min = 0.0,
            const std::string & rs = "");

        // Assign pixel coordinates for latest Landmark sighting.
        //    : param uv : (U, V) coordinates.
        void set_current_uv(const cv::Vec2d& rUV);

        // Determine world position and pointing direction
        // given relative horizontal positions of landmarks
        // and previous triangulation result(angle, range).
        // param u_var : Horizontal coordinate of variable landmark
        // param ang : Angle to this landmark
        // param r : Ground range to this landmark
        // return X,Z in world coordinates
        cv::Vec2d  calc_world_xz(const double u, const double ang, const double r);

        // Convert camera's azimuth to LM to world azimuth.
        // Relative azimuth in camera view is also considered.
        // param u_var : U coordinate of variable LM
        // param ang : Angle between camera and LM1-to-LM2 vector
        // param rel_azim : Relative azimuth to LM1 as seen in image
        // return : World azimuth (radians)
        double calc_world_azim(double u, double ang, double rel_azim);

    public:

        cv::Point3d xyz;      // world coordinates
        double ang_u1max;   // adjustment when #1 is RIGHT (max u coord) landmark.
        double ang_u1min;   // adjustment when #1 is LEFT (min u coord) landmark.
        cv::Vec2d uv;       // pixel coordinates (u horizontal, v vertical)
        std::string name;
    };
}

#endif // XYZ_LANDMARK_H_