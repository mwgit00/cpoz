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
            const cv::Point3d& world_xyz,
            const double u1max = 0.0,
            const double u1min = 0.0,
            const std::string & rs = "");

        // Assign pixel coordinates for latest Landmark sighting.
        // param uv : (U,V) coordinates.
        void set_img_xy(const cv::Point2d& rpt);

    public:

        cv::Point3d world_xyz;  // real-world coordinates
        double ang_u1max;       // adjustment when #1 is RIGHT (max u coord) landmark.
        double ang_u1min;       // adjustment when #1 is LEFT (min u coord) landmark.
        cv::Point2d img_xy;     // image pixel coordinates (U or X horizontal, V or Y vertical)
        std::string name;
    };
}

#endif // XYZ_LANDMARK_H_