// MIT License
//
// Copyright(c) 2020 Mark Whitney
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

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "FakeLidar.h"


namespace cpoz
{
    using namespace cv;
    
    FakeLidar::FakeLidar()
    {

    }

    FakeLidar::~FakeLidar()
    {

    }

    void FakeLidar::load_floorplan(const std::string& rspath)
    {
        // "inside" pixels are black or gray
        img_floorplan = imread(rspath, IMREAD_GRAYSCALE);
        Mat img_test = Mat(img_floorplan.size(), CV_8U);
        Mat img_binary = (img_floorplan < 240);
        findContours(img_binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);// cv::CHAIN_APPROX_SIMPLE);
        drawContours(img_test, contours, 0, 128, 1);
        imwrite("crud.png", img_test);
    }

    void FakeLidar::init_scan_angs(const double deg0, const double deg1, const double step)
    {
        double ang = deg0;
        while (ang < (deg1 - 1e-6))
        {
            double ang_rad = ang * CV_PI / 180.0;
            scan_angs.push_back(ang);
            scan_cos_sin.push_back(cv::Point2d(cos(ang_rad), sin(ang_rad)));
            ang += step;
        }
    }

    void FakeLidar::draw_scan(cv::Mat& rimg, const std::vector<double>& rvec)
    {
        if (rvec.size() == scan_cos_sin.size())
        {
            img_floorplan.copyTo(rimg);
            for (size_t nn = 0; nn < rvec.size(); nn++)
            {
                double mag = rvec[nn];
                int dx = static_cast<int>(scan_cos_sin[nn].x * mag);
                int dy = static_cast<int>(scan_cos_sin[nn].y * mag);
                line(rimg, pos, { pos.x + dx, pos.y + dy }, 32);
            }
        }
    }

    void FakeLidar::run_scan(std::vector<double>& rvec)
    {
        rvec.clear();
        size_t sz = contours[0].size();

        std::vector<double> vecr;
        vecr.resize(sz);
        
        // precompute distance from sensing point to all contour points
        for (size_t nn = 0; nn < sz; nn++)
        {
            cv::Point pt0 = contours[0][nn];
            double qdx = pt0.x - pos.x;
            double qdy = pt0.y - pos.y;
            double rng = sqrt((qdx * qdx) + (qdy * qdy));
            vecr[nn] = rng;
        }

        // check all scan directions
        for (const auto& r : scan_cos_sin)
        {
            double rmin = 10000.0;

            // check all contour points
            for (size_t nn = 0; nn < sz; nn++)
            {
                cv::Point pt0 = contours[0][nn];
                double rng = vecr[nn];

                // project ray by that distance to get a point
                double qx = pos.x + (rng * r.x);
                double qy = pos.y + (rng * r.y);
                double qdx = qx - pt0.x;
                double qdy = qy - pt0.y;

                // see how close that point is to contour point
                double qrpts = sqrt((qdx * qdx) + (qdy * qdy));
                if (qrpts < sqrt(2.0))
                {
                    // possible match
                    if (rng < rmin)
                    {
                        rmin = rng;
                    }
                }
#if 0
                // solve parametric system where rays from pt0 and pt1 intersect
                //
                // a1 + (dx1 * t1) = (dx0 * t0) + a0   equ #1
                // b1 + (dy1 * t1) = (dy0 * t0) + b0   equ #2

                // t1 = ((dx0 * t0) + a0 - a1) / dx1   equ #3
                // t1 = ((dy0 * t0) + b0 - b1) / dy1   equ #4

                // substitue #3 into #2  (|dx1| > 0)
                //
                // b1 - b0 + (dy1 * (((dx0 * t0) + a0 - a1) / dx1)) = (dy0 * t0)
                // dx1*(b1 - b0) + (dy1 * (dx0 * t0)) + dy1*(a0 - a1) = (dy0 * t0)*dx1
                // dx1*(b1 - b0) + (dy1 * (a0 - a1)) = (dy0 * t0)*dx1 - (dy1 * ((dx0 * t0))

                // determine length of current segment
                // and unit vector from start point
                double dx1 = pt1.x - pt0.x;
                double dy1 = pt1.y - pt0.y;
                double seglen = sqrt((dx1 * dx1) + (dy1 * dy1));
                dx1 = dx1 / seglen;
                dy1 = dy1 / seglen;
                double a1 = pt0.x;
                double b1 = pt0.y;

                double dx0 = r.x;
                double dy0 = r.y;
                double a0 = pos.x;
                double b0 = pos.y; // neg???

                if (abs(dx1) > 1e-6)
                {
                    // solve for t0 and then t1
                    double t0 = ((dx1 * (b1 - b0)) + (dy1 * (a0 - a1))) / ((dx1 * dy0) - (dx0 * dy1));
                    double t1 = ((dx0 * t0) + a0 - a1) / dx1;
                    
                    double xr = (t0 * r.x);
                    double yr = (t0 * r.y);
                    
                    double x0 = xr + a0;
                    double y0 = yr + b0;

                    double x1 = (t1 * dx1) + a1;
                    double y1 = (t1 * dy1) + b1;

                    if ((t1 >= 0) && (t1 <= seglen))
                    {
                        double rng = sqrt((xr * xr) + (yr * yr));
                        if (rng < rmin)
                        {
                            rmin = rng;
                        }
                    }
                }
                else
                {
                }
#endif
            }

            rvec.push_back(rmin);
        }
    }

}
