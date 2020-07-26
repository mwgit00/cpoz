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
    
    
    FakeLidar::FakeLidar() :
        range_dec_pt_adjust(1),
        jitter_range_cm_u(0.2),
        jitter_angle_deg_u(0.25),
        jitter_sync_deg_u(0.25),
        is_angle_noise_enabled(true),
        is_range_noise_enabled(true)
    {

    }

    
    FakeLidar::~FakeLidar()
    {

    }

    
    void FakeLidar::load_floorplan(const std::string& rspath)
    {
        // room should be a solid blob
        // "inside" pixels are black or gray, background is white
        img_floorplan = imread(rspath, IMREAD_GRAYSCALE);
        Mat img_test = Mat(img_floorplan.size(), CV_8U);
        Mat img_binary = (img_floorplan < 240);

        // extract contours of room
        // use "simple" approximation to encode long straight segments as two points
        findContours(img_binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
#if 0
        // generate test image
        drawContours(img_test, contours, 0, 128, 1);
        imwrite("zzfloorplan.png", img_test);
#endif
    }


    void FakeLidar::draw_last_scan(cv::Mat& rimg) const
    {
        for (size_t nn = 0; nn < last_scan.size(); nn++)
        {
            double mag = last_scan[nn];
            int dx = static_cast<int>(jitter_cos_sin[nn].x * mag);
            int dy = static_cast<int>(jitter_cos_sin[nn].y * mag);

            // draw ray from real-world position
            // use noisy measurements for angle and length of ray
            // skip scan ray facing backward as visual indicator of scan orientation
            if (nn != last_scan.size() / 2)
            {
                line(rimg, world_pos, { world_pos.x + dx, world_pos.y + dy }, 64);
                circle(rimg, { world_pos.x + dx, world_pos.y + dy }, 3, 64, -1);
            }
        }
    }


    void FakeLidar::run_scan(void)
    {
        const size_t sz = contours[0].size();

        // flush previous results
        last_scan.clear();
        jitter_cos_sin.clear();
        
        // create jitter in angular sync (one offset applied to all angles)
        double noise = randu<double>();
        double sync_jitter = jitter_sync_deg_u * 2.0 * (noise - 0.5);

        // create jitter in individual measurement angles
        for (const auto& rdeg : scan_angs)
        {
            double ang_rad;
            if (is_angle_noise_enabled)
            {
                double noise = randu<double>();
                double ang_with_jitter = rdeg + world_ang + jitter_angle_deg_u * 2.0 * (noise - 0.5);
                ang_with_jitter += sync_jitter;
                ang_rad = ang_with_jitter * CV_PI / 180.0;
            }
            else
            {
                ang_rad = (rdeg + world_ang) * CV_PI / 180.0;
            }
            jitter_cos_sin.push_back(cv::Point2d(cos(ang_rad), sin(ang_rad)));
        }

        // loop through all measurement angles
        for (const auto& r : jitter_cos_sin)
        {
            double rmin = 10000.0;

            // check for intersection with all contour line segments
            // wraparound at end of array to get points for final segment
            for (size_t nn = 0; nn < sz; nn++)
            {
                cv::Point pt0 = contours[0][nn];
                cv::Point pt1 = contours[0][(nn + 1) % sz];

                // solve parametric system where rays from pt0 and pt1 intersect
                //
                // a1 + (dx1 * t1) = (dx0 * t0) + a0   equ #1
                // b1 + (dy1 * t1) = (dy0 * t0) + b0   equ #2
                //
                // t1 = ((dx0 * t0) + a0 - a1) / dx1   equ #3
                // t1 = ((dy0 * t0) + b0 - b1) / dy1   equ #4

                // substitute #3 into #2  (|dx1| > 0)
                //
                // b1 - b0 + (dy1 * (((dx0 * t0) + a0 - a1) / dx1)) = (dy0 * t0)
                // dx1*(b1 - b0) + (dy1 * (dx0 * t0)) + dy1*(a0 - a1) = (dy0 * t0)*dx1
                // dx1*(b1 - b0) + (dy1 * (a0 - a1)) = (dy0 * t0)*dx1 - (dy1 * ((dx0 * t0))

                // substitute #4 into #1  (|dy1| > 0)
                //
                // a1 - a0 + (dx1 * (((dy0 * t0) + b0 - b1) / dy1)) = (dx0 * t0)
                // dy1*(a1 - a0) + (dx1 * (dy0 * t0)) + dx1*(b0 - b1) = (dx0 * t0)*dy1
                // dy1*(a1 - a0) + (dx1 * (b0 - b1)) = (dx0 * t0)*dy1 - (dx1 * ((dy0 * t0))

                // determine length of current segment
                // and unit vector from start point of segment
                double dx1 = pt1.x - pt0.x;
                double dy1 = pt1.y - pt0.y;
                double seglen = sqrt((dx1 * dx1) + (dy1 * dy1));
                dx1 = dx1 / seglen;
                dy1 = dy1 / seglen;

                double a1 = pt0.x;
                double b1 = pt0.y;

                // get unit vector and start point for scan
                double dx0 = r.x;
                double dy0 = r.y;
                double a0 = world_pos.x;
                double b0 = world_pos.y;

                double t0;
                double t1;
                
                // determine which solution to use to prevent divide-by-zero
                // solve for t0 and then substitute t0 back into equation to get t1

                if (abs(dx1) > 1e-6)
                {
                    t0 = ((dx1 * (b1 - b0)) + (dy1 * (a0 - a1))) / ((dx1 * dy0) - (dx0 * dy1));
                    t1 = ((dx0 * t0) + a0 - a1) / dx1;
                }
                else if (abs(dy1) > 1e-6)
                {
                    t0 = ((dy1 * (a1 - a0)) + (dx1 * (b0 - b1))) / ((dx0 * dy1) - (dx1 * dy0));
                    t1 = ((dy0 * t0) + b0 - b1) / dy1;
                }

                double xr = (t0 * r.x);
                double yr = (t0 * r.y);

#if 0
                // after solving for t0 and t0
                // (x0, y0) should equal (x1, y1)
                // that can be tested here
                double x0 = xr + a0;
                double y0 = yr + b0;
                double x1 = (t1 * dx1) + a1;
                double y1 = (t1 * dy1) + b1;
#endif

                // solution must have non-negative t0 and t1 terms
                // and the length of t1 must be withing segment length
                // whichever solution is closest (min range) will be the measurement
                if ((t0 >= 0) && (t1 >= 0) && (t1 <= seglen))
                {
                    double rng = sqrt((xr * xr) + (yr * yr));
                    if (rng < rmin)
                    {
                        rmin = rng;
                    }
                }
            }

            last_scan.push_back(rmin);
        }

        // add measurement jitter
        // and apply digits-after-decimal-point adjustment
        for (auto& r : last_scan)
        {
            if (is_range_noise_enabled)
            {
                double noise = randu<double>();
                double rnoisy = r + jitter_range_cm_u * 2.0 * (noise - 0.5);
                int inew = static_cast<int>((rnoisy * range_dec_pt_adjust) + 0.5);
                double rnew = static_cast<double>(inew) / range_dec_pt_adjust;
                r = rnew;
            }
        }
    }
}
