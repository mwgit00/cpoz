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

#include <vector>

#include "GHSLAM.h"



namespace cpoz
{
    using namespace cv;

    GHSLAM::GHSLAM() :
        loc({ 0, 0}),
        ang(0.0)
    {

    }
    
    GHSLAM::~GHSLAM()
    {

    }


    void GHSLAM::init_scan_angs(
        const double deg0,
        const double deg1,
        const double degstep,
        const double offset_step,
        const size_t offset_ct)
    {
        // flush old data
        scan_angs.clear();
        scan_angs_offsets.clear();
        scan_cos_sin.clear();

        // generate ideal scan angles
        double ang = deg0;
        std::vector<Point2d> vcs;
        while (ang < (deg1 - 1e-6))
        {
            scan_angs.push_back(ang);
            ang += degstep;
        }

        // generate lookup tables for cosine and sine
        // for a range of offsets from ideal scan angles
        double ang_offset = -offset_step * (offset_ct / 2);
        for (size_t nn = 0; nn < offset_ct; nn++)
        {
            scan_angs_offsets.push_back(ang_offset);
            scan_cos_sin.push_back({});
            
            for (const auto& rang : scan_angs)
            {
                double ang_rad = (rang + ang_offset) * CV_PI / 180.0;
                scan_cos_sin.back().push_back(cv::Point2d(cos(ang_rad), sin(ang_rad)));
            }

            ang_offset += offset_step;
        }
    }

    const std::vector<double>& GHSLAM::get_scan_angs(void) const
    {
        return scan_angs;
    }


    void GHSLAM::scan_to_img(
        cv::Mat& rimg,
        cv::Point& rpt0,
        const size_t offset_index,
        const std::vector<double>& rscan)
    {
        const int PAD_BORDER = 7;

        std::vector<Point2d>& rcs = scan_cos_sin[offset_index];

        if (rscan.size() == rcs.size())
        {
            std::vector<Point> pts;
            pts.resize(rscan.size());

            int maxx = 0;
            int maxy = 0;
            for (size_t nn = 0; nn < rscan.size(); nn++)
            {
                Point2d& rpt2d = scan_cos_sin[offset_index][nn];
                double mag = rscan[nn];
                int dx = static_cast<int>(rpt2d.x * mag);
                int dy = static_cast<int>(rpt2d.y * mag);
                maxx = max(abs(dx), maxx);
                maxy = max(abs(dy), maxy);
                pts[nn] = { dx, dy };
            }
            
#if 0
            int padx = maxx + pad;
            int pady = maxy + pad;
            int w = padx * 2;
            int h = pady * 2;
            rimg.create(Size(w, h), CV_8UC1);

            // offset points so they all radiate from center of image
            for (size_t nn = 0; nn < rscan.size(); nn++)
            {
                Point ptnew = { pts[nn].x + padx, pts[nn].y + pady };
                pts[nn] = ptnew;
            }
#else
            Rect r = boundingRect(pts);
            rimg = Mat::zeros(Size(r.width + (2 * PAD_BORDER), r.height + (2 * PAD_BORDER)), CV_8UC1);
            for (size_t nn = 0; nn < rscan.size(); nn++)
            {
                Point ptnew = { pts[nn].x - r.x + PAD_BORDER, pts[nn].y - r.y + PAD_BORDER };
                pts[nn] = ptnew;
            }
#endif

            // get points into a contour data structure
            // and then draw them into the image
            std::vector<std::vector<Point>> cc;
            cc.push_back(pts);
            drawContours(rimg, cc, 0, 255, cv::FILLED, cv::LINE_AA);

            // note sensing point
#if 0
            rpt0 = { padx, pady };
#else
            rpt0 = { 0 - r.x + PAD_BORDER, 0 - r.y + PAD_BORDER };
#endif
        }
    }
}
