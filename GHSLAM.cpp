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

    constexpr size_t GMARR_SZ = 15;

    
    GHSLAM::GHSLAM() :
        loc({ 0, 0}),
        ang(0.0),
        mscale(0.5)
    {
        gmarr.resize(GMARR_SZ);
        for (auto& rgm : gmarr)
        {
            rgm.init(1, 7, 0.5, 8.0);
        }

        // 8000 samples/s, 2Hz-10Hz ???
        init_scan_angs(0.0, 360.0, 4.0, 1.0, GMARR_SZ);
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
        cv::Mat& rimgmask,
        cv::Point& rpt0,
        const size_t offset_index,
        const std::vector<double>& rscan)
    {
        const int PAD_BORDER = 7;

        // @FIXME -- sanity check this index
        std::vector<Point2d>& rcs = scan_cos_sin[offset_index];

        if (rscan.size() == rcs.size())
        {
            std::vector<Point> pts;
            pts.resize(rscan.size());

            // project all measurements using ideal measurement angles
            for (size_t nn = 0; nn < rscan.size(); nn++)
            {
                Point2d& rpt2d = scan_cos_sin[offset_index][nn];
                double mag = rscan[nn] * mscale;
                int dx = static_cast<int>(rpt2d.x * mag);
                int dy = static_cast<int>(rpt2d.y * mag);
                pts[nn] = { dx, dy };
            }
            
            // get bounding box around measurement points
            // create image same size as bounding box along with some padding
            // shift points so they will be centered in image
            Rect r = boundingRect(pts);
            Size imgsz = Size(r.width + (2 * PAD_BORDER), r.height + (2 * PAD_BORDER));
            rimg = Mat::zeros(imgsz, CV_8UC1);
            for (size_t nn = 0; nn < pts.size(); nn++)
            {
                Point ptnew = { pts[nn].x - r.x + PAD_BORDER, pts[nn].y - r.y + PAD_BORDER };
                pts[nn] = ptnew;
            }

            // put points into a contour data structure
            // and then draw them into the image as filled blob with anti-aliased lines
            std::vector<std::vector<Point>> cc;
            cc.push_back(pts);
            drawContours(rimg, cc, 0, 255, cv::FILLED, cv::LINE_AA);

            // generate a mask image
            // draw lines between points but filter out any that are too long
            // this creates a mask that will eliminate lots of useless contour points
            // threshold = (max LIDAR range * tan(angle between measurements))
            double len_thr = 1200 * mscale * tan(4 * CV_PI / 180);
            rimgmask = Mat::zeros(imgsz, CV_8UC1);
            for (size_t nn = 0; nn < pts.size(); nn++)
            {
                Point pt0 = pts[nn];
                Point pt1 = pts[(nn + 1) % pts.size()];
                Point ptdiff = pt0 - pt1;
                double r = sqrt(ptdiff.x * ptdiff.x + ptdiff.y * ptdiff.y);
                if (r < len_thr)
                {
                    line(rimgmask, pt0, pt1, 255, 7);
                }
            }

            // pre-blur prior to matching (optional)
            GaussianBlur(rimg, rimg, { 3, 3 }, 0.0, 0.0);

            // finally note the sensing point in the scan image
            rpt0 = { 0 - r.x + PAD_BORDER, 0 - r.y + PAD_BORDER };
        }
    }


    void GHSLAM::update_scan_templates(const std::vector<double>& rscan)
    {
        const size_t sz = gmarr.size();
        for (size_t ii = 0; ii < sz; ii++)
        {
            Mat img_scan;
            Mat img_mask;
            Mat img_grad;

            Point pt0;
            scan_to_img(img_scan, img_mask, pt0, ii, rscan);
            
            gmarr[ii].create_masked_gradient_orientation_img(img_scan, img_grad);
            img_grad = img_grad & img_mask;

            ghalgo::create_lookup_table(
                img_grad,
                static_cast<uint8_t>(gmarr[ii].m_angstep + 1.0), gmarr[ii].m_ghtable);

#if 0
            tpt0_scan = pt0_scan;
            tpt0_mid = (img_scan.size() / 2);
            tpt0_offset = tpt0_scan - tpt0_mid;
#endif
        }
    }


    void GHSLAM::perform_match(const cv::Point& rpt0, const cv::Mat& rin)
    {
        const size_t sz = gmarr.size();

        Mat img_grad;

        Point tptq_scan = rpt0;
        Point tptq_mid = (rin.size() / 2);
        Point tptq_offset = tptq_scan - tptq_mid;

        // they are all identical
        gmarr[0].create_masked_gradient_orientation_img(rin, img_grad);

        // search for best orientation match
        // this match will also provide the translation
        size_t qidmax = 0;
        double qallmax = 0.0;
        for (size_t ii = 0; ii < sz; ii++)
        {
            Mat img_match;
            double qmax;
            Point qptmax;

            ghalgo::apply_ghough_transform_allpix<uint8_t, CV_16U, uint16_t>(
                img_grad, img_match, gmarr[ii].m_ghtable, 1);
            minMaxLoc(img_match, nullptr, &qmax, nullptr, &qptmax);

            if (qmax > qallmax)
            {
                qallmax = qmax;
                qidmax = ii;
            }
        }
    }
}
