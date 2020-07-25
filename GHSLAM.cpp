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

    constexpr int PAD_BORDER = 31;              // big enough so rotation doesn't chop off pixels
    constexpr size_t GMARR_SZ = 31;             // big enough to provide enough angle resolution
    constexpr double ANG_STEP_SEARCH = 1.0;     // degrees
    constexpr double ANG_STEP_LIDAR = 4.0;      // degrees
    constexpr double MAX_RNG_LIDAR = 1200.0;    // 12m

    
    GHSLAM::GHSLAM() :
        slam_loc({ 0, 0}),
        slam_ang(0.0),
        mscale(0.25)
    {
        gmarr.resize(GMARR_SZ);
        tpt0_offset.resize(GMARR_SZ);

        for (auto& rgm : gmarr)
        {
            rgm.init(1, 3, 0.2, 16.0);
        }

        // cheap COTS LIDAR:  8000 samples/s, 2Hz-10Hz ???
        init_scan_angs(0.0, 360.0, ANG_STEP_LIDAR, ANG_STEP_SEARCH, GMARR_SZ);
    }
    
    
    GHSLAM::~GHSLAM()
    {
        // does nothing
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
        // @FIXME -- sanity check this index
        std::vector<Point2d>& rcs = scan_cos_sin[offset_index];

        if (rscan.size() == rcs.size())
        {
            std::vector<Point> pts;
            pts.resize(rscan.size());

            // project all measurements using ideal measurement angles
            for (size_t nn = 0; nn < rscan.size(); nn++)
            {
                Point2d& rpt2d = rcs[nn];
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
            // and then draw them into the image as filled blob
            std::vector<std::vector<Point>> cc;
            cc.push_back(pts);
            drawContours(rimg, cc, 0, 255, cv::FILLED);

            // generate a mask image
            // draw lines between points but filter out any that are too long
            // this creates a mask that will eliminate lots of useless contour points
            // threshold is distance between two measurements at max range
            double len_thr = MAX_RNG_LIDAR * mscale * tan(ANG_STEP_LIDAR * CV_PI / 180.0);
            rimgmask = Mat::zeros(imgsz, CV_8UC1);
            for (size_t nn = 0; nn < pts.size(); nn++)
            {
                Point pt0 = pts[nn];
                Point pt1 = pts[(nn + 1) % pts.size()];
                Point ptdiff = pt0 - pt1;
                double r = sqrt(ptdiff.x * ptdiff.x + ptdiff.y * ptdiff.y);
                if (r < len_thr)
                {
                    line(rimgmask, pt0, pt1, 255, 7);  // FIXME -- is width okay ???
                }
            }

#if 0
            // pre-blur prior to matching (optional)
            GaussianBlur(rimg, rimg, { 3, 3 }, 0.0, 0.0);
#endif
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

            Point tpt0_mid = (img_scan.size() / 2);
            tpt0_offset[ii] = pt0 - tpt0_mid;
        }
    }


    void GHSLAM::perform_match(
        const std::vector<double>& rscan,
        cv::Point& roffset,
        double& rang)
    {
        const size_t sz = gmarr.size();

        // convert scan to an image (no rotation)
        scan_to_img(m_img_scan, m_img_mask, m_pt0_scan, sz / 2, rscan);

        Point tptq_mid = (m_img_scan.size() / 2);
        Point tptq_offset = m_pt0_scan - tptq_mid;

        // they are all identical so pick matcher 0
        // to get gradient image for running match
        gmarr[0].create_masked_gradient_orientation_img(m_img_scan, m_img_grad);
        m_img_grad = m_img_grad & m_img_mask;

        // search for best orientation match
        // this match will also provide the translation
        size_t qidmax = 0;
        double qallmax = 0.0;
        Point qptmax_offset;
        for (size_t ii = 0; ii < sz; ii++)
        {
            Mat img_match;
            double qmax;
            Point qptmax;

            ghalgo::apply_ghough_transform_allpix<uint8_t, CV_16U, uint16_t>(
                m_img_grad, img_match, gmarr[ii].m_ghtable, 1);
            minMaxLoc(img_match, nullptr, &qmax, nullptr, &qptmax);
            qmax = qmax / static_cast<double>(gmarr[ii].m_ghtable.max_votes);
            
            if (qmax > qallmax)
            {
                qallmax = qmax;
                qidmax = ii;
                qptmax_offset = qptmax - tptq_mid;
            }
        }

        roffset = tptq_offset - tpt0_offset[qidmax] - qptmax_offset;
        rang = scan_angs_offsets[qidmax];
    }
}
