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

    const int PAD_BORDER = 31;              // big enough so rotation doesn't chop off pixels
    const size_t GMARR_SZ = 61;             // big enough to provide enough angle resolution
    const double ANG_STEP_SEARCH = 0.5;     // degrees
    const double ANG_STEP_LIDAR = 1.0;      // degrees
    const double MAX_RNG_LIDAR = 1200.0;    // 12m

    GHSLAM::GHSLAM() :
        slam_loc({ 0, 0 }),
        slam_ang(0.0),
        mscale(0.25),
        m_mask_line_width(1)
    {
        tpt0_offset.resize(GMARR_SZ);

        // cheap COTS LIDAR:  8000 samples/s, 2Hz-10Hz ???
        init_scan_angs(0.0, 360.0, ANG_STEP_LIDAR, ANG_STEP_SEARCH, GMARR_SZ);

        m_preproc.resize(scan_angs.size());
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


    void GHSLAM::scan_to_img2(
        cv::Mat& rimg,
        cv::Point& rpt0,
        const double scale,
        const std::vector<double>& rscan)
    {
        std::vector<Point2d>& rcs = scan_cos_sin[GMARR_SZ / 2];

        if (rscan.size() == rcs.size())
        {
            std::vector<Point> pts;
            pts.resize(rscan.size());

            // project all measurements using ideal measurement angles
            for (size_t nn = 0; nn < rscan.size(); nn++)
            {
                Point2d& rpt2d = rcs[nn];
                double mag = rscan[nn] * scale;
                int dx = static_cast<int>((rpt2d.x * mag) + 0.5);
                int dy = static_cast<int>((rpt2d.y * mag) + 0.5);
                pts[nn] = { dx, dy };
            }

            // get bounding box around measurement points
            // shift points so they will be centered in image
            Rect r = boundingRect(pts);
            for (size_t nn = 0; nn < pts.size(); nn++)
            {
                Point ptnew = { pts[nn].x - r.x + PAD_BORDER, pts[nn].y - r.y + PAD_BORDER };
                pts[nn] = ptnew;
            }

            // create image same size as bounding box along with some padding
            Size imgsz = Size(r.width + (2 * PAD_BORDER), r.height + (2 * PAD_BORDER));
            rimg = Mat::zeros(imgsz, CV_8UC1);

            // draw representation of the scan...
            // draw lines between scan points but filter out any that are too long
            // the threshold is distance between two measurements at max LIDAR range
            double len_thr = MAX_RNG_LIDAR * mscale * tan(ANG_STEP_LIDAR * CV_PI / 180.0);
            for (size_t nn = 0; nn < pts.size(); nn++)
            {
                Point pt0 = pts[nn];
                Point pt1 = pts[(nn + 1) % pts.size()];
                double dx = pt1.x - pt0.x;
                double dy = pt1.y - pt0.y;
                double r = sqrt((dx * dx) + (dy * dy));
                if (r < len_thr)
                {
                    line(rimg, pt0, pt1, 255, m_mask_line_width);
                }
            }

            // finally note the sensing point in the scan image
            rpt0 = { 0 - r.x + PAD_BORDER, 0 - r.y + PAD_BORDER };
        }
    }


    void GHSLAM::preprocess_scan(
        const size_t offset_index,
        const std::vector<double>& rscan,
        const double scale)
    {
        // look up the 0 degree cos and sin table
        std::vector<Point2d>& rveccs = scan_cos_sin[offset_index];
        
        if (rscan.size() == m_preproc.size())
        {
            int imaxx = INT_MIN;
            int iminx = INT_MAX;
            int imaxy = INT_MIN;
            int iminy = INT_MAX;

            // project all measurements using ideal measurement angles
            // also determine bounds of the X,Y coordinates
            for (size_t nn = 0; nn < m_preproc.size(); nn++)
            {
                double mag = rscan[nn] * scale;
                int dx = static_cast<int>((rveccs[nn].x * mag) + 0.5);
                int dy = static_cast<int>((rveccs[nn].y * mag) + 0.5);
                m_preproc[nn].pt = { dx, dy };
                m_preproc[nn].range = mag;
                
                imaxx = max(imaxx, dx);
                iminx = min(iminx, dx);
                imaxy = max(imaxy, dy);
                iminy = min(iminy, dy);
            }

            // set bounding box around projected points
            m_preproc_bbox = Rect(Point(iminx, iminy), Point(imaxx, imaxy));

            // check for adjacent points that are too far away from each other
            // they are likely not on the same surface and can be ignored
            // the threshold is distance between two measurements at max LIDAR range
            const double len_thr = MAX_RNG_LIDAR * scale * tan(ANG_STEP_LIDAR * CV_PI / 180.0);
            for (size_t nn = 0; nn < m_preproc.size(); nn++)
            {
                // look at 3 neighboring points
                Point ptn = m_preproc[(nn + m_preproc.size() - 1) % m_preproc.size()].pt;
                Point pt0 = m_preproc[nn].pt;
                Point ptp = m_preproc[(nn + 1) % m_preproc.size()].pt;

                // get vectors from ptn to pt0 and pt0 to ptp
                // add them and take atan2 to get an "average" angle for the 2 vectors
                double dxn = pt0.x - ptn.x;
                double dyn = pt0.y - ptn.y;
                double dxp = ptp.x - pt0.x;
                double dyp = ptp.y - pt0.y;
                double dx = dxn + dxp;
                double dy = dyn + dyp;
                double ang = (atan2(dy, dx) * 180.0) / CV_PI;
                m_preproc[nn].ang = (ang < 0.0) ? ang + 360.0 : ang;

                // apply range threshold
                double rn = sqrt((dxn * dxn) + (dyn * dyn));
                double rp = sqrt((dxp * dxp) + (dyp * dyp));
                m_preproc[nn].flags = ((rp < len_thr) && (rn < len_thr)) ? 1 : 0;
            }

            // finally note the sensing point in the scan image
            //Point rpt0 = { 0 - rbox.x, 0 - rbox.y };
        }
    }

    void GHSLAM::draw_preprocessed_scan(
        cv::Mat& rimg,
        cv::Point& rpt0,
        const int shrink)
    {
        // create image same size as bounding box along with some padding
        Size imgsz = Size(
            (m_preproc_bbox.width / shrink) + (2 * PAD_BORDER),
            (m_preproc_bbox.height / shrink) + (2 * PAD_BORDER));
        rimg = Mat::zeros(imgsz, CV_8UC1);
            
        // shift points so they will be centered in image
        std::vector<Point> pts;
        pts.resize(m_preproc.size());
        for (size_t nn = 0; nn < m_preproc.size(); nn++)
        {
            Point ptnew = {
                ((m_preproc[nn].pt.x - m_preproc_bbox.x) / shrink) + PAD_BORDER,
                ((m_preproc[nn].pt.y - m_preproc_bbox.y) / shrink)  + PAD_BORDER };
            pts[nn] = ptnew;
        }

        // draw lines between scan points that met "closeness" criteria
        for (size_t nn = 0; nn < m_preproc.size(); nn++)
        {
            T_SAMPLE& rsamp0 = m_preproc[nn];
            T_SAMPLE& rsamp1 = m_preproc[(nn + 1) % pts.size()];
            Point pt0 = pts[nn];
            Point pt1 = pts[(nn + 1) % pts.size()];

            if (rsamp0.flags && rsamp1.flags)
            {
                line(rimg, pt0, pt1, 255, m_mask_line_width);
            }
        }

        // finally note the sensing point in the scan image
        rpt0 = {
            ((0 - m_preproc_bbox.x) / shrink) + PAD_BORDER,
            ((0 - m_preproc_bbox.y) / shrink) + PAD_BORDER };
    }


    void GHSLAM::update_scan_templates(const std::vector<double>& rscan)
    {
#if 0
        const size_t sz = GMARR_SZ;
        for (size_t ii = 0; ii < sz; ii++)
        {
            Mat img_scan;
            Mat img_grad;

            Point pt0;
            draw_preprocessed_scan(img_scan, pt0, ii, rscan);

            Point tpt0_mid = (img_scan.size() / 2);
            tpt0_offset[ii] = pt0 - tpt0_mid;

            if (ii == sz / 2)
            {
                img_scan.copyTo(m_img_template_ang_0);
                m_pt0_template_ang_0 = pt0;
            }
        }
#endif
    }


    void GHSLAM::perform_match(
        const std::vector<double>& rscan,
        cv::Point& roffset,
        double& rang)
    {
#if 0
        const size_t sz = GMARR_SZ;

        // convert scan to an image (no rotation)
        draw_preprocessed_scan(m_img_scan, m_pt0_scan, sz / 2, rscan);

        Point tptq_mid = (m_img_scan.size() / 2);
        Point tptq_offset = m_pt0_scan - tptq_mid;

        // get template for running match ???

        // search for best orientation match
        // this match will also provide the translation
        // but an "un-rotation" is required after the best match is found
        // (instead of linear search, maybe the previous match could be start for this match ???)
        size_t qidmax = 0;
        double qallmax = 0.0;
        Point qptmax_offset;
        for (size_t ii = 0; ii < sz; ii++)
        {
            Mat img_match;
            double qmax = 0.0;
            Point qptmax = { 0, 0 };

            // GH ???

            double qmaxtotal = 0.0;
            if (qmaxtotal > 0.0)
            {
                qmax = qmax / qmaxtotal;
                if (qmax > qallmax)
                {
                    qallmax = qmax;
                    qidmax = ii;
                    qptmax_offset = qptmax - tptq_mid;
                }
            }
        }

        roffset = tptq_offset - tpt0_offset[qidmax] - qptmax_offset;
        rang = scan_angs_offsets[qidmax];

        // un-rotate offset by matched orientation angle
        Point p0 = roffset;
        double rang_rad = rang * CV_PI / 180.0;
        double cos0 = cos(rang_rad);
        double sin0 = sin(rang_rad);
#if 0
        roffset.x = static_cast<int>(p0.x * cos0 - p0.y * sin0);
        roffset.y = static_cast<int>(p0.x * sin0 + p0.y * cos0);
#else
        roffset.x = static_cast<int>( p0.x * cos0 + p0.y * sin0);
        roffset.y = static_cast<int>(-p0.x * sin0 + p0.y * cos0);
#endif
#endif
    }


    void GHSLAM::add_waypoint(GHSLAM::T_WAYPOINT& rwp)
    {
        m_waypoints.push_back(rwp);
    }
}
