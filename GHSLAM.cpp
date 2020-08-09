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
#include <map>

#include "GHSLAM.h"



namespace cpoz
{
    using namespace cv;

    const int PAD_BORDER = 25;              // add some space around border of scan images

    // cheap COTS LIDAR:  8000 samples/s, 2Hz-10Hz ???

    class cmpPtByXY
    {
    public:
        bool operator()(const cv::Point& a, const cv::Point& b) const
        {
            if (a.x == b.x)
                return a.y < b.y;
            return a.x < b.x;
        }
    };

    GHSLAM::GHSLAM() :
        m_scan_ang_ct(341),                 // 340 degree scan (+1 for 0 degree sample)
        m_scan_ang_min(-170.0),             // 20 degree "blind spot" behind robot
        m_scan_ang_max(170.0),
        m_scan_ang_step(1.0),               // 1 degree between each measurement
        m_scan_max_rng(1200.0),             // 12m max range from LIDAR
        slam_loc({ 0, 0 }),
        slam_ang(0.0),
        mscale(0.25),
        m_angcode_ct(8),
        m_search_ang_ct(61),
        m_search_ang_step(1.0)              // 1 degree between each search step
    {
        tpt0_offset.resize(m_search_ang_ct);

        init_scan_angs();
    }
    
    
    GHSLAM::~GHSLAM()
    {
        // does nothing
    }


    void GHSLAM::init_scan_angs(void)
    {
        // flush old data
        scan_angs.clear();
        scan_angs_offsets.clear();
        scan_cos_sin.clear();

        // generate ideal scan angles
        scan_angs.resize(m_scan_ang_ct);
        double ang = m_scan_ang_min;
        for (size_t ii = 0; ii < m_scan_ang_ct; ii++)
        {
            scan_angs[ii] = ang;
            ang += m_scan_ang_step;
        }

        // generate lookup tables for cosine and sine
        // for a range of offsets from ideal scan angles
        double ang_offset = -m_search_ang_step * (m_search_ang_ct / 2);
        for (size_t nn = 0; nn < m_search_ang_ct; nn++)
        {
            scan_angs_offsets.push_back(ang_offset);
            scan_cos_sin.push_back({});
            
            for (const auto& rang : scan_angs)
            {
                double ang_rad = (rang + ang_offset) * CV_PI / 180.0;
                scan_cos_sin.back().push_back(cv::Point2d(cos(ang_rad), sin(ang_rad)));
            }

            ang_offset += m_search_ang_step;
        }
    }

    
    void GHSLAM::preprocess_scan(
        tVecSamples& rvec,
        cv::Rect& rbbox,
        const size_t offset_index,
        const std::vector<double>& rscan)
    {
        // look up the 0 degree cos and sin table
        std::vector<Point2d>& rveccs = scan_cos_sin[offset_index];
        const size_t sz = rvec.size();
        
        if (rscan.size() == sz)
        {
            Point ptmin = { INT_MAX, INT_MAX };
            Point ptmax = { INT_MIN, INT_MIN };

            // project all measurements using ideal measurement angles
            // also determine bounds of the X,Y coordinates
            for (size_t nn = 0; nn < rvec.size(); nn++)
            {
                double mag = rscan[nn];
                int dx = static_cast<int>((rveccs[nn].x * mag) + 0.5);
                int dy = static_cast<int>((rveccs[nn].y * mag) + 0.5);
                rvec[nn].pt = { dx, dy };
                rvec[nn].range = mag;
                
                ptmax.x = max(ptmax.x, dx);
                ptmin.x = min(ptmin.x, dx);
                ptmax.y = max(ptmax.y, dy);
                ptmin.y = min(ptmin.y, dy);
            }

            // set bounding box around projected points
            rbbox = Rect(ptmin, ptmax);

            // check for adjacent points that are too far away from each other
            // they are likely not on the same surface and can be ignored
            // the threshold is distance between two measurements at max LIDAR range
            const double len_thr = m_scan_max_rng * tan(m_scan_ang_step * CV_PI / 180.0);
            for (size_t nn = 0; nn < rvec.size(); nn++)
            {
                // look at 3 neighboring points
                Point ptn = rvec[(nn + sz - 1) % sz].pt;
                Point pt0 = rvec[nn].pt;
                Point ptp = rvec[(nn + 1) % sz].pt;

                // get vectors from ptn to pt0 and pt0 to ptp
                // add them and take atan2 to get an "average" angle for the 2 vectors
                Point vn0 = pt0 - ptn;
                Point v0p = ptp - pt0;
                Point vsum = vn0 + v0p;
                double angdeg = atan2(vsum.y, vsum.x) * 180.0 / CV_PI;
                angdeg = (angdeg < 0.0) ? angdeg + 360.0 : angdeg;
                rvec[nn].ang = angdeg;

                // convert angle to a byte code
                // at 360 the angle code is equal to m_angcode_ct so wrap it back
                // to angle code 0 to keep angle code in range 0 to (m_angcode_ct - 1)
                rvec[nn].angcode = static_cast<uint8_t>((m_angcode_ct * angdeg) / 360.0);
                if (rvec[nn].angcode == m_angcode_ct)
                {
                    rvec[nn].angcode = 0;
                }

                // apply range threshold
                double rn = sqrt((vn0.x * vn0.x) + (vn0.y * vn0.y));
                double rp = sqrt((v0p.x * v0p.x) + (v0p.y * v0p.y));
                rvec[nn].is_range_ok = ((rp < len_thr) && (rn < len_thr));
            }
        }
    }


    void GHSLAM::draw_preprocessed_scan(
        cv::Mat& rimg,
        cv::Point& rpt0,
        const GHSLAM::tVecSamples& rvec,
        const cv::Rect& rbbox,
        const int shrink)
    {
        // create image same size as bounding box along with some padding
        Size imgsz = Size(
            (rbbox.width / shrink) + (2 * PAD_BORDER),
            (rbbox.height / shrink) + (2 * PAD_BORDER));
        rimg = Mat::zeros(imgsz, CV_8UC1);
            
        // shift points so they will be centered in image
        std::vector<Point> pts;
        pts.resize(rvec.size());
        for (size_t nn = 0; nn < rvec.size(); nn++)
        {
            Point ptnew = {
                ((rvec[nn].pt.x - rbbox.x) / shrink) + PAD_BORDER,
                ((rvec[nn].pt.y - rbbox.y) / shrink)  + PAD_BORDER };
            pts[nn] = ptnew;
        }

        for (size_t nn = 0; nn < rvec.size(); nn++)
        {
            // @TODO -- add an option for this ???
            if (false)
            {
                // draw lines between scan points that meet "closeness" criteria
                const T_SAMPLE& rsamp0 = rvec[nn];
                const T_SAMPLE& rsamp1 = rvec[(nn + 1) % pts.size()];
                Point pt0 = pts[nn];
                Point pt1 = pts[(nn + 1) % pts.size()];
                if (rsamp0.is_range_ok && rsamp1.is_range_ok)
                {
                    line(rimg, pt0, pt1, 255, 1);
                }
            }
            else
            {
                // just draw a dot
                circle(rimg, pts[nn], 0, 255, 1);
            }
        }

        // finally note the sensing point in the scan image
        rpt0 = {
            ((0 - rbbox.x) / shrink) + PAD_BORDER,
            ((0 - rbbox.y) / shrink) + PAD_BORDER };
    }


    void GHSLAM::update_match_templates(const std::vector<double>& rscan)
    {
        m_vtemplates.clear();
        m_vtemplates.resize(m_search_ang_ct);
        
        for (size_t ii = 0; ii < m_search_ang_ct; ii++)
        {
            m_vtemplates[ii].vsamp.resize(m_scan_ang_ct);
            preprocess_scan(m_vtemplates[ii].vsamp, m_vtemplates[ii].bbox, ii, rscan);
            
            m_vtemplates[ii].lookup.resize(m_angcode_ct);
            
            tVecSamples& rvec = m_vtemplates[ii].vsamp;
            for (size_t jj = 0; jj < rvec.size(); jj++)
            {
                T_SAMPLE& rsamp = rvec[jj];
                if (rsamp.is_range_ok)
                {
                    m_vtemplates[ii].lookup[rsamp.angcode].push_back(rsamp.pt);
                }
            }
        }
    }


    void GHSLAM::perform_match(
        const std::vector<double>& rscan,
        cv::Point& roffset,
        double& rang)
    {
        tVecSamples vsamp;
        cv::Rect bbox;
        vsamp.resize(m_scan_ang_ct);

        const int BLOOM_BOUNDS = 50;
        const int BLOOM_SIZE = 2;
        const int BLOOM_PAD = BLOOM_BOUNDS + BLOOM_SIZE;

        m_img_foo = Mat::zeros(BLOOM_PAD * 2 + 1, BLOOM_PAD * 2 + 1, CV_16U);

        // use 0 angle
        preprocess_scan(vsamp, bbox, m_search_ang_ct / 2, rscan);

        //for (size_t jj = 0; jj < m_search_ang_ct; jj++)
        size_t jj = m_search_ang_ct / 2;
        {
            T_TEMPLATE& rt = m_vtemplates[jj];

            for (size_t ii = 0; ii < vsamp.size(); ii++)
            {
                T_SAMPLE& rs = vsamp[ii];

                if (rs.is_range_ok)
                {
                    for (const auto& rpt : rt.lookup[rs.angcode])
                    {
                        Point votept = rs.pt - rpt;

                        if ((abs(votept.x) < BLOOM_BOUNDS) && (abs(votept.y) < BLOOM_BOUNDS))
                        {
                            // bloom
                            for (int mm = -BLOOM_SIZE; mm <= BLOOM_SIZE; mm++)
                            {
                                for (int nn = -BLOOM_SIZE; nn <= BLOOM_SIZE; nn++)
                                {
                                    int q = BLOOM_SIZE + 1 - max(abs(nn), abs(mm));
                                    Point d = { mm, nn };
                                    Point boo = { BLOOM_PAD, BLOOM_PAD };
                                    Point e = votept + d + boo;
                                    uint16_t upix = m_img_foo.at<uint16_t>(e);
                                    upix += static_cast<uint16_t>(q);
                                    m_img_foo.at<uint16_t>(e) = upix;
                                }
                            }
                        }
                    }
                }
            }
        }

#if 0
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
