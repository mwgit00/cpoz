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

#ifndef GHSLAM_H_
#define GHSLAM_H_

#include <vector>
#include <list>

namespace cpoz
{
    class GHSLAM
    {
    public:

        typedef struct _T_SAMPLE_struct
        {
            cv::Point pt;
            double ang;
            double range;
            uint8_t angcode;
            bool is_range_ok;
        } T_SAMPLE;

        typedef std::vector<T_SAMPLE> tVecSamples;

        typedef struct _T_GHSLAM_WAYPOINT_struct
        {
            cv::Point pt;               ///< best-guess location of scan
            double ang;                 ///< best-guress orientation of scan
            std::vector<double> scan;   ///< original scan data
        } T_WAYPOINT;

        GHSLAM();
        virtual ~GHSLAM();

        void init_scan_angs(
            const double offset_step,
            const size_t offset_ct);

        size_t get_scan_ang_ct(void) const { return m_scan_ang_ct; }
        double get_match_scale(void) const { return mscale; }
        uint8_t get_ang_step(void) const { return m_angcode_ct; }
        
        const std::vector<double>& get_scan_angs(void) const;

        void preprocess_scan(
            tVecSamples& rvec,
            cv::Rect& rbbox,
            const size_t offset_index,
            const std::vector<double>& rscan);

        void draw_preprocessed_scan(
            cv::Mat& rimg,
            cv::Point& rpt0,
            const GHSLAM::tVecSamples& rvec,
            const cv::Rect& rbbox,
            const int shrink = 4);

        void update_match_templates(const std::vector<double>& rscan);

        void perform_match(
            const std::vector<double>& rscan,
            cv::Point& roffset,
            double& rang);

        void add_waypoint(GHSLAM::T_WAYPOINT& rwp);

        const std::list<GHSLAM::T_WAYPOINT>& get_waypoints(void) const { return m_waypoints; }

    public:

        cv::Mat m_img_scan;
        //cv::Point m_pt0_scan;

        cv::Mat m_img_template_ang_0;   // 0 degree match template for display
        cv::Point m_pt0_template_ang_0; // center of 0 degree match template for display

        std::list<T_WAYPOINT> m_waypoints;

    private:

        size_t m_scan_ang_ct;   ///< number of angles (elements) in a LIDAR scan
        double m_scan_ang_min;  ///< negative angle from 0 (front)
        double m_scan_ang_max;  ///< positive angle from 0 (front)
        double m_scan_ang_step; ///< step between angles in LIDAR scan
        double m_scan_max_rng;  ///< max range possible from LIDAR

        cv::Point slam_loc; ///< calculated position
        double slam_ang;    ///< calculated heading

        double mscale;          ///< scale for doing matching
        uint8_t m_angcode_ct;      ///< angle step for matching

        std::vector<double> scan_angs;          ///< ideal scan angles
        std::vector<double> scan_angs_offsets;  ///< offsets for angle search
        std::vector<std::vector<cv::Point2d>> scan_cos_sin; ///< ideal cos and sin for scan angles

        std::vector<cv::Point> tpt0_offset;         ///< center points for all templates
    };
}

#endif GHSLAM_H_
