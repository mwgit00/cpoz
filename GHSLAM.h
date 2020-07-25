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
#include "GradientMatcher.h"

namespace cpoz
{
    class GHSLAM
    {
    public:
        GHSLAM();
        virtual ~GHSLAM();

        void init_scan_angs(
            const double deg0,
            const double deg1,
            const double step,
            const double offset_step,
            const size_t offset_ct);

        const std::vector<double>& get_scan_angs(void) const;

        void scan_to_img(
            cv::Mat& rimg,
            cv::Mat& rimgmask,
            cv::Point& rpt0,
            const size_t offset_index,
            const std::vector<double>& rscan);

        void update_scan_templates(const std::vector<double>& rscan);

        void perform_match(
            const std::vector<double>& rscan,
            cv::Point& roffset,
            double& rang);

    public:

        cv::Mat m_img_scan;
        cv::Mat m_img_mask;
        cv::Mat m_img_grad;
        cv::Point m_pt0_scan;

    private:
        cv::Point slam_loc;  ///< calculated position
        double slam_ang;     ///< calculated heading

        double mscale;
        
        std::vector<double> scan_angs;          ///< ideal scan angles
        std::vector<double> scan_angs_offsets;  ///< offsets for angle search
        std::vector<std::vector<cv::Point2d>> scan_cos_sin; ///< ideal cos and sin for scan angles

        std::vector<ghalgo::GradientMatcher> gmarr;
        std::vector<cv::Point> tpt0_offset;
    };
}

#endif GHSLAM_H_
