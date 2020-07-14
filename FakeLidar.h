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

#ifndef FAKE_LIDAR_H_
#define FAKE_LIDAR_H_

#include <string>
#include <vector>

namespace cpoz
{
    class FakeLidar
    {
    public:
        FakeLidar();
        virtual ~FakeLidar();

        void load_floorplan(const std::string& rspath);

        void set_pos(const cv::Point& rpt) { pos = rpt; }
        void init_scan_angs(const double deg0, const double deg1, const double step);

        void run_scan(std::vector<double>& rvec);
        void run_scan2(std::vector<double>& rvec);
        void draw_scan(cv::Mat& rimg, const std::vector<double>& rvec);

    private:
        int sample_ct;
        cv::Mat img_floorplan;
        cv::Point pos;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<double> scan_angs;
        std::vector<cv::Point2d> scan_cos_sin;
    };
}

#endif // FAKE_LIDAR_H_
