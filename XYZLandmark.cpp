// MIT License
// 
// Copyright(c) 2019 Mark Whitney
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this softwareand associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
// 
// The above copyright noticeand this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "cpoz_def.h"
#include "XYZLandmark.h"


XYZLandmark::XYZLandmark()
{

}

XYZLandmark::~XYZLandmark()
{

}


XYZLandmark::XYZLandmark(
    double x,
    double y,
    double z,
    double u1max,
    double u1min,
    const std::string& rs) :
    ang_u1max(u1max),
    ang_u1min(u1min),
    name(rs)
{
    xyz = { x, y, z };
}



void XYZLandmark::set_current_uv(double u, double v)
{
    uv = { u, v };
}


void XYZLandmark::calc_world_xz(double u, double ang, double r)
{
    double ang_adj = 0.0;
    if (uv.x > u)
    {
        ang_adj = ang_u1max * DEG2RAD - ang;
    }
    else
    {
        ang_adj = ang_u1min * DEG2RAD + ang;
    }

    // result is in X, Z plane so need negative sine in math below
    // to keep azimuth direction consistent (positive azimuth is clockwise)
    double world_x = xyz[0] + cos(ang_adj) * r;
    double world_z = xyz[2] - sin(ang_adj) * r;

//    return world_x, world_z
}


double XYZLandmark::calc_world_azim(double u, double ang, double rel_azim)
{
    double offset_rad = 0.0;
    double world_azim = 0.0;
    
    // there's a 90 degree rotation from camera view to world angle

    if (uv.x > u)
    {
        offset_rad = ang_u1max * DEG2RAD;
        world_azim = offset_rad - ang - rel_azim - (CV_PI / 2.0);
    }
    else
    {
        offset_rad = ang_u1min * DEG2RAD;
        world_azim = offset_rad + ang - rel_azim - (CV_PI / 2.0);
    }
     
    // clunky way ensure 0 <= world_azim < 360
    if (world_azim < 0.0) { world_azim += (2.0 * CV_PI); }
    if (world_azim < 0.0) { world_azim += (2.0 * CV_PI); }
    
    return world_azim;
}
