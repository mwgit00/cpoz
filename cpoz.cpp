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

#include <iostream>

#include "room1.h"
#include "CameraHelper.h"
#include "XYZLandmark.h"


const double EPS = 0.01;

CameraHelper cam;


void landmark_test(
    const XYZLandmark& lm1,
    const XYZLandmark& lm2,
    const cv::Vec3d& cam_xyz,
    const cv::Vec2d& cam_angs)
{
    // for the two landmarks :
    // - translate landmark by camera offset
    // - rotate by azimuth and elevation
    // - project into image
    
    double azim = cam_angs[0];
    double elev = cam_angs[1];
    
    // determine pixel location of fixed LM 1
    cv::Vec3d xyz1 = lm1.xyz - cam_xyz;
    cv::Vec3d xyz1_rot = calc_xyz_after_rotation_deg(xyz1, elev, azim, 0);
    cv::Vec2d uv1 = cam.project_xyz_to_uv(xyz1_rot);

    // determine pixel location of left / right LM
    cv::Vec3d xyz2 = lm2.xyz - cam_xyz;
    cv::Vec3d xyz2_rot = calc_xyz_after_rotation_deg(xyz2, elev, azim, 0);
    cv::Vec2d uv2 = cam.project_xyz_to_uv(xyz2_rot);

    if (cam.is_visible(uv1) && cam.is_visible(uv2))
    {
        // all good
    }
    else
    {
        //print
        //print "Image Landmark //1:", uv1
        //print "Image Landmark //2:", uv2
        //print "At least one landmark is NOT visible!"
        //return False, 0., 0., 0.
    }
    
    //
    //// all is well so proceed with test...
    //
    //// landmarks have been acquired
    //// camera elevation and world Y also need updating
    //cam.elev = _ele * pu.DEG2RAD
    //cam.world_y = _y
    //
    //lm1.set_current_uv(uv1)
    //lm2.set_current_uv(uv2)
    //world_x, world_z, world_azim = cam.triangulate_landmarks(lm1, lm2)
    //
    //// this integer coordinate stuff is disabled for now...
    //if False:
    //print "Now try with integer pixel coords and known Y coords..."
    //lm1.set_current_uv((int(u1 + 0.5), int(v1 + 0.5)))
    //lm2.set_current_uv((int(u2 + 0.5), int(v2 + 0.5)))
    //print lm1.uv
    //print lm2.uv
    //
    //world_x, world_z, world_azim = cam.triangulate_landmarks(lm1, lm2)
    //print "Robot is at", world_x, world_z, world_azim * pu.RAD2DEG
    //print
    //
    //return True, world_x, world_z, world_azim * pu.RAD2DEG
}



bool room_test(
    const tMapStrToAZEL& lm_vis,
    const cv::Vec3d& xyz,
    const std::string& lm_name,
    const double elev_offset = 0.0)
{
    bool result = true;
    for (const auto& r : lm_vis)
    {
        double cam_azim = r.second.azim;
        double cam_elev = r.second.elev + elev_offset;
        cv::Vec2d angs = { cam_azim, cam_elev };

        tMapStrToXYZRL& lm_map = all_landmarks[lm_name];

        bool flag = false;
        double x = 0;
        double z = 0;
        double a = 0;

//        flag, x, z, a = landmark_test(mark1[key], markx[key], xyz, angs);
        
        if (!flag)
        {
            result = false;
        }

        if (abs(x - xyz[0]) >= EPS)
        {
            result = false;
        }

        if (abs(z - xyz[2]) >= EPS)
        {
            result = false;
        }

        if ((abs(a - cam_azim) >= EPS) && (abs(a - 360.0 - cam_azim) >= EPS))
        {
            result = false;
        }
    }
    
    return result;
}



//class TestUtil(unittest.TestCase) :
//
//    def test_room_x1_z1_y2_lm2_elev00(self) :
//    // LM name mapped to[world_azim, elev] for visibility at world(1, 1)
//    // has one case where one landmark is not visible
//    xyz = [1., -2., 1.]
//    self.assertFalse(room_test(lm_vis_1_1, xyz, "mark2"))
//
//    def test_room_x1_z1_y2_lm3_elev00(self):
//        // LM name mapped to[world_azim, elev] for visibility at world(1, 1)
//            xyz = [1., -2., 1.]
//            self.assertTrue(room_test(lm_vis_1_1, xyz, "mark3"))
//
//            def test_room_x1_z1_y2_lm2_elev10(self) :
//            // LM name mapped to[world_azim, elev] for visibility at world(1, 1)
//            // camera is at(1, 1) and -2 units high, elevation offset 10 degrees
//            xyz = [1., -2., 1.]
//            self.assertTrue(room_test(lm_vis_1_1, xyz, "mark2", elev_offset = 10.0))
//
//            def test_room_x1_z1_y2_lm3_elev10(self) :
//            // LM name mapped to[world_azim, elev] for visibility at world(1, 1)
//            xyz = [1., -2., 1.]
//            self.assertTrue(room_test(lm_vis_1_1, xyz, "mark3", elev_offset = 10.0))
//
//            def test_room_x1_z1_y3_lm_2elev00(self) :
//            // LM name mapped to[world_azim, elev] for visibility at world(1, 1)
//            xyz = [1., -3., 1.]
//            self.assertTrue(room_test(lm_vis_1_1, xyz, "mark2"))
//
//            def test_room_x1_z1_y3_lm2_elev00(self) :
//            // LM name mapped to[world_azim, elev] for visibility at world(1, 1)
//            xyz = [1., -3., 1.]
//            self.assertTrue(room_test(lm_vis_1_1, xyz, "mark3"))
//
//            def test_room_x7_z6_y2_lm2_elev00(self) :
//            // LM name mapped to[world_azim, elev] for visibility at world(7, 6)
//            // camera is at(7, 6) and -2 units high
//            xyz = [7., -2., 6.]
//            self.assertTrue(room_test(lm_vis_7_6, xyz, "mark2"))
//
//            def test_room_x7_z6_y2_lm3_elev00(self) :
//            // LM name mapped to[world_azim, elev] for visibility at world(7, 6)
//            // camera is at(7, 6) and -2 units high
//            xyz = [7., -2., 6.]
//            self.assertTrue(room_test(lm_vis_7_6, xyz, "mark3"))
//
//



int main()
{
    std::cout << "CPOZ Test Application\n";
}
