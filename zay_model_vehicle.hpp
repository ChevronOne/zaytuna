

//  Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
//
//  This file is part of the Zaytuna Simulator Project.
//
//  Distributed under the terms of the GNU General Public License
//  as published by the Free Software Foundation; You should have
//  received a copy of the GNU General Public License.
//  If not, see <http://www.gnu.org/licenses/>.
//
//
//  This library is distributed in the hope that it will be useful, but WITHOUT
//  WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
//  WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND
//  NON-INFRINGEMENT. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR ANYONE
//  DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY,
//  WHETHER IN CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. See the GNU
//  General Public License for more details.

/*
 * Copyright Abbas Mohammed Murrey 2019-20
 *
 * Permission to use, copy, modify, distribute and sell this software
 * for any purpose is hereby granted without fee, provided that the
 * above copyright notice appear in all copies and that both the copyright
 * notice and this permission notice appear in supporting documentation.
 * I make no representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */





#ifndef ZAY_MODEL_VEHICLE_HPP
#define ZAY_MODEL_VEHICLE_HPP

#define __GLM__

#include "zay_headers.hpp"
#include "zay_utility.hpp"
#include "zay_cam.hpp"

namespace zaytuna {


class model_vehicle
{
public:
    model_vehicle();
    const double L{0.24};

    glm::dmat4 update_rotationMat(void);

    void move_forward(const float&);
    void move_backward(const float&);
    void actuate_vehic(const float&);


    glm::dvec3 vehic_direction;
    const glm::dvec3 up_direction{0.0, 1.0, 0.0};
    glm::dvec3 bit;  //  position of back ideal tire

    glm::dvec3 fit; // position of front ideal tire
//    glm::dvec3 frontCamDir;

    glm::dmat4 transform, _rotation;
    glm::dmat4 hRotation;

    double STEERING_WHEEL;  // in degrees
    double MOVEMENT_SPEED;  //  need to be adjusted for m/s

    double ORIENTATION;


    glm::dvec3 old_bit;
    double accumulate_dist;
    double traveled_dist;
    double rem_dist;
    uint8_t ticks;

    double getRadius(void);
    double getTraveled_dist(void);

    double rad; // radius of rotation

    glm::dvec3 cen; // center of rotation


    void update_rotation_att(const double& time);
    void update_steerin(void);
    camera frontCam;
    const glm::dvec3 camHeight = glm::dvec3(0.0, 0.466, 0.0);

//    const glm::dmat4 f_rightT = glm::translate(glm::dvec3( 0.157, 0.031,  0.159));
//    const glm::dmat4 f_leftT =  glm::translate(glm::dvec3( 0.157, 0.031, -0.159));
//    const glm::dmat4 backT =    glm::translate(glm::dvec3(-0.129, 0.031,    0.0));
//    const glm::dmat4 lidar =    glm::translate(glm::dvec3(-0.013, 0.417,    0.0));

    const glm::dmat4 f_rightT = glm::translate(glm::dvec3( 0.286, 0.031,  0.159));
    const glm::dmat4 f_leftT =  glm::translate(glm::dvec3( 0.286, 0.031, -0.159));
    const glm::dmat4 backT =    glm::translate(glm::dvec3( 0.0,  0.031,    0.0));
    const glm::dmat4 lidar =    glm::translate(glm::dvec3( 0.118, 0.419,   0.0));
    const glm::dvec4 camPos =  glm::dvec4(0.13, 0.466, 0.0, 1.0);





};


} // namespace zaytuna


//===================================================================
//class model_vehicle
//{
//public:
//    model_vehicle();
//    const double L{0.24};

//    glm::dmat4 update_rotationMat(void);

//    void move_forward(const float&);
//    void move_backward(const float&);
//    void actuate_vehic(const float&);


//    glm::dvec3 vehic_direction;
//    glm::dvec3 up_direction;
//    glm::dvec3 bit;  //  position of back ideal tire

//    glm::dvec3 fit; // position of front ideal tire
////    glm::dvec3 frontCamDir;

//    glm::dmat4 transform, _rotation;

//    double STEERING_WHEEL;  // in degrees
//    double MOVEMENT_SPEED;  //  need to be adjusted for m/s

//    double ORIENTATION;


//    glm::dvec3 old_bit;
//    float accumulate_dist;
//    float traveled_dist;
//    float rem_dist;
//    uint8_t ticks;

//    double getRadius(void);
//    float getTraveled_dist(void);

//    double rad; // radius of rotation

//    glm::dvec3 cen; // center of rotation


//    void update_rotation_att(const double& time);
//    void update_steerin(void);
//    camera frontCam;
//    const glm::dvec3 camHeight = glm::dvec3(0.0, 0.466, 0.0);

//    const glm::dmat4 f_rightT = glm::translate(glm::dvec3( 0.157, 0.031,  0.159));
//    const glm::dmat4 f_leftT =  glm::translate(glm::dvec3( 0.157, 0.031, -0.159));
//    const glm::dmat4 backT =    glm::translate(glm::dvec3(-0.129, 0.031,    0.0));
//    const glm::dmat4 lidar =    glm::translate(glm::dvec3(-0.013, 0.417,    0.0));





//};



#endif // ZAY_MODEL_VEHICLE_HPP
