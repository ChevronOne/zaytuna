

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
//  This software is distributed in the hope that it will be useful, but WITHOUT
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

#include "zay_headers.hpp"
#include "zay_utility.hpp"
#include "zay_cam.hpp"
#include "zay_shape_data.hpp"

namespace zaytuna {

class model_vehicle;
class _scene_widg;


class vehicle_attribute
{
//    Q_OBJECT

    QImage local_cam_img;

    std::chrono::time_point<std::chrono::_V2::system_clock,
                    std::chrono::nanoseconds> timer_t;


    std::string name{"uninitialized object name"};

    double elapsed_t;


    friend class model_vehicle;
    friend class _scene_widg;


public:
    vehicle_attribute() = default;
    explicit vehicle_attribute(const std::string&,
                 const glm::dmat4 _rotaion = glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation = glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    ~vehicle_attribute();

    void actuate();


    glm::dvec3 vehic_direction;
    const glm::dvec3 up_direction{0.0, 1.0, 0.0};
    glm::dvec3 front_ideal_tire; // position of front ideal tire
    glm::dvec3 back_ideal_tire;  //  position of back ideal tire
    glm::dvec3 old_back_ideal_tire; // position of back ideal tire in the previous frame


    glm::dmat4 rotationMat; // rotation matrix of model vehicle;

    double AMOUNT_OF_ROTATION; // amount of rotation of the model vehicle 'per frame'
    double MOVEMENT_SPEED;  //  need to be adjusted for m/s
    double STEERING_WHEEL;  // in degrees, 'amount of vertical rotation of the wheels'
    double amount_of_hRotation{0.0}; // amount of horizontal rotation of the wheels
    double amount_of_rotation_Lidar{0.0}; // amount of rotation of lidar
    double lidar_spin_speed{10.0}; // per frame


    double accumulated_dist; // accumulated distance
    double traveled_dist; // traveled distance per frame
    uint32_t ticks_counter; // ticks counter per frame


    double radius_of_rotation; // radius of rotation of the model vehicle

    glm::dvec3 center_of_rotation; // center of rotation of the model vehicle

    void update_attribs();
    void update_rotation_att();
    void update_steerin(void);
    void update_cent(void);
    void update_positional_attributes(const glm::dmat4&, const glm::dmat4&);

    void pubFront_img(void);


    glm::dmat4 transformationMats[5]; // /model vehicle/, /right front tire/, /left front tire/, /back tires/, /lidar/
    glm::dmat4 tires_hRotation, // rotation matrix of horizontal rotation of the wheels
            front_tires_vRotation; // rotation matrix of vertical rotation of the front tires

    zaytuna::camera frontCam; // front camera


    // Default Positional Parameters
    const glm::dvec3 camHeight = glm::dvec3(0.0, 0.466, 0.0);
    const glm::dmat4 f_rightT = glm::translate(glm::dvec3( 0.286, 0.031,  0.159));
    const glm::dmat4 f_leftT =  glm::translate(glm::dvec3( 0.286, 0.031, -0.159));
    const glm::dmat4 backT =    glm::translate(glm::dvec3( 0.0,  0.031,    0.0));
    const glm::dmat4 lidar =    glm::translate(glm::dvec3( 0.118, 0.419,   0.0));
    const glm::dvec4 camPos =  glm::dvec4(0.13, 0.466, 0.0, 1.0);
    const glm::dvec4 frontCamViewDirection = glm::dvec4(0.5 , 0.278 , 0.0, 1.0); // should be adjusted!


    // Physical Specifications
    const double front_back_distance{0.2862}; // distance between back and front ideal tires
    const double ticks_per_meter{173.0}; // 173 ticks per meter 'can be adjusted'
    const double meters_per_tick{1.0/ticks_per_meter}; // meters per tick
    const double tires_radius{0.0311};
    const double PI2{2.0*M_PI};
    const double tires_circumference{PI2*tires_radius};



};



} // namespace zaytuna




#endif // ZAY_MODEL_VEHICLE_HPP




