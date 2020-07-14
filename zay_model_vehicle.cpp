

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





#include "zay_model_vehicle.hpp"
#include "zay_scene_widg.hpp"
//#include "zay_headers.hpp"

namespace zaytuna {


// for debugging
extern double GLOBAL_MOVEMENT_SPEED;
extern double GLOBAL_STEERING_WHEEL;

glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c,
                  const double angle, const bool ccw);

glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2,
                     const glm::dvec3 p3, const glm::dvec3 p4);


zaytuna::vehicle_attribute::vehicle_attribute(const std::string& _name,
                                    const glm::dmat4 _rotaion,
                                    const glm::dmat4 _translation):
    name{_name}, elapsed_t{0.0}, AMOUNT_OF_ROTATION(0.0), MOVEMENT_SPEED(0.0),
    STEERING_WHEEL(0.00000001), accumulated_dist(0.0), traveled_dist(0.0),
    ticks_counter(0), radius_of_rotation(0.0), center_of_rotation(glm::dvec3(0.0,0.0,0.0))
{
    frontCam.FIELD_OF_VIEW = 55.0; // sould be adjusted!
    update_positional_attributes(_translation, _rotaion);

    transformationMats[1] = f_rightT;
    transformationMats[2] = f_leftT;
    transformationMats[3] = backT;
    transformationMats[4] = lidar;
}

void vehicle_attribute::update_positional_attributes(const glm::dmat4 &translation_,
                                                 const glm::dmat4 &rotation_)
{
    transformationMats[0] = translation_*rotation_;

    vehic_direction = glm::dvec3(1.0, 0.0, 0.0);
    back_ideal_tire = old_back_ideal_tire = glm::dvec3(0.0, 0.0, 0.0);
    front_ideal_tire = glm::dvec3(0.2862 , 0.0 , 0.0);

    rotationMat = glm::rotate(0.0, up_direction);

    vehic_direction = glm::dmat3(rotation_) * vehic_direction;
    frontCam.view_direction  = glm::dmat3(rotation_)
                               * frontCam.view_direction;

    back_ideal_tire = old_back_ideal_tire = transformationMats[0]
                                        * glm::vec4(back_ideal_tire, 1.0);
    front_ideal_tire = transformationMats[0]
                        * glm::vec4(front_ideal_tire, 1.0);

    frontCam.camera_position = transformationMats[0] * camPos;
    frontCam.view_direction  = rotation_
                            * (frontCamViewDirection - camPos);
    frontCam.updateWorld_to_viewMat();

    tires_hRotation = glm::rotate(amount_of_hRotation,
                                  glm::dvec3(0.0, 0.0, 1.0));
    front_tires_vRotation = glm::rotate(STEERING_WHEEL,
                                        glm::dvec3(0.0, 1.0, 0.0));
}


void vehicle_attribute::update_attribs()
{
    MOVEMENT_SPEED = GLOBAL_MOVEMENT_SPEED;
    STEERING_WHEEL = GLOBAL_STEERING_WHEEL;
    if(MOVEMENT_SPEED != 0.0){

        update_rotation_att();
        accumulated_dist = traveled_dist + accumulated_dist;
        ticks_counter = static_cast<uint32_t>(accumulated_dist/meters_per_tick);
        accumulated_dist = fmod(accumulated_dist, meters_per_tick);
    } else
        timer_t = std::chrono::high_resolution_clock::now();
    actuate();
}

void vehicle_attribute::update_rotation_att()
{
    // the radius of rotation
    radius_of_rotation = std::abs( front_back_distance/std::tan(STEERING_WHEEL));

    // center of rotaion
    if(STEERING_WHEEL > 0)
        center_of_rotation =
                (glm::normalize(glm::cross(vehic_direction, up_direction))
                 * radius_of_rotation)
                + back_ideal_tire;
    else
        center_of_rotation =
                (-glm::normalize(glm::cross(vehic_direction, up_direction))
                 * radius_of_rotation)
                + back_ideal_tire;


    elapsed_t = std::chrono::duration<double,
            std::ratio< 1, 1>>
            (std::chrono::high_resolution_clock::now() - timer_t).count();
    timer_t = std::chrono::high_resolution_clock::now();

    // amount of rotaion
    AMOUNT_OF_ROTATION = (std::tan(STEERING_WHEEL)
                          * MOVEMENT_SPEED * elapsed_t)/front_back_distance;


    rotationMat =  glm::translate(center_of_rotation)
            * glm::rotate(glm::radians(AMOUNT_OF_ROTATION), up_direction)
            * glm::translate(-center_of_rotation);

    transformationMats[0] = rotationMat * transformationMats[0];

    old_back_ideal_tire = back_ideal_tire;

    back_ideal_tire =  rotationMat * glm::dvec4(back_ideal_tire, 1.0);
    front_ideal_tire =  rotationMat * glm::dvec4(front_ideal_tire, 1.0);

    traveled_dist = glm::radians(std::abs(AMOUNT_OF_ROTATION))
                    * radius_of_rotation;

    frontCam.camera_position = transformationMats[0] * camPos;



    vehic_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                            up_direction))
                      * vehic_direction;
    frontCam.view_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                                    up_direction))
                              * frontCam.view_direction;


}

void vehicle_attribute::actuate()
{
    if(std::abs(STEERING_WHEEL) < 0.0001){
        transformationMats[1] =  transformationMats[0] * f_rightT;
        transformationMats[2] = transformationMats[0] * f_leftT;
    }else{
        front_tires_vRotation = glm::rotate(-STEERING_WHEEL,
                                            glm::dvec3(0.0, 1.0, 0.0));
        transformationMats[1] =  transformationMats[0]
                                 * f_rightT * front_tires_vRotation;
        transformationMats[2] = transformationMats[0]
                                * f_leftT * front_tires_vRotation;
    }

    transformationMats[3] = transformationMats[0]  * backT;

    if(MOVEMENT_SPEED != 0.0){
        if(MOVEMENT_SPEED < 0.0)
            amount_of_hRotation -= traveled_dist/tires_radius;
        else
            amount_of_hRotation += traveled_dist/tires_radius;

        if(amount_of_hRotation>PI2)
            amount_of_hRotation -= PI2;
        else if (amount_of_hRotation< -PI2)
            amount_of_hRotation+=PI2;

        tires_hRotation = glm::rotate(amount_of_hRotation,
                                      glm::dvec3(0.0, 0.0, 1.0));

    }
    transformationMats[1] = glm::dmat4(transformationMats[1]) * tires_hRotation;
    transformationMats[2] = glm::dmat4(transformationMats[2]) * tires_hRotation;
    transformationMats[3] = glm::dmat4(transformationMats[3]) * tires_hRotation;


    transformationMats[4] = transformationMats[0] * lidar
            * glm::rotate(glm::radians(amount_of_rotation_Lidar <= -360 ?
                amount_of_rotation_Lidar = 0
                : amount_of_rotation_Lidar -= lidar_spin_speed),
                  glm::dvec3(0.0, 1.0, 0.0));

    frontCam.updateWorld_to_viewMat();
}


vehicle_attribute::~vehicle_attribute()
{
//    clean_up();
}

void vehicle_attribute::pubFront_img()
{
    local_cam_img.save((name+".jpg").c_str());
}


//===================================================================

// old approache
glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c,
                  const double angle, const bool ccw){
    double _sin = std::sin(angle);
    double _cos = std::cos(angle);
    p -= c;

    if(ccw)// ccw: counter clockwise rotation
        p = glm::dvec3(  p.x * _cos + p.z * _sin, 0.0f,
                         p.z * _cos - p.x * _sin);

    else // cw: clockwise rotation
        p = glm::dvec3(  p.x * _cos - p.z * _sin, 0.0f,
                         p.z * _cos + p.x * _sin);


    return p+c;
}

glm::dvec3 intersect(const glm::dvec3 p1,
                     const glm::dvec3 p2,
                     const glm::dvec3 p3,
                     const glm::dvec3 p4){
    double temp = ( (p4.x - p3.x)*(p1.z-p3.z) - (p4.z-p3.z)*(p1.x-p3.x) ) /
            ((p4.z - p3.z) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.z - p1.z));

    return glm::dvec3{ p1.x + temp * (p2.x - p1.x), 0.0f, p1.z + temp * (p2.z - p1.z) };
}


void vehicle_attribute::update_steerin()
{

}

void vehicle_attribute::update_cent()
{
        glm::dvec3 p1, p2;

        if(STEERING_WHEEL > 0){
            p1 = rotate(front_ideal_tire,
                        back_ideal_tire,
                        -M_PI_2, true);
            p2 = rotate(back_ideal_tire,
                        front_ideal_tire,
                        -(M_PI_2-STEERING_WHEEL), false);
        }else{
            p1 = rotate(front_ideal_tire,
                        back_ideal_tire,
                        M_PI_2, false);
            p2 = rotate(back_ideal_tire,
                        front_ideal_tire,
                        M_PI_2-STEERING_WHEEL, true);
        }

        // center of rotaion
        center_of_rotation = intersect(back_ideal_tire, p1,
                                       front_ideal_tire, p2);
}



} // namespace  zaytuna





