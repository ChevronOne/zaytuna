

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





#include "zay_model_vehicle.hpp"

namespace zaytuna {

glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c, const double angle, const bool ccw);

glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2, const glm::dvec3 p3, const glm::dvec3 p4);



model_vehicle::model_vehicle() : STEERING_WHEEL(0.0), MOVEMENT_SPEED(0.0), ORIENTATION(0.0), accumulate_dist(0.0), traveled_dist(0.0), rem_dist(0.0), ticks(0), rad(0.0), cen(glm::dvec3(0.0,0.0,0.0))
{
    vehic_direction = glm::dvec3(1.0, 0.0, 0.0);
    up_direction = glm::dvec3(0.0, 1.0, 0.0);
//    bit = old_bit = glm::dvec3(-0.129, 0.0, 0.0);
    bit = old_bit = glm::dvec3(0.0, 0.0, 0.0);


    fit = glm::dvec3(0.157 , 0.0 , 0.0);
//    frontCamDir = glm::dvec3(0.12 , 0.2 , 0.0);

    transform = glm::translate(glm::dvec3(0.0, 0.0, 0.0));
    _rotation = glm::rotate(0.0, up_direction);


//    frontCam.view_direction = vehic_direction;
    frontCam.camera_position = camPos ;// glm::dvec3(0.13, 0.466, 0.0); // bit + camHeight; //  glm::dvec3(0.129, 0.466, 0.0); //
//    frontCam.view_direction = frontCamDir - frontCam.camera_position;
    frontCam.view_direction = glm::dvec3(0.5 , 0.17 , 0.0) - frontCam.camera_position;

    hRotation = glm::rotate(0.0, glm::dvec3(0.0, 0.0, 1.0));

}

glm::dmat4 model_vehicle::update_rotationMat(void)
{
    _rotation =  glm::translate(cen) * glm::rotate(glm::radians(ORIENTATION), up_direction) * glm::translate(-cen);
    return _rotation;
}

void model_vehicle::move_forward(const float& time)
{
    std::cout << time<< "\n";
}

void model_vehicle::move_backward(const float& time)
{
    std::cout << time<< "\n";
}

void model_vehicle::actuate_vehic(const float& time)
{
    std::cout << time<< "\n";
}

double model_vehicle::getRadius()
{
    return rad;
}

double model_vehicle::getTraveled_dist()
{
    return  traveled_dist;
}

void model_vehicle::update_rotation_att(const double& time)
{


    glm::dvec3 p1, p2;

    if(STEERING_WHEEL > 0){
        p1 = rotate(fit, bit, -M_PI_2, true);
        p2 = rotate(bit, fit, -(M_PI_2-STEERING_WHEEL), false);
    }else{
        p1 = rotate(fit, bit, M_PI_2, false);
        p2 = rotate(bit, fit, M_PI_2-STEERING_WHEEL, true);
    }

    // the radius of rotation
    rad = std::abs( L/std::tan(STEERING_WHEEL));

    // center of rotaion
    cen = intersect(bit, p1, fit, p2);

    // amount of rotaion
    ORIENTATION = (std::tan(STEERING_WHEEL) * MOVEMENT_SPEED * time)/L;

    update_rotationMat();

    transform = _rotation * transform;

    old_bit = bit;

    bit =  _rotation * glm::dvec4(bit, 1.0);
    fit =  _rotation * glm::dvec4(fit, 1.0);
//    if(MOVEMENT_SPEED != 0.0)
//        vehic_direction = _rotation * glm::dvec4(1.0, 0.0, 0.0, 1.0);

//    frontCamDir = _rotation * glm::dvec4(frontCamDir, 1.0);

//    // traveled dist from last fram is given by the length of arch bwtween old position and new position
//    double d = std::sqrt( std::pow( old_bit.x-bit.x,2)+std::pow(old_bit.z-bit.z ,2) );
//    traveled_dist =  rad * std::acos(1.0 - (d*d)/(2*rad*rad) );

//    std::cout << "orientation: " << ORIENTATION << "\n";
//    std::cout << "rad: " << rad << "\n";
    traveled_dist = glm::radians(std::abs(ORIENTATION)) * rad;

//    frontCam.camera_position = bit + camHeight;
    frontCam.camera_position = transform * camPos;


//    frontCam.view_direction =  frontCamDir - frontCam.camera_position;

//    if(MOVEMENT_SPEED != 0.0){
        vehic_direction = glm::mat3(glm::rotate(glm::radians(ORIENTATION), up_direction)) * vehic_direction;
        frontCam.view_direction = glm::mat3(glm::rotate(glm::radians(ORIENTATION), up_direction)) * frontCam.view_direction;
//    }

//    vehic_direction = glm::mat3(glm::rotate(glm::radians(ORIENTATION), up_direction)) * vehic_direction;


}

glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c, const double angle, const bool ccw){
    double _sin = std::sin(angle);
    double _cos = std::cos(angle);
    p -= c;

    if(ccw){ // ccw: counter clockwise rotation
        p = glm::dvec3(  p.x * _cos + p.z * _sin, 0.0f,
                         p.z * _cos - p.x * _sin);

    }else{ // cw: clockwise rotation
        p = glm::dvec3(  p.x * _cos - p.z * _sin, 0.0f,
                         p.z * _cos + p.x * _sin);
    }

    return p+c;
}

glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2, const glm::dvec3 p3, const glm::dvec3 p4){
    double temp = ( (p4.x - p3.x)*(p1.z-p3.z) - (p4.z-p3.z)*(p1.x-p3.x) ) /
            ((p4.z - p3.z) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.z - p1.z));

    return glm::dvec3{ p1.x + temp * (p2.x - p1.x), 0.0f, p1.z + temp * (p2.z - p1.z) };
}


void model_vehicle::update_steerin()
{

}

} // namespace  zaytuna


//=====================================================================================
//std::ostream& operator<<(std::ostream& out, const glm::vec3& vec)
//{
//    out.precision(std::numeric_limits<float>::max_digits10);
//    return out << std::fixed <<
//         ": " << vec.x <<
//         " , " << vec.y <<
//         " , " << vec.z << "\n";

//}

//glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c, const double angle, const bool ccw);

//glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2, const glm::dvec3 p3, const glm::dvec3 p4);



//model_vehicle::model_vehicle() : STEERING_WHEEL(0.0), MOVEMENT_SPEED(0.0), ORIENTATION(0.0), accumulate_dist(0.f), traveled_dist(0.0), rem_dist(0.0), ticks(0), rad(0.0), cen(glm::dvec3(0.0,0.0,0.0))
//{
//    vehic_direction = glm::dvec3(1.0, 0.0, 0.0);
//    up_direction = glm::dvec3(0.0, 1.0, 0.0);
//    bit = old_bit = glm::dvec3(-0.129, 0.0, 0.0);


//    fit = glm::dvec3(0.157 , 0.0 , 0.0);
////    frontCamDir = glm::dvec3(0.12 , 0.2 , 0.0);

//    transform = glm::translate(glm::dvec3(0.0, 0.0, 0.0));
//    _rotation = glm::rotate(0.0, up_direction);


////    frontCam.view_direction = vehic_direction;
//    frontCam.camera_position = glm::dvec3(0.0, 0.466, 0.0); // bit + camHeight; //  glm::dvec3(0.129, 0.466, 0.0); //
////    frontCam.view_direction = frontCamDir - frontCam.camera_position;
//    frontCam.view_direction = glm::dvec3(0.5 , 0.17 , 0.0) - frontCam.camera_position;

////    glm::dmat4 m1 = glm::translate(glm::vec3(0.0, 0.0, 0.0));
////    glm::dmat4 m2 = glm::translate(glm::vec3(0.0, 0.0, 0.0));

////    glm::mat4 m = m1 * m2;
//}

//glm::dmat4 model_vehicle::update_rotationMat(void)
//{
//    _rotation =  glm::translate(cen) * glm::rotate(glm::radians(ORIENTATION), up_direction) * glm::translate(-cen);
////    _rotation =  glm::rotate(glm::radians(ORIENTATION), up_direction);
//    return _rotation;
//}

//void model_vehicle::move_forward(const float& time)
//{
//    std::cout << time<< "\n";
//}

//void model_vehicle::move_backward(const float& time)
//{
//    std::cout << time<< "\n";
//}

//void model_vehicle::actuate_vehic(const float& time)
//{
//    std::cout << time<< "\n";
//}

//double model_vehicle::getRadius()
//{
//    return rad;
//}

//float model_vehicle::getTraveled_dist()
//{
//    return  traveled_dist;
//}

//void model_vehicle::update_rotation_att(const double& time)
//{

//    // rotation_as , p2
//    // vehic_position , p1

//    glm::dvec3 p1, p2;

//    if(STEERING_WHEEL > 0){
//        p1 = rotate(fit, bit, -M_PI_2, true);
//        p2 = rotate(bit, fit, -(M_PI_2-STEERING_WHEEL), false);
//    }else{
//        p1 = rotate(fit, bit, M_PI_2, false);
//        p2 = rotate(bit, fit, M_PI_2-STEERING_WHEEL, true);
//    }

//    // the radius of rotation
//    rad = std::abs( L/std::tan(STEERING_WHEEL));

//    // center of rotaion
//    cen = intersect(bit, p1, fit, p2);

//    // amount of rotaion
//    ORIENTATION = (std::tan(STEERING_WHEEL) * MOVEMENT_SPEED * time)/L;

//    update_rotationMat();

//    transform = _rotation * transform;

//    old_bit = bit;

//    bit =  _rotation * glm::dvec4(bit, 1.0);
//    fit =  _rotation * glm::dvec4(fit, 1.0);
////    frontCamDir = _rotation * glm::dvec4(frontCamDir, 1.0);

//    // traveled dist from last fram is given by the length of arch bwtween old position and new position
//    double d = std::sqrt( std::pow( old_bit.x-bit.x,2)+std::pow(old_bit.z-bit.z ,2) );
//    traveled_dist =  rad * std::acos(1.0 - (d*d)/(2*rad*rad) );


////    frontCam.camera_position = bit + camHeight;
//    frontCam.camera_position = transform * glm::dvec4(glm::dvec3(0.0, 0.466, 0.0), 1.0);


////    frontCam.view_direction =  frontCamDir - frontCam.camera_position;

//    frontCam.view_direction = glm::mat3(glm::rotate(glm::radians(ORIENTATION), up_direction)) * frontCam.view_direction;
////    vehic_direction = glm::mat3(glm::rotate(glm::radians(ORIENTATION), up_direction)) * vehic_direction;


//}

//glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c, const double angle, const bool ccw){
//    double _sin = std::sin(angle);
//    double _cos = std::cos(angle);
//    p -= c;

//    if(ccw){ // ccw: counter clockwise rotation
//        p = glm::dvec3(  p.x * _cos + p.z * _sin, 0.0f,
//                         p.z * _cos - p.x * _sin);

//    }else{ // cw: clockwise rotation
//        p = glm::dvec3(  p.x * _cos - p.z * _sin, 0.0f,
//                         p.z * _cos + p.x * _sin);
//    }

//    return p+c;
//}

//glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2, const glm::dvec3 p3, const glm::dvec3 p4){
//    double temp = ( (p4.x - p3.x)*(p1.z-p3.z) - (p4.z-p3.z)*(p1.x-p3.x) ) /
//            ((p4.z - p3.z) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.z - p1.z));

//    return glm::dvec3{ p1.x + temp * (p2.x - p1.x), 0.0f, p1.z + temp * (p2.z - p1.z) };
//}


//void model_vehicle::update_steerin()
//{

//}
