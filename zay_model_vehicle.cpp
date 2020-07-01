

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



model_vehicle::model_vehicle() : STEERING_WHEEL(0.0), MOVEMENT_SPEED(0.0),
    AMOUNT_OF_ROTATION(0.0), accumulated_dist(0.0), traveled_dist(0.0),
    ticks_counter(0), radius_of_rotation(0.0), center_of_rotation(glm::dvec3(0.0,0.0,0.0))
{
    vehic_direction = glm::dvec3(1.0, 0.0, 0.0);
    back_ideal_tire = old_back_ideal_tire = glm::dvec3(0.0, 0.0, 0.0);

    front_ideal_tire = glm::dvec3(0.157 , 0.0 , 0.0);

//    transform = glm::translate(glm::dvec3(0.0, 0.0, 0.0));
    transformationMats[0] = glm::translate(glm::dvec3(0.0, 0.0, 0.0));
    rotationMat = glm::rotate(0.0, up_direction);


    frontCam.camera_position = camPos ;// glm::dvec3(0.13, 0.466, 0.0); // bit + camHeight; //  glm::dvec3(0.129, 0.466, 0.0); //
    frontCam.view_direction = glm::dvec3(0.5 , 0.17 , 0.0) - frontCam.camera_position;

//    hRotation = glm::rotate(0.0, glm::dvec3(0.0, 0.0, 1.0)); // xxxxxx
    tires_hRotation = glm::rotate(amount_of_hRotation, glm::dvec3(0.0, 0.0, 1.0));
    front_tires_vRotation = glm::rotate(STEERING_WHEEL, glm::dvec3(0.0, 1.0, 0.0));



    transformationMats[1] = f_rightT;
    transformationMats[2] = f_leftT;
    transformationMats[3] = backT;
    transformationMats[4] = lidar;

}


void model_vehicle::move_forward(const float& time)
{
    std::cout << time<< "\n";
}

void model_vehicle::move_backward(const float& time)
{
    std::cout << time<< "\n";
}

void model_vehicle::actuate()
{
    if(std::abs(STEERING_WHEEL) < 0.0001){
        transformationMats[1] =  transformationMats[0] * f_rightT;
        transformationMats[2] = transformationMats[0] * f_leftT;
    }else{
        front_tires_vRotation = glm::rotate(-STEERING_WHEEL, glm::dvec3(0.0, 1.0, 0.0));
        transformationMats[1] =  transformationMats[0] * f_rightT * front_tires_vRotation;
        transformationMats[2] = transformationMats[0] * f_leftT * front_tires_vRotation;
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

        tires_hRotation = glm::rotate(amount_of_hRotation, glm::dvec3(0.0, 0.0, 1.0));

    }
    transformationMats[1] = glm::dmat4(transformationMats[1]) * tires_hRotation;
    transformationMats[2] = glm::dmat4(transformationMats[2]) * tires_hRotation;
    transformationMats[3] = glm::dmat4(transformationMats[3]) * tires_hRotation;


    transformationMats[4] = transformationMats[0] * lidar
            * glm::rotate(glm::radians(amount_of_rotation_Lidar <= -360 ?
                                           amount_of_rotation_Lidar = 0 : amount_of_rotation_Lidar -= lidar_spin_speed),
                          glm::dvec3(0.0, 1.0, 0.0));
}

double model_vehicle::getRadius()
{
    return radius_of_rotation;
}

double model_vehicle::getTraveled_dist()
{
    return  traveled_dist;
}

void model_vehicle::update_attribs(const double & elapsed_t)
{
    if(MOVEMENT_SPEED != 0.0){

        update_rotation_att(elapsed_t);
        accumulated_dist = traveled_dist + accumulated_dist;
        ticks_counter = static_cast<uint32_t>(accumulated_dist/meters_per_tick);
        accumulated_dist = fmod(accumulated_dist, meters_per_tick);
    }
    actuate();
}

void model_vehicle::update_rotation_att(const double& elapsed_t)
{
    // the radius of rotation
    radius_of_rotation = std::abs( front_back_distance/std::tan(STEERING_WHEEL));

    // center of rotaion
    if(STEERING_WHEEL > 0)
        center_of_rotation = (glm::normalize(glm::cross(vehic_direction, up_direction))
                              * radius_of_rotation) + back_ideal_tire;
    else
        center_of_rotation = (-glm::normalize(glm::cross(vehic_direction, up_direction))
                              * radius_of_rotation) + back_ideal_tire;


    // amount of rotaion
    AMOUNT_OF_ROTATION = (std::tan(STEERING_WHEEL)
                          * MOVEMENT_SPEED * elapsed_t)/front_back_distance;


    rotationMat =  glm::translate(center_of_rotation) *
            glm::rotate(glm::radians(AMOUNT_OF_ROTATION), up_direction)
            * glm::translate(-center_of_rotation);

    transformationMats[0] = rotationMat * transformationMats[0];

    old_back_ideal_tire = back_ideal_tire;

    back_ideal_tire =  rotationMat * glm::dvec4(back_ideal_tire, 1.0);
    front_ideal_tire =  rotationMat * glm::dvec4(front_ideal_tire, 1.0);

    traveled_dist = glm::radians(std::abs(AMOUNT_OF_ROTATION)) * radius_of_rotation;

    frontCam.camera_position = transformationMats[0] * camPos;



    vehic_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION), up_direction)) * vehic_direction;
    frontCam.view_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION), up_direction)) * frontCam.view_direction;


}

void model_vehicle::render_the_model(QOpenGLFunctions_3_0* _widg,
            zaytuna::camera* activeCam,
            const GLint transformMatLocation,
            const GLint inverse_transpose_transformMatLocation)
{
    modeltransformMat = activeCam->transformationMat * transformationMats[0];
    inverse_transpose_transformMat =glm::inverse(glm::transpose(transformationMats[0]));

    _widg->glUseProgram(programID);
    _widg->glBindTexture(GL_TEXTURE_2D, textureID);


    // render the model
    _widg->glBindVertexArray(modelVAO_ID);
    _widg->glUniformMatrix4fv(transformMatLocation, 1, GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation, 1, GL_FALSE, &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(GL_TRIANGLES, modelNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(model_indOffset));


    // render lidar
    _widg->glBindVertexArray(lidarVAO_ID);
    modeltransformMat = activeCam->transformationMat * transformationMats[4];
    inverse_transpose_transformMat =glm::inverse(glm::transpose(transformationMats[4]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1, GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation, 1, GL_FALSE, &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(GL_TRIANGLES, lidarNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(lidar_indOffset));


    // render fronttires
    _widg->glBindVertexArray(fronttiresVAO_ID);
    modeltransformMat = activeCam->transformationMat * transformationMats[1];
    inverse_transpose_transformMat =glm::inverse(glm::transpose(transformationMats[1]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1, GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation, 1, GL_FALSE, &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(GL_TRIANGLES, fronttiresNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(fronttires_indOffset));

    modeltransformMat = activeCam->transformationMat * transformationMats[2];
    inverse_transpose_transformMat =glm::inverse(glm::transpose(transformationMats[2]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1, GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation, 1, GL_FALSE, &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(GL_TRIANGLES, fronttiresNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(fronttires_indOffset));


    // render back tires
    _widg->glBindVertexArray(backtiresVAO_ID);
    modeltransformMat = activeCam->transformationMat * transformationMats[3];
    inverse_transpose_transformMat =glm::inverse(glm::transpose(transformationMats[3]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1, GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation, 1, GL_FALSE, &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(GL_TRIANGLES, backtiresNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(backtires_indOffset));

}




//------------------------------------------------------------------------

// old approache
glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c, const double angle, const bool ccw){
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

glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2, const glm::dvec3 p3, const glm::dvec3 p4){
    double temp = ( (p4.x - p3.x)*(p1.z-p3.z) - (p4.z-p3.z)*(p1.x-p3.x) ) /
            ((p4.z - p3.z) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.z - p1.z));

    return glm::dvec3{ p1.x + temp * (p2.x - p1.x), 0.0f, p1.z + temp * (p2.z - p1.z) };
}


void model_vehicle::update_steerin()
{

}

void model_vehicle::get_cent()
{
        glm::dvec3 p1, p2;

        if(STEERING_WHEEL > 0){
            p1 = rotate(front_ideal_tire, back_ideal_tire, -M_PI_2, true);
            p2 = rotate(back_ideal_tire, front_ideal_tire, -(M_PI_2-STEERING_WHEEL), false);
        }else{
            p1 = rotate(front_ideal_tire, back_ideal_tire, M_PI_2, false);
            p2 = rotate(back_ideal_tire, front_ideal_tire, M_PI_2-STEERING_WHEEL, true);
        }



        // center of rotaion
        center_of_rotation = intersect(back_ideal_tire, p1, front_ideal_tire, p2);
}



} // namespace  zaytuna


