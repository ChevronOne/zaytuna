

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
 * Copyright Abbas Mohammed Murrey 2019-21
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



vehicle_attributes::vehicle_attributes
            (ZAY_USED_GL_VERSION* _widg,
             QGLFramebufferObject *const FBO_,
             const veh_transform_attribs<GLdouble> attribs,
             rect_collistion_object<GLdouble> const*const proj_rect,
             rect_collistion_pack<GLdouble>* coll_objs,
             zaytuna::vehicle_state<GLdouble>* v_state,
             ZAY_MSG_LOGGER* message_logger):
        _widg{_widg}, attribs{attribs}, coll_rect_orig{proj_rect}, coll_rect{*proj_rect}, 
        coll_pack{coll_objs}, v_state{v_state}, zay_msg_logger{message_logger}, elapsed_t{0.0},
        AMOUNT_OF_ROTATION{0.0}, MOVEMENT_SPEED{0.0}, DESIRED_SPEED{0.0},
        STEERING_WHEEL(ZAY_STEERING_MARGIN_OF_ERROR), DESIRED_STEERING{0.0}, 
        accumulated_dist(0.0),
        traveled_dist(0.0), ticks_counter(0),
        radius_of_rotation(0.0),
        center_of_rotation(glm::dvec3(0.0,0.0,0.0)){

    frontCam.FIELD_OF_VIEW = ZAY_DEFAULT_FIELD_OF_VIEW; // should be adjusted!
    topics.reset(new vehicle_topics<double, void_alloc>(attribs.name, FBO_));

    coll_rect.ID = attribs.name;
    update_positional_attributes(attribs);
    
    transformationMats[1] = f_rightT;
    transformationMats[2] = f_leftT;
    transformationMats[3] = backT;
    transformationMats[4] = lidar;
}


void vehicle_attributes::update_positional_attributes
        (const veh_transform_attribs<GLdouble> attribs_){

    this->attribs = attribs_;
    transformationMats[0] = attribs_.transformMat();

    vehic_direction = glm::dvec3(1.0, 0.0, 0.0);
    back_ideal_tire = old_back_ideal_tire = glm::dvec3(0.0, 0.0, 0.0);
    front_ideal_tire = glm::dvec3(0.2862 , 0.0 , 0.0);

    rotationMat = glm::rotate(0.0, up_direction);

    topics->vehicle_orientation = vehic_direction =
            glm::dmat3(attribs_.rotationMat()) * vehic_direction;

    topics->vehicle_pos = back_ideal_tire =
            old_back_ideal_tire = transformationMats[0]
            * glm::vec4(back_ideal_tire, 1.0);

    front_ideal_tire = transformationMats[0]
                        * glm::vec4(front_ideal_tire, 1.0);

    frontCam.camera_position = transformationMats[0] * camPos;
    frontCamViewDirection = glm::dvec4(cos(glm::radians(attribs_.front_cam_v_angle-90.0)), 
                                       cos(glm::radians(180.0-attribs_.front_cam_v_angle)), 
                                       0.0, 1.0);
    
    frontCam.view_direction  = attribs_.rotationMat()
                            * (frontCamViewDirection - camPos);
    frontCam.updateWorld_to_viewMat();

    tires_hRotation = glm::rotate(amount_of_hRotation,
                                  glm::dvec3(0.0, 0.0, 1.0));
    front_tires_vRotation = glm::rotate(STEERING_WHEEL,
                                        glm::dvec3(0.0, 1.0, 0.0));
    frontCam.updateProjection(ZAY_SCENE_WIDTH, ZAY_SCENE_HEIGHT);

    topics->vehicle_geometry.update(back_ideal_tire, front_ideal_tire, vehic_direction);


    coll_rect.points = coll_rect_orig->points;
    // coll_rect.ID = attribs_.name;
    for(glm::dvec3& vec:coll_rect.points)
        vec = transformationMats[0]*glm::dvec4(vec,1.0);
    coll_rect.uniqueV[0] = coll_rect.points[1]-coll_rect.points[0];
    coll_rect.uniqueV[1] = coll_rect.points[2]-coll_rect.points[1];

    is_collided = 0;
    if(collision_tester == nullptr){
        collision_tester = std::make_unique<coll_tester<GLdouble>>
            (&attribs.name, &is_collided, &elapsed_t, &collided_object, &coll_rect, coll_pack);

        collision_tester->detach();
    }

}


void vehicle_attributes::update_actuators_commands(const double frame_rate){
    
    if(is_detached){
        DESIRED_SPEED = topics->REMOTE_SPEED;
        DESIRED_STEERING = topics->REMOTE_STEERING;
    }else{
        DESIRED_SPEED =  v_state->MOVEMENT_SPEED;
        DESIRED_STEERING = v_state->STEERING_WHEEL;
    }

    if(std::abs(MOVEMENT_SPEED-DESIRED_SPEED)>ZAY_DELAY_MARGIN_OF_ERROR)
        MOVEMENT_SPEED+= (DESIRED_SPEED-MOVEMENT_SPEED)/(frame_rate*speed_delay);
    else
        MOVEMENT_SPEED = DESIRED_SPEED;
    
    if(std::abs(DESIRED_STEERING-STEERING_WHEEL)>ZAY_DELAY_MARGIN_OF_ERROR)
        STEERING_WHEEL+= (DESIRED_STEERING-STEERING_WHEEL)/(frame_rate*steering_delay);
    else
        STEERING_WHEEL=DESIRED_STEERING;

    if((MOVEMENT_SPEED == 0.0) & collision_msg)
        collision_msg^=1;
    
    if(is_collided &(MOVEMENT_SPEED != 0.0)){
        MOVEMENT_SPEED = 0.0;
        if(!collision_msg){
            std::string warn_msg{attribs.name + " is/was collided with "+collided_object+"! relocate the vehicle first"};
            zay_msg_logger->setStyleSheet("color:red");
            zay_msg_logger->showMessage(warn_msg.c_str(), 5000);
            ROS_WARN_STREAM(warn_msg);
            collision_msg^=1;
        }

    }
}


void vehicle_attributes::update_attribs(const double frame_rate)
{

    update_actuators_commands(frame_rate);

    if(MOVEMENT_SPEED != 0.0){

        update_rotation_att();
        accumulated_dist = traveled_dist + accumulated_dist;
        topics->current_ticks = ticks_counter = static_cast<uint32_t>(accumulated_dist/meters_per_tick);
        accumulated_dist = fmod(accumulated_dist, meters_per_tick);
        if(collision_tester == nullptr){
            collision_tester = std::make_unique<coll_tester<GLdouble>>
                (&attribs.name, &is_collided, &elapsed_t, &collided_object, &coll_rect, coll_pack);
            collision_tester->detach();
        }

    }else{
        if(collision_tester != nullptr){
            collision_tester->terminate();
            collision_tester.reset(nullptr);
        }
        elapsed_t = durSec(std::chrono::high_resolution_clock::now() - timer_t).count();
        timer_t = std::chrono::high_resolution_clock::now();
    }
    actuate();
}


void vehicle_attributes::update_rotation_att()
{
    // the radius of rotation
    radius_of_rotation = std::abs( front_back_distance/std::tan(STEERING_WHEEL));

    // center of rotaion
    if(STEERING_WHEEL > 0.0)
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

    topics->vehicle_pos = back_ideal_tire =  rotationMat * glm::dvec4(back_ideal_tire, 1.0);
    front_ideal_tire =  rotationMat * glm::dvec4(front_ideal_tire, 1.0);

    update_projection_rect();

    traveled_dist = glm::radians(std::abs(AMOUNT_OF_ROTATION))
                    * radius_of_rotation;

    frontCam.camera_position = transformationMats[0] * camPos;

    topics->vehicle_orientation = vehic_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                            up_direction))
                      * vehic_direction;
    frontCam.view_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                                    up_direction))
                              * frontCam.view_direction;

}


void vehicle_attributes::update_projection_rect(void){
    for(glm::dvec3& vec:coll_rect.points)
        vec = rotationMat*glm::dvec4(vec,1.0);
    coll_rect.uniqueV[0] = coll_rect.points[1]-coll_rect.points[0];
    coll_rect.uniqueV[1] = coll_rect.points[2]-coll_rect.points[1];
}


void vehicle_attributes::actuate()
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
                : amount_of_rotation_Lidar -= (lidar_spin_speed*elapsed_t)),
                  glm::dvec3(0.0, 1.0, 0.0));


    frontCam.updateWorld_to_viewMat();
    topics->is_collided = is_collided;
    topics->advertise_current_state(back_ideal_tire, 
                                    front_ideal_tire, 
                                    vehic_direction);

}


vehicle_attributes::~vehicle_attributes(){
    topics->shutdown();
    MOVEMENT_SPEED=0.0;
    while(topics->ok()){
        std::this_thread::sleep_for(10ms);
    }
}


veh_transform_attribs<GLdouble>
vehicle_attributes::current_state(void){

    GLdouble angle{glm::degrees
                (glm::angle(glm::dvec3(1.0, 0.0, 0.0),
                 vehic_direction))};

    attribs.angle = vehic_direction.z < 0.0 ? angle:-angle;
    attribs.translation_vec = back_ideal_tire;

    return attribs;

}





} // namespace  zaytuna





