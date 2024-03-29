

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





#include "zay_cam.hpp"


namespace zaytuna {

camera::camera() : 
            FIELD_OF_VIEW{ZAY_DEFAULT_FIELD_OF_VIEW}, NEAR_PLANE{ZAY_CAM_ACCESSIBLE_MIN_Y},
            FAR_PLANE{ZAY_FIELD_DIAMETER}, auto_perspective{1},
            view_direction{ZAY_GLOB_CAM_DEFAULT_VIEW_DIR},
            camera_position{ZAY_GLOB_CAM_DEFAULT_POS},
            ROTATION_SPEED{ZAY_CAM_DEFAULT_ROTATION_SPEED}, 
            MOVEMENT_SPEED{ZAY_CAM_DEFAULT_MOVEMENT_SPEED},
            view_point{ZAY_ORIG_3},
            mouse_position{ZAY_ORIG_2}{

    projectionMat = glm::perspective(glm::radians(FIELD_OF_VIEW),
                                           static_cast<double>(ZAY_SCENE_WIDTH) / ZAY_SCENE_HEIGHT,
                                           NEAR_PLANE,
                                           FAR_PLANE);
    world_to_viewMat = glm::lookAt(camera_position,
                                 camera_position + view_direction,
                                 ZAY_CAM_UP_DIR);
    transformationMat = projectionMat * world_to_viewMat;
    
}


void camera::set_rotation_scalar(const double val_){
    ROTATION_SPEED=val_;
}


void camera::set_movement_scalar(const double val_){
    MOVEMENT_SPEED=val_;
}


double camera::get_rotation_scalar(void) const{
    return ROTATION_SPEED;
}


double camera::get_movement_scalar(void) const{
    return MOVEMENT_SPEED;
}


void camera::mouse_update(const glm::dvec2& new_mouse_position, const double rate_)
{

    glm::dvec2 mouse_delta = new_mouse_position - mouse_position;
    double l_{glm::length(mouse_delta)};
    
    if( (rate_<=0.0) | ((rate_>ZAY_MIN_FRAMES_STABLE) & ( l_ > ZAY_MOUSE_DELTA_IGNORE)) 
        | (l_ > ZAY_FRAMES_STABLE_SCALAR/rate_) ){

        mouse_position = new_mouse_position;
        return;

    }

    //// v rotation
    view_direction =
            glm::dmat3(glm::rotate(glm::radians(-mouse_delta.x*ROTATION_SPEED),
                                  ZAY_CAM_UP_DIR))
                                  * view_direction;
    //// h rotation
    view_direction = glm::dmat3(glm::rotate(glm::radians(-mouse_delta.y*ROTATION_SPEED),
                                            glm::cross(view_direction,
                                                       ZAY_CAM_UP_DIR)))
                     * view_direction;
    mouse_position = new_mouse_position;

}


void camera::mouse_held_update(const glm::dvec2& new_mouse_position, const double rate_){

    glm::dvec2 mouse_delta = new_mouse_position - mouse_position;
    double l_{glm::length(mouse_delta)};

    if ( (rate_<=0.0) | ((rate_>ZAY_MIN_FRAMES_STABLE) & ( l_ > ZAY_MOUSE_DELTA_IGNORE)) 
        | (l_ > ZAY_FRAMES_STABLE_SCALAR/rate_) | (view_direction.y == 0.0) )
    {

        mouse_position = new_mouse_position;
        return;

    }

    view_point = glm::dvec3(((-camera_position.y*view_direction.x)
                              +camera_position.x*view_direction.y)/view_direction.y,
                            0.0,
                            ((-camera_position.y*view_direction.z)
                              +camera_position.z*view_direction.y)/view_direction.y);

    //// v rotation
    glm::dmat4 rotaionMat = glm::rotate(glm::radians(-mouse_delta.x*ROTATION_SPEED),
                                                     ZAY_CAM_UP_DIR);
    glm::dmat4 pos_rotationMat = glm::translate(view_point) * rotaionMat * glm::translate(-view_point);

    view_direction = glm::mat3(rotaionMat) * view_direction;
    camera_position = pos_rotationMat * glm::dvec4(camera_position, 1.0);

    //// h rotation
    rotaionMat = glm::rotate(glm::radians(-mouse_delta.y*ROTATION_SPEED),
                                        glm::cross(view_direction,
                                                   ZAY_CAM_UP_DIR));
    pos_rotationMat = glm::translate(view_point) * rotaionMat * glm::translate(-view_point);

    view_direction = glm::mat3(rotaionMat) * view_direction;
    camera_position = pos_rotationMat * glm::dvec4(camera_position, 1.0);

    mouse_position = new_mouse_position;

}


void camera::move_forward(double scalar){
    camera_position += MOVEMENT_SPEED * scalar * view_direction;
}

void camera::move_backward(double scalar){
    camera_position -= MOVEMENT_SPEED * scalar * view_direction;
}

void camera::move_horizontal_forward(double scalar){
    camera_position += MOVEMENT_SPEED * scalar 
            * glm::normalize(glm::dvec3(view_direction.x, 0.0, view_direction.z));
}

void camera::move_horizontal_backward(double scalar){
    camera_position -= MOVEMENT_SPEED * scalar 
            * glm::normalize(glm::dvec3(view_direction.x, 0.0, view_direction.z));
}

void camera::move_up(double scalar){
    camera_position += MOVEMENT_SPEED * scalar * ZAY_CAM_UP_DIR;
}

void camera::move_down(double scalar){
    camera_position -= MOVEMENT_SPEED * scalar * ZAY_CAM_UP_DIR;
}

void camera::right_shifting(double scalar){
    glm::dvec3 strafe_direction = glm::cross(view_direction, ZAY_CAM_UP_DIR);
    camera_position += MOVEMENT_SPEED * scalar * strafe_direction;
}

void camera::left_shifting(double scalar)
{
    glm::dvec3 strafe_direction = glm::cross(view_direction, ZAY_CAM_UP_DIR);
    camera_position -= MOVEMENT_SPEED * scalar * strafe_direction;
}

void camera::updateProjection(const double w, const double h)
{
    projectionMat = glm::perspective(glm::radians(FIELD_OF_VIEW),
                                     w / h,
                                     NEAR_PLANE,
                                     FAR_PLANE);
    updateWorld_to_viewMat();
}

void camera::updateWorld_to_viewMat()
{
    
   if(camera_position.y < ZAY_CAM_ACCESSIBLE_MIN_Y)
       camera_position.y = ZAY_CAM_ACCESSIBLE_MIN_Y;
   else if(camera_position.y > ZAY_CAM_ACCESSIBLE_MAX_Y)
       camera_position.y = ZAY_CAM_ACCESSIBLE_MAX_Y;

   if(camera_position.x < ZAY_CAM_ACCESSIBLE_MIN_X)
       camera_position.x = ZAY_CAM_ACCESSIBLE_MIN_X;
   else if(camera_position.x > ZAY_CAM_ACCESSIBLE_MAX_X)
       camera_position.x = ZAY_CAM_ACCESSIBLE_MAX_X;

   if(camera_position.z < ZAY_CAM_ACCESSIBLE_MIN_Z)
       camera_position.z = ZAY_CAM_ACCESSIBLE_MIN_Z;
   else if(camera_position.z > ZAY_CAM_ACCESSIBLE_MAX_Z)
       camera_position.z = ZAY_CAM_ACCESSIBLE_MAX_Z;

    world_to_viewMat = glm::lookAt(
                camera_position,
                camera_position + view_direction,
                        ZAY_CAM_UP_DIR);
    transformationMat = projectionMat * world_to_viewMat;
}


} // namespace  zaytuna





