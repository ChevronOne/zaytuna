

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





#include "zay_cam.hpp"


namespace zaytuna {

camera::camera() : FIELD_OF_VIEW{55.0}, NEAR_PLANE{0.01}, 
            FAR_PLANE{400.0}, auto_perspective{1},
            view_direction{glm::dvec3(-0.41f , -0.6f , -0.68f)},
            up_direction{glm::dvec3(0.0f, 1.0f, 0.0f)},
            camera_position{glm::dvec3(+8.55f , 3.26f, 3.5f)} {
    MOVEMENT_SPEED = 0.024;
    ROTATION_SPEED = 0.3;
}

glm::dmat4 camera::getWorld_to_view_Mat()
{
    return glm::lookAt(
        camera_position,
        camera_position + view_direction,
                up_direction);
}

void camera::updateProjection(const double& w, const double& h)
{
    projectionMat = glm::perspective(glm::radians(FIELD_OF_VIEW),
                                     w / h,
                                     NEAR_PLANE,
                                     FAR_PLANE);
    updateWorld_to_viewMat();
}

void camera::updateWorld_to_viewMat()
{
   if(camera_position.y < 0.1)
       camera_position.y = 0.1;
   if(camera_position.y > 100.0)
       camera_position.y = 100.0;

//    if(camera_position.y < 0.011)
//        camera_position.y = 0.011;
//    if(camera_position.y > 100.0)
//        camera_position.y = 100.0;

   if(camera_position.x < -147.8)
       camera_position.x = -147.8;
   if(camera_position.x > 147.8)
       camera_position.x = 147.8;

   if(camera_position.z < -147.8)
       camera_position.z = -147.8;
   if(camera_position.z > 147.8)
       camera_position.z = 147.8;

    world_to_viewMat = glm::lookAt(
                camera_position,
                camera_position + view_direction,
                        up_direction);
    transformationMat = projectionMat * world_to_viewMat;
}

glm::dmat4 camera::getProjection(){
    return projectionMat;
}

camera::~camera(){}

void camera::mouse_update(const glm::dvec2& new_mouse_position)
{
    glm::dvec2 mouse_delta = new_mouse_position - mouse_position;
    if (glm::length(mouse_delta) > MOUSE_DELTA_IGNORE)
    {
        mouse_position = new_mouse_position;
        return;
    }
    view_direction =
            glm::dmat3(glm::rotate(glm::radians(-mouse_delta.x*ROTATION_SPEED),
                                  up_direction))
                                  * view_direction;

    glm::dvec3 verticalRotaionVec = glm::cross(view_direction,
                                               up_direction);
    view_direction = glm::dmat3(glm::rotate(glm::radians(-mouse_delta.y*ROTATION_SPEED),
                                            verticalRotaionVec))
                                            * view_direction;
    mouse_position = new_mouse_position;
}
glm::dvec2 camera::get_moutse_position(){
    return mouse_position;
}

void camera::move_forward(void){
    camera_position += MOVEMENT_SPEED * view_direction;
}

void camera::move_backward(void){
    camera_position += -MOVEMENT_SPEED * view_direction;
}

void camera::move_up(void){
    camera_position += MOVEMENT_SPEED * up_direction;
}

void camera::move_down(void){
    camera_position += -MOVEMENT_SPEED * up_direction;
}

void camera::strafe_right(void){
    glm::dvec3 strafe_direction = glm::cross(view_direction, up_direction);
    camera_position += MOVEMENT_SPEED * strafe_direction;
}

void camera::strafe_left(void)
{
    glm::dvec3 strafe_direction = glm::cross(view_direction, up_direction);
    camera_position += -MOVEMENT_SPEED * strafe_direction;
}

} // namespace  zaytuna



