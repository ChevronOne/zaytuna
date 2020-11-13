

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

camera::camera() : ROTATION_SPEED{0.3}, MOVEMENT_SPEED{0.024},
            FIELD_OF_VIEW{55.0}, NEAR_PLANE{0.01},
            FAR_PLANE{400.0}, auto_perspective{1},
            view_direction{glm::dvec3(-0.41 , -0.6 , -0.68)},
            up_direction{glm::dvec3(0.0, 1.0, 0.0)},
            camera_position{glm::dvec3(8.55, 3.26, 3.5)},
            view_point{glm::dvec3(0.0, 0.0, 0.0)},
            mouse_position{glm::dvec2(0.0, 0.0)},
            projectionMat{glm::perspective(glm::radians(FIELD_OF_VIEW),
                                           static_cast<double>(ZAY_SCENE_WIDTH) / ZAY_SCENE_HEIGHT,
                                           NEAR_PLANE,
                                           FAR_PLANE)},
            world_to_viewMat{glm::lookAt(camera_position,
                                         camera_position + view_direction,
                                         up_direction)},
            transformationMat{projectionMat * world_to_viewMat}{}

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
//    if(camera_position.y < 0.1)
//        camera_position.y = 0.1;
//    if(camera_position.y > 100.0)
//        camera_position.y = 100.0;

   if(camera_position.y < 0.011)
       camera_position.y = 0.011;
   if(camera_position.y > 100.0)
       camera_position.y = 100.0;

   if(camera_position.x < -148.9)
       camera_position.x = -148.9;
   if(camera_position.x > 148.9)
       camera_position.x = 148.9;

   if(camera_position.z < -148.9)
       camera_position.z = -148.9;
   if(camera_position.z > 148.9)
       camera_position.z = 148.9;

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
    if (glm::length(mouse_delta) > ZAY_MOUSE_DELTA_IGNORE)
    {
        mouse_position = new_mouse_position;
        return;
    }

    //// v rotation
    view_direction =
            glm::dmat3(glm::rotate(glm::radians(-mouse_delta.x*ROTATION_SPEED),
                                  up_direction))
                                  * view_direction;
    //// h rotation
    view_direction = glm::dmat3(glm::rotate(glm::radians(-mouse_delta.y*ROTATION_SPEED),
                                            glm::cross(view_direction,
                                                       up_direction)))
                     * view_direction;
    mouse_position = new_mouse_position;
}


void camera::mouse_held_update(const glm::dvec2& new_mouse_position){
    glm::dvec2 mouse_delta = new_mouse_position - mouse_position;
    if (glm::length(mouse_delta) > ZAY_MOUSE_DELTA_IGNORE || view_direction.y == 0.0)
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
                                                     up_direction);
    glm::dmat4 pos_rotationMat = glm::translate(view_point) * rotaionMat * glm::translate(-view_point);

    view_direction = glm::mat3(rotaionMat) * view_direction;
    camera_position = pos_rotationMat * glm::dvec4(camera_position, 1.0);

    //// h rotation
    rotaionMat = glm::rotate(glm::radians(-mouse_delta.y*ROTATION_SPEED),
                                        glm::cross(view_direction,
                                                   up_direction));
    pos_rotationMat = glm::translate(view_point) * rotaionMat * glm::translate(-view_point);

    view_direction = glm::mat3(rotaionMat) * view_direction;
    camera_position = pos_rotationMat * glm::dvec4(camera_position, 1.0);

    mouse_position = new_mouse_position;
}

glm::dvec2 camera::get_mouse_position(){
    return mouse_position;
}

void camera::move_forward(double scalar){
    camera_position += MOVEMENT_SPEED * scalar * view_direction;
}

void camera::move_backward(double scalar){
    camera_position += -MOVEMENT_SPEED * scalar * view_direction;
}

void camera::move_horizontal_forward(double scalar){
    camera_position += MOVEMENT_SPEED * scalar 
            * glm::normalize(glm::dvec3(view_direction.x, 0.0, view_direction.z));
}

void camera::move_horizontal_backward(double scalar){
    camera_position += -MOVEMENT_SPEED * scalar 
            * glm::normalize(glm::dvec3(view_direction.x, 0.0, view_direction.z));
}

void camera::move_up(double scalar){
    camera_position += MOVEMENT_SPEED * scalar * up_direction;
}

void camera::move_down(double scalar){
    camera_position += -MOVEMENT_SPEED * scalar * up_direction;
}

void camera::strafe_right(double scalar){
    glm::dvec3 strafe_direction = glm::cross(view_direction, up_direction);
    camera_position += MOVEMENT_SPEED * scalar * strafe_direction;
}

void camera::strafe_left(double scalar)
{
    glm::dvec3 strafe_direction = glm::cross(view_direction, up_direction);
    camera_position += -MOVEMENT_SPEED * scalar * strafe_direction;
}

} // namespace  zaytuna



