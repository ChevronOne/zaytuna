

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





#include "zay_front_cam.hpp"

namespace zaytuna {

front_came::front_came()
{
    ROTATION_SPEED = 0;
    MOVEMENT_SPEED = 0.1;
}

void front_came::mouse_update(const glm::vec2& new_mouse_position)
{
    std::cout << new_mouse_position.x << "\n";
//    glm::vec2 mouse_delta = new_mouse_position - mouse_position;
//    if (glm::length(mouse_delta) > 15.0f)
//    {
//        mouse_position = new_mouse_position;
//        return;
//    }
//    view_direction = glm::mat3(glm::rotate(glm::radians(-mouse_delta.x*ROTATION_SPEED), up_direction)) * view_direction;

////    glm::vec3 verticalRotaionVec = glm::cross(view_direction, up_direction);
////    view_direction = glm::mat3(glm::rotate(glm::radians(-mouse_delta.y*ROTATION_SPEED), verticalRotaionVec)) * view_direction;
    //    mouse_position = new_mouse_position;
}

glm::mat4 front_came::getRotation_Mat(const float& angle)
{
    return glm::translate(rotation_axis) * glm::rotate(glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f)) * glm::translate(- rotation_axis);

}

void front_came::move_forward(void)
{
    camera_position += MOVEMENT_SPEED * view_direction;
    rotation_axis += MOVEMENT_SPEED * view_direction;
}
void front_came::move_backward(void)
{
    camera_position += -MOVEMENT_SPEED * view_direction;
    rotation_axis += -MOVEMENT_SPEED * view_direction;
}


} // namespace  zaytuna
