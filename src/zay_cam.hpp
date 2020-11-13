

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





#ifndef ZAY_CAM_HPP
#define ZAY_CAM_HPP



#include "zay_headers.hpp"
#include "zay_utility.hpp"
namespace zaytuna {


class camera
{
public:
    camera();
    glm::dmat4 getWorld_to_view_Mat();
    glm::dmat4 getProjection();
    void updateProjection(const double&, const double&);
    void updateWorld_to_viewMat();
    void mouse_update(const glm::dvec2&);
    void mouse_held_update(const glm::dvec2&);
    glm::dvec2 get_mouse_position();
    void move_forward(double scalar=1.0);
    void move_backward(double scalar=1.0);
    void move_horizontal_forward(double scalar=1.0);
    void move_horizontal_backward(double scalar=1.0);
    void move_up(double scalar=1.0);
    void move_down(double scalar=1.0);
    void strafe_left(double scalar=1.0);
    void strafe_right(double scalar=1.0);


    double ROTATION_SPEED;
    double MOVEMENT_SPEED;
    double FIELD_OF_VIEW;
    double NEAR_PLANE;
    double FAR_PLANE;
    bool auto_perspective;


    glm::dvec3 view_direction;
    glm::dvec3 up_direction;
    glm::dvec3 camera_position;
    glm::dvec3 view_point;

    glm::dvec2 mouse_position;
    glm::dmat4 projectionMat;
    glm::dmat4 world_to_viewMat;
    glm::dmat4 transformationMat;


    ~camera();
};

} // namespace zaytuna

#endif // ZAY_CAM_HPP




