

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





#ifndef ZAY_MODEL_VEHICLE_HPP
#define ZAY_MODEL_VEHICLE_HPP


#include "zay_coll_tester.hpp"


namespace zaytuna {




class model_vehicle;
class _scene_widg;
class primary_win;


typedef std::allocator<void> void_alloc;

class vehicle_attributes
{
    typedef std::chrono::time_point<std::chrono::high_resolution_clock,
                    std::chrono::nanoseconds> nanSec;
    typedef std::chrono::duration<double,
                        std::ratio< 1, 1>> durSec;


    std::unique_ptr<vehicle_topics<double, void_alloc>> topics{nullptr};
    inline void bind(){
        topics->localView_buffer->bind();
    }

    inline void advertise_frontCam(){
        topics->grab_buffer();
    }


    nanSec timer_t;
    
    ZAY_USED_GL_VERSION* _widg{nullptr};
    veh_transform_attribs<GLdouble> attribs;
    rect_collistion_object<GLdouble> const * coll_rect_orig{nullptr};
    rect_collistion_object<GLdouble> coll_rect;
    rect_collistion_pack<GLdouble> const * coll_pack{nullptr};
    zaytuna::vehicle_state<GLdouble> *v_state{nullptr};
    ZAY_MSG_LOGGER* zay_msg_logger{nullptr};


    double elapsed_t;
    bool is_collided{0}, collision_msg{0};
    std::string collided_object{"unkown_collision_object_name"};
    std::unique_ptr<coll_tester<GLdouble>> collision_tester{nullptr};
    

    friend class model_vehicle;
    friend class _scene_widg;
    friend class primary_win;
    friend class coll_tester<GLdouble>;



protected:

    glm::dvec3 vehic_direction;
    glm::dvec3 front_ideal_tire; // position of front ideal tire
    glm::dvec3 back_ideal_tire;  //  position of back ideal tire
    glm::dvec3 old_back_ideal_tire; // position of back ideal tire in the previous frame


    glm::dmat4 rotationMat; // rotation matrix of model vehicle;

    double AMOUNT_OF_ROTATION; // amount of rotation of the model vehicle 'per frame'
    double MOVEMENT_SPEED, DESIRED_SPEED; 
    double STEERING_WHEEL, DESIRED_STEERING; 
    double amount_of_hRotation{0.0}; // amount of horizontal rotation of the wheels
    double amount_of_rotation_Lidar{0.0}; // amount of rotation of lidar
    bool is_detached{1};


    double accumulated_dist; // accumulated distance
    double traveled_dist; // traveled distance per frame
    uint32_t ticks_counter; // ticks counter per frame


    double radius_of_rotation; // radius of rotation of the model vehicle

    glm::dvec3 center_of_rotation; // center of rotation of the model vehicle

    void update_attribs(const double);
    inline void update_actuators_commands(const double);
    inline void actuate();
    void update_rotation_att();
    void update_positional_attributes(const veh_transform_attribs<GLdouble>);
    inline void update_projection_rect(void);

    veh_transform_attribs<GLdouble> current_state(void);


    glm::dmat4 transformationMats[5]; // /model vehicle/, /right front tire/, /left front tire/, /back tires/, /lidar/
    glm::dmat4 tires_hRotation, // rotation matrix of horizontal rotation of the wheels
            front_tires_vRotation; // rotation matrix of vertical rotation of the front tires

    zaytuna::camera frontCam; // front camera
    glm::dvec4 frontCamViewDirection = glm::dvec4(0.5 , 0.278 , 0.0, 1.0); // should be adjusted!


public:

    vehicle_attributes() = default;
    explicit vehicle_attributes
            (ZAY_USED_GL_VERSION*,
             QGLFramebufferObject *const,
             const veh_transform_attribs<GLdouble>,
             rect_collistion_object<GLdouble> const*const,
             rect_collistion_pack<GLdouble>*,
             zaytuna::vehicle_state<GLdouble>*,
             ZAY_MSG_LOGGER*);
    ~vehicle_attributes();


};




} // namespace zaytuna




#endif // ZAY_MODEL_VEHICLE_HPP




