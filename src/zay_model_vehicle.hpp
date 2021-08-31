

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


#include "zay_topics.hpp"





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
    rect_collistion_object<GLdouble> const *coll_rect_orig;
    rect_collistion_object<GLdouble> coll_rect;
    rect_collistion_pack<GLdouble>* coll_pack;
    zaytuna::vehicle_state<GLdouble> *v_state{nullptr};
    ZAY_MSG_LOGGER* zay_msg_logger;


    double elapsed_t;
    bool is_collided{0}, collision_msg{0};
    std::string collided_object{"unkown_collision_object_name"};
    std::unique_ptr<std::thread> collision_tester{nullptr};
    

    friend class model_vehicle;
    friend class _scene_widg;
    friend class primary_win;



protected:

    glm::dvec3 vehic_direction;
    const glm::dvec3 up_direction{0.0, 1.0, 0.0};
    glm::dvec3 front_ideal_tire; // position of front ideal tire
    glm::dvec3 back_ideal_tire;  //  position of back ideal tire
    glm::dvec3 old_back_ideal_tire; // position of back ideal tire in the previous frame


    glm::dmat4 rotationMat; // rotation matrix of model vehicle;

    double AMOUNT_OF_ROTATION; // amount of rotation of the model vehicle 'per frame'
    double MOVEMENT_SPEED, DESIRED_SPEED; 
    double STEERING_WHEEL, DESIRED_STEERING; 
    double amount_of_hRotation{0.0}; // amount of horizontal rotation of the wheels
    double amount_of_rotation_Lidar{0.0}; // amount of rotation of lidar
    const double lidar_spin_speed{750.0}; // constant scalar 'affects only rendering'
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
    // void update_cent(void);
    void update_positional_attributes(const veh_transform_attribs<GLdouble>);

    veh_transform_attribs<GLdouble> current_state(void);


    glm::dmat4 transformationMats[5]; // /model vehicle/, /right front tire/, /left front tire/, /back tires/, /lidar/
    glm::dmat4 tires_hRotation, // rotation matrix of horizontal rotation of the wheels
            front_tires_vRotation; // rotation matrix of vertical rotation of the front tires

    zaytuna::camera frontCam; // front camera


    // Default Positional Parameters
    const glm::dvec3 camHeight = glm::dvec3(0.0, 0.466, 0.0);
    const glm::dmat4 f_rightT = glm::translate(glm::dvec3( 0.286, 0.031,  0.159));
    const glm::dmat4 f_leftT =  glm::translate(glm::dvec3( 0.286, 0.031, -0.159));
    const glm::dmat4 backT =    glm::translate(glm::dvec3( 0.0,  0.031,    0.0));
    const glm::dmat4 lidar =    glm::translate(glm::dvec3( 0.118, 0.419,   0.0));
    const glm::dvec4 camPos =  glm::dvec4(0.13, 0.466, 0.0, 1.0);
    glm::dvec4 frontCamViewDirection = glm::dvec4(0.5 , 0.278 , 0.0, 1.0); // should be adjusted!


    // Physical Specifications
    const double front_back_distance{0.2862}; // distance between back and front ideal tires
    const double ticks_per_meter{173.0}; // 173 ticks per meter 'can be adjusted'
    const double meters_per_tick{1.0/ticks_per_meter}; // meters per tick
    const double tires_radius{0.0311};
    const double PI2{2.0*M_PI};
    const double tires_circumference{PI2*tires_radius};

    const double steering_delay{0.2}; // sec
    const double speed_delay{0.3}; // sec


    
    inline void update_projection_rect(void);

    // for debugging purposes
    typedef glm::vec<ZAY_POINT_D, GLdouble, glm::qualifier::packed_highp> coll_vert;
    inline void collision_test(rect_collistion_pack<GLdouble> const * const);
    inline bool axis_inter(const coll_vert, 
                           const boost::array<coll_vert, ZAY_RECTANGLE_P>&) const;




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
    // ~vehicle_attributes() = default;
    ~vehicle_attributes();


};




} // namespace zaytuna




#endif // ZAY_MODEL_VEHICLE_HPP




