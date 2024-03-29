


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




#ifndef ZAY_SCENE_WIDG_HPP
#define ZAY_SCENE_WIDG_HPP



#include "zay_item.hpp"


namespace zaytuna {



class primary_win;
class _scene_widg : public ZAY_QGL_WIDGET_VERSION, protected ZAY_USED_GL_VERSION
{
    Q_OBJECT

    typedef std::unique_ptr<zaytuna::basic_program> progPtr;
    typedef std::unique_ptr<zaytuna::scene_object> sceneObjPtr;
    typedef std::unique_ptr<zaytuna::obstacle_pack<GLdouble>> obsPackPtr;
    typedef std::unique_ptr<zaytuna::model_vehicle> vehPtr;



    std::vector<progPtr> programs_;
    std::vector<sceneObjPtr> basic_objects;
    vehPtr model_vehicles{nullptr};
    vehicle_attributes* current_model{nullptr};
    sceneObjPtr skybox{nullptr};
    obsPackPtr obstacle_objects{nullptr};
    std::vector<sceneObjPtr> environmental_objects;
    default_settings<GLdouble> default_objects;
    zaytuna::rect_collistion_pack<GLdouble> collision_pack;
    ZAY_MSG_LOGGER* message_logger{nullptr};
    const std::string ZAY_PACKAGE_PATH{ros::package::getPath(ZAY_PACKAGE_NAME)};

    inline void set_up(void);
    void update_cam(void);
    void add_default_obj(void);
    void send_data(void);
    void updateProjection(void);
    void update_time_interval(uint32_t);
    void update_fc_time_interval(double);
    inline void update_contrl_attribs(void);
    void draw_local(void);
    inline void render_main_scene(zaytuna::camera const*const);
    inline void render_local_scene(zaytuna::camera const*const);
    void add_vehicle(const veh_transform_attribs<GLdouble>&);
    void add_obstacle(const obstacle_attribs<GLdouble>&);
    void delete_vehicle(const std::string&);
    void delete_obstacle(const std::string&);
    // void edit_obstacle(const obstacle_attribs<GLdouble>&); // replaced by remove & add
    void update_current_vehicle(const std::string&);
    void add_static_collition_object(const obstacle_attribs<GLdouble>&);
    void load_external_collition_object(const std::string&,
                                        const std::string&);

    obstacle_attribs<GLdouble> get_obstacle(const std::string&);
    zaytuna::vehicle_attributes* getOtherVeh(const std::string&);
    double elap_accumulated{0.0}, cam_freq_accumulated{0.0}, elapsed{0.0};  // nanSec
    double frame_rate{0.0};  // f/Sec
    double front_cam_freq{ZAY_DEFAULT_FRONT_CAM_FREQUENCY}; // Hz
    double imgs_sec{1.0/ZAY_DEFAULT_FRONT_CAM_FREQUENCY};
    double local_control_speed{0.0},
           local_control_steering{ZAY_STEERING_MARGIN_OF_ERROR};
    uint32_t frames_counter{0};
    uint32_t limited_frames{100};
    std::chrono::time_point<std::chrono::high_resolution_clock,
                    std::chrono::nanoseconds> start_t;
    camera mainCam ;
    camera* activeCam{&mainCam};
    QGLFramebufferObjectFormat fboFormat;
    bool coord_checked{true}, grid_checked{true};
    QTimer main_loop_timer;
    bool key_control{0}, fixed_cam{0};
    bool k_forward{0}, k_backward{0},
         k_left{0}, k_right{0},
         k_up{0}, k_down{0},
         l_forward{0}, l_backward{0},
         l_left{0}, l_right{0};
    GLuint theBufferID;
    double SLIDER_MOVEMENT_SPEED{0.0}, 
           SLIDER_STEERING_WHEEL{ZAY_STEERING_MARGIN_OF_ERROR};
    zaytuna::vehicle_state<GLdouble> v_state;


    friend class zaytuna::primary_win;


protected:
    virtual void initializeGL() override;
    virtual void paintGL() override;
    virtual void resizeGL(int,int) override;
    virtual void mouseMoveEvent(QMouseEvent*) override;
    virtual void keyPressEvent(QKeyEvent*) override;
    virtual void wheelEvent(QWheelEvent*) override;
    virtual void keyReleaseEvent(QKeyEvent*) override;



public:
    explicit _scene_widg(ZAY_MSG_LOGGER*, QGLFormat, QWidget* parent = nullptr);
    explicit _scene_widg(ZAY_MSG_LOGGER*, QWidget* parent = nullptr);


    virtual ~_scene_widg() override;
    static double sX, sY; //, sZ;
    static double delta_sX, delta_sY;//, delta_sZ;
    static double glX, glY; //, glZ;



public slots:
        void animate();




};



} //namespace zaytuna



#endif // ZAY_SCENE_WIDG_HPP



