


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




#ifndef ZAY_SCENE_WIDG_HPP
#define ZAY_SCENE_WIDG_HPP




#include "zay_headers.hpp"
#include <QtGui/QMouseEvent>
#include <QtGui/QKeyEvent>
#include <QtGui/QWheelEvent>

#include <ostream>
#include <limits>

#include "zay_cam.hpp"
#include "zay_vertex.hpp"
#include "zay_shape_maker.hpp"
#include "zay_shape_data.hpp"
#include "zay_model_vehicle.hpp"
#include "zay_item.hpp"


namespace zaytuna {



enum class FileStatus { UNDEFINED, LOADED, FAILED };
enum class VarType { PROGRAM, SHADER };
class primary_win;

//class _scene_widg : public QOpenGLWidget,
//                    protected USED_GL_VERSION // QOpenGLExtraFunctions
class _scene_widg : public QGL_WIDGET_VERSION, protected USED_GL_VERSION // QOpenGLExtraFunctions
{
    Q_OBJECT

    std::string getShader(const std::string&);
    FileStatus fStatus;
    GLuint programs[PROGRAMS_NUM];

    ptr_vector<zaytuna::scene_object*> basic_objects;
    zaytuna::model_vehicle* model_vehicles{nullptr};
    vehicle_attributes* current_model{nullptr};
    zaytuna::obstacle_pack<GLdouble>* obstacle_objects{nullptr};
    ptr_vector<zaytuna::scene_object*> lap_objects;
    ptr_vector<zaytuna::scene_object*> environmental_objects;
    default_settings<GLdouble> default_objects;
    unsigned int _Shaders[SHADERS_NUM];
    void checkError(GLuint, GLuint,
                    VarType, const std::string&);
    unsigned int compileShader(const std::string&, GLenum);
    void update_cam(void);
    void add_default_obj(void);

    void initShader(const std::string&, GLuint&,
                    const std::size_t&);
    void makeUnderUse(GLuint&);
    void detachProgram(void);
    void send_data(void);
    void updateProjection(void);
    inline void update_contrl_attribs(void);
    void cleanUp();
    void draw_local(void);
    inline void render_main_scene(zaytuna::camera const*const);
    inline void render_local_scene(zaytuna::camera const*const);
    void add_vehicle(const transform_attribs<GLdouble>&);
    void add_obstacle(const obstacle_attribs<GLdouble>&);
    void delete_vehicle(const std::string&);
    void delete_obstacle(const std::string&);
    void edit_obstacle(const obstacle_attribs<GLdouble>&);
    void update_current_vehicle(const std::string&);
    obstacle_attribs<GLdouble> get_obstacle(const std::string&);
    zaytuna::vehicle_attributes* getOtherVeh(const std::string&);
    double elap_accumulated{0.0}, cam_freq_accumulated{0.0}, elapsed{0.0};  // nanSec
    double frame_rate{0};  // f/Sec
    double front_cam_freq{FRONT_CAM_FREQUENCY}; // Hz
    double imgs_sec{1.0/FRONT_CAM_FREQUENCY};
    double local_control_speed{0.0},
           local_control_steering{STEERING_MARGIN_OF_ERROR};
    uint32_t frames_counter{0};
    std::chrono::time_point<std::chrono::high_resolution_clock,
                    std::chrono::nanoseconds> start_t;
    camera mainCam ;
    camera* activeCam{&mainCam};
    QGLFramebufferObjectFormat fboFormat;
    bool coord_checked{true}, grid_checked{true};
    QTimer timer;
    bool key_control{0};
    bool k_forward, k_backward,
         k_left, k_right,
         k_up, k_down,
         l_forward, l_backward,
         l_left, l_right;
    GLuint theBufferID;
    friend class zaytuna::primary_win;

protected:
    virtual void initializeGL() override;
    virtual void paintGL() override;
    virtual void resizeGL(int,int) override;
    virtual void mouseMoveEvent(QMouseEvent*) override;
    virtual void keyPressEvent(QKeyEvent*) override;
    virtual void wheelEvent(QWheelEvent*) override;
    virtual void keyReleaseEvent(QKeyEvent*) override;
//    virtual void mousePressEvent(QMouseEvent*) override;

public:
    explicit _scene_widg(QGLFormat, QWidget* parent = nullptr);
    explicit _scene_widg(QWidget* parent = nullptr);

    virtual ~_scene_widg() override;
    static double sX, sY; //, sZ;
    static double delta_sX, delta_sY;//, delta_sZ;
    static double glX, glY; //, glZ;

public slots:
        void animate();


};



} //namespace zaytuna


#endif // ZAY_SCENE_WIDG_HPP



