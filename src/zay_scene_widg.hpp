


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
//#include <QWheelEvent>
#include <QtGui/QWheelEvent>

#include <boost/ptr_container/ptr_vector.hpp>
#include<boost/ptr_container/ptr_array.hpp>



#include <ostream>
#include <limits>


#include "zay_cam.hpp"
#include "zay_vertex.hpp"
#include "zay_shape_maker.hpp"
#include "zay_shape_data.hpp"
#include "zay_model_vehicle.hpp"
#include "zay_item.hpp"


//#include <opencv2/opencv.hpp>


//#include <QOpenGLFunctions_4_2_Core>

//#include <QOpenGLFunctions_4_0>





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
    zaytuna::obstacle_pack<GLdouble>* obstacle_objects{nullptr};
    ptr_vector<zaytuna::scene_object*> lap_objects;
    ptr_vector<zaytuna::scene_object*> environmental_objects;
    default_settings<GLdouble> default_objects;
    unsigned int _Shaders[SHADERS_NUM];
    void checkError(GLuint, GLuint,
                    VarType, const std::string&);
    unsigned int compileShader(const std::string&, GLenum);
    void updat_cam(void);
    void add_default_obj(void);

    void initShader(const std::string&, GLuint&,
                    const std::size_t&);
    void makeUnderUse(GLuint&);
    void detachProgram(void);
    unsigned int getProgram(void) const;

    void send_data(void);
    void updateProjection(void);

    void cleanUp();
    void draw_local(void);
    inline void render_scene(zaytuna::camera const*const);
    void add_vehicle(const transform_attribs<GLdouble>&);
    void add_obstacle(const obstacle_attribs<GLdouble>&);
    void delete_vehicle(const std::string&);
    void delete_obstacle(const std::string&);
    void edit_vehicle(const transform_attribs<GLdouble>&);
    void edit_obstacle(const obstacle_attribs<GLdouble>&);
    void update_current_vehicle(const std::string&);
    obstacle_attribs<GLdouble> get_obstacle(const std::string&);
    zaytuna::vehicle_attribute* getOtherVeh(const std::string&);
    double elap_accumulated{0.0};
    double frame_rate{0};
    uint32_t frames_counter{0};
    std::chrono::time_point<std::chrono::_V2::system_clock,
                    std::chrono::nanoseconds> start_t;

    camera mainCam ;
    camera* activeCam;
    QGLFramebufferObjectFormat fboFormat;

    vehicle_attribute* current_model{nullptr};
    bool coord_checked{true}, grid_checked{true};

    QTimer timer;

    bool k_forward, k_backward,
         k_left, k_right,
         k_up, k_back;

    GLuint theBufferID;
//    QImage img;


    friend class zaytuna::primary_win;

protected:

    virtual void initializeGL() override;
    virtual void paintGL() override;
    virtual void resizeGL(int,int) override;


    virtual void mouseMoveEvent(QMouseEvent*) override;
    virtual void keyPressEvent(QKeyEvent*) override;
    virtual void wheelEvent(QWheelEvent*) override;
    virtual void keyReleaseEvent(QKeyEvent *event) override;


public:
    explicit _scene_widg(QGLFormat, QWidget* parent = nullptr);
//    explicit _scene_widg(QWidget* parent = nullptr);
//    std::vector<GLfloat> depth{std::vector<GLfloat>( this->width() * this->height(), 0.f )};

    virtual ~_scene_widg() override;
    static double sX, sY; //, sZ;
    static double delta_sX, delta_sY;//, delta_sZ;
    static double glX, glY; //, glZ;

public slots:
        void animate();


};



} //namespace zaytuna


#endif // ZAY_SCENE_WIDG_HPP



