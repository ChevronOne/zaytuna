


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






#ifndef ZAY__SCENE_WIDG_HPP
#define ZAY__SCENE_WIDG_HPP


#include "zay_win_mainliner.hpp"

//#include <QGLWidget>
#include <QOpenGLWidget>
#include <QTimer>
//#include <QOpenGLFunctions>

#include <QOpenGLFunctions_3_0>
#include <QtGui/QMouseEvent>
#include <QtGui/QKeyEvent>
//#include <QWheelEvent>
#include <QtGui/QWheelEvent>

#include <boost/ptr_container/ptr_vector.hpp>
#include<boost/ptr_container/ptr_array.hpp>



#include <ostream>
#include <limits>
#include "zay_headers.hpp"

#include "zay_cam.hpp"
#include "zay_vertex.hpp"
#include "zay_shape_maker.hpp"
#include "zay_shape_data.hpp"
#include "zay_model_vehicle.hpp"
#include "zay_item.hpp"
#include "zay_clock.hpp"


//#include <opencv2/opencv.hpp>


//#include <QOpenGLFunctions_4_2_Core>

//#include <QOpenGLFunctions_4_0>


#include <QImage>
#include <QGLFormat>
#include <QSurfaceFormat>


namespace zaytuna {



enum class FileStatus { UNDEFINED, LOADED, FAILED };
enum class VarType { PROGRAM, SHADER };




class _scene_widg : public QOpenGLWidget, protected QOpenGLFunctions_3_0 // QOpenGLExtraFunctions
//class _scene_widg : public QGLWidget, protected QOpenGLFunctions_3_0 // QOpenGLExtraFunctions
{
    Q_OBJECT

    std::string getShader(const std::string&);
    FileStatus fStatus;
    GLuint programs[PROGRAMS_NUM];

    ptr_vector<zaytuna::scene_object*> beings;
    unsigned int _Shaders[SHADERS_NUM];
    void checkError(GLuint, GLuint, VarType, const std::string&);
    unsigned int compileShader(const std::string&, GLenum);
    void updat_cam(void);
    inline void load_tex(QImage&, const QString&, const char*, bool, bool);

    double accum{0};
    std::chrono::time_point<std::chrono::_V2::system_clock,
                    std::chrono::nanoseconds> start_t;


    QTimer timer;

    bool k_forward, k_backward, k_left, k_right, k_up, k_back;


    GLuint theBufferID;

protected:

    virtual void initializeGL() override;
    virtual void paintGL() override;
    virtual void resizeGL(int,int) override;


    virtual void mouseMoveEvent(QMouseEvent*) override;
    virtual void keyPressEvent(QKeyEvent*) override;
    virtual void wheelEvent(QWheelEvent*) override;
    virtual void keyReleaseEvent(QKeyEvent *event) override;


public:
//    explicit _scene_widg(QGLFormat, QWidget* parent = nullptr);
    explicit _scene_widg(QWidget* parent = nullptr);



    void initShader(const std::string&, GLuint&, const std::size_t&);
    void makeUnderUse(GLuint&);
    void detachProgram(void);
    unsigned int getProgram(void) const;

    void send_data(void);
    void updateProjection(void);

    virtual ~_scene_widg() override;


    static double sX, sY; //, sZ;
    static double delta_sX, delta_sY;//, delta_sZ;
    static double glX, glY; //, glZ;

    camera mainCam ;
    camera* activeCam;

    model_vehicle* model;
    QGLFormat format;

    void cleanUp();


    bool coord_checked{true}, grid_checked{true};


public slots:
        void animate();



};

} //namespace zaytuna




#endif // ZAY_SCENE_WIDG_HPP
