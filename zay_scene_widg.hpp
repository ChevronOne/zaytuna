


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


#define __OPENGL__
#define __GLM__

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



#include <ostream>
#include <limits>
#include "zay_headers.hpp"

#include "zay_cam.hpp"
#include "zay_vertex.hpp"
#include "zay_shape_maker.hpp"
#include "zay_shape_data.hpp"
//#include "zay_front_cam.hpp"
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



//namespace Ui {
//class _scene_widg;
//}

class _scene_widg : public QOpenGLWidget, protected QOpenGLFunctions_3_0 // QOpenGLExtraFunctions
//class _scene_widg : public QGLWidget, protected QOpenGLFunctions_3_0 // QOpenGLExtraFunctions
{
    Q_OBJECT
public:
//    explicit _scene_widg(QGLFormat, QWidget* parent = nullptr);
    explicit _scene_widg(QWidget* parent = nullptr);




    void initGLAtrib();

        void initShader(const std::string&, GLuint&, const std::size_t&);
        void makeUnderUse(GLuint&);
        void detachProgram(void);
        unsigned int getProgram(void) const;

        void send_data(void);
        void updateProjection(void);

        virtual ~_scene_widg() override;

        //-----------------


        void setup_win(float, float, float, float) const;


        static double sX, sY; //, sZ;
        static double delta_sX, delta_sY;//, delta_sZ;
        static double glX, glY; //, glZ;
    //    static bool clicked;

        camera mainCam ;
        camera* activeCam;
    //    front_came carCam;

        model_vehicle model;
        QGLFormat format;

        void cleanUp();



    //    shape_data plane, grid, coord, cube, sphere, pyramide;

        bool coord_checked{true}, grid_checked{true};

    //    std::vector<GLuint> programsssssss;
        std::vector<glm::dmat4> translateMat
        {
            /*0*/ glm::translate(glm::dvec3(0.0f, 0.0f, 0.0f)),  // translation mat of the plane
            /*1*/ glm::translate(glm::dvec3(0.0f, 0.0f, 0.0f)),  // translation of grid
            /*2*/ glm::translate(glm::dvec3(0.0f, 0.0f, 0.0f)),  //  ... of coord
            /*3*/ glm::translate(glm::dvec3(0.0f, 0.0f, 0.0f)), //  ...  of the cube
            /*4*/ glm::translate(glm::dvec3(7.0f, 0.5f,-5.0f)),  //   ...  of the sphere
            /*5*/ glm::translate(glm::dvec3(0.0f, 1.0f, -7.0f)), //  ...  of pyramid
            /*6*/ glm::translate(glm::dvec3(0.0f, 0.1f, 0.0f)), //  ...  of came shpere
            /*7*/ glm::translate(glm::dvec3(0.0f, 0.0f, 0.0f)), //  ...  of lap
        };

        std::vector<glm::dmat4> scaleMat
        {
            /*0*/ glm::scale(glm::dvec3(0.0f, 0.0f, 0.0f)),  // scale mat of plane
            /*1*/ glm::scale(glm::dvec3(0.0f, 0.0f, 0.0f)),  // scale of grid
            /*2*/ glm::scale(glm::dvec3(0.0f, 0.0f, 0.0f)),  //  ... of coord
            /*3*/ glm::scale(glm::dvec3(0.2f, 0.05f, 0.05f)), //  ...  of the cube
            /*4*/ glm::scale(glm::dvec3(0.0f, 0.0f, 0.0f)),  //   ...  of the sphere
            /*5*/ glm::scale(glm::dvec3(0.0f, 0.0f, 0.0f)), //  ...  of pyramid
        };

    public slots:
        void animate();

    protected:
        virtual void initializeGL() override;
        virtual void paintGL() override;
        virtual void resizeGL(int,int) override;


        virtual void mouseMoveEvent(QMouseEvent*) override;
        virtual void keyPressEvent(QKeyEvent*) override;
        virtual void wheelEvent(QWheelEvent*) override;
        virtual void keyReleaseEvent(QKeyEvent *event) override;






    private:

        std::string getShader(const std::string&);
        FileStatus fStatus;
    //    unsigned int program1, program2;
    //    const GLuint programs_num{2};
        std::vector<GLuint> programsssssss;
        GLuint* programs;
    //    std::vector<zaytuna::_item*> ging{2};
        std::vector<std::vector<zaytuna::_item*>> beings; //{programs_num};
        unsigned int _Shaders[SHADERS_NUM];
        void checkError(GLuint, GLuint, VarType, const std::string&);
        unsigned int compileShader(const std::string&, GLenum);
        void updat_cam(void);
        inline void load_tex(QImage&, const QString&, const char*, bool, bool);

    //    double _time;
        double accum{0};
        std::chrono::time_point<std::chrono::_V2::system_clock,
                        std::chrono::nanoseconds> _time, start_t;

        // ----------


    //    SDL_Window * windo;
    //    SDL_GLContext glContex;
    //    bool win_Status;
    //    static int W;
    //    static int H;
        //size_t nIndicis;

        glm::dmat4 projectionMat;
        std::vector<glm::mat4> transformationMat;
        GLint transformMatLocation, transformMatLocation1, transformMatLocation2;

        QTimer timer, key_timer;

        bool k_forward, k_backward, k_left, k_right, k_up, k_back;

        //---------------------

        GLuint theBufferID;

        GLuint textureID1, textureID2, textureID3, textureID4, textureID5, textureID6;
    //    cv::Mat lapTex;

        //    shape_data plane, grid, coord, cube, sphere, pyramide;


        GLuint planeVAO_ID;  // vertex array object ID
        GLuint gridVAO_ID;
        GLuint coordVAO_ID;
        GLuint cubeVAO_ID;
        GLuint sphereVAO_ID;
        GLuint pyramideVAO_ID;
        GLuint camSphereVAO_ID;
        GLuint lapVAO_ID;
        GLuint lap2VAO_ID;
        GLuint fenceVAO_ID;
        GLuint texsphereVAO_ID;
        GLuint skyboxVAO_ID;
        GLuint modelVAO_ID;
        GLuint fronttiresVAO_ID;
        GLuint backtiresVAO_ID;
        GLuint lidarVAO_ID;


        GLuint plane_indOffset;
        GLuint grid_indOffset;
        GLuint coord_indOffset;
        GLuint cube_indOffset;
        GLuint sphere_indOffset;
        GLuint pyramide_indOffset;
        GLuint camSphere_indOffset;
        GLuint lap_indOffset;
        GLuint lap2_indOffset;
        GLuint fence_indOffset;
        GLuint texsphere_indOffset;
        GLuint skybox_indOffset;
        GLuint model_indOffset;
        GLuint fronttires_indOffset;
        GLuint backtires_indOffset;
        GLuint lidar_indOffset;


        GLsizei planeNumIndices;
        GLsizei gridNumIndices;
        GLsizei coordNumIndices;
        GLsizei cubeNumIndices;
        GLsizei sphereNumIndices;
        GLsizei pyramideNumIndices;
        GLsizei camSphereNumIndices;
        GLsizei lapNumIndices;
        GLsizei lap2NumIndices;
        GLsizei fenceNumIndices;
        GLsizei texsphereNumIndices;
        GLsizei skyboxNumIndices;
        GLsizei modelNumIndices;
        GLsizei fronttiresNumIndices;
        GLsizei backtiresNumIndices;
        GLsizei lidarNumIndices;

};

} //namespace zaytuna




#endif // ZAY_SCENE_WIDG_HPP
