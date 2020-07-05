

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









#include "zay_scene_widg.hpp"

namespace zaytuna {


/*

        QGLFormat zat_format;
        zat_format.setSamples(8);
        SQwidget = new _scene_widg(zat_format, centralWidget);


*/



//static uint32_t total_tiks{0};
//static double accum_dist{0.0};
static uint32_t f = 0;



double _scene_widg::sX, _scene_widg::sY; //, _scene_widg::sZ;
double _scene_widg::delta_sX, _scene_widg::delta_sY;
double _scene_widg::glX, _scene_widg::glY; //, _scene_widg::glZ;





_scene_widg::_scene_widg(QWidget* parent): QOpenGLWidget(parent),
//_scene_widg::_scene_widg(QGLFormat _format, QWidget* parent): QGLWidget(_format, parent),
    fStatus{ FileStatus::UNDEFINED },
    k_forward{0}, k_backward{0}, k_left{0}, k_right{0}, k_up{0}, k_back{0}
{
    QSurfaceFormat _format;
    _format.setSamples(8);
    this->setFormat(_format);


    activeCam = &mainCam;


    connect(&timer, SIGNAL(timeout()), this, SLOT(animate()));
    timer.start(0);

    makeCurrent();
    setMouseTracking(true);
    this->setFocusPolicy(Qt::StrongFocus);

    accum = 0;


}

_scene_widg::~_scene_widg()
{

    for(std::size_t i = 0; i<PROGRAMS_NUM; ++i)
        glDeleteProgram(programs[i]);

    glUseProgram(0);

    cleanUp();

}

void _scene_widg::cleanUp()
{

    glDeleteBuffers(1, &theBufferID);
    detachProgram();
}

void _scene_widg::initializeGL()
{

    initializeOpenGLFunctions();
    glClearColor(0.86f, 0.86f, 0.86f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glViewport(0, 0, this->width(), this->height());
    for(std::size_t i = 0; i<PROGRAMS_NUM; ++i)
        initShader("./Shaders/source"+std::to_string(i), programs[i], i);

    send_data();

}

void _scene_widg::paintGL()
{

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if(activeCam == &mainCam){
        updat_cam();
        if(activeCam->auto_perspective ){
            if(activeCam->camera_position.y < 1.0)
                activeCam->NEAR_PLANE = 0.01;
            else if(activeCam->camera_position.y < 3.0)
                activeCam->NEAR_PLANE = 0.05;
            else if(activeCam->camera_position.y < 5.0)
                activeCam->NEAR_PLANE = 0.5;
            else if (activeCam->camera_position.y < 10.0)
                activeCam->NEAR_PLANE = 3.01;
            else if (activeCam->camera_position.y < 15.0)
                activeCam->NEAR_PLANE = 5.01;
            else
                activeCam->NEAR_PLANE = 10.01;
        }
    }



    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    model->update_attribs();


    activeCam->updateWorld_to_viewMat();

    for(const auto& _obj:beings)
        _obj->render_obj(activeCam);

    // frame's rate
    accum += std::chrono::duration<double, std::ratio< 1, 1>>(std::chrono::high_resolution_clock::now() - start_t).count();
    ++f;
    if(accum>=1.0) {
//        std::cout << "--------------------\n";
        std::cout << "frame freq counter: "<< 1.0/(accum/f) << " f/s \n";
//        // std::cout.precision(15);
//        // std::cout << model->traveled_dist << " meters \n";
//        // std::cout << glm::distance(model->bit, model->old_bit) << " meters \n";
        accum = f = 0;
//        std::cout << "Pos: " << model->back_ideal_tire;
//        std::cout << "Direc: " << model->vehic_direction;
//        std::cout << "Ticks-Counter: " << model->ticks_counter <<"\n";
    }
    // std::cout << "Ticks-Counter: " << model->ticks_counter <<"\n";
    start_t = std::chrono::high_resolution_clock::now();

}

void _scene_widg::resizeGL(int W, int H)
{
    glViewport(0, 0, W, H);
    mainCam.updateProjection(W, H);
    model->frontCam.updateProjection(W, H);
//    projectionMat = activeCam->projectionMat;
    repaint();
}

void _scene_widg::updateProjection()
{
    activeCam->updateProjection(width(), height());
}

void _scene_widg::mouseMoveEvent(QMouseEvent *ev)
{
    if(activeCam == &mainCam){
        if(ev->buttons()){
            delta_sX = sX - ev->pos().x();
            delta_sY = sY - ev->pos().y();
            if( delta_sX > 0)
                mainCam.strafe_right();
            if(delta_sX < 0)
                mainCam.strafe_left();
            if(delta_sY > 0)
                mainCam.move_down();
            if(delta_sY < 0)
                mainCam.move_up();
            sX = (double)ev->x();
            sY = (double)ev->y();
        }
        else{
            sX = static_cast<double>(ev->x());
            sY = (double)ev->y();
            glX =  (sX/ ((double)this->width()/2) ) -1;
            glY =  -((sY/ ((double)this->height()/2) ) -1);
            mainCam.mouse_update(glm::vec2(ev->x(),ev->y()));
        }
    }
}


void _scene_widg::updat_cam()
{
    if(k_forward)
        mainCam.move_forward();
    else if(k_backward)
        mainCam.move_backward();
    else if(k_left)
        mainCam.strafe_left();
    else if(k_right)
        mainCam.strafe_right();
    else if(k_up)
        mainCam.move_up();
    else if(k_back)
        mainCam.move_down();
}


void _scene_widg::keyPressEvent(QKeyEvent* ev)
{

    if( (activeCam != &mainCam) | k_forward | k_backward | k_left | k_right | k_up | k_back)
        return;

    switch (ev->key())
    {
        case Qt::Key::Key_W:
            k_forward=1;
            break;
        case Qt::Key::Key_S:
            k_backward=1;
            break;
        case Qt::Key::Key_Left:
            k_left^=1;
            break;
        case Qt::Key::Key_Right:
            k_right=1;
            break;
        case Qt::Key::Key_Up:
            k_up=1;
            break;
        case Qt::Key::Key_Down:
            k_back=1;
            break;
        case Qt::Key::Key_A:
            k_left=1;
            break;
        case Qt::Key::Key_D:
            k_right=1;
            break;
    }
}

void _scene_widg::wheelEvent(QWheelEvent *ev)
{
    if(ev->delta() > 0)
        mainCam.move_forward();
    else mainCam.move_backward();
}

void _scene_widg::keyReleaseEvent(QKeyEvent *ev)
{
    if(activeCam == &mainCam)
    switch (ev->key())
    {
        case Qt::Key::Key_W:
            k_forward=0;
            break;
        case Qt::Key::Key_S:
            k_backward=0;
            break;
        case Qt::Key::Key_Left:
            k_left=0;
            break;
        case Qt::Key::Key_Right:
            k_right=0;
            break;
        case Qt::Key::Key_Up:
            k_up=0;
            break;
        case Qt::Key::Key_Down:
            k_back=0;
            break;
        case Qt::Key::Key_A:
            k_left=0;
            break;
        case Qt::Key::Key_D:
            k_right=0;
            break;
    }
}

//void _scene_widg::mousePressEvent(QMouseEvent *e)
//{
//    std::cout << "clicked\n";
//    clicked = true;
//}

//void _scene_widg::mouseReleaseEvent(QMouseEvent *e)
//{
//    clicked = false;
//    std::cout << "released\n" ;
//}


void _scene_widg::animate()
{
    repaint();
}


std::string _scene_widg::getShader(const std::string& file_dir)
{
    fStatus = FileStatus::FAILED;
    std::ifstream _stream(file_dir.c_str(), std::ios::in);
    if (!_stream)
    {
        std::cerr << "file coudl not be opened: " << file_dir << std::endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        std::string program = std::string(std::istreambuf_iterator<char>(_stream), std::istreambuf_iterator<char>());
        _stream.close();
        fStatus = FileStatus::LOADED;
        return program;
    }

}

void _scene_widg::checkError(GLuint Object, GLuint ObjectParameter, VarType ObjectType, const std::string& _attachment)
{
    // GL error handling
    int _status{0};
    if (ObjectType == VarType::PROGRAM)
        glGetProgramiv(Object, ObjectParameter, &_status);
    else if (ObjectType == VarType::SHADER)
        glGetShaderiv(Object, ObjectParameter, &_status);

    if (_status != GL_TRUE)
    {
        int len;
        char* errorMessage{ nullptr };
        if (ObjectType == VarType::PROGRAM)
        {
            glGetProgramiv(Object, GL_INFO_LOG_LENGTH, &len);
            errorMessage = new char[(unsigned)len];
            glGetProgramInfoLog(Object, len, nullptr, errorMessage);
        }
        else if (ObjectType == VarType::SHADER)
        {
            glGetShaderiv(Object, GL_INFO_LOG_LENGTH, &len);
            errorMessage = new char[(unsigned)len];
            glGetShaderInfoLog(Object, len, nullptr, errorMessage);
        }

        std::cout << _attachment << ": " << errorMessage << std::endl;

        delete[] errorMessage;
    }
}

unsigned int _scene_widg::compileShader(const std::string& _shaders, GLenum _type)
{

    GLuint _SH = glCreateShader(_type);

    if (_SH == 0){
        std::cerr << "error occurs creating the shader object " << _type << std::endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        const GLchar* adapter = reinterpret_cast<const GLchar*>(_shaders.c_str());
        const int leng = static_cast<int>(_shaders.length());

        glShaderSource(_SH, 1, &adapter, &leng);

        glCompileShader(_SH);
        checkError(_SH, GL_COMPILE_STATUS, VarType::SHADER, "Compilation Error: ");
    }
    return _SH;
}



void _scene_widg::initShader(const std::string& file_Dir, GLuint& programLinker, const std::size_t& attrib_location)
{
    programLinker = glCreateProgram();
    // load vertex shader
    std::string _program{ getShader(file_Dir + ".vsh") };
    if (fStatus == FileStatus::LOADED)
    {
        // compile vertex shader
        _Shaders[0] = compileShader(_program, GL_VERTEX_SHADER);
        if (_Shaders[0] != 0)
        {
            // load fragment shader
            _program = getShader(file_Dir + ".fsh");
            if (fStatus == FileStatus::LOADED)
            {
                // compile fragment shader
                _Shaders[1] = compileShader(_program, GL_FRAGMENT_SHADER);
                if (_Shaders[1] != 0)
                {
                    // attach the both vertex and fragment shaders to one program
                    for (size_t _ind = 0; _ind != SHADERS_NUM; ++_ind)
                        glAttachShader(programLinker, _Shaders[_ind]);

                    if(attrib_location == 0){
                        glBindAttribLocation(programLinker, 0, "vertPos");
                        glBindAttribLocation(programLinker, 1, "vertColor");
                        glBindAttribLocation(programLinker, 2, "vertNorm");
                    }else if(attrib_location == 1){
                        glBindAttribLocation(programLinker, 0, "vertPos");
                        glBindAttribLocation(programLinker, 1, "vertNorm");
                        glBindAttribLocation(programLinker, 2, "texCoor");
                    }else if(attrib_location == 2){
                        glBindAttribLocation(programLinker, 0, "vertPos");
//                        glBindAttribLocation(programLinker, 1, "vertNorm");
//                        glBindAttribLocation(programLinker, 2, "texCoor");
                    }else if(attrib_location == 3){
                        glBindAttribLocation(programLinker, 0, "vertPos");
                        glBindAttribLocation(programLinker, 1, "vertNorm");
                        glBindAttribLocation(programLinker, 2, "texCoor");

                    }else{
                        std::cout << "program " << file_Dir << " not initialized!\n";
                        exit(EXIT_FAILURE);
                    }

                    glLinkProgram(programLinker);

                    // check for linker errors
                    checkError(programLinker, GL_LINK_STATUS, VarType::PROGRAM, "Object Linking Error: ");
                    glValidateProgram(programLinker);
                    checkError(programLinker, GL_VALIDATE_STATUS, VarType::PROGRAM, "Program Validation Error: ");

                    for (size_t ind = 0; ind != SHADERS_NUM; ++ind)
                        glDeleteShader(_Shaders[ind]);
                }
            }
        }
    }
}

void _scene_widg::makeUnderUse(GLuint& p_linker)
{
    glUseProgram(p_linker);
}

void _scene_widg::detachProgram()
{
    // calling this function with '0' argument will remove the program
    glUseProgram(0);
}

unsigned int _scene_widg::getProgram() const
{
    return programs[0];
}

void _scene_widg::send_data()
{

        beings = {
            new external_obj(this,
                             programs[1],
                             "plane",
                             "./primitives/plane1sm_300x300m_150subdivs.obj",
                             "tex/plane_grass_1024x1024.jpg",
                             GL_QUADS
//                             ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                             glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new external_obj(this,
                             programs[1],
                             "fence",
                             "./primitives/fence2m300x300m.obj",
                             "tex/fence_brick_1024x1024.jpg",
                             GL_QUADS
//                             ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                             glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new external_obj(this,
                             programs[1],
                             "mini_lap",
                             "./primitives/mini-lap.obj",
                             "tex/mini_lap_exemplar.jpg",
                             GL_QUADS
//                             ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                             glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new external_obj(this,
                             programs[1],
                             "lap1",
                             "./primitives/lap1_first_attep.obj",
                             "tex/lap1_exemplar.jpg",
                             GL_QUADS
//                             ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                              glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new external_obj(this,
                             programs[1],
                             "lap2",
                             "./primitives/lap2_first_attep.obj",
                             "tex/lap2_exemplar.jpg",
                             GL_QUADS
//                             ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                              glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new external_obj(this,
                             programs[1],
                             "lap3",
                             "./primitives/lap3_first_attep.obj",
                             "tex/lap3_exemplar.jpg",
                             GL_QUADS
//                             ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                              glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new coord_sys(this,
                          programs[0],
                          "coord_sys",
                          10.f
//                        ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new grid_plane(this,
                          programs[0],
                          "grid_plane",
                          150.f,
                          150.f,
                          1.f
//                        ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new skybox_obj(this,
                           programs[2],
                           "skybox"
//                         ,glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                         glm::translate(glm::dvec3(0.0, 0.0, 0.0))
            ),
            new model_vehicle(this,
                          programs[3],
                          "model_vehicle",
                          "./primitives/zaytuna_model",
                          "tex/zaytuna-fragments.png",
                          GL_TRIANGLES
//                        ,glm::rotate(45.0, glm::dvec3(0.0, 1.0, 0.0)),
//                        glm::translate(glm::dvec3(3.0, 0.0, 2.5))
            )
        };

        std::cout << "num of objects: " << beings.size() << "\n";
        GLsizeiptr BUF_SIZE{0};
        for(const auto& _obj:beings)
            BUF_SIZE+=_obj->buffer_size();

        glGenBuffers(1, &theBufferID);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        glBufferData(GL_ARRAY_BUFFER, BUF_SIZE, nullptr, GL_STATIC_DRAW);

        GLintptr current_offset{0};
        for(auto& _obj:beings)
            _obj->carry_data(current_offset);

        GLuint priv_offset{0};
        for(const auto& _obj:beings)
            _obj->parse_VertexArraysObject(theBufferID, priv_offset);

        model = dynamic_cast<model_vehicle*>(beings.back());

}

void _scene_widg::load_tex(QImage& buff, const QString& _dir,
                          const char* _format, bool hMir, bool vMir)
{
    if(!(buff.load(_dir, _format))){
        std::cout << "image couldn't be loaded <" << _dir.toStdString() << ">!\n";
        exit(EXIT_FAILURE);
    }

    buff = QGLWidget::convertToGLFormat(buff.mirrored(hMir, vMir));
    if(buff.isNull()){
        std::cout << "error occurred while converting the image <" <<_dir.toStdString() <<">!\n";
        exit(EXIT_FAILURE);
    }
}



} // namespace  zaytuna
