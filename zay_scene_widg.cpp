

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



//glm::highp_dvec3 hf(0.0,0.0,0.0);

const uint NUM_VERTICES_PER_TRI = 3;
const uint NUM_VERTICES_PER_TEXCOR = 2;
const uint NUM_FLOATS_PER_VERTICE0 = 9;
const uint NUM_FLOATS_PER_VERTICE1 = 8;
const uint VERTEX_BYTE_SIZE = NUM_FLOATS_PER_VERTICE0 * sizeof(float);
const uint VERTEX_BYTE_SIZE1 = NUM_FLOATS_PER_VERTICE1 * sizeof(float);

//const float ticks_per_m = 0.01369863f;
static const double PI2 = 2*M_PI;
static const double t_rad =  0.0311;// circumference
static const double CIRCUMFERENCE = PI2 * t_rad;// circumference



static uint32_t total_tiks{0};
static double accum_dist{0.0};
uint32_t f = 0;


/*

        QGLFormat format;
        format.setSamples(8);
        SQwidget = new _scene_widg(format, centralWidget);
*/

static GLdouble angle1{ 0.0 };
//static GLdouble angle2{ 0.0 };
//static GLdouble angle3{ 0.0 };


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

//    QGLFormat _format;
//    _format.setSamples(4);
//    this->setFormat(_format);

    beings.resize(programs_num);

    activeCam = &mainCam;

    projectionMat = activeCam->projectionMat;


    transformationMat.resize(11);

    model.MOVEMENT_SPEED = 0.0;
    model.STEERING_WHEEL = 0.00000001 ; //0.001f;

    transformationMat[3] =   glm::translate(glm::dvec3(0.0f,0.0f,0.0f));

    connect(&timer, SIGNAL(timeout()), this, SLOT(animate()));
    timer.start(0);

    makeCurrent();
    setMouseTracking(true);
    this->setFocusPolicy(Qt::StrongFocus);

    _time = start_t = std::chrono::high_resolution_clock::now();
    accum = 0;


}

_scene_widg::~_scene_widg()
{

    for(std::size_t i = 0; i<programs_num; ++i)
        glDeleteProgram(programs[i]);
//    delete[] programs;

    glUseProgram(0);

    cleanUp();

}

void _scene_widg::cleanUp()
{

    glDeleteBuffers(1, &theBufferID);

    glDeleteVertexArrays(1, &planeVAO_ID);
    glDeleteVertexArrays(1, &gridVAO_ID);
    glDeleteVertexArrays(1, &coordVAO_ID);
    glDeleteVertexArrays(1, &cubeVAO_ID);
    glDeleteVertexArrays(1, &sphereVAO_ID);
    glDeleteVertexArrays(1, &pyramideVAO_ID);
    glDeleteVertexArrays(1, &camSphereVAO_ID);
    glDeleteVertexArrays(1, &lapVAO_ID);
    glDeleteVertexArrays(1, &lap2VAO_ID);
    glDeleteVertexArrays(1, &skyboxVAO_ID);
    glDeleteVertexArrays(1, &skyboxVAO_ID);
    glDeleteVertexArrays(1, &fenceVAO_ID);
    glDeleteVertexArrays(1, &texsphereVAO_ID);
    glDeleteVertexArrays(1, &modelVAO_ID);
    glDeleteVertexArrays(1, &fronttiresVAO_ID);
    glDeleteVertexArrays(1, &backtiresVAO_ID);
    glDeleteVertexArrays(1, &lidarVAO_ID);

    glDeleteTextures(1, &textureID1);
    glDeleteTextures(1, &textureID2);
    glDeleteTextures(1, &textureID3);
    glDeleteTextures(1, &textureID4);
    glDeleteTextures(1, &textureID5);
    glDeleteTextures(1, &textureID6);
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

//        programs = new GLuint[programs_num];
        for(std::size_t i = 0; i<programs_num; ++i)
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



    model.update_attribs(std::chrono::duration<double,
                         std::ratio< 1, 1>>
                         (std::chrono::high_resolution_clock::now() - _time).count());


    _time = std::chrono::high_resolution_clock::now();



        activeCam->updateWorld_to_viewMat();





        transformationMat[0] = activeCam->transformationMat;
        glUseProgram(programs[1]);

        //    // render a plane
        glBindVertexArray(planeVAO_ID);
        glBindTexture(GL_TEXTURE_2D, textureID4);
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, &transformationMat[0][0][0]);
        glDrawElements(GL_QUADS, planeNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(plane_indOffset));

        // fence
        glBindVertexArray(fenceVAO_ID);
        glBindTexture(GL_TEXTURE_2D, textureID5);
        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &transformationMat[0][0][0]);
        glDrawElements(GL_QUADS, fenceNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(fence_indOffset));

    //    // texsphere
    //    glBindVertexArray(texsphereVAO_ID);
    //    glUniformMatrix4fv(transformMatLocation1, 1, GL_FALSE, &transformationMat[0][0][0]);
    //    glDrawElements(GL_TRIANGLES, texsphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(texsphere_indOffset));


        // mini-lap
        glBindVertexArray(lapVAO_ID);
        glBindTexture(GL_TEXTURE_2D, textureID1);
        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &transformationMat[0][0][0]);
        glDrawElements(GL_QUADS, lapNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(lap_indOffset));


        // render the lap2
        glBindVertexArray(lap2VAO_ID);
        glBindTexture(GL_TEXTURE_2D, textureID2);
        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &transformationMat[0][0][0]);
        glDrawElements(GL_QUADS, lap2NumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(lap2_indOffset));


        //-----------------------------------
        model.render_the_model(this, activeCam, transformMatLocation_1, itt_MatLocation_P3);

//        glUseProgram(programs[3]);

//        glm::mat4 modeltransform = activeCam->transformationMat * model.transformationMats[0];
//        glm::mat4 it_modeltransform =glm::inverse(glm::transpose(model.transformationMats[0]));
//        // render the model
//        glBindVertexArray(modelVAO_ID);
//        transformationMat[3] = modeltransform;
//        glBindTexture(GL_TEXTURE_2D, textureID6);
//        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &modeltransform[0][0]);
//        glUniformMatrix4fv(itt_MatLocation_P3, 1, GL_FALSE, &it_modeltransform[0][0]);
//        glDrawElements(GL_TRIANGLES, modelNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(model_indOffset));


//        // render lidar
//        glBindVertexArray(lidarVAO_ID);
//        modeltransform = activeCam->transformationMat * model.transformationMats[4];
////        transformationMat[10] = modeltransform * lidarMat;
//        it_modeltransform =glm::inverse(glm::transpose(model.transformationMats[4]));
//        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &modeltransform[0][0]);
//        glUniformMatrix4fv(itt_MatLocation_P3, 1, GL_FALSE, &it_modeltransform[0][0]);
//        this->glDrawElements(GL_TRIANGLES, lidarNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(lidar_indOffset));



//        glUseProgram(programs[1]);
//        // render fronttires
//        glBindVertexArray(fronttiresVAO_ID);
//        modeltransform = activeCam->transformationMat * model.transformationMats[1];
//        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &modeltransform[0][0]);
//        glDrawElements(GL_TRIANGLES, fronttiresNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(fronttires_indOffset));
//        modeltransform = activeCam->transformationMat * model.transformationMats[2];
//        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &modeltransform[0][0]);
//        glDrawElements(GL_TRIANGLES, fronttiresNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(fronttires_indOffset));

//        // render back tires
//        glBindVertexArray(backtiresVAO_ID);
//        modeltransform = activeCam->transformationMat * model.transformationMats[3];
//        glUniformMatrix4fv(transformMatLocation_1, 1, GL_FALSE, &modeltransform[0][0]);
//        glDrawElements(GL_TRIANGLES, backtiresNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(backtires_indOffset));
        //-------------------------------------





        glUseProgram(programs[0]);


        glLineWidth(1.0f);
        if(grid_checked){
            // render a grid
            glBindVertexArray(gridVAO_ID);
            transformationMat[1] = activeCam->transformationMat; // projectionMat * mainCam.getWorld_to_view_Mat();
            glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, &transformationMat[1][0][0]);
            glDrawElements(GL_LINES, gridNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(grid_indOffset));
        }


        if(coord_checked){
            // render a coord
            glLineWidth(1.5f);
            glBindVertexArray(coordVAO_ID);
            transformationMat[2] = activeCam->transformationMat; // projectionMat * mainCam.getWorld_to_view_Mat();
            glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, &transformationMat[2][0][0]);
            glDrawElements(GL_LINES, coordNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(coord_indOffset));
        }


    //    // render the cude
    //    glBindVertexArray(cubeVAO_ID);
    //    transformationMat[3] = activeCam->transformationMat * model.transform;
    //    glUniformMatrix4fv(transformMatLocation, 1, GL_FALSE, &transformationMat[3][0][0]);
    //    glDrawElements(GL_TRIANGLES, cubeNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(cube_indOffset));


        // render the sphere
        glBindVertexArray(sphereVAO_ID);
        glm::dmat4 sphereR = glm::rotate(glm::radians(angle1 >= 360 ? angle1 = 0 : angle1 += 0.2), glm::dvec3(0.0, 1.0, 0.0));
        transformationMat[4] = activeCam->transformationMat * translateMat[4] * sphereR;
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, glm::value_ptr(transformationMat[4]));
        glDrawElements(GL_TRIANGLE_STRIP, sphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(sphere_indOffset));


    //    //=====================================
        transformationMat[4] = activeCam->transformationMat * glm::translate(glm::dvec3(4.0f, 0.5f,5.0f)) * sphereR;
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, glm::value_ptr(transformationMat[4]));
        glDrawElements(GL_TRIANGLE_STRIP, sphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(sphere_indOffset));

        transformationMat[4] = activeCam->transformationMat * glm::translate(glm::dvec3(-4.0f, 0.5f,5.0f)) * sphereR;
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, glm::value_ptr(transformationMat[4]));
        glDrawElements(GL_TRIANGLE_STRIP, sphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(sphere_indOffset));

        transformationMat[4] = activeCam->transformationMat * glm::translate(glm::dvec3(0.0f, 0.5f,5.0f)) * sphereR;
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, glm::value_ptr(transformationMat[4]));
        glDrawElements(GL_TRIANGLE_STRIP, sphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(sphere_indOffset));

        transformationMat[4] = activeCam->transformationMat * glm::translate(glm::dvec3(-4.0f, 0.5f, -5.0f)) * sphereR;
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, glm::value_ptr(transformationMat[4]));
        glDrawElements(GL_QUAD_STRIP, sphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(sphere_indOffset));

        transformationMat[4] = activeCam->transformationMat * glm::translate(glm::dvec3(8.0f, 0.5f, -2.0f)) * glm::rotate(glm::radians(90.0), glm::dvec3(1.0, 0.0, 0.0)) *  sphereR;
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, glm::value_ptr(transformationMat[4]));
        glDrawElements(GL_QUAD_STRIP, sphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(sphere_indOffset));
    //    //==================================


        // render the pyramidetranslate
        glBindVertexArray(pyramideVAO_ID);
        transformationMat[5] = activeCam->transformationMat * translateMat[5] * glm::rotate(glm::radians(25.0), glm::dvec3(0.0, 1.0, 0.0));
        glUniformMatrix4fv(transformMatLocation_0, 1, GL_FALSE, &transformationMat[5][0][0]);
        glDrawElements(GL_TRIANGLES, pyramideNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(pyramide_indOffset));


    //    // render the camera sphere
    //    glBindVertexArray(camSphereVAO_ID);
    //    transformationMat[6] = activeCam->transformationMat * model.transform * translateMat[6] ; // * translateMat[5] * glm::rotate(glm::radians(25.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    //    glUniformMatrix4fv(transformMatLocation, 1, GL_FALSE, &transformationMat[6][0][0]);
    //    glDrawElements(GL_TRIANGLE_STRIP, camSphereNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(camSphere_indOffset));


        // render skybox
        glDepthFunc(GL_LEQUAL);
        glUseProgram(programs[2]);
        glBindVertexArray(skyboxVAO_ID);
        glBindTexture(GL_TEXTURE_CUBE_MAP, textureID3);
        transformationMat[0] = activeCam->transformationMat * glm::translate(activeCam->camera_position);
        glUniformMatrix4fv(transformMatLocation_2, 1, GL_FALSE, &transformationMat[0][0][0]);
        glDrawElements(GL_TRIANGLES, skyboxNumIndices, GL_UNSIGNED_INT, reinterpret_cast<void*>(skybox_indOffset));
        glDepthFunc(GL_LESS);




        // frame's rate
        accum += std::chrono::duration<double, std::ratio< 1, 1>>(std::chrono::high_resolution_clock::now() - start_t).count();
        ++f;
        if(accum>=1.0) {
            std::cout << "--------------------\n";
            std::cout << "frame freq counter: "<< 1.0/(accum/f) << " f/s \n";
    //        std::cout.precision(15);
    //        std::cout << model.traveled_dist << " meters \n";
    //        std::cout << glm::distance(model.bit, model.old_bit) << " meters \n";
    //        std::cout << angle3 << " radiance \n";
            accum = f = 0;
            std::cout << "Pos: " << model.back_ideal_tire;
            std::cout << "Direc: " << model.vehic_direction;
            std::cout << "Ticks-Counter: " << model.ticks_counter <<"\n";
        }
//        std::cout << "Ticks-Counter: " << model.ticks_counter <<"\n";
        start_t = std::chrono::high_resolution_clock::now();


//        accum_dist += model.traveled_dist;
//        total_tiks += model.ticks_counter;

//        if(accum_dist>=1.0){
//            std::cout << "dist: " << accum_dist << "\n";
//            std::cout << "total ticks: " << total_tiks << "\n";
//            exit(EXIT_SUCCESS);
//        }



}

void _scene_widg::resizeGL(int W, int H)
{
    glViewport(0, 0, W, H);
    mainCam.updateProjection(W, H);
    model.frontCam.updateProjection(W, H);
    projectionMat = activeCam->projectionMat;
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
            errorMessage = new char[len];
            glGetProgramInfoLog(Object, len, nullptr, errorMessage);
        }
        else if (ObjectType == VarType::SHADER)
        {
            glGetShaderiv(Object, GL_INFO_LOG_LENGTH, &len);
            errorMessage = new char[len];
            glGetShaderInfoLog(Object, len, nullptr, errorMessage);
        }

        std::cout << _attachment << ": " << errorMessage << std::endl;

        delete[] errorMessage;
    }
}

unsigned int _scene_widg::compileShader(const std::string& _shaders, GLenum _type)
{
    // create a shader and associate it with GLuint variable
    // this is just like returning a pointer to created shader
    GLuint _SH = glCreateShader(_type);

    // 0 is returned if the creation was failed
    if (_SH == 0){
        std::cerr << "error occurs creating the shader object " << _type << std::endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        // pointer to the shader string and the length of string
        const GLchar* adapter = reinterpret_cast<const GLchar*>(_shaders.c_str());
        const int leng = static_cast<int>(_shaders.length());

        // associate the program info to the shader we created
        glShaderSource(_SH, 1, &adapter, &leng);

        // compile the shader and check for compilation errors
        glCompileShader(_SH);
        checkError(_SH, GL_COMPILE_STATUS, VarType::SHADER, "Compilation Error: ");
    }
    return _SH;
}



void _scene_widg::initShader(const std::string& file_Dir, GLuint& programLinker, const std::size_t& attrib_location) // attrib_location
{
    programLinker = glCreateProgram();
    // load vertex shader from from a  *.vsh  file
    std::string _program{ getShader(file_Dir + ".vsh") };
    if (fStatus == FileStatus::LOADED)
    {
        // compile vertex shader
        _Shaders[0] = compileShader(_program, GL_VERTEX_SHADER);
        if (_Shaders[0] != 0)
        {
            // load vertex shader from a  *.fsh  file
            _program = getShader(file_Dir + ".fsh");
            if (fStatus == FileStatus::LOADED)
            {
                // compile fragment shader
                _Shaders[1] = compileShader(_program, GL_FRAGMENT_SHADER);
                if (_Shaders[1] != 0)
                {
                    // attach the both vertex and fragment shaders to same linker
                    for (size_t _ind = 0; _ind != SHADERS_NUM; ++_ind)
                        glAttachShader(programLinker, _Shaders[_ind]);

                    // associate the vertex attribute index with attribute variable name "vertPos" & "color"
                    // the same attribute names are defined in vertex shader
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

                    // link the program as an combination of vertex and fragment shaders
                    glLinkProgram(programLinker);

                    // check for linker errors
                    checkError(programLinker, GL_LINK_STATUS, VarType::PROGRAM, "Object Linking Error: ");
                    glValidateProgram(programLinker);
                    checkError(programLinker, GL_VALIDATE_STATUS, VarType::PROGRAM, "Program Validation Error: ");

                    //after we linked the two shaders to one linker, we can now delete them
                    for (size_t ind = 0; ind != SHADERS_NUM; ++ind)
                        glDeleteShader(_Shaders[ind]);
//                    std::cout << "shader <" << file_Dir << "> has been initialized!\n";
                }
            }
        }
    }
}

void _scene_widg::makeUnderUse(GLuint& p_linker)
{
    // use the linked program "the combination of vertex and fragment shaders "
    //  install the shaders on the GPU
    glUseProgram(p_linker);
}

void _scene_widg::detachProgram()
{
    // calling this function with '0' argument will remove the program from GPU
    glUseProgram(0);
}

unsigned int _scene_widg::getProgram() const
{
    return programs[0];
}

void _scene_widg::send_data()
{
    //    shape_data<zaytuna::vertex> plane = shape_maker<zaytuna::vertex>::makePlane(300);
        shape_data<zaytuna::vertexL1_16> plane = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/plane1sm_300x300m_150subdivs.obj");
        shape_data<zaytuna::vertexL1_12> grid = shape_maker<zaytuna::vertexL1_12>::makeGrid(150, 150, 1);
        shape_data<zaytuna::vertexL1_12> coord = shape_maker<zaytuna::vertexL1_12>::makeCoord();
        shape_data<zaytuna::vertexL1_12> cube = shape_maker<zaytuna::vertexL1_12>::makeCube();
        shape_data<zaytuna::vertexL1_12> sphere = shape_maker<zaytuna::vertexL1_12>::makeSphere(0.1f, 0.5f);
        shape_data<zaytuna::vertexL1_12> pyramide = shape_maker<zaytuna::vertexL1_12>::makePyramide();
        shape_data<zaytuna::vertexL1_12> camSphere = shape_maker<zaytuna::vertexL1_12>::makeSphere(0.1f, 0.035f, glm::vec3(0.2f, 0.0f, 0.0f));
        shape_data<zaytuna::vertexL1_16> lap = shape_maker<zaytuna::vertexL1_16>::makeLap();
        shape_data<zaytuna::vertexL1_16> lap2 = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/lap1_first_attep.obj");
        shape_data<zaytuna::vertexL1_0>  skybox = shape_maker<zaytuna::vertexL1_0>::makeCubemap();
        shape_data<zaytuna::vertexL1_16> fence = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/fence2m300x300m.obj");
        shape_data<zaytuna::vertexL1_16> texsphere = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/sphere.obj");
        shape_data<zaytuna::vertexL1_16> model = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/zaytuna-model.obj");
        shape_data<zaytuna::vertexL1_16> fronttires = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/single-tire.obj");
        shape_data<zaytuna::vertexL1_16> backtires = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/back-tires.obj");
        shape_data<zaytuna::vertexL1_16> lidar = shape_maker<zaytuna::vertexL1_16>::extractExternal("./primitives/lidar.obj");

        std::cout<<"sphere vert: " << sphere.verNum << "\n"; // 4160
        std::cout<<"sphere ind: " << sphere.indNum << "\n";
        std::cout<<"tex sphere vert: " << texsphere.verNum << "\n"; // 1260
        std::cout<<"tex sphere ind: " << texsphere.indNum << "\n";

        std::cout<<"tex model vert: " << model.verNum << "\n"; // 1260
        std::cout<<"tex model ind: " << model.indNum << "\n";


        glGenBuffers(1, &theBufferID);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        glBufferData(GL_ARRAY_BUFFER,
            plane.verBufSize() + plane.indBufSize() +
            +grid.verBufSize() + grid.indBufSize()
            +coord.verBufSize() + coord.indBufSize()
            +cube.verBufSize() + cube.indBufSize()
            +sphere.verBufSize() + sphere.indBufSize()
            +pyramide.verBufSize() + pyramide.indBufSize()
            +camSphere.verBufSize() + camSphere.indBufSize()
            +lap.verBufSize() + lap.indBufSize()
            +lap2.verBufSize() + lap2.indBufSize()
            +skybox.verBufSize() + skybox.indBufSize()
            +fence.verBufSize() + fence.indBufSize()
            +texsphere.verBufSize() + texsphere.indBufSize()
            +model.verBufSize() + model.indBufSize()
            +fronttires.verBufSize() + fronttires.indBufSize()
            +backtires.verBufSize() + backtires.indBufSize()
            +lidar.verBufSize() + lidar.indBufSize()
                     , nullptr, GL_STATIC_DRAW);

    //    makeUnderUse(program1);

        GLsizeiptr currentOffset = 0;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, plane.verBufSize(), plane.verts);
        currentOffset += plane.verBufSize();
        plane_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, plane.indBufSize(), plane.indices);
        currentOffset += plane.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, grid.verBufSize(), grid.verts);
        currentOffset += grid.verBufSize();
        grid_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, grid.indBufSize(), grid.indices);
        currentOffset += grid.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, coord.verBufSize(), coord.verts);
        currentOffset += coord.verBufSize();
        coord_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, coord.indBufSize(), coord.indices);
        currentOffset += coord.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, cube.verBufSize(), cube.verts);
        currentOffset += cube.verBufSize();
        cube_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, cube.indBufSize(), cube.indices);
        currentOffset += cube.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, sphere.verBufSize(), sphere.verts);
        currentOffset += sphere.verBufSize();
        sphere_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, sphere.indBufSize(), sphere.indices);
        currentOffset += sphere.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, pyramide.verBufSize(), pyramide.verts);
        currentOffset += pyramide.verBufSize();
        pyramide_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, pyramide.indBufSize(), pyramide.indices);
        currentOffset += pyramide.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, camSphere.verBufSize(), camSphere.verts);
        currentOffset += camSphere.verBufSize();
        camSphere_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, camSphere.indBufSize(), camSphere.indices);
        currentOffset += camSphere.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, lap.verBufSize(), lap.verts);
        currentOffset += lap.verBufSize();
        lap_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, lap.indBufSize(), lap.indices);
        currentOffset += lap.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, lap2.verBufSize(), lap2.verts);
        currentOffset += lap2.verBufSize();
        lap2_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, lap2.indBufSize(), lap2.indices);
        currentOffset += lap2.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, skybox.verBufSize(), skybox.verts);
        currentOffset += skybox.verBufSize();
        skybox_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, skybox.indBufSize(), skybox.indices);
        currentOffset += skybox.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, fence.verBufSize(), fence.verts);
        currentOffset += fence.verBufSize();
        fence_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, fence.indBufSize(), fence.indices);
        currentOffset += fence.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, texsphere.verBufSize(), texsphere.verts);
        currentOffset += texsphere.verBufSize();
        texsphere_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, texsphere.indBufSize(), texsphere.indices);
        currentOffset += texsphere.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, model.verBufSize(), model.verts);
        currentOffset += model.verBufSize();
        model_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, model.indBufSize(), model.indices);
        currentOffset += model.indBufSize();


        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, fronttires.verBufSize(), fronttires.verts);
        currentOffset += fronttires.verBufSize();
        fronttires_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, fronttires.indBufSize(), fronttires.indices);
        currentOffset += fronttires.indBufSize();


        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, backtires.verBufSize(), backtires.verts);
        currentOffset += backtires.verBufSize();
        backtires_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, backtires.indBufSize(), backtires.indices);
        currentOffset += backtires.indBufSize();

        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, lidar.verBufSize(), lidar.verts);
        currentOffset += lidar.verBufSize();
        lidar_indOffset = currentOffset;
        glBufferSubData(GL_ARRAY_BUFFER, currentOffset, lidar.indBufSize(), lidar.indices);
        currentOffset += lidar.indBufSize();



        planeNumIndices = plane.indNum;
        gridNumIndices = grid.indNum;
        coordNumIndices = coord.indNum;
        cubeNumIndices  = cube.indNum;
        sphereNumIndices = sphere.indNum;
        pyramideNumIndices = pyramide.indNum;
        camSphereNumIndices = camSphere.indNum;
        lapNumIndices = lap.indNum;
        lap2NumIndices = lap2.indNum;
        skyboxNumIndices = skybox.indNum;
        fenceNumIndices = fence.indNum;
        texsphereNumIndices = texsphere.indNum;
        modelNumIndices = model.indNum;
        fronttiresNumIndices = model.indNum;
        backtiresNumIndices = model.indNum;
        lidarNumIndices = model.indNum;



        glGenVertexArrays(1, &planeVAO_ID);
        glGenVertexArrays(1, &gridVAO_ID);
        glGenVertexArrays(1, &coordVAO_ID);
        glGenVertexArrays(1, &cubeVAO_ID);
        glGenVertexArrays(1, &sphereVAO_ID);
        glGenVertexArrays(1, &pyramideVAO_ID);
        glGenVertexArrays(1, &camSphereVAO_ID);
        glGenVertexArrays(1, &lapVAO_ID);
        glGenVertexArrays(1, &lap2VAO_ID);
        glGenVertexArrays(1, &skyboxVAO_ID);
        glGenVertexArrays(1, &fenceVAO_ID);
        glGenVertexArrays(1, &texsphereVAO_ID);
        glGenVertexArrays(1, &modelVAO_ID);
        glGenVertexArrays(1, &fronttiresVAO_ID);
        glGenVertexArrays(1, &backtiresVAO_ID);
        glGenVertexArrays(1, &lidarVAO_ID);

        glBindVertexArray(planeVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(0));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(gridVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint gridOffset = plane.verBufSize() + plane.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(gridOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(gridOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(gridOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(coordVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint coordOffset = gridOffset + grid.verBufSize() + grid.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(coordOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(coordOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(coordOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(cubeVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint cubeOffset = coordOffset + coord.verBufSize() + coord.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(cubeOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(cubeOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(cubeOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(sphereVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint sphereOffset = cubeOffset + cube.verBufSize() + cube.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(sphereOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(sphereOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(sphereOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(pyramideVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint pyramideOffset = sphereOffset + sphere.verBufSize() + sphere.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(pyramideOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(pyramideOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(pyramideOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(camSphereVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint camSphereOffset = pyramideOffset + pyramide.verBufSize() + pyramide.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(camSphereOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(camSphereOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, VERTEX_BYTE_SIZE, reinterpret_cast<void*>(camSphereOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);


        glBindVertexArray(lapVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint lapOffset = camSphereOffset + camSphere.verBufSize() + camSphere.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lapOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lapOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lapOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(lap2VAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint lap2Offset = lapOffset + lap.verBufSize() + lap.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lap2Offset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lap2Offset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lap2Offset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(skyboxVAO_ID);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint skyboxOffset = lap2Offset + lap2.verBufSize() + lap2.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<void*>(skyboxOffset));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(fenceVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint fenceOffset = skyboxOffset + skybox.verBufSize() + skybox.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(fenceOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(fenceOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(fenceOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        glBindVertexArray(texsphereVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint texsphereOffset = fenceOffset + fence.verBufSize() + fence.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(texsphereOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(texsphereOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(texsphereOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);


        glBindVertexArray(modelVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint modelOffset = texsphereOffset + texsphere.verBufSize() + texsphere.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(modelOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(modelOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(modelOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);


        glBindVertexArray(fronttiresVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint fronttiresOffset = modelOffset + model.verBufSize() + model.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(fronttiresOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(fronttiresOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(fronttiresOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);


        glBindVertexArray(backtiresVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint backtiresOffset = fronttiresOffset + fronttires.verBufSize() + fronttires.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(backtiresOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(backtiresOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(backtiresOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);


        glBindVertexArray(lidarVAO_ID);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
        GLuint lidarOffset = backtiresOffset + backtires.verBufSize() + backtires.indBufSize();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lidarOffset));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lidarOffset + sizeof(float) * 3));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(lidarOffset + sizeof(float) * 6));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);


        transformMatLocation_0 = glGetUniformLocation(programs[0], "transformMat");
        transformMatLocation_1 = glGetUniformLocation(programs[1], "transformMat");
        transformMatLocation_2 = glGetUniformLocation(programs[2], "transformMat");
        transformMatLocation_3 = glGetUniformLocation(programs[3], "transformMat");

        itt_MatLocation_P3 = glGetUniformLocation(programs[3], "it_transformMat");



        plane.cleanUP();
        grid.cleanUP();
        coord.cleanUP();
        cube.cleanUP();
        sphere.cleanUP();
        pyramide.cleanUP();
        camSphere.cleanUP();
        lap.cleanUP();
        lap2.cleanUP();
        skybox.cleanUP();
        fence.cleanUP();
        texsphere.cleanUP();
        model.cleanUP();
        fronttires.cleanUP();
        backtires.cleanUP();
        lidar.cleanUP();



        QImage tex_buffer;
        //---------------------------------------------
        glGenTextures(1, &textureID1); // mini-lape
        glBindTexture(GL_TEXTURE_2D, textureID1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        load_tex(tex_buffer, "tex/mini_lap_exemplar.png", "PNG", 0, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_buffer.width(), tex_buffer.height(), 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, tex_buffer.bits());

        //-----------------------------------------
        glGenTextures(1, &textureID2);// lap2
        glBindTexture(GL_TEXTURE_2D, textureID2);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        load_tex(tex_buffer, "tex/lap1_exemplar.jpg", "JPG", 0, 0);
    //    load_tex(tex_buffer, "textures/132148_header.jpg", "JPG"); // 132148_header.jpg
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_buffer.width(), tex_buffer.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_buffer.bits());

        //----------------------------------
        std::vector<QString> faces={
            "tex/skybox-jpg/right.jpg",
            "tex/skybox-jpg/left.jpg",
            "tex/skybox-jpg/top.jpg",
            "tex/skybox-jpg/bottom.jpg",
            "tex/skybox-jpg/front.jpg",
            "tex/skybox-jpg/back.jpg" };

        glGenTextures(1, &textureID3); // skybox
        glBindTexture(GL_TEXTURE_CUBE_MAP, textureID3);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_BASE_LEVEL, 0);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_LEVEL, 0);

        for (GLuint i = 0; i < 6; ++i){
            load_tex(tex_buffer, faces[i], "JPG", 0, 1);
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+i, 0, 3, tex_buffer.width(), tex_buffer.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_buffer.bits());
        }

       //---------------------------------------------
       glGenTextures(1, &textureID4); // plane
       glBindTexture(GL_TEXTURE_2D, textureID4);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //   load_tex(tex_buffer, "textures/green+grass-1024x1024.png", "PNG"); // TexturesCom_Grass0003_1_seamless_S.jpg
       load_tex(tex_buffer, "tex/plane_grass_1024x1024.jpg", "JPG", 0, 1);
//       load_tex(tex_buffer, "tex/gras.jpg", "JPG", 0, 0);
       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_buffer.width(), tex_buffer.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_buffer.bits());
       //------------------------------
       glGenTextures(1, &textureID5); // fence
       glBindTexture(GL_TEXTURE_2D, textureID5);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
       load_tex(tex_buffer, "tex/fence_brick_1024x1024.jpg", "JPG", 0, 1); // TexturesCom_MetalBare0169_2_seamless_S.jpg
    //   load_tex(tex_buffer, "textures/TexturesCom_ConcreteBunkerDirty0070_1_S.jpg", "JPG"); // TexturesCom_MetalBare0169_2_seamless_S.jpg
       glTexImage2D(GL_TEXTURE_2D, 0, 3, tex_buffer.width(), tex_buffer.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_buffer.bits());

       //------------------------------
       glGenTextures(1, &textureID6); // zaytuna model
       glBindTexture(GL_TEXTURE_2D, textureID6);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
       glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //   load_tex(tex_buffer, "textures/zaytuna-materials.png", "PNG"); // TexturesCom_MetalBare0169_2_seamless_S.jpg

//       tex_buffer.load("textures/zaytuna-fragments2.png", "PNG");
       if(!tex_buffer.load("tex/zaytuna-fragments.png", "PNG")){
            std::cout << "image couldn't be loaded <" << "textures/zaytuna-fragments2.png" << ">!\n";
            exit(EXIT_FAILURE);
       }
       tex_buffer = QGLWidget::convertToGLFormat(tex_buffer);
       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_buffer.width(), tex_buffer.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_buffer.bits());





       this->model.programID = programs[3];
       this->model.textureID = textureID6;

       this->model.modelVAO_ID = modelVAO_ID;
       this->model.fronttiresVAO_ID = fronttiresVAO_ID;
       this->model.backtiresVAO_ID = backtiresVAO_ID;
       this->model.lidarVAO_ID = lidarVAO_ID;

       this->model.model_indOffset = model_indOffset;
       this->model.fronttires_indOffset = fronttires_indOffset;
       this->model.backtires_indOffset = backtires_indOffset;
       this->model.lidar_indOffset = lidar_indOffset;

       this->model.modelNumIndices = modelNumIndices;
       this->model.fronttiresNumIndices = fronttiresNumIndices;
       this->model.backtiresNumIndices = backtiresNumIndices;
       this->model.lidarNumIndices = lidarNumIndices;
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



void _scene_widg::initGLAtrib()

{

}

} // namespace  zaytuna
