

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







#include "zay_scene_widg.hpp"
#include "zay_primary_win.hpp"


namespace zaytuna {


//// for debugging
double GLOBAL_MOVEMENT_SPEED{0.0};
double GLOBAL_STEERING_WHEEL{0.00000001};
extern double SLIDER_MOVEMENT_SPEED;
extern double SLIDER_STEERING_WHEEL;

double _scene_widg::sX, _scene_widg::sY; //, _scene_widg::sZ;
double _scene_widg::delta_sX, _scene_widg::delta_sY;
double _scene_widg::glX, _scene_widg::glY; //, _scene_widg::glZ;


//_scene_widg::_scene_widg(QWidget* parent): QGL_WIDGET_VERSION(parent),
_scene_widg::_scene_widg(QGLFormat _format, QWidget* parent):
    QGL_WIDGET_VERSION(_format, parent),
    elap_accumulated{0.0}, cam_freq_accumulated{0.0}, elapsed{0.0},
    frame_rate{1.0}, front_cam_freq{FRONT_CAM_FREQUENCY}, 
    imgs_sec{1.0/FRONT_CAM_FREQUENCY}, local_control_speed{6.0},
    local_control_steering{0.39269908}, frames_counter{0},
    limited_frames{60}, activeCam{&mainCam},
    k_forward{0}, k_backward{0}, k_left{0}, k_right{0}, k_up{0}, k_down{0},
    l_forward{0}, l_backward{0}, l_left{0}, l_right{0}
{
    //    QSurfaceFormat _format;
    //    _format.setSamples(NUM_SAMPLES_PER_PIXEL); 
    //    setFormat(_format);

    mainCam.updateProjection(WIDTH, HEIGHT);
    connect(&timer, SIGNAL(timeout()), this, SLOT(animate()));
    makeCurrent();
    setMouseTracking(true);
    this->setFocusPolicy(Qt::StrongFocus);
    timer.setTimerType(Qt::PreciseTimer);
}

void _scene_widg::update_contrl_attribs(void){
    if(key_control){
        GLOBAL_MOVEMENT_SPEED = l_forward? -local_control_speed:
          (l_backward? local_control_speed:0.0);

        GLOBAL_STEERING_WHEEL = l_right? local_control_steering:
          (l_left? -local_control_steering:STEERING_MARGIN_OF_ERROR);

    }else{
        GLOBAL_MOVEMENT_SPEED = SLIDER_MOVEMENT_SPEED;
        GLOBAL_STEERING_WHEEL = SLIDER_STEERING_WHEEL;
    }
}

_scene_widg::~_scene_widg(){
    cleanUp();
    if(model_vehicles != nullptr)
        delete model_vehicles;
    if(obstacle_objects != nullptr)
        delete obstacle_objects;
    if(skybox != nullptr)
        delete skybox;
}

void _scene_widg::cleanUp(){
    glDeleteBuffers(1, &theBufferID);
}

void _scene_widg::initializeGL(){

    initializeOpenGLFunctions();
    glClearColor(0.86f, 0.86f, 0.86f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glViewport(0, 0, this->width(), this->height());
    programs_ = {
        new static_program(this, 
                          ZAY_PACKAGE_PATH+"/programs/source0"),
        new animated_program(this, 
                          ZAY_PACKAGE_PATH+"/programs/source1"),
        new basic_program(this, 
                          ZAY_PACKAGE_PATH+"/programs/source2"),
        new animated_program(this, 
                          ZAY_PACKAGE_PATH+"/programs/source3")
    };
    send_data();
    std::cout << "  Zaytuna Simulator " <<ZAYTUNA_VERSION << "." << ZAYTUNA_MINOR_VERSION << ", "
              << "running with following system specifications:\n" << std::flush;
             
    ROS_INFO_STREAM("Vendor: " << reinterpret_cast<const char*>(glGetString(GL_VENDOR)));
    ROS_INFO_STREAM("Renderer: " << reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
    ROS_INFO_STREAM("OpenGL Version: " << reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    ROS_INFO_STREAM("GL Shading Language Version: " << reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION)));

    timer.start(1000/limited_frames);
    start_t = std::chrono::high_resolution_clock::now();
}

void _scene_widg::update_time_interval(uint32_t val){
    limited_frames = val;
    timer.setInterval(1000/val);
}

void _scene_widg::render_local_scene(camera const*const current_cam){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    programs_[1]->makeUnderUse();
    uint32_t i{0};
	for(; i<environmental_objects.size(); ++i)
        environmental_objects[i]->render_obj(current_cam);
    programs_[2]->makeUnderUse();
    skybox->render_obj(current_cam);

////----- disable 'grid' and 'coordinate axes' from vehicles cam--------
/*
    programs_[0]->makeUnderUse();
    if(coord_checked)
        basic_objects[0]->render_obj(current_cam);
    if(grid_checked)
        basic_objects[1]->render_obj(current_cam);
*/

    programs_[3]->makeUnderUse();
    obstacle_objects->render_obj(current_cam);
    model_vehicles->render_obj(current_cam);
}

void _scene_widg::paintGL(){
    ros::spinOnce();
    update_contrl_attribs();
    if(activeCam == &mainCam){
        update_cam();
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

    for(uint32_t i{0}; i<model_vehicles->vehicles.size(); ++i)
        model_vehicles->vehicles[i].update_attribs(frame_rate);
    if(activeCam==&mainCam)
        mainCam.updateWorld_to_viewMat();
    render_main_scene(activeCam);

    // frame rate
    elapsed = std::chrono::duration<double,
              std::ratio< 1, 1>>(std::chrono::high_resolution_clock::now()
                                 - start_t).count();
    start_t = std::chrono::high_resolution_clock::now();

    elap_accumulated += elapsed;
    cam_freq_accumulated += elapsed;
    ++frames_counter;

    if(elap_accumulated>=NUM_SEC_FRAME_RATE) {
//        std::cout << DebugGLerr(glGetError()) << "\n";
        frame_rate = 1.0/(elap_accumulated/frames_counter);
        elap_accumulated = frames_counter = 0;
    }

    if(cam_freq_accumulated>=imgs_sec){
        for(uint32_t i{0}; i<model_vehicles->vehicles.size(); ++i){
            model_vehicles->vehicles[i].localView_buffer->bind();
            render_local_scene(&(model_vehicles->vehicles[i].frontCam));
            model_vehicles->vehicles[i].grab_buffer();
        }
        glBindFramebuffer(GL_FRAMEBUFFER,  0);
        cam_freq_accumulated = 0.0;
    }

}

void _scene_widg::render_main_scene(camera const*const current_cam){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    programs_[0]->makeUnderUse();
    if(coord_checked)
        basic_objects[0]->render_obj(current_cam);
    if(grid_checked)
        basic_objects[1]->render_obj(current_cam);
    programs_[1]->makeUnderUse();
    uint32_t i{0};
	for(; i<environmental_objects.size(); ++i)
        environmental_objects[i]->render_obj(current_cam);
    programs_[2]->makeUnderUse();
    skybox->render_obj(current_cam);
    programs_[3]->makeUnderUse();
    obstacle_objects->render_obj(current_cam);
    model_vehicles->render_obj(current_cam);
}

void _scene_widg::resizeGL(int W, int H){
    glViewport(0, 0, W, H);
    mainCam.updateProjection(W, H);
    for(uint32_t i{0}; i<model_vehicles->vehicles.size(); ++i){
        model_vehicles->vehicles[i].frontCam.updateProjection(W, H);
    }
    repaint();
}

void _scene_widg::updateProjection(){
    mainCam.updateProjection(WIDTH, HEIGHT);
    for(uint32_t i{0}; i<model_vehicles->vehicles.size(); ++i){
        model_vehicles->vehicles[i].frontCam.updateProjection
                (WIDTH, HEIGHT);
    }
}

void _scene_widg::mouseMoveEvent(QMouseEvent *ev){
    if(activeCam == &mainCam){
        if(ev->buttons()==Qt::RightButton){
            delta_sX = sX - ev->pos().x();
            delta_sY = sY - ev->pos().y();
            if( delta_sX > 0.0)
                mainCam.strafe_right();
            if(delta_sX < 0.0)
                mainCam.strafe_left();
            if(delta_sY > 0.0)
                mainCam.move_backward();
            if(delta_sY < 0.0)
                mainCam.move_forward();
            sX = static_cast<double>(ev->x());
            sY = static_cast<double>(ev->y());
        }
        else{
            sX = static_cast<double>(ev->x());
            sY = static_cast<double>(ev->y());
            glX =  (sX/ (static_cast<double>
                         (this->width())/2.0) ) -1;
            glY =  -((sY/ (static_cast<double>
                           (this->height())/2.0) ) -1);
            if(ev->buttons()==Qt::LeftButton)
                mainCam.mouse_held_update(glm::vec2(ev->x(),ev->y()));
            else
                mainCam.mouse_update(glm::vec2(ev->x(),ev->y()));
        }
    }
}

void _scene_widg::update_cam(){
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
    else if(k_down)
        mainCam.move_down();
}

void _scene_widg::keyPressEvent(QKeyEvent* ev){

    if(activeCam == &mainCam){
        switch (ev->key()){
            case Qt::Key::Key_W:
                k_forward=1;
                return;
            case Qt::Key::Key_S:
                k_backward=1;
                return;
            case Qt::Key::Key_Left: case Qt::Key::Key_A:
                k_left=1;
                return;
            case Qt::Key::Key_Right: case Qt::Key::Key_D:
                k_right=1;
                return;
            case Qt::Key::Key_Up:
                k_up=1;
                return;
            case Qt::Key::Key_Down:
                k_down=1;
                return;
        }
    }
    switch (ev->key()){
        case Qt::Key::Key_T:
            l_forward=1;
            break;
        case Qt::Key::Key_G:
            l_backward=1;
            break;
        case Qt::Key::Key_F:
            l_left=1;
            break;
        case Qt::Key::Key_H:
            l_right=1;
    }
}

void _scene_widg::wheelEvent(QWheelEvent *ev){
    if(ev->delta() > 0)
        mainCam.move_forward();
    else mainCam.move_backward();
}

void _scene_widg::keyReleaseEvent(QKeyEvent *ev)
{
    switch (ev->key()){
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
            k_down=0;
            break;
        case Qt::Key::Key_A:
            k_left=0;
            break;
        case Qt::Key::Key_D:
            k_right=0;
            break;
        case Qt::Key::Key_T:
            l_forward=0;
            break;
        case Qt::Key::Key_G:
            l_backward=0;
            break;
        case Qt::Key::Key_F:
            l_left=0;
            break;
        case Qt::Key::Key_H:
            l_right=0;
    }
}

//void _scene_widg::mousePressEvent(QMouseEvent *ev)
//{
////    if(activeCam != &mainCam)
////        return;
////    if(ev->buttons()==Qt::LeftButton){
////        mainCam.update_view_point();
////    }
//}

//void _scene_widg::mouseReleaseEvent(QMouseEvent *ev)
//{
//    clicked = false;
//}

void _scene_widg::animate(){
    repaint();
}
void _scene_widg::send_data()
{
    basic_objects = {
        new coord_sys(this,
                      programs_[0]->program_handler(),
                      "coord_sys",
                      20.f,
                      1.5f
//                      ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                      glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        ),
        new grid_plane(this,
                      programs_[0]->program_handler(),
                      "grid_plane",
                      150.f,
                      150.f,
                      1.0f,
                      1.0f
//                      ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                      glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        )
    };
    //---------------------------
    environmental_objects = {
        new external_obj(this,
                         programs_[1]->program_handler(),
                         "plane",
                         ZAY_PACKAGE_PATH+"/primitives/zay_plane_300x300_1sub-div",
                         "/tex/plane_grass_1024x1024",
                         GL_QUADS
//                         ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                         glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        )
        ,new external_obj(this,
                         programs_[1]->program_handler(),
                         "fence",
                         ZAY_PACKAGE_PATH+"/primitives/zay_fence_300x300_2H_1W",
                         "/tex/fence_brick_1024x1024",
                         GL_QUADS
//                         ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                         glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        )
        ,new external_obj(this,
                         programs_[1]->program_handler(),
                         "mini_lap",
                         ZAY_PACKAGE_PATH+"/primitives/zay_mini_lap",
                         "/tex/mini_lap_exemplar",
                         GL_QUADS
//                         ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                         glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        )
        ,new external_obj(this,
                         programs_[1]->program_handler(),
                         "lap1",
                         ZAY_PACKAGE_PATH+"/primitives/zay_lap1",
                         "/tex/lap1_exemplar",
                         GL_QUADS
//                         ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                         glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        )
        ,new external_obj(this,
                         programs_[1]->program_handler(),
                         "lap2",
                         ZAY_PACKAGE_PATH+"/primitives/zay_lap2",
                         "/tex/lap2_exemplar",
                         GL_QUADS
//                         ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                         glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        )
        ,new external_obj(this,
                         programs_[1]->program_handler(),
                         "lap3",
                         ZAY_PACKAGE_PATH+"/primitives/zay_lap3",
                         "/tex/lap3_exemplar",
                         GL_QUADS
//                        ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
        )

    };
    //-------------------------
    skybox = new skybox_obj(this,
                            programs_[2]->program_handler(),
                            "skybox"
//                             ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
//                             glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                        );
    //---------------------------------
    obstacle_objects =
            new obstacle_pack<GLdouble>(this,
                programs_[3]->program_handler(),
                Obstacle_Type::CARTON_BOX,
                ZAY_PACKAGE_PATH+"/primitives/zay_carton_box",
                "/tex/carton_box",
                GL_QUADS);
    obstacle_objects->add_category
            (Obstacle_Type::BRICK_WALL,
             ZAY_PACKAGE_PATH+"/primitives/zay_brick_wall",
             "/tex/brick_wall");
    obstacle_objects->add_category
            (Obstacle_Type::STONE_WALL,
             ZAY_PACKAGE_PATH+"/primitives/zay_stone_wall",
             "/tex/stone_wall_1");
    //------------------------------

    fboFormat.setSamples(NUM_SAMPLES_PER_PIXEL);
    fboFormat.setAttachment
            (QGLFramebufferObject::CombinedDepthStencil);
    model_vehicles = new model_vehicle(this,
           programs_[3]->program_handler(),
           ZAY_PACKAGE_PATH+"/primitives/zaytuna_model",
           "/tex/zaytuna-fragments",
           GL_TRIANGLES );

    //-------------------------------

    GLsizeiptr BUF_SIZE{0};
    for(const auto& _obj:basic_objects)
        BUF_SIZE+=_obj->buffer_size();
    for(const auto& _obj:environmental_objects)
        BUF_SIZE+=_obj->buffer_size();

    BUF_SIZE+=skybox->buffer_size();
    BUF_SIZE+=obstacle_objects->buffer_size();
    BUF_SIZE+=model_vehicles->buffer_size();

    glGenBuffers(1, &theBufferID);
    glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
    glBufferData(GL_ARRAY_BUFFER,
                 BUF_SIZE, nullptr,
                 GL_STATIC_DRAW);

    GLintptr current_offset{0};
    GLuint previous_offset{0};
    for(auto& _obj:basic_objects)
        _obj->transmit_data(current_offset,
                            theBufferID,
                            previous_offset);
    for(auto& _obj:environmental_objects)
        _obj->transmit_data(current_offset,
                            theBufferID,
                            previous_offset);
    skybox->transmit_data(current_offset,
                          theBufferID,
                          previous_offset);
    obstacle_objects->transmit_data(current_offset,
                                    theBufferID,
                                    previous_offset);
    model_vehicles->transmit_data(current_offset,
                                  theBufferID,
                                  previous_offset);

    add_default_obj();
}
void _scene_widg::add_default_obj(){
    for(uint32_t i{0}; i<default_objects.vehicles.size(); ++i)
        model_vehicles->add_vehicle
                (new QGLFramebufferObject(WIDTH,
                                          HEIGHT, fboFormat),
                 default_objects.vehicles[i]);

    update_current_vehicle("any_vehicle");
    for(uint32_t i{0}; i<default_objects.obstacles.size(); ++i)
        add_obstacle(default_objects.obstacles[i]);
}
void _scene_widg::update_current_vehicle
    (const std::string& _name){
    auto it = model_vehicles->find(_name);
    if(it == model_vehicles->vehicles.end())
        if(model_vehicles->vehicles.size() != 0)
            current_model = &model_vehicles->vehicles[0];
        else current_model = nullptr;
    else current_model = &(*it);
}
void _scene_widg::add_vehicle
    (const transform_attribs<GLdouble>& attribs){
    model_vehicles->add_vehicle
        (new QGLFramebufferObject(WIDTH,
         HEIGHT, fboFormat),
         attribs);
}

void _scene_widg::delete_vehicle
    (const std::string& _name){
    auto it = model_vehicles->find(_name);
    model_vehicles->vehicles.erase(it);
}
zaytuna::vehicle_attributes*
_scene_widg::getOtherVeh(const std::string& _name){
    zaytuna::vehicle_attributes* other{nullptr};
    for(uint32_t i{0}; i<model_vehicles->vehicles.size(); ++i)
        if(model_vehicles->vehicles[i].attribs.name != _name)
            return &model_vehicles->vehicles[i];
    return other;
}
void _scene_widg::add_obstacle
    (const obstacle_attribs<GLdouble>& attribs){
    obstacle_objects->categories
            [attribs.type]->instances.push_back(attribs);
    obstacle_objects->category[attribs.name] = attribs.type;
}
void _scene_widg::delete_obstacle
    (const std::string& _name){
    obstacle_objects->delete_obstacle(_name);
    obstacle_objects->category.erase(_name);
}
obstacle_attribs<GLdouble>
_scene_widg::get_obstacle(const std::string& _name){
    return obstacle_objects->get_attribs(_name);
}

void _scene_widg::edit_obstacle
    (const obstacle_attribs<GLdouble>& attribs){
    auto it = obstacle_objects->find(attribs.name);
    it->edit(attribs);
}


} // namespace  zaytuna



