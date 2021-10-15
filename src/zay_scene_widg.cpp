

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







#include "zay_scene_widg.hpp"


namespace zaytuna {


double _scene_widg::sX{0}, _scene_widg::sY{0}; //, _scene_widg::sZ;
double _scene_widg::delta_sX{0}, _scene_widg::delta_sY{0};
double _scene_widg::glX{-1}, _scene_widg::glY{1}; //, _scene_widg::glZ;


// _scene_widg::_scene_widg(ZAY_MSG_LOGGER* msg_log, QWidget* parent):
//     ZAY_QGL_WIDGET_VERSION(parent), message_logger{msg_log},
//     elap_accumulated{0.0}, cam_freq_accumulated{0.0}, elapsed{0.0},
//     frame_rate{0.0}, front_cam_freq{ZAY_DEFAULT_FRONT_CAM_FREQUENCY}, 
//     imgs_sec{1.0/ZAY_DEFAULT_FRONT_CAM_FREQUENCY}, local_control_speed{ZAY_DEFAULT_LOCAL_CONTROL_SPEED},
//     local_control_steering{ZAY_DEFAULT_LOCAL_CONTROL_STEERING}, frames_counter{0},
//     limited_frames{ZAY_DEFAULT_FRAME_RATE}, activeCam{&mainCam},
//     key_control{0}, fixed_cam{0},
//     k_forward{0}, k_backward{0}, k_left{0}, k_right{0}, k_up{0}, k_down{0},
//     l_forward{0}, l_backward{0}, l_left{0}, l_right{0}
// { 
//     if(std::is_same<ZAY_QGL_WIDGET_VERSION, ZAY_Q_OPEN_GL_W>::value){
//         QSurfaceFormat format_;
//         format_.setSamples(ZAY_NUM_SAMPLES_PER_PIXEL); 
//         ((ZAY_Q_OPEN_GL_W*)(this))->setFormat(format_);
//     }

//     set_up();
// }

_scene_widg::_scene_widg(ZAY_MSG_LOGGER* msg_log, QGLFormat format_, QWidget* parent):
    ZAY_QGL_WIDGET_VERSION(format_, parent), message_logger{msg_log},
    elap_accumulated{0.0}, cam_freq_accumulated{0.0}, elapsed{0.0},
    frame_rate{0.0}, front_cam_freq{ZAY_DEFAULT_FRONT_CAM_FREQUENCY}, 
    imgs_sec{1.0/ZAY_DEFAULT_FRONT_CAM_FREQUENCY}, local_control_speed{ZAY_DEFAULT_LOCAL_CONTROL_SPEED},
    local_control_steering{ZAY_DEFAULT_LOCAL_CONTROL_STEERING}, frames_counter{0},
    limited_frames{ZAY_DEFAULT_FRAME_RATE}, activeCam{&mainCam},
    key_control{0}, fixed_cam{0},
    k_forward{0}, k_backward{0}, k_left{0}, k_right{0}, k_up{0}, k_down{0},
    l_forward{0}, l_backward{0}, l_left{0}, l_right{0}
{ set_up();}


void _scene_widg::set_up(){

    mainCam.updateProjection(ZAY_SCENE_WIDTH, ZAY_SCENE_HEIGHT);

    main_loop_timer.setTimerType(Qt::PreciseTimer);
    connect(&main_loop_timer, SIGNAL(timeout()), this, SLOT(animate()));

    makeCurrent();
    setMouseTracking(true);
    this->setFocusPolicy(Qt::StrongFocus);

}


void _scene_widg::update_contrl_attribs(void){

    if(key_control){
        v_state.MOVEMENT_SPEED = l_forward? -local_control_speed:
          (l_backward? local_control_speed:0.0);

        v_state.STEERING_WHEEL = l_right? local_control_steering:
          (l_left? -local_control_steering:ZAY_STEERING_MARGIN_OF_ERROR);

    }else{
        v_state.MOVEMENT_SPEED = SLIDER_MOVEMENT_SPEED;
        v_state.STEERING_WHEEL = SLIDER_STEERING_WHEEL;
    }

}


_scene_widg::~_scene_widg(){

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
    
    programs_.reserve(ZAY_PROGRAMS_NUM);
    programs_.emplace_back(new static_program(this, ZAY_PACKAGE_PATH+"/programs/source0"));
    programs_.emplace_back(new animated_program(this, ZAY_PACKAGE_PATH+"/programs/source1"));
    programs_.emplace_back(new basic_program(this, ZAY_PACKAGE_PATH+"/programs/source2"));
    programs_.emplace_back(new animated_program(this, ZAY_PACKAGE_PATH+"/programs/source3"));


    send_data();
    std::cout << "  Zaytuna Simulator " <<ZAYTUNA_VERSION << "." << ZAYTUNA_MINOR_VERSION << ", "
              << "running with following system specifications:\n" << std::flush;
             
    ROS_INFO_STREAM("Vendor: " << reinterpret_cast<const char*>(glGetString(GL_VENDOR)));
    ROS_INFO_STREAM("Renderer: " << reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
    ROS_INFO_STREAM("OpenGL Version: " << reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    ROS_INFO_STREAM("GL Shading Language Version: " << reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION)));

    main_loop_timer.start(1000/limited_frames);
    start_t = std::chrono::high_resolution_clock::now();

}


void _scene_widg::update_time_interval(uint32_t val){

    limited_frames = val;
    main_loop_timer.stop();

    if(val!=0)
        main_loop_timer.start(1000/val);
    else
        frame_rate = 0.0;
    
}


void _scene_widg::update_fc_time_interval(double val){

    front_cam_freq = val;
    if(val>0.0){
        cam_freq_accumulated=imgs_sec = 1.0/val;

    }else{
        imgs_sec = std::numeric_limits<decltype(imgs_sec)>::max();
        cam_freq_accumulated=0;
    }
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

    if(frame_rate!=0.0)
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

    if(elap_accumulated>=ZAY_NUM_SEC_FRAME_RATE) {
//        std::cout << DebugGLerr(glGetError()) << "\n";

        if(limited_frames == 0)
            frame_rate = 0.0;
        else
            frame_rate = frames_counter/elap_accumulated;
        
        elap_accumulated = frames_counter = 0;
        if(front_cam_freq > 0.0 && cam_freq_accumulated > 2*imgs_sec)
            cam_freq_accumulated=2*imgs_sec;
        
    }

    if(cam_freq_accumulated>=imgs_sec){

        for(uint32_t i{0}; i<model_vehicles->vehicles.size(); ++i){

            model_vehicles->vehicles[i].bind();
            render_local_scene(&(model_vehicles->vehicles[i].frontCam));

            model_vehicles->vehicles[i].advertise_frontCam();

        }

        glBindFramebuffer(GL_FRAMEBUFFER,  0);
        cam_freq_accumulated-=imgs_sec;
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

    mainCam.updateProjection(ZAY_SCENE_WIDTH, ZAY_SCENE_HEIGHT);
    for(uint32_t i{0}; i<model_vehicles->vehicles.size(); ++i){
        model_vehicles->vehicles[i].frontCam.updateProjection
                (ZAY_SCENE_WIDTH, ZAY_SCENE_HEIGHT);
    }

}

void _scene_widg::animate(){
    repaint();
}

void _scene_widg::send_data()
{
    basic_objects.reserve(ZAY_REMOVABLE_OBJS_NUM);
    basic_objects.emplace_back(new coord_sys(this,
                    programs_[0]->program_handler(),
                    "coord_sys",
                    ZAY_DEF_AXES_LENGTH,
                    ZAY_DEF_AXES_THICKNESS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));
    basic_objects.emplace_back(new grid_plane(this,
                    programs_[0]->program_handler(),
                    "grid_plane",
                    ZAY_DEF_GRID_LENGTH,
                    ZAY_DEF_GRID_WIDTH,
                    ZAY_DEF_GRID_TESSELLATION,
                    ZAY_DEF_GRID_LINE_THICKNESS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));

    //---------------------------
    environmental_objects.reserve(ZAY_ENVIRONMENTAL_OBJS_NUM);
    environmental_objects.emplace_back(new external_obj(this,
                    programs_[1]->program_handler(),
                    "plane",
                    ZAY_PACKAGE_PATH+"/primitives/zay_plane_300x300_1sub-div",
                    "/resources/plane_grass_1024x1024",
                    GL_QUADS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));
    environmental_objects.emplace_back(new external_obj(this,
                    programs_[1]->program_handler(),
                    "fence",
                    ZAY_PACKAGE_PATH+"/primitives/zay_fence_300x300_2H_1W",
                    "/resources/fence_brick_1024x1024",
                    GL_QUADS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));
    environmental_objects.emplace_back(new external_obj(this,
                    programs_[1]->program_handler(),
                    "mini_lap",
                    ZAY_PACKAGE_PATH+"/primitives/zay_mini_lap",
                    "/resources/mini_lap_exemplar",
                    GL_QUADS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));
    environmental_objects.emplace_back(new external_obj(this,
                    programs_[1]->program_handler(),
                    "lap1",
                    ZAY_PACKAGE_PATH+"/primitives/zay_lap1",
                    "/resources/lap1_exemplar",
                    GL_QUADS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));
    environmental_objects.emplace_back(new external_obj(this,
                    programs_[1]->program_handler(),
                    "lap2",
                    ZAY_PACKAGE_PATH+"/primitives/zay_lap2",
                    "/resources/lap2_exemplar",
                    GL_QUADS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));
    environmental_objects.emplace_back(new external_obj(this,
                    programs_[1]->program_handler(),
                    "lap3",
                    ZAY_PACKAGE_PATH+"/primitives/zay_lap3",
                    "/resources/lap3_exemplar",
                    GL_QUADS
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));
    //-------------------------
    skybox.reset(new skybox_obj(this,
                    programs_[2]->program_handler(),
                    "skybox"
                    // ,glm::rotate(glm::radians(0.0), glm::dvec3(0.0, 1.0, 0.0)),
                    // glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                ));

    //---------------------------------
    obstacle_objects.reset(new obstacle_pack<GLdouble>(this,
                programs_[3]->program_handler(),
                Obstacle_Type::CARTON_BOX,
                ZAY_PACKAGE_PATH+"/primitives/zay_carton_box",
                ZAY_PACKAGE_PATH+"/primitives/zay_carton_box_projection",
                "/resources/carton_box",
                GL_QUADS));
    
    obstacle_objects->add_category
            (Obstacle_Type::BRICK_WALL,
             ZAY_PACKAGE_PATH+"/primitives/zay_brick_wall",
             ZAY_PACKAGE_PATH+"/primitives/zay_brick_wall_projection",
             "/resources/brick_wall");
    
    obstacle_objects->add_category
            (Obstacle_Type::STONE_WALL,
             ZAY_PACKAGE_PATH+"/primitives/zay_stone_wall",
             ZAY_PACKAGE_PATH+"/primitives/zay_stone_wall_projection",
             "/resources/stone_wall_1");
    //------------------------------

    fboFormat.setSamples(ZAY_NUM_SAMPLES_PER_PIXEL);
    fboFormat.setAttachment
            (QGLFramebufferObject::CombinedDepthStencil);
    
    model_vehicles.reset(new model_vehicle(this,
           programs_[3]->program_handler(),
           ZAY_PACKAGE_PATH+"/primitives/zaytuna_model",
           ZAY_PACKAGE_PATH+"/primitives/zaytuna_model_projection",
           "/resources/zaytuna-fragments",
           &collision_pack,
           GL_TRIANGLES));

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

    load_external_collition_object(ZAY_PACKAGE_PATH+"/primitives/zay_fence-front_projection", "front_fence");
    load_external_collition_object(ZAY_PACKAGE_PATH+"/primitives/zay_fence-back_projection", "back_fence");
    load_external_collition_object(ZAY_PACKAGE_PATH+"/primitives/zay_fence-right_projection", "right_fence");
    load_external_collition_object(ZAY_PACKAGE_PATH+"/primitives/zay_fence-left_projection", "left_fence");

    for(uint32_t i{0}; i<default_objects.vehicles.size(); ++i){
        model_vehicles->add_vehicle
                (new QGLFramebufferObject(ZAY_SCENE_WIDTH,
                                          ZAY_SCENE_HEIGHT, 
                                          fboFormat),
                 default_objects.vehicles[i], &v_state, message_logger);

        collision_pack.dyn_objs.emplace_back(&(model_vehicles->vehicles.back().coll_rect));
    }

    update_current_vehicle("any_vehicle");
    for(uint32_t i{0}; i<default_objects.obstacles.size(); ++i)
        add_obstacle(default_objects.obstacles[i]);

    default_objects.clear();

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
    (const veh_transform_attribs<GLdouble>& attribs){

    model_vehicles->add_vehicle
        (new QGLFramebufferObject(ZAY_SCENE_WIDTH,
         ZAY_SCENE_HEIGHT, fboFormat), attribs, &v_state, message_logger);

    if(model_vehicles->vehicles.size()==1)
        current_model = &model_vehicles->vehicles[0];

    collision_pack.dyn_objs.emplace_back(&(model_vehicles->vehicles.back().coll_rect));

}


void _scene_widg::delete_vehicle
    (const std::string& _name){

    collision_pack.erase_veh(_name);

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

    add_static_collition_object(attribs);
}


void _scene_widg::delete_obstacle
    (const std::string& _name){

    obstacle_objects->delete_obstacle(_name);
    obstacle_objects->category.erase(_name);

    collision_pack.erase_obs(_name);
}


obstacle_attribs<GLdouble>
_scene_widg::get_obstacle(const std::string& _name){
    return obstacle_objects->get_attribs(_name);
}


void _scene_widg::load_external_collition_object
            (const std::string& _dir, const std::string& _name){

    rect_collistion_object<GLdouble> coll_obj;
    obj_parser::extractProjectionRect(_dir, coll_obj.points);

    coll_obj.ID = _name;
    coll_obj.uniqueV[0]=coll_obj.points[1]-coll_obj.points[0];
    coll_obj.uniqueV[1]=coll_obj.points[2]-coll_obj.points[1];

    collision_pack.static_objs.emplace_back(coll_obj);
}


void _scene_widg::add_static_collition_object
        (const obstacle_attribs<GLdouble>& attribs){

    rect_collistion_object<GLdouble> coll_obj;

    coll_obj.ID = attribs.name;
    coll_obj.points = obstacle_objects->rect_projections[attribs.type].points;

    glm::dmat4 M{attribs.transformMat()};
    for(glm::dvec3& vec:coll_obj.points)
        vec = M*glm::dvec4(vec, 1.0);

    coll_obj.uniqueV[0]=coll_obj.points[1]-coll_obj.points[0];
    coll_obj.uniqueV[1]=coll_obj.points[2]-coll_obj.points[1];

    collision_pack.static_objs.emplace_back(coll_obj);

}

/*****simply just delete the obstacle and add new one*****/
// void _scene_widg::edit_obstacle
//     (const obstacle_attribs<GLdouble>& attribs){
//     auto it = obstacle_objects->find(attribs.name);
//     it->edit(attribs);

//     ////==================================
//     collision_pack.edit_obs(attribs);
// }


void _scene_widg::mouseMoveEvent(QMouseEvent *ev){
    if(limited_frames==0)
        return;

    if(activeCam == &mainCam){
        if(ev->buttons()==Qt::RightButton){

            delta_sX = sX - ev->pos().x();
            delta_sY = sY - ev->pos().y();

            if( delta_sX > 0.0)
                mainCam.right_shifting(delta_sX*mainCam.camera_position.y*ZAY_CAM_HORIZONTAL_MOVEMENT_SCALAR);
            else if(delta_sX < 0.0)
                mainCam.left_shifting(-delta_sX*mainCam.camera_position.y*ZAY_CAM_HORIZONTAL_MOVEMENT_SCALAR);
            
            if(delta_sY > 0.0)
                mainCam.move_horizontal_backward(delta_sY*mainCam.camera_position.y*ZAY_CAM_HORIZONTAL_MOVEMENT_SCALAR);
            else if(delta_sY < 0.0)
                mainCam.move_horizontal_forward(-delta_sY*mainCam.camera_position.y*ZAY_CAM_HORIZONTAL_MOVEMENT_SCALAR);
            
            sX = static_cast<double>(ev->x());
            sY = static_cast<double>(ev->y());

        }else{

            sX = static_cast<double>(ev->x());
            sY = static_cast<double>(ev->y());

            glX =  (sX/ (static_cast<double>
                         (this->width())/2.0) ) -1;
            glY =  -((sY/ (static_cast<double>
                           (this->height())/2.0) ) -1);
            
            if(ev->buttons()==Qt::LeftButton){
                mainCam.mouse_held_update(glm::vec2(ev->x(),ev->y()), frame_rate);
            }
            else if(!fixed_cam){
                mainCam.mouse_update(glm::vec2(ev->x(),ev->y()), frame_rate);
            }
        }
    }
}

void _scene_widg::update_cam(){

    if(k_forward)
        mainCam.move_forward();
    else if(k_backward)
        mainCam.move_backward();
    else if(k_left)
        mainCam.left_shifting();
    else if(k_right)
        mainCam.right_shifting();
    else if(k_up)
        mainCam.move_up();
    else if(k_down)
        mainCam.move_down();
}

void _scene_widg::keyPressEvent(QKeyEvent* ev){

    if(limited_frames==0)
        return;

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

    if(limited_frames==0)
        return;

    if(ev->delta() > 0)
        mainCam.move_forward(ZAY_MOUSE_WHEEL_SCALAR);

    else mainCam.move_backward(ZAY_MOUSE_WHEEL_SCALAR);

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
        case Qt::Key::Key_Left: case Qt::Key::Key_A:
            k_left=0;
            break;
        case Qt::Key::Key_Right: case Qt::Key::Key_D:
            k_right=0;
            break;
        case Qt::Key::Key_Up:
            k_up=0;
            break;
        case Qt::Key::Key_Down:
            k_down=0;
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




} // namespace  zaytuna






