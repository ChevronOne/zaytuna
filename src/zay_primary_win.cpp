

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





#include "zay_primary_win.hpp"
#include "ui_zay_primary_win.h"


namespace zaytuna {



primary_win::primary_win(const QString& title_, const QIcon& icon_,
                         const std::string& p_path, QWidget *parent) :
    QMainWindow(parent), ui{new Ui::primary_win}, 
    ZAY_PACKAGE_PATH{p_path}, zay_icon{icon_}
{
    
    ZAY_PACKAGE_PATH = ros::package::getPath(ZAY_PACKAGE_NAME);

    ui->setupUi(this);
    this->setWindowTitle(title_);
    this->setWindowIcon(zay_icon);


    if(std::is_same<ZAY_QGL_WIDGET_VERSION, ZAY_QGL_W>::value){
        QGLFormat format_;
        format_.setSamples(ZAY_NUM_SAMPLES_PER_PIXEL);

        _scene_widget = new _scene_widg(ui->statusBar, format_, this);
        
    }else{

        _scene_widget = new _scene_widg(ui->statusBar, this);
        
    }

    
    _scene_widget->setObjectName(QStringLiteral("_scene_widget"));
    _scene_widget->setEnabled(true);
    _scene_widget->setGeometry(QRect(630, 25, ZAY_SCENE_WIDTH, ZAY_SCENE_HEIGHT));

    QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(_scene_widget->sizePolicy().hasHeightForWidth());
    _scene_widget->setSizePolicy(sizePolicy);

    _scene_widget->setMinimumSize(QSize(ZAY_SCENE_WIDTH, ZAY_SCENE_HEIGHT));
    _scene_widget->setMaximumSize(QSize(ZAY_SCENE_WIDTH, ZAY_SCENE_HEIGHT));

    

    obstacle_counter[Obstacle_Type::CARTON_BOX] = {"carton_box_%1",1};
    obstacle_counter[Obstacle_Type::BRICK_WALL] = {"brick_wall_%1",1};
    obstacle_counter[Obstacle_Type::STONE_WALL] = {"stone_wall_%1",1};

    scene_objects = new QTreeWidget(ui->edit_frame);
    scene_objects->setGeometry(QRect(305, 10, 290, 255));
    scene_objects->setColumnCount(1);
    scene_objects->setObjectName(QStringLiteral("objects"));
    scene_objects->setEnabled(true);
    scene_objects->headerItem()->setText(0, "Scene Objects");

    scene_objects->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(scene_objects, &QTreeWidget::customContextMenuRequested,
            this, &primary_win::menus);

    vehicle_type = new QTreeWidgetItem(scene_objects);
    vehicle_type->setText(0, "Vehicles");
    

    scene_objects->addTopLevelItem(vehicle_type);
    menus_popups["Vehicles"]=&primary_win::vehicle_type_menu;

    obstacle_type = new QTreeWidgetItem(scene_objects);
    obstacle_type->setText(0, "Obstacles");
    obstacle_type->setExpanded(1);
    scene_objects->addTopLevelItem(obstacle_type);
    menus_popups["Obstacles"]=&primary_win::obstacle_type_menu;

    
    ui->max_speed_SpinBox->setValue(_scene_widget->local_control_speed/ZAY_SPEED_SCALAR);
    ui->max_steering_SpinBox->setValue(glm::degrees(_scene_widget->local_control_steering)/ZAY_MAX_TURN_ANGLE);

    ui->lcdCamViewX->display(_scene_widget->mainCam.view_direction.x);
    ui->lcdCamViewY->display(_scene_widget->mainCam.view_direction.y);
    ui->lcdCamViewZ->display(_scene_widget->mainCam.view_direction.z);

    ui->lcdCamCoordX->display(_scene_widget->mainCam.camera_position.x);
    ui->lcdCamCoordY->display(_scene_widget->mainCam.camera_position.y);
    ui->lcdCamCoordZ->display(_scene_widget->mainCam.camera_position.z);

    ui->lcdFrameRate->display(_scene_widget->frame_rate);
    ui->front_cam_freq_SpinBox->setValue(ZAY_DEFAULT_FRONT_CAM_FREQUENCY);

    ui->limited_frames_spinBox->setValue
            (static_cast<int>(_scene_widget->limited_frames));


    ui->coord_ref_lab->setPixmap(QPixmap
        ((ZAY_PACKAGE_PATH+"/resources/coord_ref").c_str()).scaled
            (ui->coord_ref_lab->width(), 
            ui->coord_ref_lab->height(), 
            Qt::KeepAspectRatio));

    connect(&timer, SIGNAL(timeout()), this, SLOT(update_displays()));
    timer.start(1000);




    /*********default objects**********/
    add_vehicle({ZAY_DEFAULT_FRONT_CAM_V_ANGLE_MIN,
                 std::string(),
                 180.0,
                 {0.0, 1.0, 0.0},
                 {6.0, 0.0, -1.25}},1);


    add_obstacle(obstacle_attribs<GLdouble>
                    (Obstacle_Type::CARTON_BOX,
                     std::string(),
                     -70.0,
                     {0.0, 1.0, 0.0},
                     {2.0, 0.0, -1.0}),1);
    
    add_obstacle(obstacle_attribs<GLdouble>
                    (Obstacle_Type::BRICK_WALL,
                     std::string(),
                     25.0,
                     {0.0, 1.0, 0.0},
                     {8.8, 0.0, -3.0}),1);
    
    add_obstacle(obstacle_attribs<GLdouble>
                    (Obstacle_Type::STONE_WALL,
                     std::string(),
                     15.0,
                     {0.0, 1.0, 0.0},
                     {4.0, 0.0, -3.0}),1);

    // /////////////////////////////////////////
    // for(int i{-150}; i < 150; ++i){
    //     add_obstacle(obstacle_attribs<GLdouble>
    //                 (Obstacle_Type::STONE_WALL,
    //                  std::string(),
    //                  15.0,
    //                  {0.0, 1.0, 0.0},
    //                  {0.0, 0.0, i}), 1);

    //     // add_obstacle(obstacle_attribs<GLdouble>
    //     //             (Obstacle_Type::STONE_WALL,
    //     //              std::string(),
    //     //              15.0,
    //     //              {0.0, 1.0, 0.0},
    //     //              {i, 0.0, 0.0}), 1);
        
    //     // add_obstacle(obstacle_attribs<GLdouble>
    //     //             (Obstacle_Type::STONE_WALL,
    //     //              std::string(),
    //     //              15.0,
    //     //              {0.0, 1.0, 0.0},
    //     //              {i, 0.0, i}), 1);
        
    //     add_obstacle(obstacle_attribs<GLdouble>
    //                 (Obstacle_Type::STONE_WALL,
    //                  std::string(),
    //                  15.0,
    //                  {0.0, 1.0, 0.0},
    //                  {i, 0.0, -i}), 1);
    // }

    // for(int i{2}; i<20; i+=2){
    //     add_vehicle({ZAY_DEFAULT_FRONT_CAM_V_ANGLE_MIN,
    //              std::string(),
    //              180.0,
    //              {0.0, 1.0, 0.0},
    //              {25, 0.0, -i}}, 1);
    // }
    

    setToolTipLabels();

}


void primary_win::new_vehicle(void)
{
    if(vehicles.size()>=ZAY_LIMITED_VEH_NUM){
        QMessageBox::information(this, "Limited num of vehicles",
                                 "Only up to " + QString::number(ZAY_LIMITED_VEH_NUM) + " model vehicles can be added!");
        return;
    }

    item_inputs_form inputs_d(ZAY_PACKAGE_PATH);

    inputs_d.setWindowTitle("new vehicle");
    inputs_d.setWindowIcon(zay_icon);
    inputs_d.setModal(1);
        
    if (inputs_d.exec() == QDialog::Accepted)
        add_vehicle(inputs_d.attribs,0);

}


void primary_win::add_vehicle
    (veh_transform_attribs<GLdouble> attribs,
     bool is_default)
{

    QString veh_name{QString("vehicle_%1").arg(vehicle_counter++)};
    attribs.name = veh_name.toStdString();

    if(is_default)
        _scene_widget->default_objects.vehicles.emplace_back(attribs);
    else
        _scene_widget->add_vehicle(attribs);
    
    QTreeWidgetItem* veh = new QTreeWidgetItem();
    veh->setText(0, veh_name);
    vehicle_type->addChild(veh);

    menus_popups[veh_name]=&primary_win::vehicle_menu;
    vehicles[veh_name] = veh;

    scene_objects->resizeColumnToContents(0);

}


void primary_win::edit_vehicle(const QString& _name){

    auto veh{_scene_widget->model_vehicles->find
                (_name.toStdString())};

    item_inputs_form inputs_d(veh->current_state(), ZAY_PACKAGE_PATH);
    inputs_d.setWindowTitle("edit vehicle");
    inputs_d.setWindowIcon(zay_icon);
    inputs_d.setModal(1);

    if (inputs_d.exec() == QDialog::Accepted){
        veh->update_positional_attributes(inputs_d.attribs);
        scene_objects->resizeColumnToContents(0);
    }
}


void primary_win::delete_vehicle(const QString& _name){

    if(QMessageBox::question(this,
                "delete vehicle",
                "delete vehicle '"+_name+"' ?",
                QMessageBox::Ok|QMessageBox::Cancel) == QMessageBox::Ok){

        if( (_scene_widget->current_model != nullptr) 
            & (_scene_widget->current_model->attribs.name == _name.toStdString())) {

            _scene_widget->current_model
                    = _scene_widget->getOtherVeh
                    (_name.toStdString());

            if(ui->radioButton_Local->isChecked()){

                ui->radioButton_Global->setChecked(1);
                this->on_radioButton_Global_clicked(); 
            }
        }

        _scene_widget->delete_vehicle(_name.toStdString());
        vehicle_type->removeChild(vehicles[_name]);

        if(vehicles[_name] != nullptr)
            delete vehicles[_name];

        vehicles.erase(_name);
        scene_objects->resizeColumnToContents(0);
    }
}


void primary_win::new_obstacle(void)
{
    if(obstacles.size()>=ZAY_LIMITED_OBS_NUM){

        QMessageBox::information(this, "Limited num of obstacles",
                                 "Only up to " + QString::number(ZAY_LIMITED_OBS_NUM) + " obstacles can be added!");
        return;
    }

    obstacle_inputs_form inputs_d;
    
    inputs_d.setWindowTitle("new obstacle");
    inputs_d.setWindowIcon(zay_icon);
    inputs_d.setModal(1);

    if (inputs_d.exec() == QDialog::Accepted)
        add_obstacle(inputs_d.attribs,0);

}


void primary_win::edit_obstacle(const QString& _name){

    obstacle_attribs<GLdouble> attribs
            = _scene_widget->get_obstacle(_name.toStdString());
    
    obstacle_inputs_form inputs_d(attribs);
    inputs_d.setWindowTitle("edit obstacle");
    inputs_d.setWindowIcon(zay_icon);
    inputs_d.setModal(1);

    if (inputs_d.exec() == QDialog::Accepted){
        delete_obstacle(_name);
        add_obstacle(inputs_d.attribs, 0);
    }

}


void primary_win::add_obstacle
    (obstacle_attribs<GLdouble> attribs,
    bool is_default)
{

    QString obs_name{QString(obstacle_counter[attribs.type].category).arg
        (obstacle_counter[attribs.type].counter++)
        +"_X"+QString::number(attribs.translation_vec.x, 'f', 2) 
        +"_Z"+QString::number(attribs.translation_vec.z, 'f', 2)};


    attribs.name = obs_name.toStdString();

    QTreeWidgetItem* obs = new QTreeWidgetItem();
    obs->setText(0, obs_name);
    obstacle_type->addChild(obs);

    menus_popups[obs_name]=&primary_win::obstacle_menu;
    obstacles[obs_name] = obs;

    if(is_default){

        _scene_widget->default_objects.obstacles.emplace_back(attribs);

    }else{

        _scene_widget->add_obstacle(attribs);
        obs->setSelected(1);
    }

    scene_objects->resizeColumnToContents(0);
}


void primary_win::delete_obstacle
                (const QString& _name){

    _scene_widget->delete_obstacle(_name.toStdString());

    obstacle_type->removeChild(obstacles[_name]);

    if(obstacles[_name] != nullptr)
        delete obstacles[_name];

    obstacles.erase(_name);
    menus_popups.erase(_name);

    scene_objects->resizeColumnToContents(0);
}


void primary_win::delete_obstacle_confirm
                (const QString& _name){

    if(QMessageBox::question(this,
            "delete obstacle",
            "delete obstacle '"+_name+"' ?",
            QMessageBox::Ok|QMessageBox::Cancel) == QMessageBox::Ok){
        delete_obstacle(_name);
    }
}


void primary_win::front_cam_to_screen
                (const QString& _name){

    _scene_widget->update_current_vehicle(_name.toStdString());
    ui->radioButton_Local->setChecked(1);
    this->on_radioButton_Local_clicked();

}


void primary_win::attach_to_control_panel
                   (const QString& _name){

    auto it = _scene_widget->model_vehicles->find(_name.toStdString());
    it->is_detached = 0;

}


void primary_win::detach_from_control_panel
                    (const QString& _name){
                        
    auto it = _scene_widget->model_vehicles->find(_name.toStdString());
    it->is_detached = 1;

}


void primary_win::vehicle_type_menu(const QPoint& pos){

    ui->statusBar->setStyleSheet("color:black");

    QAction *addItem_act = new QAction(QIcon(), tr("&New Vehicle"), this);
    addItem_act->setStatusTip(tr("add new vehicle"));

    connect(addItem_act, SIGNAL(triggered()), this, SLOT(new_vehicle()));

    QMenu menu(this);
    menu.addAction(addItem_act);
    menu.exec( scene_objects->mapToGlobal(pos) );
}


void primary_win::vehicle_menu(const QPoint& pos)
{

    ui->statusBar->setStyleSheet("color:black");
    QAction *moveToScreen_act = new QAction(QIcon(), tr("&Front cam"), this);
    moveToScreen_act->setStatusTip(tr("move front cam to screen"));
    connect(moveToScreen_act, &QAction::triggered, this,
            [=]{front_cam_to_screen(scene_objects->itemAt(pos)->text(0));} );


    QAction *attach_to_panel = new QAction(QIcon(), tr("&Connect to control panel"), this);
    attach_to_panel->setStatusTip(tr("connect selected vehicle to control panel"));
    connect(attach_to_panel, &QAction::triggered, this,
            [=]{attach_to_control_panel(scene_objects->itemAt(pos)->text(0));} );


    QAction *detach_from_panel = new QAction(QIcon(), tr("&Detach from control panel"), this);
    detach_from_panel->setStatusTip(tr("detach selected vehicle from control panel"));
    connect(detach_from_panel, &QAction::triggered, this,
            [=]{detach_from_control_panel(scene_objects->itemAt(pos)->text(0));} );


    QAction *editItem_act = new QAction(QIcon(), tr("&Edit"), this);
    editItem_act->setStatusTip(tr("edit selected vehicle"));
    connect(editItem_act, &QAction::triggered, this,
            [=]{edit_vehicle(scene_objects->itemAt(pos)->text(0));} );


    QAction *delItem_act = new QAction(QIcon(), tr("&Delete"), this);
    delItem_act->setStatusTip(tr("delete selected vehicle"));
    connect(delItem_act, &QAction::triggered, this,
            [=]{delete_vehicle(scene_objects->itemAt(pos)->text(0));} );


    QMenu menu(this);
    menu.addAction(moveToScreen_act);
    menu.addAction(attach_to_panel);
    menu.addAction(detach_from_panel);
    menu.addAction(editItem_act);
    menu.addAction(delItem_act);

    menu.exec( scene_objects->mapToGlobal(pos) );


}


void primary_win::obstacle_type_menu(const QPoint& pos)
{

    ui->statusBar->setStyleSheet("color:black");
    QAction *newAct = new QAction(QIcon(), tr("&New Obstacle"), this);
    newAct->setStatusTip(tr("add new obstacle"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(new_obstacle()));

    QMenu menu(this);
    menu.addAction(newAct);
    menu.exec( scene_objects->mapToGlobal(pos) );


}


void primary_win::obstacle_menu(const QPoint& pos)
{

    ui->statusBar->setStyleSheet("color:black");
    QAction *editItem_act = new QAction(QIcon(), tr("&Edit"), this);
    editItem_act->setStatusTip(tr("edit selected obstacle"));
    connect(editItem_act, &QAction::triggered, this,
            [=]{edit_obstacle(scene_objects->itemAt(pos)->text(0));} );

    QAction *delItem_act = new QAction(QIcon(), tr("&Delete"), this);
    delItem_act->setStatusTip(tr("delete selected obstacle"));
    connect(delItem_act, &QAction::triggered, this,
            [=]{delete_obstacle_confirm(scene_objects->itemAt(pos)->text(0));} );

    QMenu menu(this);
    menu.addAction(editItem_act);
    menu.addAction(delItem_act);
    menu.exec( scene_objects->mapToGlobal(pos) );
}


void primary_win::menus(const QPoint& pos){

    QTreeWidgetItem *selected = scene_objects->itemAt(pos);
    (this->*menus_popups[selected->text(0)])(pos);

}


void primary_win::closeEvent(QCloseEvent *event){

     std::cout << "exited normally\n" << "close ptr: " << event << "\n";
}


void primary_win::update_displays()
{

    ui->lcdCamViewX->display(_scene_widget->activeCam->view_direction.x);
    ui->lcdCamViewY->display(_scene_widget->activeCam->view_direction.y);
    ui->lcdCamViewZ->display(_scene_widget->activeCam->view_direction.z);

    ui->lcdCamCoordX->display(_scene_widget->activeCam->camera_position.x);
    ui->lcdCamCoordY->display(_scene_widget->activeCam->camera_position.y);
    ui->lcdCamCoordZ->display(_scene_widget->activeCam->camera_position.z);

    ui->lcdFrameRate->display(_scene_widget->frame_rate);

    ui->cam_movement_speed->setValue(_scene_widget->mainCam.get_movement_scalar());
    ui->cam_rotation_speed->setValue(_scene_widget->mainCam.get_rotation_scalar());

    ui->SpinBox_FieldOfView->setValue(_scene_widget->activeCam->FIELD_OF_VIEW);
    ui->SpinBox_Near->setValue(_scene_widget->activeCam->NEAR_PLANE);
    ui->SpinBox_Far->setValue(_scene_widget->activeCam->FAR_PLANE);
    
    
}


void primary_win::on_grid_check_clicked
        (bool checked){

    _scene_widget->grid_checked = checked;
    _scene_widget->repaint();

}


void primary_win::on_coord_check_clicked
        (bool checked){

    _scene_widget->coord_checked = checked;
    _scene_widget->repaint();

}


void primary_win::on_fixed_cam_check_clicked
(bool checked){

    _scene_widget->fixed_cam = checked;

}


void primary_win::on_cam_movement_speed_valueChanged
        (double arg){

    _scene_widget->mainCam.set_movement_scalar(arg);

}


void primary_win::on_cam_rotation_speed_valueChanged
        (double arg){

    _scene_widget->mainCam.set_rotation_scalar(arg);

}


void primary_win::on_SpinBox_Near_valueChanged
        (double arg){

    _scene_widget->activeCam->NEAR_PLANE = arg;
    _scene_widget->updateProjection();

}


void primary_win::on_SpinBox_Far_valueChanged
        (double arg){

    _scene_widget->activeCam->FAR_PLANE = arg;
    _scene_widget->updateProjection();

}



void primary_win::on_SpinBox_FieldOfView_valueChanged
        (double arg){

    _scene_widget->activeCam->FIELD_OF_VIEW = arg;
    _scene_widget->updateProjection();

}


void primary_win::on_radioButton_Global_clicked(){

    _scene_widget->activeCam = &(_scene_widget->mainCam);
    ui->camTransformation->setEnabled(1);
    _scene_widget->updateProjection();

}


void primary_win::on_radioButton_Local_clicked()
{

    if(_scene_widget->current_model == nullptr)
        return;
    _scene_widget->activeCam = &(_scene_widget->current_model->frontCam);
    ui->camTransformation->setEnabled(0);
    _scene_widget->updateProjection();

}


void primary_win::on_steeringV_valueChanged(int value){

    ui->steeringDis->setText(QString::number
          (static_cast<double>(value)/ZAY_MAX_TURN_ANGLE, 'f', 3));
    if(value == 0)
        _scene_widget->SLIDER_STEERING_WHEEL = ZAY_STEERING_MARGIN_OF_ERROR;
    else
        _scene_widget->SLIDER_STEERING_WHEEL = static_cast<double>(value)*ZAY_RADIAN;
    
}


void primary_win::on_speedV_valueChanged(int value){

    ui->speedDis->setText(QString::number
         (static_cast<double>(value)/ZAY_SPEED_SCALAR, 'f', 3));
    _scene_widget->SLIDER_MOVEMENT_SPEED = static_cast<double>(-value);

}


void primary_win::on_auto_perspective_radio_clicked(){

    _scene_widget->activeCam->FAR_PLANE = ZAY_FIELD_DIAMETER;
    ui->frame_perspective->setDisabled(1);
    _scene_widget->activeCam->auto_perspective = 1;

}


void primary_win::on_custom_perspective_radio_clicked()
{
    ui->frame_perspective->setEnabled(1);
    _scene_widget->activeCam->auto_perspective = 0;
}


void primary_win::on_front_cam_freq_SpinBox_valueChanged
        (double arg){

    _scene_widget->update_fc_time_interval(arg);
}


void primary_win::on_key_control_radio_clicked(){

    ui->steering_groupBox->setEnabled(0);
    ui->speed_groupBox->setEnabled(0);
    ui->max_speed_SpinBox->setEnabled(1);
    ui->max_steering_SpinBox->setEnabled(1);

    _scene_widget->key_control=1;

}


void primary_win::on_slider_control_radio_clicked(){

    ui->steering_groupBox->setEnabled(1);
    ui->speed_groupBox->setEnabled(1);
    ui->max_speed_SpinBox->setEnabled(0);
    ui->max_steering_SpinBox->setEnabled(0);

    _scene_widget->key_control=0;

}


void primary_win::on_max_steering_SpinBox_valueChanged
        (double val){

    _scene_widget->local_control_steering = val==0.0? ZAY_STEERING_MARGIN_OF_ERROR :
            (glm::radians(val*ZAY_MAX_TURN_ANGLE));

}


void primary_win::on_max_speed_SpinBox_valueChanged
        (double val){

    _scene_widget->local_control_speed = ZAY_SPEED_SCALAR*val;
}


void zaytuna::primary_win::on_limited_frames_spinBox_valueChanged
        (int val){

    _scene_widget->update_time_interval(static_cast<uint32_t>(val));
}


primary_win::~primary_win()
{
    delete _scene_widget;
    delete scene_objects;
    timer.deleteLater();
    delete ui;
}


void primary_win::setToolTipLabels(void){

    ui->limited_frames_spinBox->setToolTip
            ("set a limitation for the frame rate per second");

    ui->radioButton_Global->setToolTip
            ("switch to main camera");

    ui->radioButton_Local->setToolTip
            ("switch to a local cam of one available vehicle");

    ui->front_cam_freq_SpinBox->setToolTip
            ("set a frequency of publishing images from vehicle's front cam");

    ui->grid_check->setToolTip
            ("show the grid");

    ui->coord_check->setToolTip
            ("show coordinate axes");

    ui->cam_movement_speed->setToolTip
            ("movement speed when navigating in the scene");

    ui->cam_rotation_speed->setToolTip
            ("rotation speed when hovering with mouse");

    ui->SpinBox_FieldOfView->setToolTip
            ("angle of view 'AOV'");

    ui->auto_perspective_radio->setToolTip
            ("set values of far- and near plane automatically");

    ui->custom_perspective_radio->setToolTip
            ("set custom values of near- and far-plane");

    ui->SpinBox_Near->setToolTip
            ("set a value for near-plane");

    ui->SpinBox_Far->setToolTip
            ("set a value for far-plane");

    ui->slider_control_radio->setToolTip
            ("control a connected vehicle with sliders");

    ui->key_control_radio->setToolTip
            ("control a connected vehicle with T,G,F,H keys");

    ui->steeringV->setToolTip
            ("set value of steering when controlling by slider");

    ui->speedV->setToolTip
            ("set value of speed when controlling by slider");

    ui->max_steering_SpinBox->setToolTip
            ("set a max value for steering when controlling by keys");

    ui->max_speed_SpinBox->setToolTip
            ("set a max value for speed when controlling by keys");

}



} // namespace  zaytuna


