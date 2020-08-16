

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





#include "zay_primary_win.hpp"
#include "ui_zay_primary_win.h"
#include <QString>

namespace zaytuna {


//// for debugging
double GLOBAL_MOVEMENT_SPEED{0.0};
double GLOBAL_STEERING_WHEEL{0.00000001};

primary_win::primary_win(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::primary_win)
{
    ui->setupUi(this);
    QGLFormat _format;
    _format.setSamples(NUM_SAMPLES_PER_PIXEL);

    _scene_widget = new _scene_widg(_format, this);


    ////////////////////////////////////
    _scene_widget->setObjectName(QStringLiteral("Place_Tracker"));
    _scene_widget->setEnabled(true);
    _scene_widget->setGeometry(QRect(630, 25, WIDTH, HEIGHT));
    QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(_scene_widget->sizePolicy().hasHeightForWidth());
    _scene_widget->setSizePolicy(sizePolicy);
    _scene_widget->setMinimumSize(QSize(WIDTH, HEIGHT));
    _scene_widget->setMaximumSize(QSize(WIDTH, HEIGHT));

    ///////////////////////////////////////
    scene_objects = new QTreeWidget(ui->edit_frame);
    scene_objects->setGeometry(QRect(300, 10, 150, 250));
    scene_objects->setColumnCount(1);
    scene_objects->setObjectName(QStringLiteral("objects"));
    scene_objects->setEnabled(true);
    scene_objects->headerItem()->setText(0, "Scene Objects");

    scene_objects->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(scene_objects, &QTreeWidget::customContextMenuRequested,
            this, &primary_win::menus);

    vehicle_type = new QTreeWidgetItem(scene_objects);
    vehicle_type->setText(0, "Vehicles");
    vehicle_type->setExpanded(1);
    scene_objects->addTopLevelItem(vehicle_type);
    menus_popups["Vehicles"]=&primary_win::vehicle_type_menu;

    obstacle_type = new QTreeWidgetItem(scene_objects);
    obstacle_type->setText(0, "Obstacles");
    obstacle_type->setExpanded(1);
    scene_objects->addTopLevelItem(obstacle_type);
    menus_popups["Obstacles"]=&primary_win::obstacle_type_menu;

    /////////////////////////////////////////
    ui->lcdCamViewX->display(_scene_widget->mainCam.view_direction.x);
    ui->lcdCamViewY->display(_scene_widget->mainCam.view_direction.y);
    ui->lcdCamViewZ->display(_scene_widget->mainCam.view_direction.z);

    ui->lcdCamCoordX->display(_scene_widget->mainCam.camera_position.x);
    ui->lcdCamCoordY->display(_scene_widget->mainCam.camera_position.y);
    ui->lcdCamCoordZ->display(_scene_widget->mainCam.camera_position.z);

    ui->lcdFrameRate->display(_scene_widget->frame_rate);

    ui->speedDis->setText(QString::number(0));
    ui->steeringDis->setText(QString::number(0));

    connect(&timer, SIGNAL(timeout()), this, SLOT(update_displays()));
    timer.start(1000);

    ////-------default--objects--------
    add_vehicle({std::string(),
                 180.0,
                 {0.0, 1.0, 0.0},
                 {6.0, 0.0, -1.25}},1);

    add_obstacle(obstacle_attribs<GLdouble>
                    (Obstacle_Type::CARTON_BOX,
                     std::string(),
                     0.0,
                     {0.0, 1.0, 0.0},
                     {0.0, 0.0, 0.0}),1);
    add_obstacle(obstacle_attribs<GLdouble>
                    (Obstacle_Type::BRICK_WALL,
                     std::string(),
                     10.0,
                     {0.0, 1.0, 0.0},
                     {2.0, 0.0, -1.0}),1);
    add_obstacle(obstacle_attribs<GLdouble>
                    (Obstacle_Type::STONE_WALL,
                     std::string(),
                     15.0,
                     {0.0, 1.0, 0.0},
                     {4.0, 0.0, -3.0}),1);

}
void primary_win::new_vehicle(void)
{
    item_inputs_form inputs_d;
    inputs_d.setWindowTitle("new vehicle");
    inputs_d.setModal(1);

    if (inputs_d.exec() == QDialog::Accepted)
        add_vehicle(inputs_d.attribs,0);
}
void primary_win::add_vehicle
    (transform_attribs<GLdouble> attribs,
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
}
void primary_win::edit_vehicle(const QString& _name){
    auto veh{_scene_widget->model_vehicles->find
                (_name.toStdString())};
    item_inputs_form inputs_d(veh->current_state());
    inputs_d.setWindowTitle("edit vehicle");
    inputs_d.setModal(1);

    if (inputs_d.exec() == QDialog::Accepted)
        veh->update_positional_attributes(inputs_d.attribs);
}
void primary_win::new_obstacle(void)
{
    obstacle_inputs_form inputs_d;
    inputs_d.setWindowTitle("new obstacle");
    inputs_d.setModal(1);
    if (inputs_d.exec() == QDialog::Accepted){
        add_obstacle(inputs_d.attribs,0);
    }
}

void primary_win::edit_obstacle(const QString& _name){
    obstacle_attribs<GLdouble> attribs
            = _scene_widget->get_obstacle(_name.toStdString());
    obstacle_inputs_form inputs_d(attribs);
    inputs_d.setWindowTitle("edit obstacle");
    inputs_d.setModal(1);
    if (inputs_d.exec() == QDialog::Accepted){
        if(inputs_d.attribs.type == attribs.type)
            _scene_widget->edit_obstacle(inputs_d.attribs);
        else{
            _scene_widget->delete_obstacle(_name.toStdString());
            _scene_widget->add_obstacle(inputs_d.attribs);
        }
    }
}
void primary_win::add_obstacle
    (obstacle_attribs<GLdouble> attribs,
     bool is_default)
{
    QString obs_name{QString("obstacle_%1").arg(obstacle_counter++)};
    attribs.name = obs_name.toStdString();
    if(is_default)
        _scene_widget->default_objects.obstacles.emplace_back(attribs);
    else
        _scene_widget->add_obstacle(attribs);
    QTreeWidgetItem* obs = new QTreeWidgetItem();
    obs->setText(0, obs_name);
    obstacle_type->addChild(obs);
    menus_popups[obs_name]=&primary_win::obstacle_menu;
    obstacles[obs_name] = obs;
}

void primary_win::front_cam_to_screen(const QString& _name){
    _scene_widget->update_current_vehicle(_name.toStdString());
    ui->radioButton_Local->setChecked(1);
    this->on_radioButton_Local_clicked();

}
void primary_win::delete_vehicle(const QString& _name){
    if(QMessageBox::question(this,
                "delete vehicle",
                "delete vehicle '"+_name+"' ?",
                QMessageBox::Ok|QMessageBox::Cancel) == QMessageBox::Ok){
        if(_scene_widget->current_model != nullptr)
            if(_scene_widget->current_model->attribs.name == _name.toStdString()){
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
        vehicles.erase(_name);
    }
}
void primary_win::delete_obstacle(const QString& _name){
    if(QMessageBox::question(this,
            "delete obstacle",
            "delete obstacle '"+_name+"' ?",
            QMessageBox::Ok|QMessageBox::Cancel) == QMessageBox::Ok){
        _scene_widget->delete_obstacle(_name.toStdString());
        obstacle_type->removeChild(obstacles[_name]);
        obstacles.erase(_name);
    }
}

void primary_win::vehicle_type_menu(const QPoint& pos){
    QAction *addItem_act = new QAction(QIcon(), tr("&New Vehicle"), this);
    addItem_act->setStatusTip(tr("add new vehicle"));
    connect(addItem_act, SIGNAL(triggered()), this, SLOT(new_vehicle()));
    QMenu menu(this);
    menu.addAction(addItem_act);
    menu.exec( scene_objects->mapToGlobal(pos) );
}
void primary_win::vehicle_menu(const QPoint& pos)
{
    QAction *moveToScreen_act = new QAction(QIcon(), tr("&front cam"), this);
    moveToScreen_act->setStatusTip(tr("move front cam to screen"));
    connect(moveToScreen_act, &QAction::triggered, this,
            [=]{front_cam_to_screen(scene_objects->itemAt(pos)->text(0));} );

    QAction *editItem_act = new QAction(QIcon(), tr("&edit"), this);
    editItem_act->setStatusTip(tr("edit selected vehicle"));
    connect(editItem_act, &QAction::triggered, this,
            [=]{edit_vehicle(scene_objects->itemAt(pos)->text(0));} );

    QAction *delItem_act = new QAction(QIcon(), tr("&delete"), this);
    delItem_act->setStatusTip(tr("delete selected vehicle"));
    connect(delItem_act, &QAction::triggered, this,
            [=]{delete_vehicle(scene_objects->itemAt(pos)->text(0));} );

    QMenu menu(this);
    menu.addAction(moveToScreen_act);
    menu.addAction(editItem_act);
    menu.addAction(delItem_act);

    menu.exec( scene_objects->mapToGlobal(pos) );
}
void primary_win::obstacle_type_menu(const QPoint& pos)
{
    QAction *newAct = new QAction(QIcon(), tr("&New Obstacle"), this);
    newAct->setStatusTip(tr("add new obstacle"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(new_obstacle()));

    QMenu menu(this);
    menu.addAction(newAct);
    menu.exec( scene_objects->mapToGlobal(pos) );
}
void primary_win::obstacle_menu(const QPoint& pos)
{
    QAction *editItem_act = new QAction(QIcon(), tr("&edit"), this);
    editItem_act->setStatusTip(tr("edit selected obstacle"));
    connect(editItem_act, &QAction::triggered, this,
            [=]{edit_obstacle(scene_objects->itemAt(pos)->text(0));} );

    QAction *delItem_act = new QAction(QIcon(), tr("&delete"), this);
    delItem_act->setStatusTip(tr("delete selected obstacle"));
    connect(delItem_act, &QAction::triggered, this,
            [=]{delete_obstacle(scene_objects->itemAt(pos)->text(0));} );

    QMenu menu(this);
    menu.addAction(editItem_act);
    menu.addAction(delItem_act);
    menu.exec( scene_objects->mapToGlobal(pos) );
}

void primary_win::menus(const QPoint& pos){
    QTreeWidgetItem *selected = scene_objects->itemAt(pos);
    (this->*menus_popups[selected->text(0)])(pos);
}

primary_win::~primary_win()
{
    delete _scene_widget;
    delete scene_objects;
    timer.deleteLater();
    delete ui;
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

    ui->cam_movement_speed->setValue(_scene_widget->mainCam.MOVEMENT_SPEED);
    ui->cam_rotation_speed->setValue(_scene_widget->mainCam.ROTATION_SPEED);

    ui->SpinBox_FieldOfView->setValue(_scene_widget->activeCam->FIELD_OF_VIEW);
    ui->SpinBox_Near->setValue(_scene_widget->activeCam->NEAR_PLANE);
    ui->SpinBox_Far->setValue(_scene_widget->activeCam->FAR_PLANE);
    ui->front_cam_freq_SpinBox->setValue(_scene_widget->front_cam_freq);


}

void primary_win::on_grid_check_clicked(bool checked){
    _scene_widget->grid_checked = checked;
    _scene_widget->repaint();
}

void primary_win::on_coord_check_clicked(bool checked){
    _scene_widget->coord_checked = checked;
    _scene_widget->repaint();
}

void primary_win::on_cam_movement_speed_valueChanged(double arg){
    _scene_widget->mainCam.MOVEMENT_SPEED = arg;
}

void primary_win::on_cam_rotation_speed_valueChanged(double arg){
    _scene_widget->mainCam.ROTATION_SPEED = arg;
}

void primary_win::on_SpinBox_Near_valueChanged(double arg){
    _scene_widget->activeCam->NEAR_PLANE = arg;
    _scene_widget->updateProjection();
}

void primary_win::on_SpinBox_Far_valueChanged(double arg){
    _scene_widget->activeCam->FAR_PLANE = arg;
    _scene_widget->updateProjection();
}

void primary_win::on_SpinBox_FieldOfView_valueChanged(double arg){
    _scene_widget->activeCam->FIELD_OF_VIEW = arg;
    _scene_widget->updateProjection();
}

void primary_win::on_radioButton_Global_clicked(){
    _scene_widget->activeCam = &_scene_widget->mainCam;
    ui->camTransformation->setEnabled(1);
    _scene_widget->updateProjection();
}

void primary_win::on_radioButton_Local_clicked()
{
    if(_scene_widget->current_model == nullptr)
        return;
    _scene_widget->activeCam = &_scene_widget->current_model->frontCam;
    ui->camTransformation->setEnabled(0);
    _scene_widget->updateProjection();
}

void primary_win::on_steeringV_valueChanged(int value)
{
        ui->steeringDis->setText(QString::number(value));

       if(value == 0){
           GLOBAL_STEERING_WHEEL = STEERING_MARGIN_OF_ERROR;
       }else{
           GLOBAL_STEERING_WHEEL = (static_cast<double>(value)*M_PI) /180.0;
       }
}

void primary_win::on_speedV_valueChanged(int value)
{
    ui->speedDis->setText(QString::number(value));
   GLOBAL_MOVEMENT_SPEED = static_cast<double>(-value);
}

void primary_win::on_auto_perspective_radio_clicked()
{
    ui->frame_perspective->setDisabled(1);
    _scene_widget->activeCam->auto_perspective = 1;
}

void primary_win::on_custom_perspective_radio_clicked()
{
    ui->frame_perspective->setEnabled(1);
    _scene_widget->activeCam->auto_perspective = 0;
}

void primary_win::on_front_cam_freq_SpinBox_valueChanged(double arg)
{
    _scene_widget->front_cam_freq = arg;
    _scene_widget->imgs_sec = 1.0/arg;
}



} // namespace  zaytuna




