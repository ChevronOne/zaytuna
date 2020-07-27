

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

//#include "zay_headers.hpp"


namespace zaytuna {


//// for debugging
double GLOBAL_MOVEMENT_SPEED{0.0};
double GLOBAL_STEERING_WHEEL{0.00000001};

uint32_t vehicle_counter{1};
uint32_t obstacle_counter{1};
//QString vehicle_name{"vehicle"};
//QString obstacle_name{"obstacle"};

primary_win::primary_win(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::primary_win)
{
    ui->setupUi(this);
    QGLFormat _format;
    _format.setSamples(8);

    _scene_widget = new _scene_widg(_format, this);


    //-------------------------------------------------
//    Place_Tracker = new QWidget(centralWidget);
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





    // -------------------------------------------------
    scene_objects = new QTreeWidget(ui->edit_frame);
    scene_objects->setGeometry(QRect(300, 10, 150, 250));
    scene_objects->setColumnCount(1);
    scene_objects->setObjectName(QStringLiteral("objects"));
    scene_objects->setEnabled(true);
    scene_objects->headerItem()->setText(0, "Scene Objects");

    scene_objects->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(scene_objects, &QTreeWidget::customContextMenuRequested, this, &primary_win::menus);


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


    new_vehicle();
    new_obstacle();
    new_obstacle();





    //--------------------------------------------------
    ui->lcdCamViewX->display(_scene_widget->mainCam.view_direction.x);
    ui->lcdCamViewY->display(_scene_widget->mainCam.view_direction.y);
    ui->lcdCamViewZ->display(_scene_widget->mainCam.view_direction.z);

    ui->lcdCamCoordX->display(_scene_widget->mainCam.camera_position.x);
    ui->lcdCamCoordY->display(_scene_widget->mainCam.camera_position.y);
    ui->lcdCamCoordZ->display(_scene_widget->mainCam.camera_position.z);

    ui->lcdFrameRate->display(_scene_widget->frame_rate);

    ui->speedDis->setText(QString::number(0));
    ui->steeringDis->setText(QString::number(0));

    connect(&timer, SIGNAL(timeout()), this, SLOT(update_displys()));
    timer.start(1000); // 100


//    this->setMouseTracking(true);
//    this->setFocusPolicy(Qt::StrongFocus);

////    setMouseTracking(true);


//    centralWidget()->setAttribute(Qt::WA_TransparentForMouseEvents);
//    setMouseTracking(true);

//    qApp->installEventFilter(this);

//    ui->lcdCamCoordX->display(ui->SQwidget->mainCam.camera_position.x);
//    ui->lcdCamCoordY->display(ui->SQwidget->mainCam.camera_position.y);
//    ui->lcdCamCoordZ->display(ui->SQwidget->mainCam.camera_position.z);

//    setFocusPolicy()
//    this->setFocusPolicy(Qt::TabFocus);


}
void primary_win::new_vehicle(void)
{
    QString veh_name{QString("vehicle_%1").arg(vehicle_counter++)};
    QTreeWidgetItem* veh = new QTreeWidgetItem();
    veh->setText(0, veh_name);
    vehicle_type->addChild(veh);
    menus_popups[veh_name]=&primary_win::vehicle_menu;
    vehicles[veh_name] = veh;


    qDebug() << "added vehicle: " << veh_name << " \n";
}
void primary_win::new_obstacle(void)
{
    QString obs_name{QString("obstacle_%1").arg(obstacle_counter++)};
    QTreeWidgetItem* obs = new QTreeWidgetItem();
    obs->setText(0, obs_name);
    obstacle_type->addChild(obs);
    menus_popups[obs_name]=&primary_win::obstacle_menu;
    obstacles[obs_name] = obs;


    qDebug() << "added obstacle: " << obs_name << " \n";
}
void primary_win::delete_vehicle(const QString& _name)
{
    qDebug() << "delete vehicle: "<< _name <<" \n";
}
void primary_win::delete_obstacle(const QString& _name)
{
    qDebug() << "delete obstacle: "<< _name<< " \n";
}
void primary_win::edit_vehicle(const QString& _name)
{
    qDebug() << "edit vehicle: "<< _name <<" \n";
}
void primary_win::edit_obstacle(const QString& _name)
{
    qDebug() << "edit obstacle: "<< _name<< " \n";
}

void primary_win::vehicle_type_menu(const QPoint& pos)
{

    qDebug() << pos << ", vehicle type call \n";
    QAction *addItem_act = new QAction(QIcon(), tr("&New Vehicle"), this);
    addItem_act->setStatusTip(tr("add new vehicle"));
    connect(addItem_act, SIGNAL(triggered()), this, SLOT(new_vehicle()));


    QMenu menu(this);
    menu.addAction(addItem_act);

//    QPoint pt(pos);
    menu.exec( scene_objects->mapToGlobal(pos) );
}
void primary_win::vehicle_menu(const QPoint& pos)
{
    qDebug() << pos << "a vehicle call \n";
    QAction *editItem_act = new QAction(QIcon(), tr("&edit"), this);
    editItem_act->setStatusTip(tr("edit selected vehicle"));
    connect(editItem_act, &QAction::triggered, this,
            [=]{edit_vehicle(scene_objects->itemAt(pos)->text(0));} );

    QAction *delItem_act = new QAction(QIcon(), tr("&delete"), this);
    delItem_act->setStatusTip(tr("delete selected vehicle"));
    connect(delItem_act, &QAction::triggered, this,
            [=]{delete_vehicle(scene_objects->itemAt(pos)->text(0));} );


    QMenu menu(this);
    menu.addAction(editItem_act);
    menu.addAction(delItem_act);

    menu.exec( scene_objects->mapToGlobal(pos) );
}
void primary_win::obstacle_type_menu(const QPoint& pos)
{
    qDebug() << pos <<", obstacle type call \n";

    QAction *newAct = new QAction(QIcon(), tr("&New Obstacle"), this);
    newAct->setStatusTip(tr("add new obstacle"));
    connect(newAct, SIGNAL(triggered()), this, SLOT(new_obstacle()));


    QMenu menu(this);
    menu.addAction(newAct);

//    QPoint pt(pos);
    menu.exec( scene_objects->mapToGlobal(pos) );
}
void primary_win::obstacle_menu(const QPoint& pos)
{
    qDebug() << pos <<"an obstacle call \n";
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

void primary_win::menus(const QPoint& pos)
{
    QTreeWidgetItem *selected = scene_objects->itemAt(pos);

    qDebug() << pos << (selected->text(0));

//    QTreeWidgetItem *selected = tree->itemAt(pos);

    qDebug() << "item type: " << selected->type() << "\n";

    (this->*menus_popups[selected->text(0)])(pos);

}


primary_win::~primary_win()
{
    delete _scene_widget;
    delete scene_objects;
    timer.deleteLater();
    delete ui;
}

void primary_win::closeEvent(QCloseEvent *event)
{
    std::cout << "exited normally with code 0\n" << "close ptr: " << event << "\n";
//    ui->SQwidget->cleanUp();

}

void primary_win::update_displys()
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


}

//void primary_win::mouseMoveEvent(QMouseEvent *ev)
//{
//    ui->lcdCamCoordX->display(ui->SQwidget->mainCam.view_direction.x);
//    ui->lcdCamCoordY->display(ui->SQwidget->mainCam.view_direction.y);
//    ui->lcdCamCoordZ->display(ui->SQwidget->mainCam.view_direction.z);


////    ui->label_3->setText(QString::number(ui->SQwidget->mainCam.camera_position.x));
//}


void primary_win::on_grid_check_clicked(bool checked)
{
    _scene_widget->grid_checked = checked;
    _scene_widget->repaint();
}

void primary_win::on_coord_check_clicked(bool checked)
{
    _scene_widget->coord_checked = checked;
    _scene_widget->repaint();
}

void primary_win::on_cam_movement_speed_valueChanged(double arg1)
{
//    _scene_widget->mainCam.MOVEMENT_SPEED = arg1;
    _scene_widget->mainCam.MOVEMENT_SPEED = arg1;
}

void primary_win::on_cam_rotation_speed_valueChanged(double arg1)
{
    _scene_widget->mainCam.ROTATION_SPEED = arg1;
}



void primary_win::on_speedV_valueChanged(int value)
{
    ui->speedDis->setText(QString::number(value));
    GLOBAL_MOVEMENT_SPEED = static_cast<double>(-value);
//    _scene_widget->model->MOVEMENT_SPEED = static_cast<double>(-value);
}


void primary_win::on_radioButton_clicked()
{
    ui->frame_perspective->setDisabled(1);
    _scene_widget->activeCam->auto_perspective = 1;
}

void primary_win::on_radioButton_2_clicked()
{
    ui->frame_perspective->setEnabled(1);
    _scene_widget->activeCam->auto_perspective = 0;
}

void primary_win::on_SpinBox_Near_valueChanged(double arg1)
{
    _scene_widget->activeCam->NEAR_PLANE = arg1;
    _scene_widget->updateProjection();
}

void primary_win::on_SpinBox_Far_valueChanged(double arg1)
{
    _scene_widget->activeCam->FAR_PLANE = arg1;
    _scene_widget->updateProjection();
}

void primary_win::on_SpinBox_FieldOfView_valueChanged(double arg1)
{
    _scene_widget->activeCam->FIELD_OF_VIEW = arg1;
    _scene_widget->updateProjection();
}

void primary_win::on_radioButton_Global_clicked()
{
    _scene_widget->activeCam = &_scene_widget->mainCam;
    ui->camTransformation->setEnabled(1);
    _scene_widget->updateProjection();
}

void primary_win::on_radioButton_Local_clicked()
{
    _scene_widget->activeCam = &_scene_widget->model->frontCam;
    ui->camTransformation->setEnabled(0);
    _scene_widget->updateProjection();
}

void primary_win::on_steeringV_valueChanged(int value)
{
        ui->steeringDis->setText(QString::number(value));

        if(value == 0){
            GLOBAL_STEERING_WHEEL = 0.00000001;  // 0.001f;
//            _scene_widget->model->STEERING_WHEEL = 0.00000001;  // 0.001f;
        }else{
            GLOBAL_STEERING_WHEEL = (static_cast<double>(value)*M_PI) /180.0;
//            _scene_widget->model->STEERING_WHEEL = (static_cast<double>(value)*M_PI) /180.0;
        }
}

} // namespace  zaytuna

