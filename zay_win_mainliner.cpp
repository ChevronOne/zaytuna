

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





#include "zay_win_mainliner.hpp"
#include "ui_zay_win_mainliner.h"
#include <QString>


namespace zaytuna {

win_mainliner::win_mainliner(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::win_mainliner)
{
    ui->setupUi(this);



//    QPainter painter(this);
//    painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform | QPainter::TextAntialiasing);

    ui->lcdCamViewX->display(ui->SQwidget->mainCam.view_direction.x);
    ui->lcdCamViewY->display(ui->SQwidget->mainCam.view_direction.y);
    ui->lcdCamViewZ->display(ui->SQwidget->mainCam.view_direction.z);

    ui->lcdCamCoordX->display(ui->SQwidget->mainCam.camera_position.x);
    ui->lcdCamCoordY->display(ui->SQwidget->mainCam.camera_position.y);
    ui->lcdCamCoordZ->display(ui->SQwidget->mainCam.camera_position.z);

    ui->speedDis->setText(QString::number(0));
    ui->steeringDis->setText(QString::number(0));

    connect(&timer, SIGNAL(timeout()), this, SLOT(update_displys()));
    timer.start(20);


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

win_mainliner::~win_mainliner()
{
    timer.deleteLater();
    delete ui;
}

void win_mainliner::closeEvent(QCloseEvent *event)
{
    std::cout << "exited normally with code 0\n" << "close ptr: " << event << "\n";
//    ui->SQwidget->cleanUp();

}

void win_mainliner::update_displys()
{

    ui->lcdCamViewX->display(ui->SQwidget->activeCam->view_direction.x);
    ui->lcdCamViewY->display(ui->SQwidget->activeCam->view_direction.y);
    ui->lcdCamViewZ->display(ui->SQwidget->activeCam->view_direction.z);

    ui->lcdCamCoordX->display(ui->SQwidget->activeCam->camera_position.x);
    ui->lcdCamCoordY->display(ui->SQwidget->activeCam->camera_position.y);
    ui->lcdCamCoordZ->display(ui->SQwidget->activeCam->camera_position.z);



    ui->cam_movement_speed->setValue(ui->SQwidget->mainCam.MOVEMENT_SPEED);
    ui->cam_rotation_speed->setValue(ui->SQwidget->mainCam.ROTATION_SPEED);

    ui->SpinBox_FieldOfView->setValue(ui->SQwidget->activeCam->FIELD_OF_VIEW);
    ui->SpinBox_Near->setValue(ui->SQwidget->activeCam->NEAR_PLANE);
    ui->SpinBox_Far->setValue(ui->SQwidget->activeCam->FAR_PLANE);


}

//void win_mainliner::mouseMoveEvent(QMouseEvent *ev)
//{
//    ui->lcdCamCoordX->display(ui->SQwidget->mainCam.view_direction.x);
//    ui->lcdCamCoordY->display(ui->SQwidget->mainCam.view_direction.y);
//    ui->lcdCamCoordZ->display(ui->SQwidget->mainCam.view_direction.z);


////    ui->label_3->setText(QString::number(ui->SQwidget->mainCam.camera_position.x));
//}


void win_mainliner::on_grid_check_clicked(bool checked)
{
    ui->SQwidget->grid_checked = checked;
    ui->SQwidget->repaint();
}

void win_mainliner::on_coord_check_clicked(bool checked)
{
    ui->SQwidget->coord_checked = checked;
    ui->SQwidget->repaint();
}

void win_mainliner::on_cam_movement_speed_valueChanged(double arg1)
{
    ui->SQwidget->mainCam.MOVEMENT_SPEED = arg1;
}

void win_mainliner::on_cam_rotation_speed_valueChanged(double arg1)
{
    ui->SQwidget->mainCam.ROTATION_SPEED = arg1;
}



void win_mainliner::on_speedV_valueChanged(int value)
{
    ui->speedDis->setText(QString::number(value));
    ui->SQwidget->model->MOVEMENT_SPEED = static_cast<double>(-value);
}

//void win_mainliner::on_steeringAng_valueChanged(int value)
//{
//    ui->steeringDis->setText(QString::number(value));

//    if(value == 0){
//        ui->SQwidget->model.STEERING_WHEEL = 0.00000001;  // 0.001f;
//    }else{
//        ui->SQwidget->model.STEERING_WHEEL = (static_cast<double>(value)*PI) /180.0  ;///30;
//    }
//}

void win_mainliner::on_radioButton_clicked()
{
    ui->frame_perspective->setDisabled(1);
    ui->SQwidget->activeCam->auto_perspective = 1;
}

void win_mainliner::on_radioButton_2_clicked()
{
    ui->frame_perspective->setEnabled(1);
    ui->SQwidget->activeCam->auto_perspective = 0;
}

void win_mainliner::on_SpinBox_Near_valueChanged(double arg1)
{
    ui->SQwidget->activeCam->NEAR_PLANE = arg1;
    ui->SQwidget->updateProjection();
}

void win_mainliner::on_SpinBox_Far_valueChanged(double arg1)
{
    ui->SQwidget->activeCam->FAR_PLANE = arg1;
    ui->SQwidget->updateProjection();
}

void win_mainliner::on_SpinBox_FieldOfView_valueChanged(double arg1)
{
    ui->SQwidget->activeCam->FIELD_OF_VIEW = arg1;
    ui->SQwidget->updateProjection();
}

void win_mainliner::on_radioButton_Global_clicked()
{
    ui->SQwidget->activeCam = &ui->SQwidget->mainCam;
    ui->camTransformation->setEnabled(1);
    ui->SQwidget->updateProjection();
}

void win_mainliner::on_radioButton_Local_clicked()
{
    ui->SQwidget->activeCam = &ui->SQwidget->model->frontCam;
    ui->camTransformation->setEnabled(0);
    ui->SQwidget->updateProjection();
}

void win_mainliner::on_steeringV_valueChanged(int value)
{
        ui->steeringDis->setText(QString::number(value));

        if(value == 0){
            ui->SQwidget->model->STEERING_WHEEL = 0.00000001;  // 0.001f;
        }else{
            ui->SQwidget->model->STEERING_WHEEL = (static_cast<double>(value)*PI) /180.0  ;///30;
        }
}

} // namespace  zaytuna
