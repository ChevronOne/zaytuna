

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



#include "zay_item_inputs_form.hpp"
#include "ui_zay_item_inputs_form.h"

namespace zaytuna {


item_inputs_form::item_inputs_form(const std::string& ZAY_PACKAGE_PATH,
                                   QWidget *parent) :
    QDialog(parent),
    ui(new Ui::item_inputs_form){

    ui->setupUi(this);
    set_tool_tips(ZAY_PACKAGE_PATH);

    ui->angle->setValue(0.0);
    ui->T_X->setValue(0.0);
    ui->T_Z->setValue(0.0);
    ui->front_cam_v_angle->setValue(ZAY_DEFAULT_FRONT_CAM_V_ANGLE_MIN);

}


item_inputs_form::item_inputs_form(veh_transform_attribs<GLdouble> attribs, 
    const std::string& ZAY_PACKAGE_PATH, QWidget *parent) :
    QDialog(parent), ui(new Ui::item_inputs_form)
{

    this->attribs = attribs;

    ui->setupUi(this);
    set_tool_tips(ZAY_PACKAGE_PATH);
    ui->angle->setValue(attribs.angle);
    ui->T_X->setValue(attribs.translation_vec.x);
    ui->T_Z->setValue(attribs.translation_vec.z);
    ui->front_cam_v_angle->setValue(attribs.front_cam_v_angle);

}


item_inputs_form::~item_inputs_form(){
    delete ui;
}


void item_inputs_form::on_decision_tools_accepted(){

    this->attribs.angle = ui->angle->value();
    this->attribs.translation_vec.x = ui->T_X->value();
    this->attribs.translation_vec.z = ui->T_Z->value();
    this->attribs.front_cam_v_angle = ui->front_cam_v_angle->value();

}


void item_inputs_form::set_tool_tips(const std::string& ZAY_PACKAGE_PATH){

    ui->T_X->setToolTip("[" + QString::number(ZAY_ACCESSIBLE_MIN_X) + ", +" +QString::number(ZAY_ACCESSIBLE_MAX_X)+"]");
    ui->T_Z->setToolTip("[" + QString::number(ZAY_ACCESSIBLE_MIN_Z) + ", +" +QString::number(ZAY_ACCESSIBLE_MAX_Z)+"]");
    ui->angle->setToolTip("[" + QString::number(ZAY_ANGLE_DEGREE_MIN) + ", +" +QString::number(ZAY_ANGLE_DEGREE_MAX)+"]");
    ui->front_cam_v_angle->setMaximum(ZAY_FRONT_CAM_V_ANGLE_MAX);
    ui->front_cam_v_angle->setMinimum(ZAY_FRONT_CAM_V_ANGLE_MIN);
    ui->front_cam_v_angle->setToolTip("vertical angle [" + QString::number(ZAY_FRONT_CAM_V_ANGLE_MIN) + ", +" +QString::number(ZAY_FRONT_CAM_V_ANGLE_MAX)+"]");
    ui->front_cam_v_angle_ref_lab->setPixmap(QPixmap((ZAY_PACKAGE_PATH+"/resources/cam_v-angle").c_str()).scaled
                       (ui->front_cam_v_angle_ref_lab->width(),
                       ui->front_cam_v_angle_ref_lab->height(),
                       Qt::KeepAspectRatio));

}




} // namespace  zaytuna




