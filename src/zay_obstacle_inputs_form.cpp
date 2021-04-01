

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



#include "zay_obstacle_inputs_form.hpp"
#include "ui_zay_obstacle_inputs_form.h"



namespace zaytuna {



obstacle_inputs_form::obstacle_inputs_form(QWidget *parent) :
        QDialog(parent),
        ui(new Ui::obstacle_inputs_form){

    ui->setupUi(this);
    set_tool_tips();

}



obstacle_inputs_form::obstacle_inputs_form(obstacle_attribs<GLdouble> attribs,
                                           QWidget *parent) :
    QDialog(parent),
    ui(new Ui::obstacle_inputs_form)
{

    ui->setupUi(this);
    ui->angle->setValue(attribs.angle);

    ui->T_X->setValue(attribs.translation_vec.x);
    ui->T_Z->setValue(attribs.translation_vec.z);

    set_tool_tips();

    this->attribs.name = attribs.name;

    switch (attribs.type) {

        case Obstacle_Type::CARTON_BOX:
            ui->carton_box_radio->setChecked(1);
            break;

        case Obstacle_Type::BRICK_WALL:
            ui->wall1_radio->setChecked(1);
            break;

        default: // Obstacle_Type::BRICK_WALL by default
            ui->wall2_radio->setChecked(1);

    }

}



obstacle_inputs_form::~obstacle_inputs_form(){

    delete ui;
}


void obstacle_inputs_form::on_decision_tools_accepted()
{

    if(ui->carton_box_radio->isChecked())
        this->attribs.type = Obstacle_Type::CARTON_BOX;

    else if(ui->wall1_radio->isChecked())
        this->attribs.type = Obstacle_Type::BRICK_WALL;

    else this->attribs.type = Obstacle_Type::STONE_WALL;


    this->attribs.angle = ui->angle->value();
    this->attribs.translation_vec.x = ui->T_X->value();
    this->attribs.translation_vec.z = ui->T_Z->value();

}



void obstacle_inputs_form::set_tool_tips(){

    ui->T_X->setToolTip("[" + QString::number(ZAY_ACCESSIBLE_MIN_X) + ", +" +QString::number(ZAY_ACCESSIBLE_MAX_X)+"]");
    ui->T_Z->setToolTip("[" + QString::number(ZAY_ACCESSIBLE_MIN_Z) + ", +" +QString::number(ZAY_ACCESSIBLE_MAX_Z)+"]");
    ui->angle->setToolTip("[" + QString::number(ZAY_ANGLE_DEGREE_MIN) + ", +" +QString::number(ZAY_ANGLE_DEGREE_MAX)+"]");

}



} //  namespace zaytuna




