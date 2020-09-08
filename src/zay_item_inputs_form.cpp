

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



#include "zay_item_inputs_form.hpp"
#include "ui_zay_item_inputs_form.h"

namespace zaytuna {


item_inputs_form::item_inputs_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::item_inputs_form){
    ui->setupUi(this);
}
item_inputs_form::item_inputs_form
(transform_attribs<GLdouble> attribs, QWidget *parent) :
    QDialog(parent), ui(new Ui::item_inputs_form)
{
    this->attribs = attribs;
    ui->setupUi(this);
    ui->angle->setValue(attribs.angle);
    ui->T_X->setValue(attribs.translation_vec.x);
    ui->T_Z->setValue(attribs.translation_vec.z);
}

item_inputs_form::~item_inputs_form(){
    delete ui;
}

void item_inputs_form::on_decision_tools_accepted(){
    this->attribs.angle = ui->angle->value();
    this->attribs.translation_vec.x = ui->T_X->value();
    this->attribs.translation_vec.z = ui->T_Z->value();
}

} // namespace  zaytuna


