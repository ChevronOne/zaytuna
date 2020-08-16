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


