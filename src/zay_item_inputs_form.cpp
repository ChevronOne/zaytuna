#include "zay_item_inputs_form.hpp"
#include "ui_zay_item_inputs_form.h"

namespace zaytuna {


item_inputs_form::item_inputs_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::item_inputs_form)
{
    ui->setupUi(this);
}

item_inputs_form::~item_inputs_form()
{
    delete ui;
}


} // namespace  zaytuna

void zaytuna::item_inputs_form::on_decision_tools_accepted()
{
    this->transform.angle = ui->angle->value();
    this->transform.translation_vec.x = ui->T_X->value();
    this->transform.translation_vec.z = ui->T_Z->value();
}
