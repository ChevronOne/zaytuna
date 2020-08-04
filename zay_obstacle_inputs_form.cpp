#include "zay_obstacle_inputs_form.hpp"
#include "ui_zay_obstacle_inputs_form.h"
namespace zaytuna {
obstacle_inputs_form::obstacle_inputs_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::obstacle_inputs_form)
{
    ui->setupUi(this);
}

obstacle_inputs_form::~obstacle_inputs_form()
{
    delete ui;
}

void obstacle_inputs_form::on_decision_tools_accepted()
{
    if(ui->carton_box_radio->isChecked())
        this->attribs.type = Obstacle_Type::CARTON_BOX;
    else if(ui->wall1_radio->isCheckable())
        this->attribs.type = Obstacle_Type::WALL_1;
    else this->attribs.type = Obstacle_Type::WALL_2;
    this->attribs.angle = ui->angle->value();
    this->attribs.translation_vec.x = ui->T_X->value();
    this->attribs.translation_vec.z = ui->T_Z->value();
}

}
