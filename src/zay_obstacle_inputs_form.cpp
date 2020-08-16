#include "zay_obstacle_inputs_form.hpp"
#include "ui_zay_obstacle_inputs_form.h"

namespace zaytuna {

obstacle_inputs_form::obstacle_inputs_form(QWidget *parent) :
        QDialog(parent),
        ui(new Ui::obstacle_inputs_form){
    ui->setupUi(this);
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
    this->attribs.name = attribs.name;
    switch (attribs.type) {
    case Obstacle_Type::CARTON_BOX:
        ui->carton_box_radio->setChecked(1);
        break;
    case Obstacle_Type::BRICK_WALL:
        ui->wall1_radio->setChecked(1);
        break;
    default:
        ui->wall2_radio->setChecked(1);
        break;
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

} //  namespace zaytuna




