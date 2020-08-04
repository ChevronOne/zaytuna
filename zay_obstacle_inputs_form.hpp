#ifndef ZAY_OBSTACLE_INPUTS_FORM_HPP
#define ZAY_OBSTACLE_INPUTS_FORM_HPP

#include <QDialog>
#include "zay_utility.hpp"

namespace Ui {
class obstacle_inputs_form;
}

namespace zaytuna{
class primary_win;
class obstacle_inputs_form : public QDialog
{
    Q_OBJECT
friend class primary_win;
public:
    explicit obstacle_inputs_form(QWidget *parent = 0);
    ~obstacle_inputs_form();

private slots:
    void on_decision_tools_accepted();

private:
    obstacle_attribs<GLdouble> attribs;
    Ui::obstacle_inputs_form *ui;
};


}
#endif // ZAY_OBSTACLE_INPUTS_FORM_HPP
