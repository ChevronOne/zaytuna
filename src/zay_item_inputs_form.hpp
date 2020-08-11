#ifndef ZAY_ITEM_INPUTS_FORM_HPP
#define ZAY_ITEM_INPUTS_FORM_HPP

#include <QDialog>
#include "zay_utility.hpp"

namespace Ui {
class item_inputs_form;
}




namespace zaytuna {

class primary_win;
class item_inputs_form : public QDialog
{
    Q_OBJECT

    friend class primary_win;
public:
    explicit item_inputs_form(QWidget *parent = 0);
    explicit item_inputs_form(transform_attribs<GLdouble>,
                              QWidget *parent = 0);
    ~item_inputs_form();

private slots:
    void on_decision_tools_accepted();

private:
    transform_attribs<GLdouble> attribs;
    Ui::item_inputs_form *ui;
};

} // namespace  zaytuna

#endif // ZAY_ITEM_INPUTS_FORM_HPP
