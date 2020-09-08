

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


