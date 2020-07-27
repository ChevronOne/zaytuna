

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





#ifndef ZAY_WIN_MAINLINER_HPP
#define ZAY_WIN_MAINLINER_HPP

#include <QMainWindow>
#include <QWidget>
#include <QtGui/QMouseEvent>
#include <QtGui/QKeyEvent>
#include <QTimer>
#include <QTreeWidget>
#include <QDebug>
#include "zay_scene_widg.hpp"



namespace Ui {
class primary_win;
}

namespace zaytuna {


//// for debugging
//extern double GLOBAL_MOVEMENT_SPEED;
//extern double GLOBAL_STEERING_WHEEL;

class primary_win : public QMainWindow
{
    Q_OBJECT

public:
    explicit primary_win(QWidget *parent = nullptr);
    virtual ~primary_win();


    void vehicle_type_menu(const QPoint&);
    void vehicle_menu(const QPoint&);
    void obstacle_type_menu(const QPoint&);
    void obstacle_menu(const QPoint&);



//    void update_displys(void);


protected:
//    void mouseMoveEvent(QMouseEvent*);
//    void keyPressEvent(QKeyEvent*);
    virtual void closeEvent(QCloseEvent*);

private slots:

    void on_grid_check_clicked(bool checked);

    void on_coord_check_clicked(bool checked);

    void update_displys(void);

    void on_cam_movement_speed_valueChanged(double arg1);

    void on_cam_rotation_speed_valueChanged(double arg1);

    void on_speedV_valueChanged(int value);

    void on_radioButton_clicked();

    void on_radioButton_2_clicked();

    void on_SpinBox_Near_valueChanged(double arg1);

    void on_SpinBox_Far_valueChanged(double arg1);

    void on_SpinBox_FieldOfView_valueChanged(double arg1);

    void on_radioButton_Global_clicked();

    void on_radioButton_Local_clicked();

    void on_steeringV_valueChanged(int value);

    void menus(const QPoint&);
    void new_vehicle(void);
    void new_obstacle(void);
    void delete_vehicle(const QString&);
    void edit_vehicle(const QString&);
    void delete_obstacle(const QString&);
    void edit_obstacle(const QString&);

private:
    Ui::primary_win *ui{nullptr};
    QTimer timer;
    zaytuna::_scene_widg* _scene_widget{nullptr};
    QTreeWidget* scene_objects{nullptr};
    std::map<QString, void(primary_win::*)(const QPoint&)> menus_popups;
    QTreeWidgetItem *vehicle_type{nullptr}, *obstacle_type{nullptr};
    std::map<QString, QTreeWidgetItem*> vehicles, obstacles;

    //-----------------------



};


} // namespace  zaytuna


#endif // ZAY_WIN_MAINLINER_HPP



