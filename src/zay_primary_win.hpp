

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





#ifndef ZAY_PRIMARY_WIN_HPP
#define ZAY_PRIMARY_WIN_HPP

#include <QMainWindow>
#include <QWidget>
#include <QtGui/QMouseEvent>
#include <QtGui/QKeyEvent>
#include <QTimer>
#include <QTreeWidget>
#include <QDebug>
#include "zay_scene_widg.hpp"
#include "zay_item_inputs_form.hpp"
#include "zay_obstacle_inputs_form.hpp"


namespace Ui {
class primary_win;
}

namespace zaytuna {


class primary_win : public QMainWindow
{
    Q_OBJECT

public:
    explicit primary_win(QWidget *parent = nullptr);
    virtual ~primary_win() override;
    void vehicle_type_menu(const QPoint&);
    void vehicle_menu(const QPoint&);
    void obstacle_type_menu(const QPoint&);
    void obstacle_menu(const QPoint&);


protected:
    virtual void closeEvent(QCloseEvent*) override;

private slots:
    void on_grid_check_clicked(bool);
    void on_coord_check_clicked(bool);
    void update_displays(void);
    void on_cam_movement_speed_valueChanged(double);
    void on_cam_rotation_speed_valueChanged(double);
    void on_speedV_valueChanged(int);
    void on_SpinBox_Near_valueChanged(double);
    void on_SpinBox_Far_valueChanged(double);
    void on_SpinBox_FieldOfView_valueChanged(double);
    void on_radioButton_Global_clicked();
    void on_radioButton_Local_clicked();
    void on_steeringV_valueChanged(int value);
    void menus(const QPoint&);
    void new_vehicle(void);
    void add_vehicle(transform_attribs<GLdouble>,bool);
    void new_obstacle(void);
    void add_obstacle(obstacle_attribs<GLdouble>,bool);
    void front_cam_to_screen(const QString&);
    void attach_to_control_panel(const QString&);
    void detach_from_control_panel(const QString&);
    void delete_vehicle(const QString&);
    void edit_vehicle(const QString&);
    void delete_obstacle(const QString&);
    void delete_obstacle_confirm(const QString&);
    void edit_obstacle(const QString&);
    void on_auto_perspective_radio_clicked();
    void on_custom_perspective_radio_clicked();
    void on_front_cam_freq_SpinBox_valueChanged(double);
    void on_key_control_radio_clicked();
    void on_slider_control_radio_clicked();
    void on_max_steering_SpinBox_valueChanged(double);
    void on_max_speed_SpinBox_valueChanged(double);

private:
    Ui::primary_win *ui{nullptr};
    QTimer timer;
    zaytuna::_scene_widg* _scene_widget{nullptr};
    QTreeWidget* scene_objects{nullptr};
    std::map<QString, void(primary_win::*)(const QPoint&)> menus_popups;
    QTreeWidgetItem *vehicle_type{nullptr}, *obstacle_type{nullptr};
    std::map<QString, QTreeWidgetItem*> vehicles, obstacles;
    uint32_t vehicle_counter{1};
    std::map<zaytuna::Obstacle_Type, obstacle_tracker> obstacle_counter;
};


} // namespace  zaytuna


#endif // ZAY_WIN_MAINLINER_HPP



