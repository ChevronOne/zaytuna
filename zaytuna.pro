#-------------------------------------------------
#
# Project created by QtCreator 2019-04-20T20:25:07
#
#-------------------------------------------------


#  Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
#
#  This file is part of the Zaytuna Simulator Project.
#
#  Distributed under the terms of the GNU General Public License
#  as published by the Free Software Foundation; You should have
#  received a copy of the GNU General Public License.
#  If not, see <http://www.gnu.org/licenses/>.
#
#
#  This software is distributed in the hope that it will be useful, but WITHOUT
#  WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#  WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND
#  NON-INFRINGEMENT. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR ANYONE
#  DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY,
#  WHETHER IN CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
#  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. See the GNU
#  General Public License for more details.
#
#
# Copyright Abbas Mohammed Murrey 2019-20
#
# Permission to use, copy, modify, distribute and sell this software
# for any purpose is hereby granted without fee, provided that the
# above copyright notice appear in all copies and that both the copyright
# notice and this permission notice appear in supporting documentation.
# I make no representations about the suitability of this software for any
# purpose.  It is provided "as is" without express or implied warranty.
#







QT       += core gui opengl widgets
win32:INCLUDEPATH += C:/boost/boost_1_67_0 C:/glm
LIBS += -lstdc++fs # -lopencv_imgcodecs -lopencv_core

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = zaytuna
TEMPLATE = app

#INCLUDEPATH += "/opt/ros/melodic/include"

#unix:LIBS += -L/opt/ros/melodic/lib -librosconsole # -librosconsole_backend_interface -librosconsole_bridge -librosconsole_log4cxx
#LIBS += -L/opt/ros/melodic/lib -librosconsole -librosconsole_backend_interface -librosconsole_bridge -librosconsole_log4cxx
#LIBS += -L"$$_PRO_FILE_PWD_/opt/ros/melodic/lib" -librosconsole




# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.

 DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++17

# --std=c++1z

QMAKE_CXXFLAGS += -g -Wall -std=c++1z -Wno-old-style-cast
#QMAKE_CXXFLAGS += -g -Wall -std=c++1z -Werror -Wno-old-style-cast
#QMAKE_CXXFLAGS += -g -Wall -Werror
#QMAKE_CXXFLAGS += -O2 # -g -Wall -Werror

SOURCES += \
    src/zay_utility.cpp \
    src/zay_cam.cpp \
    src/zay_item.cpp \
    src/zay_shape_maker.inl \
    src/zay_scene_widg.cpp \
    src/zaytuna.cpp \
    src/zay_model_vehicle.cpp \
    src/zay_primary_win.cpp \
    src/zay_item_inputs_form.cpp \
    src/zay_obstacle_inputs_form.cpp

HEADERS += \
    src/zay_utility.hpp \
    src/zay_clock.hpp \
    src/zay_headers.hpp \
    src/zay_item.hpp \
    src/zay_shape_data.hpp \
    src/zay_shape_maker.hpp \
    src/zay_vertex.hpp \
    src/zay_scene_widg.hpp \
    src/zay_model_vehicle.hpp \
    src/zay_cam.hpp \
    src/zay_primary_win.hpp \
    src/zay_item_inputs_form.hpp \
    src/zay_obstacle_inputs_form.hpp

FORMS += \
    src/zay_primary_win.ui \
    src/zay_item_inputs_form.ui \
    src/zay_obstacle_inputs_form.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
