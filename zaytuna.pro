#-------------------------------------------------
#
# Project created by QtCreator 2019-04-20T20:25:07
#
#-------------------------------------------------

QT       += core gui opengl widgets
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

QMAKE_CXXFLAGS += -g -Wall -std=c++1z -Werror -Wno-old-style-cast
#QMAKE_CXXFLAGS += -g -Wall -Werror
#QMAKE_CXXFLAGS += -O2 # -g -Wall -Werror

SOURCES += \
    zay_utility.cpp \
    zay_cam.cpp \
    zay_item.cpp \
    zay_shape_maker.inl \
    zay_scene_widg.cpp \
    zay_front_cam.cpp \
    zaytuna.cpp \
    zay_model_vehicle.cpp \
    zay_win_mainliner.cpp

HEADERS += \
    zay_utility.hpp \
    zay_clock.hpp \
    zay_headers.hpp \
    zay_item.hpp \
    zay_shape_data.hpp \
    zay_shape_maker.hpp \
    zay_vertex.hpp \
    zay_scene_widg.hpp \
    zay_model_vehicle.hpp \
    zay_win_mainliner.hpp \
    zay_front_cam.hpp \
    zay_cam.hpp

FORMS += \
    zay_win_mainliner.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
