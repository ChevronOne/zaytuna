

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
 * Copyright Abbas Mohammed Murrey 2019-21
 *
 * Permission to use, copy, modify, distribute and sell this software
 * for any purpose is hereby granted without fee, provided that the
 * above copyright notice appear in all copies and that both the copyright
 * notice and this permission notice appear in supporting documentation.
 * I make no representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */




#ifndef ZAY_HEADERS_HPP
#define ZAY_HEADERS_HPP



#include <fstream>
#include <iostream>
#include <string>
#include <string.h>
#include <vector>
#include <iterator>
#include <stdint.h>
#include <float.h>
#include <limits.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <math.h>
#include <map>
#include <iterator>
#include <ostream>
#include <limits>
#include <type_traits>
#include <functional>
#include <utility>
#include <memory>
#include <set>

#include <QMainWindow>
#include <QWidget>
#include <QTreeWidget>
#include <QString>
#include <QOpenGLFunctions>
#include <QGLWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_0>
#include <QTimer>
#include <QGL>
#include <QImage>
#include <QMessageBox>
#include <QGLFormat>
#include <QSurfaceFormat>
#include <QGLFramebufferObject>
#include <QIcon>
#include <QFontDatabase>
#include <QPixmap>
#include <QApplication>
#include <QScreen>
#include <QDialog>
#include <QtGui/QMouseEvent>
#include <QtGui/QKeyEvent>
#include <QtGui/QWheelEvent>
#include <QStatusBar>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "ros/package.h"
#include "ros/console.h"

#define BOOST_PYTHON_STATIC_LIB
#define ZAY_BOOST_MAJOR_VERSION BOOST_VERSION / 0x186A0 
#define ZAY_BOOST_MINOR_VERSION BOOST_VERSION / 0x64 % 0x3E8
#define ZAY_BOOST_PATCH_LEVEL BOOST_VERSION % 0x64

typedef GLfloat ZAY_DATA_TYPE;
typedef QOpenGLFunctions_3_0 ZAY_USED_GL_VERSION;
typedef QOpenGLWidget ZAY_Q_OPEN_GL_W;
typedef QGLWidget ZAY_QGL_W;
typedef QStatusBar ZAY_MSG_LOGGER;

typedef ZAY_QGL_W ZAY_QGL_WIDGET_VERSION;
// typedef ZAY_Q_OPEN_GL_W ZAY_QGL_WIDGET_VERSION;  /***Needs to be reworked on, as a Q_OBJECT class cannot be templated!***/



#if !(ZAY_BOOST_MAJOR_VERSION==0x1 && ((ZAY_BOOST_MINOR_VERSION==0x41 && \
        (ZAY_BOOST_PATCH_LEVEL==0x0 || ZAY_BOOST_PATCH_LEVEL==0x1)) || \
     (ZAY_BOOST_MINOR_VERSION==0x42 && ZAY_BOOST_PATCH_LEVEL==0x0)))
#define ZAY_BOOST_SPIRIT_X3_SEMANTIC_ACTION
#endif


#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#ifndef M_PI
# define M_E		2.7182818284590452354	/* e */
# define M_LOG2E	1.4426950408889634074	/* log_2 e */
# define M_LOG10E	0.43429448190325182765	/* log_10 e */
# define M_LN2		0.69314718055994530942	/* log_e 2 */
# define M_LN10		2.30258509299404568402	/* log_e 10 */
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
# define M_PI_4		0.78539816339744830962	/* pi/4 */
# define M_1_PI		0.31830988618379067154	/* 1/pi */
# define M_2_PI		0.63661977236758134308	/* 2/pi */
# define M_2_SQRTPI	1.12837916709551257390	/* 2/sqrt(pi) */
# define M_SQRT2	1.41421356237309504880	/* sqrt(2) */
# define M_SQRT1_2	0.70710678118654752440	/* 1/sqrt(2) */
#endif

#define ZAY_NUM_OF(arr)  sizeof(arr) / sizeof(*arr)
#define ZAY_SIGN_OF(val)  (float(0) < val) - (val < float(0))
#define ZAY_TYPE_SIZE sizeof(ZAY_DATA_TYPE)
#define ZAY_NUM_VERTICES_PER_TRI 3
#define ZAY_NUM_VERTICES_PER_TEXCOR 2
#define ZAY_NUM_FLOATS_PER_VERTEX_0 9
#define ZAY_NUM_FLOATS_PER_VERTEX_1 8
#define ZAY_NUM_FLOATS_PER_VERTEX_2 6
#define ZAY_SHADERS_NUM 2
#define ZAY_PROGRAMS_NUM 4
#define ZAY_VERTEX_BYTE_SIZE_0 ZAY_NUM_FLOATS_PER_VERTEX_0 * ZAY_TYPE_SIZE
#define ZAY_VERTEX_BYTE_SIZE_1 ZAY_NUM_FLOATS_PER_VERTEX_1 * ZAY_TYPE_SIZE
#define ZAY_VERTEX_BYTE_SIZE_2 ZAY_NUM_FLOATS_PER_VERTEX_2 * ZAY_TYPE_SIZE
#define ZAY_MOUSE_DELTA_IGNORE 7.0
#define ZAY_NUM_SEC_FRAME_RATE 1.0
#define ZAY_SCENE_WIDTH 800
#define ZAY_SCENE_HEIGHT 500
#define ZAY_ACCESSIBLE_MIN_X -149.0
#define ZAY_ACCESSIBLE_MAX_X 149.0
#define ZAY_ACCESSIBLE_MIN_Z -149.0
#define ZAY_ACCESSIBLE_MAX_Z 149.0
#define ZAY_ANGLE_DEGREE_MIN -360.0
#define ZAY_ANGLE_DEGREE_MAX 360.0
#define ZAY_FIELD_WIDTH 300
#define ZAY_FIELD_HEIGHT 300
#define ZAY_FIELD_DIAMETER 424.265
#define ZAY_ACCESSIBLE_FIELD_WIDTH 298
#define ZAY_ACCESSIBLE_FIELD_HEIGHT 298
#define ZAY_ACCESSIBLE_FIELD_DIAMETER 421.4356
#define ZAY_CAM_ACCESSIBLE_MIN_Y 0.011
#define ZAY_CAM_ACCESSIBLE_MAX_Y 100.0
#define ZAY_CAM_ACCESSIBLE_MIN_X -148.9
#define ZAY_CAM_ACCESSIBLE_MAX_X 148.9
#define ZAY_CAM_ACCESSIBLE_MIN_Z -148.9
#define ZAY_CAM_ACCESSIBLE_MAX_Z 148.9
#define ZAY_CAM_DEFAULT_ROTATION_SPEED 0.3
#define ZAY_CAM_DEFAULT_MOVEMENT_SPEED 0.024
#define ZAY_CAM_HORIZONTAL_MOVEMENT_SCALAR 0.2
#define ZAY_FRAMES_STABLE_SCALAR 420.0
#define ZAY_MIN_FRAMES_STABLE 27.0
#define ZAY_NUM_SAMPLES_PER_PIXEL 8
#define ZAY_MOUSE_WHEEL_SCALAR 5
#define ZAY_SPEED_SCALAR 600.0
#define ZAY_MAX_TURN_ANGLE 25.0
#define ZAY_RADIAN M_PI/180.0
#define ZAY_MAX_TURN_ANGLE_RAD ZAY_MAX_TURN_ANGLE*ZAY_RADIAN
#define ZAY_STEERING_MARGIN_OF_ERROR 0.00000001
#define ZAY_DELAY_MARGIN_OF_ERROR 0.01
#define ZAY_NUM_OF_CHANNELS 3
#define ZAY_FRONT_IMG_SIZE ZAY_SCENE_WIDTH*ZAY_SCENE_HEIGHT*ZAY_NUM_OF_CHANNELS
#define ZAY_DEFAULT_FRONT_CAM_FREQUENCY 20
#define ZAY_DEFAULT_LOCAL_CONTROL_SPEED 6.0
#define ZAY_DEFAULT_LOCAL_CONTROL_STEERING 0.39269908
#define ZAY_DEFAULT_FRAME_RATE 30
#define ZAY_DEFAULT_FIELD_OF_VIEW 55.0
#define ZAY_NORMALS_STRIDE 3 
#define ZAY_TEXTURE_STRIDE 6
#define ZAY_COLOR_STRIDE 3
#define ZAY_POINT_D 3
#define ZAY_RECTANGLE_P 4
#define ZAY_RECT_LIN_INDEPENDENT_V 2
#define ZAY_PUBLISHERS 6
#define ZAY_SUBSCRIBERS 2
#define ZAY_LIMITED_VEH_NUM 10
#define ZAY_LIMITED_OBS_NUM 5000
#define ZAYTUNA_VERSION 0x0001
#define ZAYTUNA_MINOR_VERSION 0x0003




////--------GLM---------------
#include "glm/vec3.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/type_ptr.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/transform.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/projection.hpp"
#include "glm/gtc/quaternion.hpp"

const glm::dvec3 ZAY_GLOB_CAM_DEFAULT_VIEW_DIR{glm::dvec3(-0.41, -0.6, -0.68)};
const glm::dvec3 ZAY_CAM_UP_DIR{glm::dvec3(0.0, 1.0, 0.0)};
const glm::dvec3 ZAY_GLOB_CAM_DEFAULT_POS{glm::dvec3(8.55, 3.26, 3.5)};
const glm::dvec2 ZAY_ORIG_2{glm::dvec2(0.0, 0.0)};
const glm::dvec3 ZAY_ORIG_3{glm::dvec3(0.0, 0.0, 0.0)};
const glm::dvec4 ZAY_ORIG_4{glm::dvec4(0.0, 0.0, 0.0, 1.0)};




using namespace std::chrono_literals;
const std::string ZAY_PACKAGE_NAME{"zaytuna"};



#endif // ZAY_HEADERS_HPP




