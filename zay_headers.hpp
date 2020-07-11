

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
//  This library is distributed in the hope that it will be useful, but WITHOUT
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
#include <iomanip>
#include <math.h>

#include <QOpenGLFunctions>
//#include <QOpenGLWidget>
#include <QGLWidget>
#include <QOpenGLFunctions_3_0>
#include <QTimer>

#include <QGL>

#include <QImage>
#include <QMessageBox>
#include <QGLFormat>
//#include <QSurfaceFormat>
#include <QGLFramebufferObject>

#if __has_include(<experimental/filesystem>)

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem::v1;

#endif


typedef GLfloat DATA_TYPE;
typedef  QOpenGLFunctions_3_0 USED_GL_VERSION;


#define _USE_MATH_DEFINES
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

//#define PI 3.14159265358979323846
#define NUM_OF(arr)  sizeof(arr) / sizeof(*arr)

#define SIGN_OF(val)  (float(0) < val) - (val < float(0))




#define TYPE_SIZE sizeof(DATA_TYPE)
#define NUM_VERTICES_PER_TRI 3
#define NUM_VERTICES_PER_TEXCOR 2
#define NUM_FLOATS_PER_VERTEX_0 9
#define NUM_FLOATS_PER_VERTEX_1 8
#define SHADERS_NUM 2
#define PROGRAMS_NUM 4
#define VERTEX_BYTE_SIZE_0  NUM_FLOATS_PER_VERTEX_0 * TYPE_SIZE
#define VERTEX_BYTE_SIZE_1 NUM_FLOATS_PER_VERTEX_1 * TYPE_SIZE
#define MOUSE_DELTA_IGNORE 7



//--------------------------

//#define GLM_FORCE_SINGLE_ONLY
#include <glm/vec3.hpp>
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>







#endif // ZAY_HEADERS_HPP




