

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
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_0>

#include <QGL>

#if __has_include(<experimental/filesystem>)

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem::v1;

#endif


typedef GLfloat DATA_TYPE;


#define _USE_MATH_DEFINES
#include <math.h>

#define PI 3.14159265358979323846
#define NUM_OF(arr)  sizeof(arr) / sizeof(*arr)

#define SIGN_OF(val)  (float(0) < val) - (val < float(0))


#define TYPE_SIZE sizeof(DATA_TYPE)
#define NUM_VERTICES_PER_TRI 3
#define NUM_VERTICES_PER_TEXCOR 2
#define NUM_FLOATS_PER_VERTICE_0 9
#define NUM_FLOATS_PER_VERTICE_1 8
#define SHADERS_NUM 2
#define PROGRAMS_NUM 4
#define VERTEX_BYTE_SIZE_0  NUM_FLOATS_PER_VERTICE_0 * TYPE_SIZE
#define VERTEX_BYTE_SIZE_1 NUM_FLOATS_PER_VERTICE_1 * TYPE_SIZE



//--------------------------

#define GLM_FORCE_SINGLE_ONLY
#include <glm/vec3.hpp>
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>







#endif // ZAY_HEADERS_HPP




