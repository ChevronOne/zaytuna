

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

//#define __GLM__

#define _USE_MATH_DEFINES
#include <math.h>

#define PI 3.14159265358979323846   // pi
#define NUM_OF(arr)  sizeof(arr) / sizeof(*arr)

#define __sign(val)  (float(0) < val) - (val < float(0))



#ifdef __OPENGL__

#include <QGL>
#include <experimental/filesystem>
#include <QOpenGLExtraFunctions>

namespace fs = std::experimental::filesystem::v1;

#define SHADERS_NUM 2
const GLuint programs_num{4};

#endif
//--------------------------

#ifdef __GLM__

#define GLM_FORCE_SINGLE_ONLY
#include <glm/vec3.hpp>
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>




#endif

//----------------------------

#ifdef __SDL__

#include <SDL2/SDL.h>

#endif


#endif // ZAY_HEADERS_HPP
