

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





#ifndef ZAY_VERTEX_HPP
#define ZAY_VERTEX_HPP

#define __GLM__

#include "zay_headers.hpp"

namespace zaytuna
{

struct vertexL0_0 {
    glm::vec2 position; // x y
};

struct vertexL0_1 {
    glm::vec2 position; // x y
    glm::vec3 color; // r g b
};

struct vertexL0_2 {
    glm::vec3 position; // x y d
    glm::vec3 color; // r g b
};

struct vertexL0_3 {
    glm::vec3 position; // x y d
    glm::vec4 color; // r g b a
};

struct vertexL0_4 {
    glm::vec2 position; // x y
    glm::vec2 texcoor; // u v
};

struct vertexL0_5 {
    glm::vec2 position; // x y
    glm::vec3 color; // r g b
    glm::vec2 texcoor; // u v
};

struct vertexL0_6 {
    glm::vec3 position; // x y d
    glm::vec3 color; // r g b
    glm::vec2 texcoor; // u v
};

struct vertexL0_F {
    glm::vec3 position; // x y d
    glm::vec4 color; // r g b a
    glm::vec2 texcoor; // u v
};


struct vertexL1_0 {
    glm::vec3 position; // x y z
};

struct vertexL1_1 {
    glm::vec3 position; // x y z
    glm::vec3 color; // r g b
};

struct vertexL1_2 {
    glm::vec4 position; // x y z d
    glm::vec3 color; // r g b
};

struct vertexL1_3 {
    glm::vec4 position; // x y z d
    glm::vec4 color; // r g b a
};

struct vertexL1_4 {
    glm::vec3 position; // x y z
    glm::vec4 color; // r g b a
};

struct vertexL1_5 {
    glm::vec3 position; // x y z
    glm::vec2 texcoor; // u v
};

struct vertexL1_6 {
    glm::vec4 position; // x y z d
    glm::vec2 texcoor; // u v
};


struct vertexL1_7 {
    glm::vec3 position; // x y z
    glm::vec3 color; // r g b
    glm::vec2 texcoor; // u v
};

struct vertexL1_8 {
    glm::vec4 position; // x y z d
    glm::vec3 color; // r g b
    glm::vec2 texcoor; // u v
};

struct vertexL1_9 {
    glm::vec3 position; // x y z
    glm::vec4 color; // r g b a
    glm::vec2 texcoor; // u v
};

struct vertexL1_10 {
    glm::vec4 position; // x y z d
    glm::vec4 color; // r g b a
    glm::vec2 texcoor; // u v
};

struct vertexL1_11 {
    glm::vec3 position; // x y z
    glm::vec3 normal; // x y z
};

struct vertexL1_12 {
    glm::vec3 position; // x y z
    glm::vec3 color; // r g b
    glm::vec3 normal; // x y z
};

struct vertexL1_13 {
    glm::vec4 position; // x y z d
    glm::vec3 color; // r g b
    glm::vec3 normal; // x y z
};

struct vertexL1_14 {
    glm::vec4 position; // x y z d
    glm::vec4 color; // r g b a
    glm::vec3 normal; // x y z
};

struct vertexL1_15 {
    glm::vec3 position; // x y z
    glm::vec4 color; // r g b a
    glm::vec3 normal; // x y z
};

struct vertexL1_16 {
    glm::vec3 position; // x y z
    glm::vec3 normal; // x y z
    glm::vec2 texcoor; // u v

};

struct vertexL1_17 {
    glm::vec4 position; // x y z d
    glm::vec2 texcoor; // u v
    glm::vec3 normal; // x y z
};


struct vertexL1_18 {
    glm::vec3 position; // x y z
    glm::vec3 color; // r g b
    glm::vec2 texcoor; // u v
    glm::vec3 normal; // x y z
};

struct vertexL1_19 {
    glm::vec4 position; // x y z d
    glm::vec3 color; // r g b
    glm::vec2 texcoor; // u v
    glm::vec3 normal; // x y z
};

struct vertexL1_20 {
    glm::vec3 position; // x y z
    glm::vec4 color; // r g b a
    glm::vec2 texcoor; // u v
    glm::vec3 normal; // x y z
};

struct vertexL1_F {
    glm::vec4 position; // x y z d
    glm::vec4 color; // r g b a
    glm::vec2 texcoor; // u v
    glm::vec3 normal; // x y z
};

struct details{
    uint32_t _begin, _strid, _size;
};



struct vertex {  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< orig
    glm::vec3 position; // x y z
    glm::vec3 color; // r g b
    glm::vec3 normal; // x y z
};

} // namespace

#endif // ZAY_VERTEX_HPP
