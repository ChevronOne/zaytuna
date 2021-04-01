

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

// #pragma once


#ifndef ZAY_SHAPE_MAKER_HPP
#define ZAY_SHAPE_MAKER_HPP


#include "zay_shape_data.hpp"


namespace zaytuna {



template <typename VERT>
class shape_maker
{

public:

    static shape_data<VERT>
    makeSphere(GLfloat PERS = 0.1f,
               GLfloat RAD = 1,
               glm::vec3 CENT = glm::vec3(0.0f, 0.0f, 0.0f));
    static shape_data<VERT> makePyramid();

    static shape_data<VERT> makeCube(void);
    static shape_data<VERT> makeCubemap(void);
    static shape_data<VERT>
    makeGrid(GLfloat length = 10.0f,
             GLfloat width = 10.0f,
             GLfloat tessellation = 1.0f);
    static shape_data<VERT>
    makeCoord(GLfloat axes_length = 10.0f);
    static shape_data<VERT> makeLap();


};





#include "zay_shape_maker.inl"



} // namespace zaytuna



#endif // ZAY_SHAPE_MAKER_HPP






