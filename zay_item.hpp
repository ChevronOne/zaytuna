

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







#ifndef ZAY_ITEM_HPP
#define ZAY_ITEM_HPP

#define __OPENGL__
#define __GLM__


#include "zay_headers.hpp"
#include "zay_vertex.hpp"

namespace zaytuna{

typedef uint16_t _itm0_indRange_t;
typedef uint16_t _itm1_indRange_t;
typedef uint16_t _itm2_indRange_t;


class _item
{

private:

    GLuint _ind_strip;


public:
    virtual const GLuint& g_ind_striop(void) const;
    virtual void s_ind_striop(const GLuint&);


    _item() = default;
    _item(const GLuint&, const GLuint&,
          const glm::dmat4&);
    virtual ~_item() = default;
    GLuint itID;
    GLuint program_id;
    glm::dmat4 transform;
};

//===========================================

class _item0 : public _item
{

private:
//    vertexL1_F* vertices;
//    _itm0_indRange_t* indices;


public:
//    uint64_t i_BufSize(void) const;
//    const vertexL1_F* g_vertices(void) const;
//    const _itm0_indRange_t* g_indices(void) const;
    virtual const GLuint& g_tex_id(void) const;
    virtual void s_tex_id(const GLuint&);


    _item0() = default;
    _item0(const GLuint&, const GLuint&,
           const glm::dmat4&, const GLuint&);
    virtual ~_item0() = default;

    GLuint tex_id;
};

//===========================================

//class _item1 : public _item
//{

//private:
//    vertexL1_12* vertices;
//    _itm0_indRange_t* indices;

//    void cleanUp();

//public:
//    uint64_t i_BufSize(void) const;
//    const vertexL1_12* g_vertices(void) const;
//    const _itm1_indRange_t* g_indices(void) const;


//    _item1();
//    _item1(zaytuna::details*&, const uint32_t&,
//           vertexL1_12*&, const uint32_t&,
//           _itm0_indRange_t*&, const uint32_t&);
//    virtual ~_item1();

//};

} // namespace zaytuna

#endif // ZAY_ITEM_HPP
