

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





#ifndef ZAY_SHAPE_DATA_HPP
#define ZAY_SHAPE_DATA_HPP


#include "zay_vertex.hpp"



namespace zaytuna {


template <typename VERT>
struct shape_data
{   

    shape_data(shape_data<VERT>&) = delete;
    shape_data<VERT>& operator=(shape_data<VERT>&) = delete;
    shape_data(shape_data<VERT>&&) = default;
    shape_data<VERT>& operator=(shape_data<VERT>&&) = default;
    shape_data() = default;
    

    GLsizeiptr verBufSize() const
    {
        return vertices.size() * sizeof(VERT);
    }

    GLsizeiptr indBufSize() const
    {
        return indices.size() * sizeof(GLuint);
    }

    void cleanUP()
    {
        vertices.clear();
        indices.clear();
    }

    
    std::vector<VERT> vertices;
    std::vector<GLuint> indices;

};



} // namespace zaytuna




#endif // ZAY_SHAPE_DATA_HPP



