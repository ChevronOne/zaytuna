

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





#ifndef ZAY_SHAPE_DATA_HPP
#define ZAY_SHAPE_DATA_HPP

#define __OPENGL__


#include "zay_headers.hpp"
#include "zay_vertex.hpp"
#include "zay_utility.hpp"
//#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/include/support_istream_iterator.hpp>
#include <boost/iterator/counting_iterator.hpp>

//namespace qi = boost::spirit::qi;
namespace x3 = boost::spirit::x3;

namespace zaytuna {


template <typename VERT>
struct shape_data
{
    shape_data() : verts(nullptr), verNum(0), indices(nullptr), indNum(0) {}
    VERT* verts;
    GLuint verNum;
    GLuint* indices;
    GLuint indNum;
//    std::vector<glm::vec3> normals;

    GLsizeiptr verBufSize() const
    {
        return verNum * sizeof(VERT);
    }
    GLsizeiptr indBufSize() const
    {
        return indNum * sizeof(unsigned int);
    }

    void cleanUP()
    {
        if(verts != nullptr){
            delete[] verts;
            verts = nullptr;
        }
        if(indices != nullptr){
            delete[] indices;
            indices = nullptr;
        }
        verNum = indNum = 0;
    }
};

} // namespace zaytuna

#endif // ZAY_SHAPE_DATA_HPP
