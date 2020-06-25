

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







#include "zay_item.hpp"
namespace zaytuna
{

//zaytuna::_item::_item() : itID{0}, compNum{0}, vNum{0}, iNum{0} {}
zaytuna::_item::_item(const GLuint& it_id, const GLuint& p_id,
                      const glm::dmat4& trans):
                itID{it_id}, program_id{p_id}, transform{trans} {}



const GLuint& zaytuna::_item::g_ind_striop(void) const{
    return _ind_strip;
}
void zaytuna::_item::s_ind_striop(const GLuint& _offset){
    _ind_strip = _offset;
}





//=====================================


zaytuna::_item0::_item0(const GLuint& it_id, const GLuint& p_id,
                        const glm::dmat4& trans, const GLuint& texid):
    _item(it_id, p_id, trans), tex_id{texid}  {}

const GLuint &_item0::g_tex_id() const
{
    return tex_id;
}

void _item0::s_tex_id(const GLuint& texid)
{
    tex_id = texid;
}

//zaytuna::_item0::~_item0(){ }






}  // namespace zaytuna

