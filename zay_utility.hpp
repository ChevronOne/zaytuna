

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





#ifndef ZAY_UTILITY_HPP
#define ZAY_UTILITY_HPP


#include "zay_headers.hpp"
#include <vector>
#include <boost/assign/list_of.hpp>
#include <QImage>

namespace zaytuna {

glm::vec3 rCol(void);

std::ostream& operator<<(std::ostream&, const glm::vec3&);

void _load_tex(QImage&, const QString&, const char*, bool, bool);

const char* DebugGLerr(unsigned);

template<class T>
class ptr_vector : public std::vector<T>
{
public:
    virtual ~ptr_vector() {}
};

template<class T>
class ptr_vector<T *> : public std::vector<T *>
{
public:
    ptr_vector&
    operator=(std::initializer_list<T*> _list)
    {
        this->assign(_list.begin(), _list.end());
        return *this;
    }

    virtual ~ptr_vector()
    {
        class std::vector< T *>::reverse_iterator it;
        for (it = this->rbegin(); it != this->rend(); ++it)
            delete *it;
    }
};

//namespace std{
//    class warning : public exception
//    {
//    public:
//         warning(const string& warn_msg) {}
//         const char* what() { return warn_msg.c_str(); } //message of warning
//    private:
//         string warn_msg;
//    };
//}


} // namespace zaytuna


#endif // ZAY_UTILITY_HPP




