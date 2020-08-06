

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






#include "zay_utility.hpp"

namespace zaytuna {

glm::vec3 rCol(void)
{
    return {
        rand() / static_cast<float>(RAND_MAX),
        rand() / static_cast<float>(RAND_MAX),
        rand() / static_cast<float>(RAND_MAX)
    };
}

void _load_tex(QImage& buff,
               const QString& _dir,
               const char* _format,
               bool hMir, bool vMir)
{
    if(!(buff.load(_dir, _format))){
        std::cout << "image couldn't be loaded <"
                  << _dir.toStdString() << ">!\n";
        exit(EXIT_FAILURE);
    }

    buff = QGLWidget::convertToGLFormat(buff.mirrored(hMir, vMir));
    if(buff.isNull()){
        std::cout << "error occurred while converting the image <"
                  <<_dir.toStdString() <<">!\n";
        exit(EXIT_FAILURE);
    }
}

const char *DebugGLerr(unsigned GL_enum)
{

    switch( GL_enum ){
        case 0:      return "GL Error Message: GL_NO_ERROR";
        case 0x0500: return "GL Error Message: GL_INVALID_ENUM";
        case 0x0501: return "GL Error Message: GL_INVALID_VALUE";
        case 0x0502: return "GL Error Message: GL_INVALID_OPERATION";
        case 0x0503: return "GL Error Message: GL_STACK_OVERFLOW";
        case 0x0504: return "GL Error Message: GL_STACK_UNDERFLOW";
        case 0x0505: return "GL Error Message: GL_OUT_OF_MEMORY";

        default:     return "GL Error Message: unknown error value";
    }
}



} // namespace  zaytuna













