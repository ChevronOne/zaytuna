

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


template<class T>
class ptr_vector : public std::vector<T>
{
public:
    virtual ~ptr_vector() {}
};

template<class T>
class ptr_vector<T*> : public std::vector<T*>
{
public:
    ptr_vector&
    operator=(std::initializer_list<T*> _list){
        this->assign(_list.begin(), _list.end());
        return *this;
    }

    virtual ~ptr_vector(){
        class std::vector< T *>::reverse_iterator it;
        for (it = this->rbegin(); it != this->rend(); ++it)
            delete *it;
    }
};
enum class Obstacle_Type { CARTON_BOX, WALL_1, WALL_2 };

template<class T>
struct transform_attribs
{
    std::string name{"uninitialized_item_name"};
    T angle{static_cast<T>(0.0)};
    glm::tvec3<T> rotation_vec
        {static_cast<T>(0.0),
         static_cast<T>(1.0),
         static_cast<T>(0.0)};
    glm::tvec3<T> translation_vec
        {static_cast<T>(0.0),
         static_cast<T>(0.0),
         static_cast<T>(0.0)};
    transform_attribs() = default;
    transform_attribs(const std::string& name,
                      T angle,
                      glm::tvec3<T> r_vec,
                      glm::tvec3<T> t_vec):
        name{name},
        angle{angle},
        rotation_vec{r_vec},
        translation_vec{t_vec}{}

    glm::tmat4x4<T> rotationMat() const {
        return glm::rotate(glm::radians(angle),
                           rotation_vec); }
    glm::tmat4x4<T> translationMat() const {
        return glm::translate(translation_vec); }
    glm::tmat4x4<T> transformMat() const{
        return translationMat()*rotationMat();
    }
};

template<class T>
struct obstacle_attribs : public transform_attribs<T>
{
    Obstacle_Type type{Obstacle_Type::WALL_2};
    obstacle_attribs() = default;
    obstacle_attribs(Obstacle_Type type,
                     const std::string& name,
                     T angle,
                     glm::tvec3<T> r_vec,
                     glm::tvec3<T> t_vec):
        transform_attribs<T>(name, angle, r_vec, t_vec),
        type{type}{}
};

template<class T>
struct default_settings{
    std::vector<transform_attribs<T>> vehicles;
    std::vector<obstacle_attribs<T>> obstacles;
};

class basic_program
{
protected:
    USED_GL_VERSION* gl_context{nullptr};
    GLuint program_ID;

public:
  basic_program(USED_GL_VERSION* const context,
                GLuint ID): gl_context{context}, program_ID{ID}{}
  virtual ~basic_program() = default;
  virtual void makeUnderUse(void){
      gl_context->glUseProgram(program_ID);
  }
  virtual void detachProgram(void){
      gl_context->glUseProgram(0);
  }
};

class static_program : public basic_program
{
protected:
    GLint transformMatLocation;
public:
    static_program(USED_GL_VERSION* const context,
                   GLuint ID, GLint matLocation):
        basic_program(context, ID),
        transformMatLocation{matLocation}{}
    virtual ~static_program() override = default;

};

class animated_program : public static_program
{
protected:
    GLint inverse_transpose_transformMatLocation;
public:
    animated_program(USED_GL_VERSION* const context,
                     GLuint ID, GLint matLocation,
                     GLint itt_matLocation):
        static_program(context, ID, matLocation),
        inverse_transpose_transformMatLocation{itt_matLocation}{}
    virtual ~animated_program() override = default;

};

glm::vec3 rCol(void);

template<class T>
std::ostream& operator<<(std::ostream&out, const glm::tvec3<T>&vec){
    unsigned def_per = out.precision();
    out.precision(std::numeric_limits<T>::max_digits10);
    return out << std::fixed <<
         " X: " << vec.x <<
         ", Y: " << vec.y <<
         ", Z: " << vec.z << "\n" << std::flush;
    out.precision(def_per);
}

void _load_tex(QImage&, const QString&, const char*, bool, bool);

const char* DebugGLerr(unsigned);

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




