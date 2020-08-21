

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
#include "zay_shape_data.hpp"

#include <vector>
#include <boost/assign/list_of.hpp>
#include <QImage>

#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/include/support_istream_iterator.hpp>
//#include <boost/iostreams/device/mapped_file.hpp>
//#include "boost/spirit/include/support_iso8859_1.hpp"
namespace x3 = boost::spirit::x3;

#ifndef BOOST_SPIRIT_X3_SEMANTIC_ACTION

#include <boost/fusion/adapted/struct.hpp>
#include <boost/fusion/include/as_vector.hpp>
#endif

namespace zaytuna {
enum class TEX_TYPE { TEX_CUBE_MAP, TEX_2D, TEX_2D_MIPMAP};

struct obj_parser{
    static shape_data<zaytuna::vertexL1_16> 
        extractExternal(const std::string&);
};

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
template <class allocator>
struct zay_vec3 : public geometry_msgs::Vector3_<allocator>{
    inline zay_vec3<allocator>& operator=(const glm::dvec3& glm_vec){
        this->x = glm_vec.x;
        this->y = glm_vec.y;
        this->z = glm_vec.z;
        return *this; }
};

template <class allocator>
struct zay_uint32 : public std_msgs::UInt32_<allocator>{
    inline zay_uint32<allocator>& operator=(const uint32_t& val){
        this->data = val;
        return *this; }
};

template<class allocator>
struct zay_geo_pose : public geometry_msgs::Pose_<allocator> {
    void update(const glm::dvec3& pos,
                const glm::dvec3& dir){
        this->position.x = pos.x;
        this->position.y = pos.y;
        this->position.z = pos.z;
        double h_angle{atan2(dir.x, dir.z)/2.0};
        this->orientation.y = sin(h_angle);
        this->orientation.w = cos(h_angle);
    }
};


enum class Obstacle_Type { CARTON_BOX, BRICK_WALL, STONE_WALL };
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
        return this->translationMat()*this->rotationMat();
    }
};

template<class T>
struct obstacle_attribs : public transform_attribs<T>
{
    Obstacle_Type type{Obstacle_Type::STONE_WALL};
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
    GLint inversed_transpose_transformMatLocation;
public:
    animated_program(USED_GL_VERSION* const context,
                     GLuint ID, GLint matLocation,
                     GLint itt_matLocation):
        static_program(context, ID, matLocation),
        inversed_transpose_transformMatLocation{itt_matLocation}{}
    virtual ~animated_program() override = default;

};

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

void _read_tex(QImage&, const std::string&, const char*, bool, bool);

const char* DebugGLerr(unsigned);

struct OBJ {
    std::vector<glm::vec3> positions, normals;
    std::vector<glm::vec2> texCoords;
    std::vector<uint32_t> faces;
};

void _load_tex(USED_GL_VERSION * const _widg,
              GLuint&,
              const std::string&,
              TEX_TYPE,
              const char* _format=nullptr,
              bool h_mirroring=0,
              bool v_mirroring=0);


} // namespace zaytuna


#endif // ZAY_UTILITY_HPP




