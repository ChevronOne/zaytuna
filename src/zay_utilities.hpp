

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



#ifndef ZAY_UTILITIES_HPP
#define ZAY_UTILITIES_HPP



#include "zay_shape_maker.hpp"

#include <boost/assign/list_of.hpp>

#include <boost/spirit/home/x3.hpp>
#include <boost/spirit/include/support_istream_iterator.hpp>
//#include <boost/iostreams/device/mapped_file.hpp>
//#include "boost/spirit/include/support_iso8859_1.hpp"
namespace x3 = boost::spirit::x3;

#ifndef ZAY_BOOST_SPIRIT_X3_SEMANTIC_ACTION

#include <boost/fusion/adapted/struct.hpp>
#include <boost/fusion/include/as_vector.hpp>
#endif


namespace zaytuna {
enum class ZAY_TEX_TYPE { TEX_CUBE_MAP, TEX_2D, TEX_2D_MIPMAP};
enum class ZAY_GL_OBJECT_TYPE { PROGRAM, SHADER };
struct obj_parser{
    typedef glm::vec<ZAY_POINT_D, GLdouble, glm::qualifier::packed_highp> vert;

    static shape_data<zaytuna::vertexL1_16> 
        extractExternal(const std::string&);
    static void extractProjectionRect(const std::string&, 
                                      boost::array<vert, ZAY_RECTANGLE_P>&);
};


template <class allocator>
struct vec3 : public geometry_msgs::Vector3_<allocator>{

    template<class v_type>
    inline vec3<allocator>& operator=(const glm::vec<ZAY_POINT_D, v_type, glm::qualifier::packed_highp>& vec_){
        this->x = vec_.x;
        this->y = vec_.y;
        this->z = vec_.z;
        return *this; }
};


template <class allocator>
struct uint32 : public std_msgs::UInt32_<allocator>{
    inline uint32<allocator>& operator=(const uint32_t& val){
        this->data = val;
        return *this; }
};


template <class allocator>
struct Bool : public std_msgs::Bool_<allocator>{
    inline Bool<allocator>& operator=(const bool& val){
        this->data = val;
        return *this; }
};

template<class v_type, class allocator>
geometry_msgs::Point_<allocator> glm2gPoint(const glm::vec<ZAY_POINT_D, v_type, glm::qualifier::packed_highp>& vec)
{
    geometry_msgs::Point_<allocator> gPoint;
    gPoint.x = vec.x;
    gPoint.y = vec.y;
    gPoint.z = vec.z;

    return gPoint;
}

template<class allocator>
struct geo_pose : public zaytuna::Pose_<allocator>{
    template<class v_type>
    void update(const glm::vec<ZAY_POINT_D, v_type, glm::qualifier::packed_highp>& bIdealT,
                const glm::vec<ZAY_POINT_D, v_type, glm::qualifier::packed_highp>& fIdealT,
                const glm::vec<ZAY_POINT_D, v_type, glm::qualifier::packed_highp>& dir_){

        this->position.back_ideal_tire = glm2gPoint<v_type, allocator>(bIdealT);
        this->position.front_ideal_tire = glm2gPoint<v_type, allocator>(fIdealT);

        double h_angle{atan2(dir_.x, dir_.z)/2.0};
        this->orientation.y = sin(h_angle);
        this->orientation.w = cos(h_angle);
    }
};


enum class Obstacle_Type { CARTON_BOX, BRICK_WALL, STONE_WALL };
template<class T>
struct obj_transform_attribs
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

    obj_transform_attribs() = default;
    obj_transform_attribs(const std::string& name,
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
struct obstacle_attribs : public obj_transform_attribs<T>
{
    Obstacle_Type type{Obstacle_Type::STONE_WALL};
    obstacle_attribs() = default;
    obstacle_attribs(Obstacle_Type type,
                     const std::string& name,
                     T angle,
                     glm::tvec3<T> r_vec,
                     glm::tvec3<T> t_vec):
        obj_transform_attribs<T>(name, angle, r_vec, t_vec),
        type{type}{}
};


template<class T>
struct veh_transform_attribs : public obj_transform_attribs<T>
{
    T front_cam_v_angle{static_cast<T>(0.0)};
    veh_transform_attribs() = default;
    veh_transform_attribs(T cam_v_angle,
                const std::string& name,
                T angle,
                glm::tvec3<T> r_vec,
                glm::tvec3<T> t_vec):
        obj_transform_attribs<T>(name, angle, r_vec, t_vec),
        front_cam_v_angle{cam_v_angle}{}
};


template<class T>
struct default_settings{
    std::vector<veh_transform_attribs<T>> vehicles;
    std::vector<obstacle_attribs<T>> obstacles;
    void clear(){
        this->vehicles.clear();
        this->obstacles.clear();
    }
};


template<class T> 
struct rect_collistion_object{
    typedef glm::vec<ZAY_POINT_D, T, glm::qualifier::packed_highp> coll_vert;
    std::string ID{"uninitialized_collistion_object_name"};
    boost::array<coll_vert, ZAY_RECTANGLE_P> points;
    boost::array<coll_vert, ZAY_RECT_LIN_INDEPENDENT_V> uniqueV;

};


template<class T> 
struct rect_collistion_pack{
    typedef std::reference_wrapper<rect_collistion_object<T>> _ref_Obj;
    std::vector<rect_collistion_object<T>> static_objs;
    // std::vector<_ref_Obj> dyn_objs;
    std::vector<rect_collistion_object<T>*> dyn_objs;
    
    
    inline void erase_obs(const std::string& _name){
        static_objs.erase(find_obs(_name));
    }

    inline void erase_veh(const std::string& _name){
        auto it = dyn_objs.begin();
        for(;it!=dyn_objs.end();++it)
            if((*it)->ID ==_name){
                dyn_objs.erase(it);
                break;
            }
    }

    inline class std::vector<rect_collistion_object<T>>::iterator 
    find_obs(const std::string& _name){
        auto it = static_objs.begin();
        for(;it!=static_objs.end();++it)
            if(it->ID ==_name)
                return it;
        return it;
    }

    inline void edit_obs(const obstacle_attribs<GLdouble>& attribs){
        auto it = find_obs(attribs.name);
        if(it==static_objs.end())
            return;
        glm::dmat4 _M{attribs.transformMat()};
        for(glm::dvec3& vec:it->points)
            vec = _M*glm::dvec4(vec, 1.0);
        
        it->uniqueV[0]=it->points[1]-it->points[0];
        it->uniqueV[1]=it->points[2]-it->points[1];
    }

};

enum class Collider_type{COLLIDER, COUNTERPART};
template<class T>
struct projection_1d{
    T proj_1d;
    zaytuna::Collider_type c_type;
    projection_1d(T p): proj_1d{p}, c_type{zaytuna::Collider_type::COLLIDER}{}
    projection_1d(T p, zaytuna::Collider_type t):proj_1d{p}, c_type{t}{}
    inline bool operator<(const projection_1d<T> &_other) const{
        return proj_1d < _other.proj_1d;
    }
};

class basic_program
{
protected:
    ZAY_USED_GL_VERSION* gl_context{nullptr};
    GLuint program_ID;

public:
  basic_program(ZAY_USED_GL_VERSION* const context,
                const std::string& _source): gl_context{context}
                {init(_source);}
  virtual ~basic_program(){
      this->detachProgram();
      gl_context->glDeleteProgram(program_ID);
  }
  virtual void makeUnderUse(void){
      gl_context->glUseProgram(program_ID);
  }
  virtual void detachProgram(void){
      gl_context->glUseProgram(0);
  }
  virtual GLuint program_handler(void) const{
      return program_ID;
  }
  void init(const std::string&);
  static void get_source(const std::string&, std::string&);
  GLuint compile_shader(const std::string&, GLenum);
  void checkErrors(GLuint, GLuint,
                   ZAY_GL_OBJECT_TYPE, 
                   const std::string&);
  virtual void get_attrib_locations(){
      gl_context->glBindAttribLocation(program_ID, 0, "vertPos");
  }
};


class static_program : public basic_program{
public:
    static_program(ZAY_USED_GL_VERSION* const context,
                   const std::string& _source):
        basic_program(context, _source){}
    virtual ~static_program() override {};
    virtual void get_attrib_locations() override{
        gl_context->glBindAttribLocation(program_ID, 0, "vertPos");
        gl_context->glBindAttribLocation(program_ID, 1, "vertColor");
    }
};


class animated_program : public basic_program{
public:
    animated_program(ZAY_USED_GL_VERSION* const context,
                     const std::string& _source):
        basic_program(context, _source){}
    virtual ~animated_program() override {};
    virtual void get_attrib_locations() override{
        gl_context->glBindAttribLocation(program_ID, 0, "vertPos");
        gl_context->glBindAttribLocation(program_ID, 1, "vertNorm");
        gl_context->glBindAttribLocation(program_ID, 2, "texCoor");
    }
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


template<class T> struct vehicle_state{
    T MOVEMENT_SPEED{0.0};
    T STEERING_WHEEL{ZAY_STEERING_MARGIN_OF_ERROR};
};


void _read_tex(QImage&, const std::string&, const char*, bool, bool);
const char* DebugGLerr(unsigned);

struct OBJ {
    std::vector<glm::vec3> positions, normals;
    std::vector<glm::vec2> texCoords;
    std::vector<uint32_t> faces;
};


void _load_tex(ZAY_USED_GL_VERSION * const _widg,
              GLuint&,
              const std::string&,
              ZAY_TEX_TYPE,
              const char* _format=nullptr,
              bool h_mirroring=0,
              bool v_mirroring=0);

struct obstacle_tracker{
    const char* category;
    uint32_t counter;
};



} // namespace zaytuna


#endif // ZAY_UTILITIES_HPP




