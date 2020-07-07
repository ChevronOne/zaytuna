

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



#include "zay_headers.hpp"
#include "zay_vertex.hpp"
#include "zay_shape_maker.hpp"
#include "zay_utility.hpp"
#include "zay_cam.hpp"

namespace zaytuna{

typedef uint16_t _itm0_indRange_t;
typedef uint16_t _itm1_indRange_t;
typedef uint16_t _itm2_indRange_t;


class scene_object
{
protected:

    QOpenGLFunctions_3_0* _widg{nullptr};
    GLuint _programID;
    GLuint _VAO_ID;
    std::string name{"uninitialized object name"};
    GLuint inds_offset;
    GLsizei num_indices;

    glm::dmat4 initial_rotaionMat{
                   glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0))
               };
    glm::dmat4 initial_translationMat{
                   glm::translate(glm::dvec3(0.0, 0.0, 0.0))
               };
    glm::dmat4 initial_transformationMat{
                   glm::translate(glm::dvec3(0.0, 0.0, 0.0))
               };


public:

    scene_object() = default;
    scene_object(QOpenGLFunctions_3_0 * const,
                 const GLuint,
                 const std::string&,
                 const glm::dmat4,
                 const glm::dmat4
                 );

    virtual GLsizeiptr buffer_size(void) const = 0;
    virtual void clean_up(void) = 0;
    virtual void carry_data(GLintptr&) = 0;
    virtual void parse_VertexArraysObject(const GLuint&, GLuint&) = 0;
    virtual void render_obj(zaytuna::camera*) = 0;
    virtual ~scene_object() = default;
};



// ---------------------------------------------

class external_obj : public scene_object
{

private:

    shape_data<zaytuna::vertexL1_16> primitives;
    GLuint _texID;
    GLenum MODE;
    glm::mat4 transformationMat{
                glm::translate(glm::dvec3(0.0, 0.0, 0.0))
              };
    static GLint transformMatLocation;


public:

    external_obj() = default;
    external_obj(QOpenGLFunctions_3_0 * const,
                 const GLuint,
                 const std::string&,
                 const std::string&,
                 const std::string&,
                 const GLenum MODE = GL_TRIANGLES,
                 const glm::dmat4 _rotaion =
                        glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation =
                        glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    virtual ~external_obj() override;
    virtual void clean_up(void) override;
    virtual void carry_data(GLintptr&) override;
    virtual void parse_VertexArraysObject(const GLuint&,
                                          GLuint&) override;
    virtual void render_obj(zaytuna::camera*) override;
    virtual GLsizeiptr buffer_size(void) const override;

};



// --------------------------------------------

class coord_sys : public scene_object
{

private:

    shape_data<zaytuna::vertexL1_12> primitives;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;
    GLfloat LINE_WIDTH;

public:

    coord_sys() = default;
    coord_sys(QOpenGLFunctions_3_0 * const,
                 const GLuint,
                 const std::string&,
                 const GLfloat axes_length = 10.f,
                 const GLfloat line_width = 1.5f,
                 const glm::dmat4 _rotaion =
                        glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation =
                        glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    virtual ~coord_sys() override;
    virtual void clean_up(void) override;
    virtual void carry_data(GLintptr&) override;
    virtual void parse_VertexArraysObject(const GLuint&,
                                          GLuint&) override;
    virtual void render_obj(zaytuna::camera*) override;
    virtual GLsizeiptr buffer_size(void) const override;

};



//-----------------------------------------

class grid_plane : public scene_object
{

private:

    shape_data<zaytuna::vertexL1_12> primitives;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;
    GLfloat LINE_WIDTH;

public:

    grid_plane() = default;
    grid_plane(QOpenGLFunctions_3_0 * const,
                 const GLuint,
                 const std::string&,
                 const GLfloat length = 150.f,
                 const GLfloat width = 150.f,
                 const GLfloat tessellation = 1.0f,
                 const GLfloat line_width = 1.0f,
                 const glm::dmat4 _rotaion =
                        glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation =
                        glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    virtual ~grid_plane() override;
    virtual void clean_up(void) override;
    virtual void carry_data(GLintptr&) override;
    virtual void parse_VertexArraysObject(const GLuint&,
                                          GLuint&) override;
    virtual void render_obj(zaytuna::camera*) override;
    virtual GLsizeiptr buffer_size(void) const override;

};




// ----------------------------------------------

class skybox_obj : public scene_object
{

private:

    shape_data<zaytuna::vertexL1_0> primitives;
    GLuint _texID;
    GLenum MODE;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;


public:

    skybox_obj() = default;
    skybox_obj(QOpenGLFunctions_3_0 * const,
                 const GLuint,
                 const std::string&,
                 const GLenum MODE = GL_TRIANGLES,
                 const glm::dmat4 _rotaion =
                        glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation =
                        glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    virtual ~skybox_obj() override;
    virtual void clean_up(void) override;
    virtual void carry_data(GLintptr&) override;
    virtual void parse_VertexArraysObject(const GLuint&,
                                          GLuint&) override;
    virtual void render_obj(zaytuna::camera*) override;
    virtual GLsizeiptr buffer_size(void) const override;

};




} // namespace zaytuna



#endif // ZAY_ITEM_HPP




