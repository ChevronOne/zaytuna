

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







#ifndef ZAY_ITEM_HPP
#define ZAY_ITEM_HPP



#include "zay_headers.hpp"
#include "zay_vertex.hpp"
#include "zay_shape_maker.hpp"
#include "zay_utility.hpp"
#include "zay_cam.hpp"
#include "zay_model_vehicle.hpp"
#include <boost/ptr_container/ptr_vector.hpp>







namespace zaytuna{

class primary_win;

class scene_object
{
protected:

    USED_GL_VERSION* _widg{nullptr};
    GLuint _programID;
    GLuint _VAO_ID;
    GLuint inds_offset;
    GLsizei num_indices;

    glm::dmat4 initial_rotationMat{
                   glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0))
               };
    glm::dmat4 initial_translationMat{
                   glm::translate(glm::dvec3(0.0, 0.0, 0.0))
               };
    glm::dmat4 initial_transformationMat{
                   glm::translate(glm::dvec3(0.0, 0.0, 0.0))
               };

    virtual void clean_up(void) = 0;

public:

    scene_object() = default;
    scene_object(USED_GL_VERSION * const,
                 const GLuint,
                 const glm::dmat4 _rotaion =
                        glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation =
                        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
                 );

    virtual GLsizeiptr buffer_size(void) const = 0;

    virtual void transmit_data(GLintptr&,const GLuint&,
                            GLuint&) = 0;
    virtual void render_obj(zaytuna::camera const*const) = 0;
    virtual ~scene_object() = default;
};



// ---------------------------------------------

class external_obj : public scene_object
{

private:

    std::string name{"uninitialized_object_name"};
    shape_data<zaytuna::vertexL1_16> primitives;
    GLuint _texID;
    GLenum MODE;
    glm::mat4 transformationMat{
                glm::translate(glm::dvec3(0.0, 0.0, 0.0))
              };
    static GLint transformMatLocation;

    virtual void clean_up(void) override;
public:

    external_obj() = default;
    external_obj(USED_GL_VERSION * const,
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

    virtual void transmit_data(GLintptr&,const GLuint&,
                            GLuint&) override;
    virtual void render_obj(zaytuna::camera const*const) override;
    virtual GLsizeiptr buffer_size(void) const override;

};



// --------------------------------------------

class coord_sys : public scene_object
{

private:
    std::string name{"uninitialized_object_name"};
    shape_data<zaytuna::vertexL1_12> primitives;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;
    GLfloat LINE_WIDTH;
    virtual void clean_up(void) override;
public:

    coord_sys() = default;
    coord_sys(USED_GL_VERSION * const,
                 const GLuint,
                 const std::string&,
                 const GLfloat axes_length = 10.f,
                 const GLfloat line_width = 1.5f,
                 const glm::dmat4 _rotaion =
                        glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation =
                        glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    virtual ~coord_sys() override;

    virtual void transmit_data(GLintptr&,const GLuint&,
                            GLuint&) override;
    virtual void render_obj(zaytuna::camera const*const) override;
    virtual GLsizeiptr buffer_size(void) const override;

};



//-----------------------------------------

class grid_plane : public scene_object
{

private:
    std::string name{"uninitialized_object_name"};
    shape_data<zaytuna::vertexL1_12> primitives;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;
    GLfloat LINE_WIDTH;
    virtual void clean_up(void) override;
public:

    grid_plane() = default;
    grid_plane(USED_GL_VERSION * const,
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

    virtual void transmit_data(GLintptr&,const GLuint&,
                            GLuint&) override;
    virtual void render_obj(zaytuna::camera const*const) override;
    virtual GLsizeiptr buffer_size(void) const override;

};




// ----------------------------------------------

class skybox_obj : public scene_object
{

private:
    std::string name{"uninitialized_object_name"};
    shape_data<zaytuna::vertexL1_0> primitives;
    GLuint _texID;
    GLenum MODE;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;
    virtual void clean_up(void) override;

public:

    skybox_obj() = default;
    skybox_obj(USED_GL_VERSION * const,
                 const GLuint,
                 const std::string&,
                 const GLenum MODE = GL_TRIANGLES,
                 const glm::dmat4 _rotaion =
                        glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                 const glm::dmat4 _translation =
                        glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    virtual ~skybox_obj() override;

    virtual void transmit_data(GLintptr&,const GLuint&,
                            GLuint&) override;
    virtual void render_obj(zaytuna::camera const*const) override;
    virtual GLsizeiptr buffer_size(void) const override;

};

//---------------------------------------------------------------


class model_vehicle : public scene_object
{
    shape_data<zaytuna::vertexL1_16> model_primitives;
    shape_data<zaytuna::vertexL1_16> fronttires_primitives;
    shape_data<zaytuna::vertexL1_16> backtires_primitives;
    shape_data<zaytuna::vertexL1_16> lidar_primitives;


    // model transformation matrix
    glm::mat4 modeltransformMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };

    // the inverse of the transposed transformation matrix
    glm::mat4 inverse_transpose_transformMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };

    static GLint transformMatLocation;
    static GLint inverse_transpose_transformMatLocation;

    QString prims_dir, tex_dir;

    GLuint _texID;
    GLenum PRIMITIVES_TYPE;

    GLuint fronttiresVAO_ID;
    GLuint backtiresVAO_ID;
    GLuint lidarVAO_ID;

    GLuint fronttires_indOffset;
    GLuint backtires_indOffset;
    GLuint lidar_indOffset;

    GLsizei fronttiresNumIndices;
    GLsizei backtiresNumIndices;
    GLsizei lidarNumIndices;

//    ptr_vector<vehicle_attribute*> vehicles;
    boost::ptr_vector<vehicle_attribute> vehicles;

    boost::ptr_vector<vehicle_attribute>::iterator find(const std::string&);


    virtual void clean_up(void) override;


    friend class _scene_widg;
    friend class primary_win;

public:
    model_vehicle() = default;
//    explicit model_vehicle(USED_GL_VERSION * const,
//                 const GLuint,
//                 const std::string&,
//                 const std::string&,
//                 const std::string&,
//                 QGLFramebufferObject *const,
//                 const GLenum PRIMITIVES_TYPE = GL_TRIANGLES,
//                 const glm::dmat4 _rotaion = glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
//                 const glm::dmat4 _translation =glm::translate(glm::dvec3(0.0, 0.0, 0.0)));

    explicit model_vehicle(USED_GL_VERSION * const,
                 const GLuint,
                 const std::string&,
                 const std::string&,
                 const GLenum PRIMITIVES_TYPE = GL_TRIANGLES);

    virtual ~model_vehicle() override;

    virtual void transmit_data(GLintptr&, const GLuint&,
                            GLuint&) override;
    virtual void render_obj(zaytuna::camera const*const) override;
    virtual GLsizeiptr buffer_size(void) const override;
    void add_vehicle(const std::string&,
                     QGLFramebufferObject *const,
                     const glm::dmat4 _rotaion = glm::rotate(0.0, glm::dvec3(0.0, 1.0, 0.0)),
                     const glm::dmat4 _translation =glm::translate(glm::dvec3(0.0, 0.0, 0.0)));
    void delete_vehicle(void*);



    // useful for debugging
    void render_vectors_state(vehicle_attribute*, zaytuna::camera*);
};


} // namespace zaytuna



#endif // ZAY_ITEM_HPP




