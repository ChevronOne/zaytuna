

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







#include "zay_item.hpp"
namespace zaytuna
{

GLint zaytuna::external_obj::transformMatLocation=-1;
GLint zaytuna::coord_sys::transformMatLocation=-1;
GLint zaytuna::grid_plane::transformMatLocation=-1;
GLint zaytuna::skybox_obj::transformMatLocation=-1;

GLint zaytuna::model_vehicle::transformMatLocation=-1;
GLint zaytuna::model_vehicle::inverse_transpose_transformMatLocation=-1;



scene_object::scene_object
    (USED_GL_VERSION * const _widg,
     const GLuint programID,
     const glm::dmat4 _rotation,
     const glm::dmat4 _translation ):

    _widg{_widg},
    _programID{programID},
    initial_rotationMat{_rotation},
    initial_translationMat{_translation},
    initial_transformationMat{_translation*_rotation}{}


//==================================================================================================


external_obj::external_obj
        (USED_GL_VERSION * const _widg,
         const GLuint programID,
         const std::string& _name,
         const std::string& _dir,
         const std::string& _tex,
         const GLenum _MODE,
         const glm::dmat4 _rotation,
         const glm::dmat4 _translation):

    scene_object(_widg, programID,
                 _rotation, _translation ),
    name{_name}, MODE{_MODE}
{
    primitives = shape_maker
            <zaytuna::vertexL1_16>::extractExternal(_dir);

    QImage tex_buffer;
    _widg->glGenTextures(1, &_texID);
    _widg->glBindTexture(GL_TEXTURE_2D, _texID);

    GLfloat max_anisotropic_extention{0};
    _widg->glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT,
                       &max_anisotropic_extention);
    _widg->glTexParameterf(GL_TEXTURE_2D,
                           GL_TEXTURE_MAX_ANISOTROPY_EXT,
                           max_anisotropic_extention);

    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_WRAP_S,
                           GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_WRAP_T,
                           GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_MIN_FILTER,
                           GL_LINEAR_MIPMAP_LINEAR);
    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_MAG_FILTER,
                           GL_LINEAR);
    _load_tex(tex_buffer, _tex.c_str(), "JPG", 0, 0);
    _widg->glTexImage2D(GL_TEXTURE_2D, 0,
                        GL_RGBA, tex_buffer.width(),
                        tex_buffer.height(), 0, GL_RGBA,
                        GL_UNSIGNED_BYTE, tex_buffer.bits());
    _widg->glGenerateMipmap(GL_TEXTURE_2D);


}

external_obj::~external_obj()
{
    clean_up();
}

void external_obj::clean_up()
{
    primitives.cleanUP();
    _widg->glDeleteVertexArrays(1, &_VAO_ID);
    _widg->glDeleteTextures(1, &_texID);
}

void external_obj::transmit_data(GLintptr& _offset,const GLuint& theBufferID,
                              GLuint& off_set)
{
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.verBufSize(),
                           primitives.verts);
    _offset += primitives.verBufSize();
    inds_offset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.indBufSize(),
                           primitives.indices);
    _offset += primitives.indBufSize();

    num_indices = static_cast<GLsizei>(primitives.indNum);

    //================================================
    _widg->glGenVertexArrays(1, &_VAO_ID);

    _widg->glBindVertexArray(_VAO_ID);
    _widg->glEnableVertexAttribArray(0);
    _widg->glEnableVertexAttribArray(1);
    _widg->glEnableVertexAttribArray(2);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

    _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set));
    _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 3));
    _widg->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 6));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

    off_set += primitives.verBufSize()
            + primitives.indBufSize();

    if(transformMatLocation == -1)
        transformMatLocation
                = _widg->glGetUniformLocation(_programID, "transformMat");


    primitives.cleanUP();

}


void external_obj::render_obj(zaytuna::camera const*const activeCam)
{
    _widg->glUseProgram(_programID);
    transformationMat = activeCam->transformationMat * initial_transformationMat;
    _widg->glBindVertexArray(_VAO_ID);
    _widg->glBindTexture(GL_TEXTURE_2D, _texID);
    _widg->glUniformMatrix4fv(transformMatLocation, 1,
                              GL_FALSE,
                              &transformationMat[0][0]);
    _widg->glDrawElements(MODE, num_indices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(inds_offset));
}

GLsizeiptr external_obj::buffer_size() const
{
    return primitives.verBufSize()
           + primitives.indBufSize();
}

// // -------

zaytuna::coord_sys::coord_sys(USED_GL_VERSION * const _widg,
                              const GLuint programID,
                              const std::string& _name,
                              const GLfloat axes_lenght,
                              const GLfloat line_width,
                              const glm::dmat4 _rotation,
                              const glm::dmat4 _translation):
        scene_object(_widg, programID,
                     _rotation, _translation ),
        name{_name}, LINE_WIDTH{line_width}
{
    primitives = shape_maker<zaytuna::vertexL1_12>::makeCoord(axes_lenght);

}

coord_sys::~coord_sys()
{
    this->clean_up();
}

void coord_sys::clean_up()
{
    primitives.cleanUP();
    _widg->glDeleteVertexArrays(1, &_VAO_ID);
}

void coord_sys::transmit_data(GLintptr& _offset,const GLuint& theBufferID,
                           GLuint& off_set)
{
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.verBufSize(),
                           primitives.verts);
    _offset += primitives.verBufSize();
    inds_offset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.indBufSize(),
                           primitives.indices);
    _offset += primitives.indBufSize();

    num_indices = static_cast<GLsizei>(primitives.indNum);

    //=======================================
    _widg->glGenVertexArrays(1, &_VAO_ID);

    _widg->glBindVertexArray(_VAO_ID);
    _widg->glEnableVertexAttribArray(0);
    _widg->glEnableVertexAttribArray(1);
    _widg->glEnableVertexAttribArray(2);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

    _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_0,
                                 reinterpret_cast<void*>(off_set));
    _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_0,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 3));
    _widg->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_0,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 6));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

    off_set += primitives.verBufSize()
            + primitives.indBufSize();

    if(transformMatLocation == -1)
        transformMatLocation =
                _widg->glGetUniformLocation(_programID, "transformMat");

    primitives.cleanUP();

}


void coord_sys::render_obj(zaytuna::camera const*const activeCam)
{
    _widg->glUseProgram(_programID);
    transformationMat = activeCam->transformationMat;
    _widg->glBindVertexArray(_VAO_ID);

    _widg->glLineWidth(LINE_WIDTH);
    _widg->glUniformMatrix4fv(transformMatLocation, 1,
                              GL_FALSE,
                              &transformationMat[0][0]);
    _widg->glDrawElements(GL_LINES, num_indices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(inds_offset));
}

GLsizeiptr coord_sys::buffer_size() const
{
    return primitives.verBufSize()
           + primitives.indBufSize();
}


// // --------------------

zaytuna::grid_plane::grid_plane(USED_GL_VERSION * const _widg,
                              const GLuint programID,
                              const std::string& _name,
                              const GLfloat length,
                              const GLfloat width,
                              const GLfloat tessellatio,
                              const GLfloat line_width,
                              const glm::dmat4 _rotation,
                              const glm::dmat4 _translation):
        scene_object(_widg, programID,
                     _rotation, _translation ),
        name{_name}, LINE_WIDTH{line_width}
{
    primitives =
            shape_maker<zaytuna::vertexL1_12>::makeGrid(length,
                                                        width, tessellatio);

}

grid_plane::~grid_plane()
{
    this->clean_up();
}

void grid_plane::clean_up()
{
    primitives.cleanUP();
    _widg->glDeleteVertexArrays(1, &_VAO_ID);
}

void grid_plane::transmit_data(GLintptr& _offset,const GLuint& theBufferID,
                            GLuint& off_set)
{
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.verBufSize(),
                           primitives.verts);
    _offset += primitives.verBufSize();
    inds_offset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.indBufSize(),
                           primitives.indices);
    _offset += primitives.indBufSize();

    num_indices = static_cast<GLsizei>(primitives.indNum);

    //=================================

    _widg->glGenVertexArrays(1, &_VAO_ID);

    _widg->glBindVertexArray(_VAO_ID);
    _widg->glEnableVertexAttribArray(0);
    _widg->glEnableVertexAttribArray(1);
    _widg->glEnableVertexAttribArray(2);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

    _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_0,
                                 reinterpret_cast<void*>(off_set));
    _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_0,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 3));
    _widg->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_0,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 6));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

    off_set += primitives.verBufSize()
            + primitives.indBufSize();

    if(transformMatLocation == -1)
        transformMatLocation =
                _widg->glGetUniformLocation(_programID, "transformMat");

    primitives.cleanUP();

}


void grid_plane::render_obj(zaytuna::camera const*const activeCam)
{
    _widg->glUseProgram(_programID);
    transformationMat = activeCam->transformationMat;
    _widg->glBindVertexArray(_VAO_ID);

    _widg->glLineWidth(LINE_WIDTH);
    _widg->glUniformMatrix4fv(transformMatLocation,
                              1, GL_FALSE,
                              &transformationMat[0][0]);
    _widg->glDrawElements(GL_LINES, num_indices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(inds_offset));
}

GLsizeiptr grid_plane::buffer_size() const
{
    return primitives.verBufSize()
           + primitives.indBufSize();
}

// // -----------------


zaytuna::skybox_obj::skybox_obj(USED_GL_VERSION * const _widg,
                                    const GLuint programID,
                                    const std::string& _name,
                                    const GLenum _MODE,
                                    const glm::dmat4 _rotation,
                                    const glm::dmat4 _translation):
    scene_object(_widg, programID,
                 _rotation, _translation ),
    name{_name}, MODE{_MODE}
{
    primitives = shape_maker<zaytuna::vertexL1_0>::makeCubemap();

    QImage tex_buffer;
    std::vector<QString> faces={
        "tex/skybox-jpg/right.jpg",
        "tex/skybox-jpg/left.jpg",
        "tex/skybox-jpg/top.jpg",
        "tex/skybox-jpg/bottom.jpg",
        "tex/skybox-jpg/front.jpg",
        "tex/skybox-jpg/back.jpg" };
    _widg->glGenTextures(1, &_texID);
    _widg->glBindTexture(GL_TEXTURE_CUBE_MAP, _texID);
    _widg->glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR);
    _widg->glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_MAG_FILTER,
                    GL_LINEAR);
    _widg->glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_WRAP_S,
                    GL_CLAMP_TO_EDGE);
    _widg->glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_WRAP_T,
                    GL_CLAMP_TO_EDGE);
    _widg->glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_WRAP_R,
                    GL_CLAMP_TO_EDGE);
    _widg->glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_BASE_LEVEL, 0);
    _widg->glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_MAX_LEVEL, 0);
    for (GLuint i = 0; i < 6; ++i){
        _load_tex(tex_buffer, faces[i], "JPG", 0, 1);
        _widg->glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+i,
                     0, 3, tex_buffer.width(),
                     tex_buffer.height(), 0,
                     GL_RGBA, GL_UNSIGNED_BYTE,
                     tex_buffer.bits());
    }
}

skybox_obj::~skybox_obj()
{
    clean_up();
}

void skybox_obj::clean_up()
{
    primitives.cleanUP();
    _widg->glDeleteVertexArrays(1, &_VAO_ID);
    _widg->glDeleteTextures(1, &_texID);
}

void skybox_obj::transmit_data(GLintptr& _offset,const GLuint& theBufferID,
                            GLuint& off_set)
{
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.verBufSize(),
                           primitives.verts);
    _offset += primitives.verBufSize();
    inds_offset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           primitives.indBufSize(),
                           primitives.indices);
    _offset += primitives.indBufSize();

    num_indices = static_cast<GLsizei>(primitives.indNum);

    //======================
    _widg->glGenVertexArrays(1, &_VAO_ID);

    _widg->glBindVertexArray(_VAO_ID);

    _widg->glEnableVertexAttribArray(0);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);
    _widg->glVertexAttribPointer(0, 3, GL_FLOAT,
                                 GL_FALSE, 0,
                                 reinterpret_cast<void*>(off_set));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                        theBufferID);

    off_set += primitives.verBufSize()
            + primitives.indBufSize();


    if(transformMatLocation == -1)
        transformMatLocation =
                _widg->glGetUniformLocation(_programID, "transformMat");


    primitives.cleanUP();

}


void skybox_obj::render_obj(zaytuna::camera const*const activeCam)
{
    _widg->glDepthFunc(GL_LEQUAL);
    _widg->glUseProgram(_programID);
    _widg->glBindVertexArray(_VAO_ID);
    _widg->glBindTexture(GL_TEXTURE_CUBE_MAP, _texID);
    transformationMat = activeCam->transformationMat
            * glm::translate(activeCam->camera_position);
    _widg->glUniformMatrix4fv(transformMatLocation,
                              1, GL_FALSE,
                              &transformationMat[0][0]);
    _widg->glDrawElements(GL_TRIANGLES, num_indices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(inds_offset));
    _widg->glDepthFunc(GL_LESS);
}

GLsizeiptr skybox_obj::buffer_size() const
{
    return primitives.verBufSize()
           + primitives.indBufSize();
}

//======================================================================================================
//======================================================================================================

zaytuna::model_vehicle::model_vehicle(USED_GL_VERSION * const _widg,
                                    const GLuint programID,
                                    const std::string& _dir,
                                    const std::string& _tex,
                                    const GLenum _MODE):
    scene_object(_widg, programID),
    prims_dir{_dir.c_str()}, tex_dir{_tex.c_str()}, PRIMITIVES_TYPE{_MODE}
{

    model_primitives = shape_maker
            <zaytuna::vertexL1_16>::extractExternal
            (prims_dir.toStdString());
    fronttires_primitives = shape_maker
            <zaytuna::vertexL1_16>::extractExternal
            (prims_dir.toStdString()+"-single_tire");
    backtires_primitives = shape_maker
            <zaytuna::vertexL1_16>::extractExternal
            (prims_dir.toStdString()+"-back_tires");
    lidar_primitives = shape_maker
            <zaytuna::vertexL1_16>::extractExternal
            (prims_dir.toStdString()+"-lidar");

}

void model_vehicle::add_vehicle(const std::string& _name,
                                QGLFramebufferObject *const FBO_,
                                const glm::dmat4 _rotation,
                                const glm::dmat4 _translation){
    vehicles.push_back(new vehicle_attribute(_widg, _name, FBO_,
                                             _rotation,
                                             _translation));
}

model_vehicle::~model_vehicle(){
    clean_up();
    _widg->glDeleteVertexArrays(1, &_VAO_ID);
    _widg->glDeleteVertexArrays(1, &fronttiresVAO_ID);
    _widg->glDeleteVertexArrays(1, &backtiresVAO_ID);
    _widg->glDeleteVertexArrays(1, &lidarVAO_ID);

    _widg->glDeleteTextures(1, &_texID);

}

void model_vehicle::clean_up(){
    model_primitives.cleanUP();
    fronttires_primitives.cleanUP();
    backtires_primitives.cleanUP();
    lidar_primitives.cleanUP();
}


void model_vehicle::transmit_data(GLintptr& _offset,
                               const GLuint& theBufferID,
                               GLuint& off_set)
{
    // model
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           model_primitives.verBufSize(),
                           model_primitives.verts);
    _offset += model_primitives.verBufSize();
    inds_offset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           model_primitives.indBufSize(),
                           model_primitives.indices);
    _offset += model_primitives.indBufSize();

    num_indices = static_cast<GLsizei>(model_primitives.indNum);

    // front-tires
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           fronttires_primitives.verBufSize(),
                           fronttires_primitives.verts);
    _offset += fronttires_primitives.verBufSize();
    fronttires_indOffset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           fronttires_primitives.indBufSize(),
                           fronttires_primitives.indices);
    _offset += fronttires_primitives.indBufSize();

    fronttiresNumIndices = static_cast<GLsizei>(fronttires_primitives.indNum);

    // back-tires
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           backtires_primitives.verBufSize(),
                           backtires_primitives.verts);
    _offset += backtires_primitives.verBufSize();
    backtires_indOffset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           backtires_primitives.indBufSize(),
                           backtires_primitives.indices);
    _offset += backtires_primitives.indBufSize();

    backtiresNumIndices = static_cast<GLsizei>(backtires_primitives.indNum);

    // lidar
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           lidar_primitives.verBufSize(),
                           lidar_primitives.verts);
    _offset += lidar_primitives.verBufSize();
    lidar_indOffset = static_cast<GLuint>(_offset);
    _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                           lidar_primitives.indBufSize(),
                           lidar_primitives.indices);
    _offset += lidar_primitives.indBufSize();

    lidarNumIndices = static_cast<GLsizei>(lidar_primitives.indNum);

    //==============================================================

    // model
    _widg->glGenVertexArrays(1, &_VAO_ID);

    _widg->glBindVertexArray(_VAO_ID);
    _widg->glEnableVertexAttribArray(0);
    _widg->glEnableVertexAttribArray(1);
    _widg->glEnableVertexAttribArray(2);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

    _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set));
    _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 3));
    _widg->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 6));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

    off_set+= model_primitives.verBufSize()
            + model_primitives.indBufSize();

    // front-tires
    _widg->glGenVertexArrays(1, &fronttiresVAO_ID);

    _widg->glBindVertexArray(fronttiresVAO_ID);
    _widg->glEnableVertexAttribArray(0);
    _widg->glEnableVertexAttribArray(1);
    _widg->glEnableVertexAttribArray(2);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

    _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set));
    _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 3));
    _widg->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 6));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

    off_set+= fronttires_primitives.verBufSize()
            + fronttires_primitives.indBufSize();

    // back-tires
    _widg->glGenVertexArrays(1, &backtiresVAO_ID);

    _widg->glBindVertexArray(backtiresVAO_ID);
    _widg->glEnableVertexAttribArray(0);
    _widg->glEnableVertexAttribArray(1);
    _widg->glEnableVertexAttribArray(2);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

    _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set));
    _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 3));
    _widg->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 6));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

    off_set+= backtires_primitives.verBufSize()
            + backtires_primitives.indBufSize();

    // lidar
    _widg->glGenVertexArrays(1, &lidarVAO_ID);

    _widg->glBindVertexArray(lidarVAO_ID);
    _widg->glEnableVertexAttribArray(0);
    _widg->glEnableVertexAttribArray(1);
    _widg->glEnableVertexAttribArray(2);
    _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

    _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set));
    _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 3));
    _widg->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                                 VERTEX_BYTE_SIZE_1,
                                 reinterpret_cast<void*>(off_set + TYPE_SIZE * 6));
    _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

    off_set+= lidar_primitives.verBufSize()
            + lidar_primitives.indBufSize();



    if(transformMatLocation == -1)
        transformMatLocation =
                _widg->glGetUniformLocation(_programID, "transformMat");
    if(inverse_transpose_transformMatLocation == -1)
        inverse_transpose_transformMatLocation =
                _widg->glGetUniformLocation(_programID, "it_transformMat");

    clean_up();

    QImage tex_buffer;
    _widg->glGenTextures(1, &_texID);
    _widg->glBindTexture(GL_TEXTURE_2D, _texID);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    if(!tex_buffer.load(tex_dir, "PNG")){
         std::cerr << "image couldn't be loaded <" << tex_dir.toStdString() << ">!\n";
         exit(EXIT_FAILURE);
    }
    tex_buffer = QGLWidget::convertToGLFormat(tex_buffer);
    _widg->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_buffer.width(),
                 tex_buffer.height(), 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, tex_buffer.bits());

}



GLsizeiptr model_vehicle::buffer_size() const
{
    return model_primitives.verBufSize()
           + model_primitives.indBufSize()
           + fronttires_primitives.verBufSize()
           + fronttires_primitives.indBufSize()
           + backtires_primitives.verBufSize()
           + backtires_primitives.indBufSize()
           + lidar_primitives.verBufSize()
           + lidar_primitives.indBufSize();
}


void model_vehicle::render_obj(zaytuna::camera const*const activeCam)
{
    if(vehicles.size() == 0)
        return;

    _widg->glUseProgram(_programID);
    _widg->glBindTexture(GL_TEXTURE_2D, _texID);


    uint32_t i{0};

    // the model
    _widg->glBindVertexArray(_VAO_ID);
    for(i=0; i<vehicles.size(); ++i){
        modeltransformMat = activeCam->transformationMat
                * vehicles[i].transformationMats[0];
        inverse_transpose_transformMat =
                glm::inverse(glm::transpose(vehicles[i].transformationMats[0]));

        _widg->glUniformMatrix4fv(transformMatLocation, 1,
                                  GL_FALSE, glm::value_ptr(modeltransformMat));
        _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                                  1, GL_FALSE,
                                  &inverse_transpose_transformMat[0][0]);
        _widg->glDrawElements(PRIMITIVES_TYPE, num_indices,
                              GL_UNSIGNED_INT,
                              reinterpret_cast<void*>(inds_offset));
    }



    // front-tires
    _widg->glBindVertexArray(fronttiresVAO_ID);
    for(i=0; i<vehicles.size(); ++i){
        modeltransformMat = activeCam->transformationMat
                            * vehicles[i].transformationMats[1];
        inverse_transpose_transformMat =
                glm::inverse(glm::transpose(vehicles[i].transformationMats[1]));
        _widg->glUniformMatrix4fv(transformMatLocation, 1,
                                  GL_FALSE, &modeltransformMat[0][0]);
        _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                                  1, GL_FALSE,
                                  &inverse_transpose_transformMat[0][0]);
        _widg->glDrawElements(PRIMITIVES_TYPE, fronttiresNumIndices,
                              GL_UNSIGNED_INT,
                              reinterpret_cast<void*>(fronttires_indOffset));

        modeltransformMat = activeCam->transformationMat
                            * vehicles[i].transformationMats[2];
        inverse_transpose_transformMat
                = glm::inverse(glm::transpose(vehicles[i].transformationMats[2]));
        _widg->glUniformMatrix4fv(transformMatLocation, 1,
                                  GL_FALSE, &modeltransformMat[0][0]);
        _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                                  1, GL_FALSE,
                                  &inverse_transpose_transformMat[0][0]);
        _widg->glDrawElements(PRIMITIVES_TYPE, fronttiresNumIndices,
                              GL_UNSIGNED_INT,
                              reinterpret_cast<void*>(fronttires_indOffset));
    }



    // back-tires
    _widg->glBindVertexArray(backtiresVAO_ID);
    for(i=0; i<vehicles.size(); ++i){
        modeltransformMat = activeCam->transformationMat
                            * vehicles[i].transformationMats[3];
        inverse_transpose_transformMat
                = glm::inverse(glm::transpose(vehicles[i].transformationMats[3]));
        _widg->glUniformMatrix4fv(transformMatLocation, 1,
                                  GL_FALSE, &modeltransformMat[0][0]);
        _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                                  1, GL_FALSE,
                                  &inverse_transpose_transformMat[0][0]);
        _widg->glDrawElements(PRIMITIVES_TYPE, backtiresNumIndices,
                              GL_UNSIGNED_INT,
                              reinterpret_cast<void*>(backtires_indOffset));
    }

    // lidar
    _widg->glBindVertexArray(lidarVAO_ID);
    for(i=0; i<vehicles.size(); ++i){
        modeltransformMat = activeCam->transformationMat
                            * vehicles[i].transformationMats[4];
        inverse_transpose_transformMat =
                glm::inverse(glm::transpose(vehicles[i].transformationMats[4]));
        _widg->glUniformMatrix4fv(transformMatLocation, 1,
                                  GL_FALSE, &modeltransformMat[0][0]);
        _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                                  1, GL_FALSE,
                                  &inverse_transpose_transformMat[0][0]);
        _widg->glDrawElements(PRIMITIVES_TYPE, lidarNumIndices,
                              GL_UNSIGNED_INT,
                              reinterpret_cast<void*>(lidar_indOffset));
    }

}

boost::ptr_vector<vehicle_attribute>::iterator
model_vehicle::find(const std::string& _name)
{
    boost::ptr_vector<vehicle_attribute>::iterator it;
    for(it = vehicles.begin(); it!=vehicles.end(); ++it)
        if( (*it).name == _name)
            return it;
    return it;
}


// // useful for debugging
void model_vehicle::render_vectors_state(vehicle_attribute* vehicle, camera *activeCam)
{
    _widg->glUseProgram(0);
    _widg->glLineWidth(5.0f);
    glm::vec4 origin = activeCam->transformationMat * glm::vec4(0.f,0.f,0.f,1.f);
    glm::vec4 head;
    _widg->glBegin(GL_LINES);


    // front-cam position and view-dir
    // ------------------------------------------------
    _widg->glColor4f(0.0f, 0.0f, 1.0f, 1.f);
    _widg->glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(vehicle->frontCam.camera_position,1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);

    _widg->glColor4f(0.3f, 0.8f, 0.3f, 1.f);
    _widg->glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(vehicle->frontCam.view_direction,1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);


    // // ideal tires
    // // ------------------------------------------------
    _widg->glColor4f(0.3f, 0.8f, 0.3f, 1.f);
    _widg->glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(vehicle->back_ideal_tire,1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);

    _widg->glColor4f(0.2f, 0.6f, 0.2f, 1.f);
    _widg->glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(vehicle->front_ideal_tire,1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);

    // // vehic_direction
    // // ------------------------------------------------
    _widg->glColor4f(0.8f, 0.3f, 0.3f, 1.f);
    _widg->glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(vehicle->vehic_direction,1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);


    // // center of rotation
    // // ------------------------------------------------
    _widg->glColor4f(1.f, 0.0f, 0.0f, 1.f);
    _widg->glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(vehicle->center_of_rotation,1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);

    _widg->glColor4f(0.8f, 0.3f, 0.3f, 1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);
    head = activeCam->transformationMat
            * glm::vec4(vehicle->back_ideal_tire,1.f);
    _widg->glVertex4f(head.x, head.y, head.z, head.w);


    _widg->glEnd();
    _widg->glFlush();

}

//====================================================================
//====================================================================




}  // namespace zaytuna




