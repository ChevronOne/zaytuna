

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

GLint zaytuna::external_obj::transformMatLocation=-1;
GLint zaytuna::coord_sys::transformMatLocation=-1;
GLint zaytuna::grid_plane::transformMatLocation=-1;
GLint zaytuna::skybox_obj::transformMatLocation=-1;


zaytuna::scene_object::scene_object(USED_GL_VERSION * const _widg,
                                    const GLuint programID,
                                    const std::string& _name,
                                    const glm::dmat4 _rotaion,
                                    const glm::dmat4 _translation
                                    ):
    _widg{_widg},
    _programID{programID},
    name{_name},
    initial_rotaionMat{_rotaion},
    initial_translationMat{_translation},
    initial_transformationMat{_translation*_rotaion}{}


//==================================================================================================


zaytuna::external_obj::external_obj(USED_GL_VERSION * const _widg,
                                    const GLuint programID,
                                    const std::string& _name,
                                    const std::string& _dir,
                                    const std::string& _tex,
                                    const GLenum _MODE,
                                    const glm::dmat4 _rotaion,
                                    const glm::dmat4 _translation):
    scene_object(_widg, programID, _name,
                 _rotaion, _translation ),
    MODE{_MODE}
{
    primitives = shape_maker<zaytuna::vertexL1_16>::extractExternal(_dir);

    QImage tex_buffer;
    _widg->glGenTextures(1, &_texID);
    _widg->glBindTexture(GL_TEXTURE_2D, _texID);
    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_WRAP_S,
                           GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_WRAP_T,
                           GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_MIN_FILTER,
                           GL_LINEAR);
    _widg->glTexParameteri(GL_TEXTURE_2D,
                           GL_TEXTURE_MAG_FILTER,
                           GL_LINEAR);
    _load_tex(tex_buffer, _tex.c_str(), "JPG", 0, 0);
    _widg->glTexImage2D(GL_TEXTURE_2D, 0,
                        GL_RGBA, tex_buffer.width(),
                        tex_buffer.height(), 0, GL_RGBA,
                        GL_UNSIGNED_BYTE, tex_buffer.bits());

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

void external_obj::carry_data(GLintptr& _offset)
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

}

void external_obj::parse_VertexArraysObject(const GLuint& theBufferID,
                                            GLuint& off_set)
{
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
    transformationMat = activeCam->transformationMat;
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
                              const glm::dmat4 _rotaion,
                              const glm::dmat4 _translation):
        scene_object(_widg, programID, _name,
                     _rotaion, _translation ),
        LINE_WIDTH{line_width}
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

void coord_sys::carry_data(GLintptr& _offset)
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

}

void coord_sys::parse_VertexArraysObject(const GLuint& theBufferID,
                                         GLuint& off_set)
{
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

    _widg->glLineWidth(1.0f);
}

void coord_sys::render_obj(zaytuna::camera const*const activeCam)
{
    _widg->glUseProgram(_programID);
    transformationMat = activeCam->transformationMat;
    _widg->glBindVertexArray(_VAO_ID);

    glLineWidth(LINE_WIDTH);
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
                              const glm::dmat4 _rotaion,
                              const glm::dmat4 _translation):
        scene_object(_widg, programID, _name,
                     _rotaion, _translation ),
        LINE_WIDTH{line_width}
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

void grid_plane::carry_data(GLintptr& _offset)
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

}

void grid_plane::parse_VertexArraysObject(const GLuint& theBufferID,
                                          GLuint& off_set)
{
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

    _widg->glLineWidth(1.0f);
}

void grid_plane::render_obj(zaytuna::camera const*const activeCam)
{
    _widg->glUseProgram(_programID);
    transformationMat = activeCam->transformationMat;
    _widg->glBindVertexArray(_VAO_ID);

    glLineWidth(LINE_WIDTH);
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
                                    const glm::dmat4 _rotaion,
                                    const glm::dmat4 _translation):
    scene_object(_widg, programID, _name,
                 _rotaion, _translation ),
    MODE{_MODE}
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
    glGenTextures(1, &_texID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, _texID);
    glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_MAG_FILTER,
                    GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_WRAP_S,
                    GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_WRAP_T,
                    GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_WRAP_R,
                    GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP,
                    GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_CUBE_MAP,
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

void skybox_obj::carry_data(GLintptr& _offset)
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

}

void skybox_obj::parse_VertexArraysObject(const GLuint& theBufferID,
                                          GLuint& off_set)
{
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



}  // namespace zaytuna




