

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





#include "zay_model_vehicle.hpp"

namespace zaytuna {

glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c,
                  const double angle, const bool ccw);

glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2,
                     const glm::dvec3 p3, const glm::dvec3 p4);

GLint zaytuna::model_vehicle::transformMatLocation=-1;
GLint zaytuna::model_vehicle::inverse_transpose_transformMatLocation=-1;


zaytuna::model_vehicle::model_vehicle(QOpenGLFunctions_3_0 * const _widg,
                                    const GLuint programID,
                                    const std::string& _name,
                                    const std::string& _dir,
                                    const std::string& _tex,
                                    const GLenum _MODE,
                                    const glm::dmat4 _rotaion,
                                    const glm::dmat4 _translation):
    scene_object(_widg, programID, _name,
                 _rotaion, _translation ),
    MODE{_MODE}, AMOUNT_OF_ROTATION(0.0), MOVEMENT_SPEED(0.0),
    STEERING_WHEEL(0.00000001), accumulated_dist(0.0), traveled_dist(0.0),
    ticks_counter(0), radius_of_rotation(0.0), center_of_rotation(glm::dvec3(0.0,0.0,0.0))
{

    model_primitives = shape_maker<zaytuna::vertexL1_16>::extractExternal(_dir);
    fronttires_primitives = shape_maker<zaytuna::vertexL1_16>::extractExternal(_dir+"-single_tire");
    backtires_primitives = shape_maker<zaytuna::vertexL1_16>::extractExternal(_dir+"-back_tires");
    lidar_primitives = shape_maker<zaytuna::vertexL1_16>::extractExternal(_dir+"-lidar");

    QImage tex_buffer;
    _widg->glGenTextures(1, &_texID);
    _widg->glBindTexture(GL_TEXTURE_2D, _texID);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    _widg->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    if(!tex_buffer.load(_tex.c_str(), "PNG")){
         std::cout << "image couldn't be loaded <" << _tex << ">!\n";
         exit(EXIT_FAILURE);
    }
    tex_buffer = QGLWidget::convertToGLFormat(tex_buffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_buffer.width(),
                 tex_buffer.height(), 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, tex_buffer.bits());



    update_positional_attributes(_translation, _rotaion);

    transformationMats[1] = f_rightT;
    transformationMats[2] = f_leftT;
    transformationMats[3] = backT;
    transformationMats[4] = lidar;

}

void model_vehicle::update_positional_attributes(const glm::dmat4 &translation_,
                                                 const glm::dmat4 &rotation_)
{
    initial_transformationMat = translation_*rotation_;
    initial_translationMat = translation_;
    initial_rotaionMat = rotation_;

    vehic_direction = glm::dvec3(1.0, 0.0, 0.0);
    back_ideal_tire = old_back_ideal_tire = glm::dvec3(0.0, 0.0, 0.0);
    front_ideal_tire = glm::dvec3(0.2862 , 0.0 , 0.0);

    transformationMats[0] = initial_transformationMat;
    rotationMat = glm::rotate(0.0, up_direction);

    vehic_direction = glm::dmat3(initial_rotaionMat) * vehic_direction;
    frontCam.view_direction  = glm::dmat3(initial_rotaionMat)
                               * frontCam.view_direction;

    back_ideal_tire = old_back_ideal_tire = initial_transformationMat
                                        * glm::vec4(back_ideal_tire, 1.0);
    front_ideal_tire = initial_transformationMat
                        * glm::vec4(front_ideal_tire, 1.0);

    frontCam.camera_position = initial_transformationMat * camPos;
    frontCam.view_direction  = initial_rotaionMat
                            * (glm::dvec4(0.5 , 0.17 , 0.0, 1.0) - camPos);

    tires_hRotation = glm::rotate(amount_of_hRotation,
                                  glm::dvec3(0.0, 0.0, 1.0));
    front_tires_vRotation = glm::rotate(STEERING_WHEEL,
                                        glm::dvec3(0.0, 1.0, 0.0));
}


void model_vehicle::update_attribs()
{
    if(MOVEMENT_SPEED != 0.0){

        update_rotation_att();
        accumulated_dist = traveled_dist + accumulated_dist;
        ticks_counter = static_cast<uint32_t>(accumulated_dist/meters_per_tick);
        accumulated_dist = fmod(accumulated_dist, meters_per_tick);
    } else
        timer_t = std::chrono::high_resolution_clock::now();
    actuate();
}

void model_vehicle::update_rotation_att()
{
    // the radius of rotation
    radius_of_rotation = std::abs( front_back_distance/std::tan(STEERING_WHEEL));

    // center of rotaion
    if(STEERING_WHEEL > 0)
        center_of_rotation =
                (glm::normalize(glm::cross(vehic_direction, up_direction))
                 * radius_of_rotation)
                + back_ideal_tire;
    else
        center_of_rotation =
                (-glm::normalize(glm::cross(vehic_direction, up_direction))
                 * radius_of_rotation)
                + back_ideal_tire;


    elapsed_t = std::chrono::duration<double,
            std::ratio< 1, 1>>
            (std::chrono::high_resolution_clock::now() - timer_t).count();
    timer_t = std::chrono::high_resolution_clock::now();

    // amount of rotaion
    AMOUNT_OF_ROTATION = (std::tan(STEERING_WHEEL)
                          * MOVEMENT_SPEED * elapsed_t)/front_back_distance;


    rotationMat =  glm::translate(center_of_rotation)
            * glm::rotate(glm::radians(AMOUNT_OF_ROTATION), up_direction)
            * glm::translate(-center_of_rotation);

    transformationMats[0] = rotationMat * transformationMats[0];

    old_back_ideal_tire = back_ideal_tire;

    back_ideal_tire =  rotationMat * glm::dvec4(back_ideal_tire, 1.0);
    front_ideal_tire =  rotationMat * glm::dvec4(front_ideal_tire, 1.0);

    traveled_dist = glm::radians(std::abs(AMOUNT_OF_ROTATION))
                    * radius_of_rotation;

    frontCam.camera_position = transformationMats[0] * camPos;



    vehic_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                            up_direction))
                      * vehic_direction;
    frontCam.view_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                                    up_direction))
                              * frontCam.view_direction;


}

void model_vehicle::actuate()
{
    if(std::abs(STEERING_WHEEL) < 0.0001){
        transformationMats[1] =  transformationMats[0] * f_rightT;
        transformationMats[2] = transformationMats[0] * f_leftT;
    }else{
        front_tires_vRotation = glm::rotate(-STEERING_WHEEL,
                                            glm::dvec3(0.0, 1.0, 0.0));
        transformationMats[1] =  transformationMats[0]
                                 * f_rightT * front_tires_vRotation;
        transformationMats[2] = transformationMats[0]
                                * f_leftT * front_tires_vRotation;
    }

    transformationMats[3] = transformationMats[0]  * backT;

    if(MOVEMENT_SPEED != 0.0){
        if(MOVEMENT_SPEED < 0.0)
            amount_of_hRotation -= traveled_dist/tires_radius;
        else
            amount_of_hRotation += traveled_dist/tires_radius;

        if(amount_of_hRotation>PI2)
            amount_of_hRotation -= PI2;
        else if (amount_of_hRotation< -PI2)
            amount_of_hRotation+=PI2;

        tires_hRotation = glm::rotate(amount_of_hRotation,
                                      glm::dvec3(0.0, 0.0, 1.0));

    }
    transformationMats[1] = glm::dmat4(transformationMats[1]) * tires_hRotation;
    transformationMats[2] = glm::dmat4(transformationMats[2]) * tires_hRotation;
    transformationMats[3] = glm::dmat4(transformationMats[3]) * tires_hRotation;


    transformationMats[4] = transformationMats[0] * lidar
            * glm::rotate(glm::radians(amount_of_rotation_Lidar <= -360 ?
                amount_of_rotation_Lidar = 0
                : amount_of_rotation_Lidar -= lidar_spin_speed),
                  glm::dvec3(0.0, 1.0, 0.0));
}


model_vehicle::~model_vehicle()
{
    clean_up();
}

void model_vehicle::clean_up()
{
    model_primitives.cleanUP();
    fronttires_primitives.cleanUP();
    backtires_primitives.cleanUP();
    lidar_primitives.cleanUP();

    _widg->glDeleteVertexArrays(1, &_VAO_ID);
    _widg->glDeleteVertexArrays(1, &fronttiresVAO_ID);
    _widg->glDeleteVertexArrays(1, &backtiresVAO_ID);
    _widg->glDeleteVertexArrays(1, &lidarVAO_ID);

    _widg->glDeleteTextures(1, &_texID);
}

void model_vehicle::carry_data(GLintptr& _offset)
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

}

void model_vehicle::parse_VertexArraysObject(const GLuint& theBufferID,
                                             GLuint& off_set)
{
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


    model_primitives.cleanUP();
    fronttires_primitives.cleanUP();
    backtires_primitives.cleanUP();
    lidar_primitives.cleanUP();

    timer_t = std::chrono::high_resolution_clock::now();

}

void model_vehicle::render_obj(zaytuna::camera *activeCam)
{

    modeltransformMat = activeCam->transformationMat
            * transformationMats[0];
    inverse_transpose_transformMat =
            glm::inverse(glm::transpose(transformationMats[0]));

    _widg->glUseProgram(_programID);
    _widg->glBindTexture(GL_TEXTURE_2D, _texID);


    // the model
    _widg->glBindVertexArray(_VAO_ID);
    _widg->glUniformMatrix4fv(transformMatLocation, 1,
                              GL_FALSE, glm::value_ptr(modeltransformMat));
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                              1, GL_FALSE,
                              &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(MODE, num_indices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(inds_offset));


    // front-tires
    _widg->glBindVertexArray(fronttiresVAO_ID);
    modeltransformMat = activeCam->transformationMat
                        * transformationMats[1];
    inverse_transpose_transformMat =
            glm::inverse(glm::transpose(transformationMats[1]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1,
                              GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                              1, GL_FALSE,
                              &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(MODE, fronttiresNumIndices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(fronttires_indOffset));

    modeltransformMat = activeCam->transformationMat
                        * transformationMats[2];
    inverse_transpose_transformMat
            = glm::inverse(glm::transpose(transformationMats[2]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1,
                              GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                              1, GL_FALSE,
                              &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(MODE, fronttiresNumIndices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(fronttires_indOffset));


    // back-tires
    _widg->glBindVertexArray(backtiresVAO_ID);
    modeltransformMat = activeCam->transformationMat
                        * transformationMats[3];
    inverse_transpose_transformMat
            = glm::inverse(glm::transpose(transformationMats[3]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1,
                              GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                              1, GL_FALSE,
                              &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(MODE, backtiresNumIndices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(backtires_indOffset));


    // lidar
    _widg->glBindVertexArray(lidarVAO_ID);
    modeltransformMat = activeCam->transformationMat
                        * transformationMats[4];
    inverse_transpose_transformMat =
            glm::inverse(glm::transpose(transformationMats[4]));
    _widg->glUniformMatrix4fv(transformMatLocation, 1,
                              GL_FALSE, &modeltransformMat[0][0]);
    _widg->glUniformMatrix4fv(inverse_transpose_transformMatLocation,
                              1, GL_FALSE,
                              &inverse_transpose_transformMat[0][0]);
    _widg->glDrawElements(MODE, lidarNumIndices,
                          GL_UNSIGNED_INT,
                          reinterpret_cast<void*>(lidar_indOffset));


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


//----------------------------------

// old approache
glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c,
                  const double angle, const bool ccw){
    double _sin = std::sin(angle);
    double _cos = std::cos(angle);
    p -= c;

    if(ccw)// ccw: counter clockwise rotation
        p = glm::dvec3(  p.x * _cos + p.z * _sin, 0.0f,
                         p.z * _cos - p.x * _sin);

    else // cw: clockwise rotation
        p = glm::dvec3(  p.x * _cos - p.z * _sin, 0.0f,
                         p.z * _cos + p.x * _sin);


    return p+c;
}

glm::dvec3 intersect(const glm::dvec3 p1,
                     const glm::dvec3 p2,
                     const glm::dvec3 p3,
                     const glm::dvec3 p4){
    double temp = ( (p4.x - p3.x)*(p1.z-p3.z) - (p4.z-p3.z)*(p1.x-p3.x) ) /
            ((p4.z - p3.z) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.z - p1.z));

    return glm::dvec3{ p1.x + temp * (p2.x - p1.x), 0.0f, p1.z + temp * (p2.z - p1.z) };
}


void model_vehicle::update_steerin()
{

}

void model_vehicle::update_cent()
{
        glm::dvec3 p1, p2;

        if(STEERING_WHEEL > 0){
            p1 = rotate(front_ideal_tire,
                        back_ideal_tire,
                        -M_PI_2, true);
            p2 = rotate(back_ideal_tire,
                        front_ideal_tire,
                        -(M_PI_2-STEERING_WHEEL), false);
        }else{
            p1 = rotate(front_ideal_tire,
                        back_ideal_tire,
                        M_PI_2, false);
            p2 = rotate(back_ideal_tire,
                        front_ideal_tire,
                        M_PI_2-STEERING_WHEEL, true);
        }

        // center of rotaion
        center_of_rotation = intersect(back_ideal_tire, p1,
                                       front_ideal_tire, p2);
}


// // useful for debugging
void model_vehicle::render_vectors_state(camera *activeCam)
{
    _widg->glUseProgram(0);
    glLineWidth(5.0f);
    glm::vec4 origin = activeCam->transformationMat * glm::vec4(0.f,0.f,0.f,1.f);
    glm::vec4 head;
    glBegin(GL_LINES);


    // front-cam position and view-dir
    // ------------------------------------------------
    glColor4f(0.0f, 0.0f, 1.0f, 1.f);
    glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(frontCam.camera_position,1.f);
    glVertex4f(head.x, head.y, head.z, head.w);

    glColor4f(0.3f, 0.8f, 0.3f, 1.f);
    glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(frontCam.view_direction,1.f);
    glVertex4f(head.x, head.y, head.z, head.w);


    // // ideal tires
    // // ------------------------------------------------
    glColor4f(0.3f, 0.8f, 0.3f, 1.f);
    glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(back_ideal_tire,1.f);
    glVertex4f(head.x, head.y, head.z, head.w);

    glColor4f(0.2f, 0.6f, 0.2f, 1.f);
    glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(front_ideal_tire,1.f);
    glVertex4f(head.x, head.y, head.z, head.w);

    // // vehic_direction
    // // ------------------------------------------------
    glColor4f(0.8f, 0.3f, 0.3f, 1.f);
    glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(vehic_direction,1.f);
    glVertex4f(head.x, head.y, head.z, head.w);


    // // center of rotation
    // // ------------------------------------------------
    glColor4f(1.f, 0.0f, 0.0f, 1.f);
    glVertex4f(origin.x, origin.y, origin.z, origin.w);
    head = activeCam->transformationMat
            * glm::vec4(center_of_rotation,1.f);
    glVertex4f(head.x, head.y, head.z, head.w);

    glColor4f(0.8f, 0.3f, 0.3f, 1.f);
    glVertex4f(head.x, head.y, head.z, head.w);
    head = activeCam->transformationMat
            * glm::vec4(back_ideal_tire,1.f);
    glVertex4f(head.x, head.y, head.z, head.w);




    glEnd();
    glFlush();

}



} // namespace  zaytuna





