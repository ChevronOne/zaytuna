

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







#ifndef ZAY_ITEM_HPP
#define ZAY_ITEM_HPP



#include "zay_model_vehicle.hpp"
#include <boost/ptr_container/ptr_vector.hpp>


namespace zaytuna{



class primary_win;

class scene_object
{


protected:

    ZAY_USED_GL_VERSION* _widg{nullptr};
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
    scene_object(ZAY_USED_GL_VERSION * const,
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
    external_obj(ZAY_USED_GL_VERSION * const,
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



class coord_sys : public scene_object
{

private:

    std::string name{"uninitialized_object_name"};
    shape_data<zaytuna::vertexL1_1> primitives;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;
    GLfloat LINE_WIDTH;
    virtual void clean_up(void) override;


public:

    coord_sys() = default;
    coord_sys(ZAY_USED_GL_VERSION * const,
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



class grid_plane : public scene_object
{

private:

    std::string name{"uninitialized_object_name"};
    shape_data<zaytuna::vertexL1_1> primitives;
    glm::mat4 transformationMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };
    static GLint transformMatLocation;
    GLfloat LINE_WIDTH;
    virtual void clean_up(void) override;


public:

    grid_plane() = default;
    grid_plane(ZAY_USED_GL_VERSION * const,
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
    skybox_obj(ZAY_USED_GL_VERSION * const,
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



class model_vehicle : public scene_object
{

    shape_data<zaytuna::vertexL1_16> model_primitives;
    shape_data<zaytuna::vertexL1_16> fronttires_primitives;
    shape_data<zaytuna::vertexL1_16> backtires_primitives;
    shape_data<zaytuna::vertexL1_16> lidar_primitives;
    rect_collistion_object<GLdouble>  coll_obj;
    rect_collistion_pack<GLdouble>* coll_pack;


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

    std::string prims_dir, tex_dir;

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

    boost::ptr_vector<vehicle_attributes> vehicles;

    boost::ptr_vector<vehicle_attributes>::iterator
    find(const std::string&);
    virtual void clean_up(void) override;

    friend class _scene_widg;
    friend class primary_win;



public:

    model_vehicle() = default;
    explicit model_vehicle(ZAY_USED_GL_VERSION * const,
                 const GLuint,
                 const std::string&,
                 const std::string&,
                 const std::string&,
                 rect_collistion_pack<GLdouble>*,
                 const GLenum PRIMITIVES_TYPE = GL_TRIANGLES);

    virtual ~model_vehicle() override;

    virtual void transmit_data(GLintptr&, const GLuint&,
                            GLuint&) override;

    virtual void render_obj(zaytuna::camera const*const) override;
    virtual GLsizeiptr buffer_size(void) const override;

    void add_vehicle(QGLFramebufferObject *const,
                     const veh_transform_attribs<GLdouble>,
                     zaytuna::vehicle_state<GLdouble>*,
                     ZAY_MSG_LOGGER*);

    void delete_vehicle(void*);



    ////----------useful--for--debugging--------
    void render_vectors_state(vehicle_attributes*, zaytuna::camera const*const);

};



template<class T>
struct obstacle_instance{


    obstacle_attribs<T> attribs;

    glm::tmat4x4<T> transformMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };

    glm::mat4 inverse_transpose_transformMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };

    obstacle_instance
        (const obstacle_attribs<T>& attribs_){
            edit(attribs_);}

    void edit(const obstacle_attribs<T>& attribs_){
        this->attribs = attribs_;
        this->transformMat = attribs_.transformMat();
        inverse_transpose_transformMat = glm::inverse(glm::transpose(this->transformMat));
    }



};




template<class T>
struct obstacle_wrapper{

    ZAY_USED_GL_VERSION * _widg;
    Obstacle_Type type;
    shape_data<zaytuna::vertexL1_16> primitives;
    std::string prims_dir, tex_dir;
    GLuint _texID;
    GLuint _VAO_ID;
    GLuint inds_offset;
    GLsizei num_indices;
    std::vector<obstacle_instance<T>> instances;


    obstacle_wrapper() = default;
    obstacle_wrapper(ZAY_USED_GL_VERSION * const _widg,
                     Obstacle_Type type,
                     std::string prims_dir,
                     std::string tex_dir):
            _widg{_widg},
            type{type},
            prims_dir{prims_dir},
            tex_dir{tex_dir}{

        primitives = obj_parser::extractExternal(prims_dir);
        _load_tex(_widg, _texID, tex_dir, ZAY_TEX_TYPE::TEX_2D_MIPMAP,
                  "JPG", 0,0);

    }


    void clean_up(void){

        primitives.cleanUP();
    }


    GLsizeiptr buffer_size() const{

        return primitives.verBufSize()
               + primitives.indBufSize();

    }


    void transmit_data(GLintptr& _offset,const GLuint& theBufferID,
                                  GLuint& off_set)
    {

        _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                               primitives.verBufSize(),
                               primitives.vertices.data());


        _offset += primitives.verBufSize();
        inds_offset = static_cast<GLuint>(_offset);


        _widg->glBufferSubData(GL_ARRAY_BUFFER, _offset,
                               primitives.indBufSize(),
                               primitives.indices.data());

        _offset += primitives.indBufSize();

        num_indices = static_cast<GLsizei>(primitives.indices.size());

        _widg->glGenVertexArrays(1, &_VAO_ID);
        _widg->glBindVertexArray(_VAO_ID);
        _widg->glEnableVertexAttribArray(0);
        _widg->glEnableVertexAttribArray(1);
        _widg->glEnableVertexAttribArray(2);

        _widg->glBindBuffer(GL_ARRAY_BUFFER, theBufferID);

        _widg->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                     ZAY_VERTEX_BYTE_SIZE_1,
                                     reinterpret_cast<void*>(off_set));

        _widg->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                     ZAY_VERTEX_BYTE_SIZE_1,
                                     reinterpret_cast<void*>(off_set + ZAY_TYPE_SIZE * ZAY_NORMALS_STRIDE));

        _widg->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                                     ZAY_VERTEX_BYTE_SIZE_1,
                                     reinterpret_cast<void*>(off_set + ZAY_TYPE_SIZE * ZAY_TEXTURE_STRIDE));
        _widg->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, theBufferID);

        off_set += primitives.verBufSize()
                + primitives.indBufSize();

        primitives.cleanUP();

    }


    ~obstacle_wrapper(){
        clean_up();
        _widg->glDeleteVertexArrays(1, &_VAO_ID);
        _widg->glDeleteTextures(1, &_texID);}


};



template<class T>
struct obstacle_pack : public scene_object {

    GLenum PRIMITIVES_TYPE;
    glm::mat4 transformMat{
        glm::translate(glm::dvec3(0.0, 0.0, 0.0))
    };


    static GLint transformMatLocation;
    static GLint inverse_transpose_transformMatLocation;
    std::map<Obstacle_Type, std::shared_ptr<obstacle_wrapper<T>>> categories;


    /* this used only to get a linear iteration, otherwise 'categories' does the same! */
    std::vector<std::shared_ptr<obstacle_wrapper<T>>> obstacle_categories;
    
    std::map<std::string, Obstacle_Type> category;

    
    std::map<Obstacle_Type, rect_collistion_object<T>> rect_projections;

    class std::vector<obstacle_instance<T>>::iterator
            find(const std::string& _name){
        auto cat{&(categories[category[_name]]->instances)};
        auto begin{cat->begin()}, end{cat->end()};
        for(;begin!=end;++begin){

            if(begin->attribs.name == _name)
                return begin;

        }

        ROS_ERROR_STREAM("query was made for a non-existent object: <"<< _name.c_str() <<">, a possible bug! \n");
        ROS_FATAL_STREAM("Please report this with sufficient information on how the situation came about!");
        exit(EXIT_FAILURE);

    }


    obstacle_attribs<T> get_attribs(const std::string _name){

        return find(_name)->attribs;
    }

    void delete_obstacle(const std::string& _name){
#if __cplusplus > 201703L
        std::erase_if(categories[category[_name]]->instances,
            [=](obstacle_instance<T> instance)
            {return instance.attribs.name == _name; });
#else
        categories[category[_name]]->instances.erase(find(_name));
#endif
    }

    obstacle_pack(ZAY_USED_GL_VERSION * const _widg,
                  const GLuint programID,
                  Obstacle_Type type,
                  const std::string& prims_dir,
                  const std::string& proj_prims_dir,
                  const std::string& tex_dir,
                  GLenum PRIMITIVES_TYPE):
            scene_object(_widg, programID),
            PRIMITIVES_TYPE{PRIMITIVES_TYPE}
    {

        add_category(type, prims_dir, proj_prims_dir, tex_dir);

        if(transformMatLocation == -1)
            transformMatLocation =
                    _widg->glGetUniformLocation(_programID, "transformMat");

        if(inverse_transpose_transformMatLocation == -1)
            inverse_transpose_transformMatLocation =
                    _widg->glGetUniformLocation(_programID, "it_transformMat");

    }


    virtual ~obstacle_pack() override{}

    void add_category(Obstacle_Type type,
                      std::string prims_dir,
                      std::string proj_prims_dir,
                      std::string tex_dir){

        obstacle_categories.push_back
          (std::shared_ptr<obstacle_wrapper<T>>(new obstacle_wrapper<T>(_widg, type, prims_dir, tex_dir)));

        categories[type] = obstacle_categories.back();


        rect_collistion_object<T> coll_obj;
        obj_parser::extractProjectionRect(proj_prims_dir, coll_obj.points);
        rect_projections[type] = coll_obj;

    }


    virtual void transmit_data(GLintptr& _offset,
                       const GLuint& theBufferID,
                       GLuint& off_set) override{

        for(uint32_t i{0}; i<obstacle_categories.size(); ++i)
            obstacle_categories[i]->transmit_data(_offset, theBufferID, off_set);

    }


    virtual void render_obj(zaytuna::camera const*const activeCam) override{

        uint32_t category_{0}, instance{0};
        // this->_widg->glUseProgram(_programID);

        for(;category_<obstacle_categories.size(); ++category_){

            _widg->glBindTexture(GL_TEXTURE_2D, obstacle_categories[category_]->_texID);
            _widg->glBindVertexArray(obstacle_categories[category_]->_VAO_ID);
            for(instance=0;
                instance<obstacle_categories[category_]->instances.size();
                ++instance){

                transformMat = activeCam->transformationMat *
                        obstacle_categories[category_]->instances[instance].transformMat;

                _widg->glUniformMatrix4fv
                        (transformMatLocation, 1,
                         GL_FALSE, glm::value_ptr(transformMat));

                _widg->glUniformMatrix4fv
                        (inverse_transpose_transformMatLocation,
                         1, GL_FALSE, glm::value_ptr
                    (obstacle_categories[category_]->instances
                     [instance].inverse_transpose_transformMat));

                _widg->glDrawElements
                        (PRIMITIVES_TYPE, obstacle_categories[category_]->num_indices,
                         GL_UNSIGNED_INT,
                         reinterpret_cast<void*>
                         (obstacle_categories[category_]->inds_offset));

            }
        }
    }


    virtual GLsizeiptr buffer_size(void) const override{

        GLsizeiptr buff_size{0};
        for(uint32_t i{0}; i<obstacle_categories.size(); ++i)
            buff_size +=obstacle_categories[i]->buffer_size();

        return buff_size;

    }

    virtual void clean_up(void){}

};


template<class T>
GLint obstacle_pack<T>::transformMatLocation{-1};

template<class T>
GLint obstacle_pack<T>::inverse_transpose_transformMatLocation{-1};





} // namespace zaytuna





#endif // ZAY_ITEM_HPP




