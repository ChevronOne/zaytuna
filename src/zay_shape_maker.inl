

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




template <typename VERT>
shape_data<VERT>
shape_maker<VERT>::makeGrid(GLfloat length,
                            GLfloat width,
                            GLfloat tessellation)
{

    shape_data<VERT> plane;
    VERT v;

    for (float i = -ceil(width / 2.0f); i <= floor(width / 2.0f); i += tessellation)
    {
        v.position = glm::vec3(-ceil(length / 2), 0.0f, i);
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        plane.vertices.push_back(v);
        v.position = glm::vec3(floor(length / 2), 0.0f, i);
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        plane.vertices.push_back(v);
    }

    for (float i = -ceil(length / 2.0f); i <= floor(length / 2.0f); i += tessellation)
    {
        v.position = glm::vec3(i, 0.0f, -ceil(width / 2));
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        plane.vertices.push_back(v);
        v.position = glm::vec3(i, 0.0f, floor(width / 2));
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        plane.vertices.push_back(v);
    }

    plane.indices.resize(plane.vertices.size());
    std::iota(plane.indices.begin(), plane.indices.end(), 0);

    return plane;
}

template <typename VERT>
shape_data<VERT>
shape_maker<VERT>::makeCoord(GLfloat axes_length)
{
    shape_data<VERT> coord;
    coord.vertices.resize(6);

    if(axes_length<1.0f)
        axes_length = 1.0f;

    coord.vertices[0].position = glm::vec3(0.0f, 0.001f, 0.0f); // X coord
    coord.vertices[0].color = glm::vec3(0.0f, 0.0f, 1.0f);
//    coord.vertices[0].normal = glm::vec3(0.0f, 1.0f, 0.0f);

    coord.vertices[1].position = glm::vec3(axes_length, 0.001f, 0.0f);
    coord.vertices[1].color = glm::vec3(0.0f, 0.0f, 1.0f);
//    coord.vertices[1].normal = glm::vec3(10.0f, 1.0f, 0.0f);



    coord.vertices[2].position = glm::vec3(0.0f, 0.001f, 0.0f);  // Z coord
    coord.vertices[2].color = glm::vec3(0.0f, 1.0f, 0.0f);
//    coord.vertices[2].normal = glm::vec3(0.0f, 1.0f, 0.0f);

    coord.vertices[3].position = glm::vec3(0.0f, 0.001f, axes_length);
    coord.vertices[3].color = glm::vec3(0.0f, 1.0f, 0.0f);
//    coord.vertices[3].normal = glm::vec3(00.0f, 1.0f, -10.0f);



    coord.vertices[4].position = glm::vec3(0.0f, 0.001f, 0.0f); // Y coord
    coord.vertices[4].color = glm::vec3(1.0f, 0.0f, 0.0f);
//    coord.vertices[4].normal = glm::vec3(1.0f, 0.0f, 0.0f);

    coord.vertices[5].position = glm::vec3(0.0f, axes_length, 0.0f);
    coord.vertices[5].color = glm::vec3(1.0f, 0.0f, 0.0f);
//    coord.vertices[5].normal = glm::vec3(1.0f, 10.0f, 0.0f);

    
    coord.indices.resize(coord.vertices.size());
    std::iota(coord.indices.begin(), coord.indices.end(), 0);

    return coord;

}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeLap()
{
    shape_data<VERT> rectangle;
    rectangle.indices.assing({0, 1, 2, 3});
    rectangle.vertices.assing({glm::vec3(+0.0f, +0.0f, +0.0f),  // 0
                               glm::vec3(+0.0f, +1.0f, +0.0f),	// normal
                               glm::vec2(+1.0f, +1.0f),         // texcoor

                               glm::vec3(+0.0f, +0.0f, -4.3f),  // 1
                               glm::vec3(+0.0f, +1.0f, +0.0f),	// normal
                               glm::vec2(+1.0f, +0.0f),		    // texcoor

                               glm::vec3(+6.0f, +0.0f, -4.3f),  // 2
                               glm::vec3(+0.0f, +1.0f, +0.0f),  // normal
                               glm::vec2(+0.0f, +0.0f),         // texcoor

                               glm::vec3(+6.0f, +0.0f, +0.0f),  // 3
                               glm::vec3(+0.0f, +1.0f, +0.0f),  // normal
                               glm::vec2(+0.0f, +1.0f),         // texcoor
                            });
    return rectangle;
}

template <typename VERT>
shape_data<VERT>
shape_maker<VERT>::makeSphere(GLfloat PERS,
                              GLfloat RAD,
                              glm::vec3 CENT)
{

    shape_data<VERT> sphere;
    VERT v;

    for (float i = -M_PI / 2; i <= M_PI / 2; i += PERS)
    {
        for (float j = 0.0f; j <= M_PI * 2 + M_PI / 27; j += PERS)
        {
            v.position = glm::vec3(cosf(j) * cosf(i)
                                   * RAD, sinf(i)
                                   * RAD, sinf(j)
                                   * cosf(i)
                                   * RAD)
                                    + CENT;
            
            v.color = glm::vec3(
                    rand() / static_cast<float>(RAND_MAX),
                    rand() / static_cast<float>(RAND_MAX),
                    rand() / static_cast<float>(RAND_MAX) );

            v.normal = v.position;
            sphere.vertices.push_back(v);


            v.position = glm::vec3(cosf(j) * cosf(i + PERS)
                                   * RAD, sinf(i + PERS)
                                   * RAD, sinf(j)
                                   * cosf(i + PERS)
                                   * RAD)
                                   + CENT;
            
            v.color = glm::vec3(
                    rand() / static_cast<float>(RAND_MAX),
                    rand() / static_cast<float>(RAND_MAX),
                    rand() / static_cast<float>(RAND_MAX) );

            v.normal = v.position;
            sphere.vertices.push_back(v);
        }
    }

    sphere.indices.reserve(sphere.vertices.size());
    std::iota(sphere.indices.begin(), sphere.indices.end(), 0);

    return sphere;
}


template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeCubeMap() {

    shape_data<VERT> cube;
    cube.vertices.assign(
    {
        {glm::vec3(-1.0f, +1.0f, +1.0f)},  // 0 // Top
        {glm::vec3(+1.0f, +1.0f, +1.0f)},  // 1
        {glm::vec3(+1.0f, +1.0f, -1.0f)},  // 2
        {glm::vec3(-1.0f, +1.0f, -1.0f)},  // 3

        {glm::vec3(-1.0f, +1.0f, -1.0f)},  // 4 // Front
        {glm::vec3(+1.0f, +1.0f, -1.0f)},  // 5
        {glm::vec3(+1.0f, -1.0f, -1.0f)},  // 6
        {glm::vec3(-1.0f, -1.0f, -1.0f)},  // 7

        {glm::vec3(+1.0f, +1.0f, -1.0f)},  // 8 // Right
        {glm::vec3(+1.0f, +1.0f, +1.0f)},  // 9
        {glm::vec3(+1.0f, -1.0f, +1.0f)},  // 10
        {glm::vec3(+1.0f, -1.0f, -1.0f)},  // 11

        {glm::vec3(-1.0f, +1.0f, +1.0f)},  // 12 // Left
        {glm::vec3(-1.0f, +1.0f, -1.0f)},  // 13
        {glm::vec3(-1.0f, -1.0f, -1.0f)},  // 14
        {glm::vec3(-1.0f, -1.0f, +1.0f)},  // 15

        {glm::vec3(+1.0f, +1.0f, +1.0f)},  // 16 // Back
        {glm::vec3(-1.0f, +1.0f, +1.0f)},  // 17
        {glm::vec3(-1.0f, -1.0f, +1.0f)},  // 18
        {glm::vec3(+1.0f, -1.0f, +1.0f)},  // 19

        {glm::vec3(+1.0f, -1.0f, -1.0f)},  // 20 // Bottom
        {glm::vec3(-1.0f, -1.0f, -1.0f)},  // 21
        {glm::vec3(-1.0f, -1.0f, +1.0f)},  // 22
        {glm::vec3(+1.0f, -1.0f, +1.0f)},  // 23

    });


    cube.indices.assign({
        0,   1,  2,  0,  2,  3, // Top
        4,   5,  6,  4,  6,  7, // Front
        8,   9, 10,  8, 10, 11, // Right
        12, 13, 14, 12, 14, 15, // Left
        16, 17, 18, 16, 18, 19, // Back
        20, 22, 21, 20, 23, 22, // Bottom
    });

    return cube;
}







