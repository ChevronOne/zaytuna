

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
    std::vector<VERT> ver;
    VERT v;


    for (float i = -ceil(width / 2.0f); i <= floor(width / 2.0f); i += tessellation)
    {
        v.position = glm::vec3(-ceil(length / 2), 0.0f, i);
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
        v.position = glm::vec3(floor(length / 2), 0.0f, i);
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
    }
    for (float i = -ceil(length / 2.0f); i <= floor(length / 2.0f); i += tessellation)
    {
        v.position = glm::vec3(i, 0.0f, -ceil(width / 2));
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
        v.position = glm::vec3(i, 0.0f, floor(width / 2));
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
    }


    plane.verNum = ver.size();
    plane.verts = new VERT[ver.size()];
    //std::copy(ver.begin(), ver.end(),
    //          stdext::checked_array_iterator<Vertex*>
    //                  (plane.verts, ver.size()));
    std::copy(ver.begin(), ver.end(), plane.verts);

    plane.indNum = ver.size();
    plane.indices = new GLuint[ver.size()];
    for (size_t i{ 0 }; i < ver.size(); ++i)
        plane.indices[i] = i;

    return plane;
}

template <typename VERT>
shape_data<VERT>
shape_maker<VERT>::makeCoord(GLfloat axes_length)
{
    shape_data<VERT> coord;
    coord.verNum = 6;
    coord.indNum = 6;

    if(axes_length<1.0f)
        axes_length = 1.0f;

    coord.verts = new VERT[coord.verNum];
    coord.indices = new GLuint[coord.indNum];
    coord.verts[0].position = glm::vec3(0.0f, 0.001f, 0.0f); // X coord
    coord.verts[0].color = glm::vec3(0.0f, 0.0f, 1.0f);
//    coord.verts[0].normal = glm::vec3(0.0f, 1.0f, 0.0f);

    coord.verts[1].position = glm::vec3(axes_length, 0.001f, 0.0f);
    coord.verts[1].color = glm::vec3(0.0f, 0.0f, 1.0f);
//    coord.verts[1].normal = glm::vec3(10.0f, 1.0f, 0.0f);



    coord.verts[2].position = glm::vec3(0.0f, 0.001f, 0.0f);  // Z coord
    coord.verts[2].color = glm::vec3(0.0f, 1.0f, 0.0f);
//    coord.verts[2].normal = glm::vec3(0.0f, 1.0f, 0.0f);

    coord.verts[3].position = glm::vec3(0.0f, 0.001f, axes_length);
    coord.verts[3].color = glm::vec3(0.0f, 1.0f, 0.0f);
//    coord.verts[3].normal = glm::vec3(00.0f, 1.0f, -10.0f);



    coord.verts[4].position = glm::vec3(0.0f, 0.001f, 0.0f); // Y coord
    coord.verts[4].color = glm::vec3(1.0f, 0.0f, 0.0f);
//    coord.verts[4].normal = glm::vec3(1.0f, 0.0f, 0.0f);

    coord.verts[5].position = glm::vec3(0.0f, axes_length, 0.0f);
    coord.verts[5].color = glm::vec3(1.0f, 0.0f, 0.0f);
//    coord.verts[5].normal = glm::vec3(1.0f, 10.0f, 0.0f);

    for(GLuint i =0 ; i< coord.indNum; ++i)
        coord.indices[i] = i;

    return coord;

}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeLap()
{

    unsigned int indices[] = { 0, 1, 2, 3,};
    shape_data<VERT> rectangle;
    zaytuna::vertexL1_16 vertsT[] = {
        // Top
        glm::vec3(+0.0f, +0.0f, +0.0f),  // 0
        glm::vec3(+0.0f, +1.0f, +0.0f),	 // normal
        glm::vec2(+1.0f, +1.0f),         // texcoor

        glm::vec3(+0.0f, +0.0f, -4.3f),  // 1
        glm::vec3(+0.0f, +1.0f, +0.0f),	 // normal
        glm::vec2(+1.0f, +0.0f),		 // texcoor

        glm::vec3(+6.0f, +0.0f, -4.3f),  // 2
        glm::vec3(+0.0f, +1.0f, +0.0f),  // normal
        glm::vec2(+0.0f, +0.0f),         // texcoor

        glm::vec3(+6.0f, +0.0f, +0.0f),  // 3
        glm::vec3(+0.0f, +1.0f, +0.0f),  // normal
        glm::vec2(+0.0f, +1.0f),         // texcoor
    };
    rectangle.verNum = ZAY_NUM_OF(vertsT);
    rectangle.verts = new VERT[rectangle.verNum];
    memcpy(rectangle.verts, vertsT, sizeof(vertsT));
    rectangle.indNum = ZAY_NUM_OF(indices);
    rectangle.indices = new unsigned int[rectangle.indNum];
    memcpy(rectangle.indices, indices, sizeof(indices));
    return rectangle;
}

template <typename VERT>
shape_data<VERT>
shape_maker<VERT>::makeSphere(GLfloat PERS,
                              GLfloat RAD,
                              glm::vec3 CENT)
{
    shape_data<VERT> sphere;
    std::vector<VERT> ver;
    std::vector<GLuint> ind;
    VERT v;
//    glm::vec3 ret;

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
            ver.push_back(v);
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
            ver.push_back(v);
        }
    }

    for (size_t i{ 0 }; i < ver.size(); ++i)
    {
        ind.push_back(i);
    }

    sphere.verNum = ver.size();
    sphere.verts = new VERT[ver.size()];
    //std::copy(ver.begin(), ver.end(),
    //          stdext::checked_array_iterator<Vertex*>
    //              (sphere.verts, ver.size()));
    std::copy(ver.begin(), ver.end(), sphere.verts);

    sphere.indNum = ind.size();
    sphere.indices = new GLuint[ind.size()];
    //std::copy(ind.begin(), ind.end(),
    //          stdext::checked_array_iterator<GLuint*>
    //              (sphere.indices, ind.size()));
    std::copy(ind.begin(), ind.end(),sphere.indices);

    return sphere;
}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makePyramid()
{
    shape_data<VERT> pyramid;
    VERT verts[] = {
        glm::vec3(+0.0f, +1.0f, +0.0f), // 0
        glm::vec3(+0.0f, +0.5f, +0.2f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

        glm::vec3(+0.0f, +1.0f, +0.0f), // 1
        glm::vec3(+0.2f, +0.5f, +0.2f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

        glm::vec3(+0.0f, +1.0f, +0.0f), // 2
        glm::vec3(+0.8f, +0.6f, +0.4f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

        glm::vec3(+0.0f, +1.0f, +0.0f), // 3
        glm::vec3(+0.3f, +1.0f, +0.5f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

        glm::vec3(-1.0f, -1.0f, +1.0f), // 4
        glm::vec3(+0.9f, +0.3f, +0.7f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

        glm::vec3(+1.0f, -1.0f, +1.0f), // 5
        glm::vec3(+0.3f, +0.7f, +0.5f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

        glm::vec3(+1.0f, -1.0f, -1.0f), // 6
        glm::vec3(+0.5f, +0.7f, +0.5f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

        glm::vec3(-1.0f, -1.0f, -1.0f), // 7
        glm::vec3(+0.7f, +0.8f, +0.2f), // color
        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

    };
    unsigned int indices[] = {
        0, 4, 5, // first side
        1, 5, 6, // second side
        2, 6, 7, // third side
        3, 7, 4, // fourth side
        4, 5, 7, 5, 6, 7, // bottom
    };

    pyramid.verNum = ZAY_NUM_OF(verts);
    pyramid.verts = new VERT[pyramid.verNum];
    memcpy(pyramid.verts, verts, sizeof(verts));
    pyramid.indNum = ZAY_NUM_OF(indices);
    pyramid.indices = new unsigned int[pyramid.indNum];
    memcpy(pyramid.indices, indices, sizeof(indices));

    return pyramid;
}


template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeCube() {
    shape_data<VERT> cube;
    using glm::vec3;

    VERT verts[] =
    {
        glm::vec3(-0.2f, 0.1f, 0.05f),  // 0
        glm::vec3(+1.0f, +0.0f, +0.0f),	// Color
        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
        glm::vec3(0.2f, 0.1f, 0.05f),  // 1
        glm::vec3(+0.0f, +1.0f, +0.0f),	// Color
        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
        glm::vec3(0.2f, 0.1f, -0.05f),  // 2
        glm::vec3(+0.0f, +0.0f, +1.0f),  // Color
        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
        glm::vec3(-0.2f, 0.1f, -0.05f),  // 3
        glm::vec3(+1.0f, +1.0f, +1.0f),  // Color
        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal

        glm::vec3(-0.2f, 0.1f, -0.05f),  // 4
        glm::vec3(+1.0f, +0.0f, +1.0f),  // Color
        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
        glm::vec3(0.2f, 0.1f, -0.05f),  // 5
        glm::vec3(+0.0f, +0.5f, +0.2f),  // Color
        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
        glm::vec3(0.2f, 0.0f, -0.05f),  // 6
        glm::vec3(+0.8f, +0.6f, +0.4f),  // Color
        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
        glm::vec3(-0.2f, 0.0f, -0.05f),  // 7
        glm::vec3(+0.3f, +1.0f, +0.5f),  // Color
        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal

        glm::vec3(0.2f, 0.1f, -0.05f),  // 8
        glm::vec3(+0.8f, +0.5f, +0.2f),  // Color
        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
        glm::vec3(0.2f, 0.1f, 0.05f),  // 9
        glm::vec3(+0.9f, +0.3f, +0.5f),  // Color
        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
        glm::vec3(0.2f, 0.0f, 0.05f),  // 10
        glm::vec3(+0.9f, +0.7f, +0.5f),  // Color
        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
        glm::vec3(0.2f, 0.0f, -0.05f),  // 11
        glm::vec3(+0.9f, +0.7f, +0.5f),  // Color
        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal

        glm::vec3(-0.2f, 0.1f, 0.05f),  // 12
        glm::vec3(+0.7f, +0.8f, +0.2f),  // Color
        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
        glm::vec3(-0.2f, 0.1f, -0.05f),  // 13
        glm::vec3(+0.5f, +0.7f, +0.3f),  // Color
        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
        glm::vec3(-0.2f, 0.0f, -0.05f),  // 14
        glm::vec3(+0.4f, +0.7f, +0.7f),  // Color
        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
        glm::vec3(-0.2f, 0.0f, 0.05f),  // 15
        glm::vec3(+0.2f, +0.5f, +1.0f),  // Color
        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal

        glm::vec3(0.2f, 0.1f, 0.05f),  // 16
        glm::vec3(+0.6f, +1.0f, +0.7f),  // Color
        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
        glm::vec3( -0.2f, 0.1f, 0.05f),  // 17
        glm::vec3(+0.6f, +0.4f, +0.8f),  // Color
        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
        glm::vec3(-0.2f, 0.0f, 0.05f),  // 18
        glm::vec3(+0.2f, +0.8f, +0.7f),  // Color
        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
        glm::vec3(0.2f, 0.0f, 0.05f),  // 19
        glm::vec3(+0.2f, +0.7f, +1.0f),  // Color
        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal

        glm::vec3(0.2f, 0.0f, -0.05f),  // 20
        glm::vec3(+0.8f, +0.3f, +0.7f),  // Color
        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
        glm::vec3(-0.2f, 0.0f, -0.05f),  // 21
        glm::vec3(+0.8f, +0.9f, +0.5f),  // Color
        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
        glm::vec3(-0.2f, 0.0f, 0.05f),  // 22
        glm::vec3(+0.5f, +0.8f, +0.5f),  // Color
        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
        glm::vec3(0.2f, 0.0f, 0.05f),  // 23
        glm::vec3(+0.9f, +1.0f, +0.2f),  // Color
        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
    };

    unsigned int indices[] = {
        0,   1,  2,  0,  2,  3, // Top
        4,   5,  6,  4,  6,  7, // Front
        8,   9, 10,  8, 10, 11, // Right
        12, 13, 14, 12, 14, 15, // Left
        16, 17, 18, 16, 18, 19, // Back
        20, 22, 21, 20, 23, 22, // Bottom
    };
    cube.verNum = ZAY_NUM_OF(verts);
    cube.verts = new VERT[cube.verNum];
    memcpy(cube.verts, verts, sizeof(verts));
    cube.indNum = ZAY_NUM_OF(indices);
    cube.indices = new unsigned int[cube.indNum];
    memcpy(cube.indices, indices, sizeof(indices));

    return cube;
}



template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeCubemap() {
    shape_data<VERT> cube;
    VERT verts[] =
    {
        glm::vec3(-1.0f, +1.0f, +1.0f),  // 0 // Top
        glm::vec3(+1.0f, +1.0f, +1.0f),  // 1
        glm::vec3(+1.0f, +1.0f, -1.0f),  // 2
        glm::vec3(-1.0f, +1.0f, -1.0f),  // 3

        glm::vec3(-1.0f, +1.0f, -1.0f),  // 4 // Front
        glm::vec3(+1.0f, +1.0f, -1.0f),  // 5
        glm::vec3(+1.0f, -1.0f, -1.0f),  // 6
        glm::vec3(-1.0f, -1.0f, -1.0f),  // 7

        glm::vec3(+1.0f, +1.0f, -1.0f),  // 8 // Right
        glm::vec3(+1.0f, +1.0f, +1.0f),  // 9
        glm::vec3(+1.0f, -1.0f, +1.0f),  // 10
        glm::vec3(+1.0f, -1.0f, -1.0f),  // 11

        glm::vec3(-1.0f, +1.0f, +1.0f),  // 12 // Left
        glm::vec3(-1.0f, +1.0f, -1.0f),  // 13
        glm::vec3(-1.0f, -1.0f, -1.0f),  // 14
        glm::vec3(-1.0f, -1.0f, +1.0f),  // 15

        glm::vec3(+1.0f, +1.0f, +1.0f),  // 16 // Back
        glm::vec3(-1.0f, +1.0f, +1.0f),  // 17
        glm::vec3(-1.0f, -1.0f, +1.0f),  // 18
        glm::vec3(+1.0f, -1.0f, +1.0f),  // 19

        glm::vec3(+1.0f, -1.0f, -1.0f),  // 20 // Bottom
        glm::vec3(-1.0f, -1.0f, -1.0f),  // 21
        glm::vec3(-1.0f, -1.0f, +1.0f),  // 22
        glm::vec3(+1.0f, -1.0f, +1.0f),  // 23

    };


    unsigned int indices[] = {
        0,   1,  2,  0,  2,  3,  // Top
        4,   5,  6,  4,  6,  7, // Front
        8,   9, 10,  8, 10, 11, // Right
        12, 13, 14, 12, 14, 15, // Left
        16, 17, 18, 16, 18, 19, // Back
        20, 22, 21, 20, 23, 22, // Bottom
    };

    cube.verNum = ZAY_NUM_OF(verts);
    cube.verts = new VERT[cube.verNum];
    memcpy(cube.verts, verts, sizeof(verts));
    cube.indNum = ZAY_NUM_OF(indices);
    cube.indices = new unsigned int[cube.indNum];
    memcpy(cube.indices, indices, sizeof(indices));

    return cube;

    
}







