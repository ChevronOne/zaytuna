

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








template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeGrid(GLfloat length, GLfloat width, GLfloat tessellation)
{
    shape_data<VERT> plane;
    std::vector<VERT> ver;
    std::vector<GLuint> ind;
    VERT v;


    for (float i = -ceil(width / 2.0f); i <= floor(width / 2.0f); i += tessellation)
    {
        v.position = glm::vec3(-ceil(length / 2), 0.0f, i);
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
        v.position = glm::vec3(floor(length / 2), 0.0f, i);
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
    }
    for (float i = -ceil(length / 2.0f); i <= floor(length / 2.0f); i += tessellation)
    {
        v.position = glm::vec3(i, 0.0f, -ceil(width / 2));
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
        v.position = glm::vec3(i, 0.0f, floor(width / 2));
        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        ver.push_back(v);
    }

    for (size_t i{ 0 }; i < ver.size(); ++i)
    {
        ind.push_back(i);
    }

    plane.verNum = ver.size();
    plane.verts = new VERT[ver.size()];
    //std::copy(ver.begin(), ver.end(), stdext::checked_array_iterator<Vertex*>(plane.verts, ver.size()));
    std::copy(ver.begin(), ver.end(), plane.verts);

    plane.indNum = ind.size();
    plane.indices = new GLuint[ind.size()];
    //std::copy(ind.begin(), ind.end(), stdext::checked_array_iterator<GLuint*>(plane.indices, ind.size()));
    std::copy(ind.begin(), ind.end(),plane.indices);

//    std::cout << ver.size() << std::endl;
//    std::cout << ind.size() << std::endl;
    return plane;
}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeCoord()
{
    shape_data<VERT> coord;
    coord.verNum = 6;
    coord.indNum = 6;

    coord.verts = new VERT[coord.verNum];
    coord.indices = new GLuint[coord.indNum];
    coord.verts[0].position = glm::vec3(0.0f, 0.001f, 0.0f); // X coord
    coord.verts[0].color = glm::vec3(0.0f, 0.0f, 1.0f);
    coord.verts[0].normal = glm::vec3(0.0f, 1.0f, 0.0f);

    coord.verts[1].position = glm::vec3(10.0f, 0.001f, 0.0f);
    coord.verts[1].color = glm::vec3(0.0f, 0.0f, 1.0f);
    coord.verts[1].normal = glm::vec3(10.0f, 1.0f, 0.0f);



    coord.verts[2].position = glm::vec3(0.0f, 0.001f, 0.0f);  // Z coord
    coord.verts[2].color = glm::vec3(0.0f, 1.0f, 0.0f);
    coord.verts[2].normal = glm::vec3(0.0f, 1.0f, 0.0f);

    coord.verts[3].position = glm::vec3(0.0f, 0.001f, -10.0f);
    coord.verts[3].color = glm::vec3(0.0f, 1.0f, 0.0f);
    coord.verts[3].normal = glm::vec3(00.0f, 1.0f, -10.0f);



    coord.verts[4].position = glm::vec3(0.0f, 0.001f, 0.0f); // Y coord
    coord.verts[4].color = glm::vec3(1.0f, 0.0f, 0.0f);
    coord.verts[4].normal = glm::vec3(1.0f, 0.0f, 0.0f);

    coord.verts[5].position = glm::vec3(0.0f, 10.001f, 0.0f);
    coord.verts[5].color = glm::vec3(1.0f, 0.0f, 0.0f);
    coord.verts[5].normal = glm::vec3(1.0f, 10.0f, 0.0f);

    for(GLuint i =0 ; i< coord.indNum; ++i)
        coord.indices[i] = i;

    return coord;

}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeLap()
{

    unsigned int indices[] = { 0, 1, 2, 3,}; // GL_TRIANGLES { 0, 1, 3, 1, 2, 3, };
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
    rectangle.verNum = NUM_OF(vertsT);
    rectangle.verts = new VERT[rectangle.verNum];
    memcpy(rectangle.verts, vertsT, sizeof(vertsT));
    rectangle.indNum = NUM_OF(indices);
    rectangle.indices = new unsigned int[rectangle.indNum];
    memcpy(rectangle.indices, indices, sizeof(indices));
    return rectangle;
}

//template <typename VERT>
//shape_data<VERT> shape_maker<VERT>::extractExternal(const std::string& _dir)
//{
//    shape_data<VERT> _object;


//    std::ifstream f_stream(_dir, std::ios::in);
//    if( !f_stream.is_open())
//        throw std::runtime_error("could not open file <"+_dir+">\n");

//    std::vector<float> pos, tex, norm;
//    std::vector<uint32_t> vert;
//    std::string record;
//    while (std::getline(f_stream, record))
//    {
//        if ( record.substr(0, 2) == "v ")
//            for(; record.substr(0, 2) == "v "; std::getline(f_stream, record) )
//                qi::parse(record.begin()+2, record.end(),
//                        *(qi::float_ >> ' '|'\0'), pos);

//        if ( record.substr(0, 3) == "vt ")
//            for(;  record.substr(0, 3) == "vt "; std::getline(f_stream, record))
//                qi::parse(record.begin()+3, record.end(),
//                        *(qi::float_ >> ' '|'\0'), tex);


//        if ( record.substr(0, 3)  == "vn ")
//            for(; record.substr(0, 3)  == "vn "; std::getline(f_stream, record))
//                qi::parse(record.begin()+3, record.end(),
//                        *(qi::float_ >> ' '|'\0'), norm);

//        if ( record.substr(0, 2)  == "f ")
//            for(;record.substr(0, 2)  == "f "; std::getline(f_stream, record))
//                qi::parse(record.begin()+2, record.end(),
//                        *(qi::uint_ % '/' >> ' '|'\0'), vert);
//    }
//    for(auto& ver:vert)
//        ver-=1;

//    _object.verNum = _object.indNum =  vert.size()/3;
//    _object.verts = new VERT[_object.verNum];
//    glm::vec3*  pos_arr{reinterpret_cast<glm::vec3*>( pos.data()) },
//               *norm_arr{reinterpret_cast<glm::vec3*>(norm.data()) };
//    glm::vec2*  tex_arr{reinterpret_cast<glm::vec2*>( tex.data()) };

//    for(uint32_t i{0}, j{0}; i<_object.verNum; ++i, j+=3 ){
//        _object.verts[i] = {pos_arr[vert[j]], norm_arr[vert[j+2]], tex_arr[vert[j+1]]};
//    }

//    _object.indices = new unsigned int[_object.indNum];
//    for(uint32_t i{0}; i<_object.indNum; ++i)
//        _object.indices[i] = i;


//    return _object;
//}


template <typename VERT>
shape_data<VERT> shape_maker<VERT>::extractExternal(const std::string& _dir)
{
    std::ifstream f_stream(_dir, std::ios::in);
    if( !f_stream.is_open()){
        std::cerr << "could not open file <" << _dir << ">\n";
        exit(EXIT_FAILURE);
    }
    std::vector<float> _positions;
    std::vector<float> _texcoords;
    std::vector<float> _normals;
    std::vector<uint32_t> _faces;

    auto pos_capture  = [&](auto& ctx){ _positions.emplace_back(_attr(ctx)); };
    auto tex_capture  = [&](auto& ctx){ _texcoords.emplace_back(_attr(ctx)); };
    auto norm_capture = [&](auto& ctx){ _normals.emplace_back(_attr(ctx)); };
    auto face_capture = [&](auto& ctx){ _faces.emplace_back(_attr(ctx)-1); };


    auto pos_rule      = "v" >> x3::float_[pos_capture] >> x3::float_[pos_capture] >> x3::float_[pos_capture];
    auto tex_rule      = "vt" >> x3::float_[tex_capture] >> x3::float_[tex_capture];
    auto norm_rule     = "vn" >> x3::float_[norm_capture] >> x3::float_[norm_capture] >> x3::float_[norm_capture];
    auto VERTEX_rule   = x3::uint_[face_capture] >> '/' >> x3::uint_[face_capture] >> '/' >> x3::uint_[face_capture];
//    auto LINE_rule     = "f" >> x3::repeat(2)[VERTEX];
//    auto TRIANGLE_rule = "f" >> x3::repeat(3)[VERTEX];
//    auto QUAD_rule     = "f" >> x3::repeat(4)[VERTEX];

    // auto faces   = "f" >> VERTEX >> VERTEX >> VERTEX;
    // auto faces = "f" >> x3::repeat(3)[VERTEX];
    auto face_rule  = "f" >> +VERTEX_rule;
    // auto faces   = "f" >> (QUAD  | TRIANGLE | LINE);  XXXX


    auto skipper = x3::blank | '#' >> *(x3::char_ - x3::eol) >> (x3::eol|x3::eoi);

    auto __OBJ = x3::skip(skipper) [ *x3::eol >> *(
            +(pos_rule  >> (x3::eoi|+x3::eol)) >>
            *(tex_rule  >> (x3::eoi|+x3::eol)) >>
            *(norm_rule >> (x3::eoi|+x3::eol)) >>
            *(face_rule >> (x3::eoi|+x3::eol))
        )];


    boost::spirit::istream_iterator _f(f_stream >> std::noskipws), _l;
    shape_data<VERT> _object;

    if (x3::parse(_f, _l, __OBJ)) {

        _object.verNum = _object.indNum =  _faces.size()/3;
        _object.verts = new VERT[_object.verNum];
        glm::vec3*  pos_arr{reinterpret_cast<glm::vec3*>(_positions.data())},
                   *norm_arr{reinterpret_cast<glm::vec3*>(_normals.data())};
        glm::vec2*  tex_arr{reinterpret_cast<glm::vec2*>(_texcoords.data())};

        for(uint32_t i{0}, j{0}; i<_object.verNum; ++i, j+=3 ){
            _object.verts[i] = {pos_arr[_faces[j]], norm_arr[_faces[j+2]], tex_arr[_faces[j+1]]};
        }

        _object.indices = new unsigned int[_object.indNum];
        for(uint32_t i{0}; i<_object.indNum; ++i)
            _object.indices[i] = i;

    } else{
        std::cerr << "failed to parse file <" << _dir << ">\n";
        exit(EXIT_FAILURE);
    }
    if (_f!=_l){
        std::cerr << "WARNING: file did not parsed properly <" << _dir << ">\n";
        std::cout << "Unparsed: " << std::distance(_f,_l) << " characters remained unparsed! object may not be rendered properly\n";
    }

    return _object;
}


template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makePlaneVerts(int dim)
{
    shape_data<VERT> ret;
    ret.verNum = dim * dim;
    int half = dim / 2;
    ret.verts = new VERT[ret.verNum];
    for (int i = 0; i < dim; i++)
    {
        for (int j = 0; j < dim; j++)
        {
            VERT& thisVert = ret.verts[i * dim + j];
            thisVert.position.x = j - half;
            thisVert.position.z = i - half;
            thisVert.position.y = -0.004f;
            thisVert.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            thisVert.color = glm::vec3(0.843f, 0.8f, 0.69f); // glm::vec3(0.74f, 0.74f, 0.74f);
        }
    }
    return ret;
}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makePlaneIndices(int dim)
{
    shape_data<VERT> ret;
    ret.indNum = (dim - 1) * (dim - 1) * 2 * 3; // 2 triangles per square, 3 indices per triangle
    ret.indices = new GLuint[ret.indNum];
    int runner = 0;
    for (int row = 0; row < dim - 1; row++)
    {
        for (int col = 0; col < dim - 1; col++)
        {
            ret.indices[runner++] = dim * row + col;
            ret.indices[runner++] = dim * row + col + dim;
            ret.indices[runner++] = dim * row + col + dim + 1;

            ret.indices[runner++] = dim * row + col;
            ret.indices[runner++] = dim * row + col + dim + 1;
            ret.indices[runner++] = dim * row + col + 1;
        }
    }
    assert(runner = ret.indNum);
    return ret;
}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makePlane(int dim)
{
    shape_data<VERT> ret = makePlaneVerts(dim);
    shape_data<VERT> ret2 = makePlaneIndices(dim);
    ret.indNum = ret2.indNum;
    ret.indices = ret2.indices;
    return ret;
}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeSphere(GLfloat PERS, GLfloat RAD, glm::vec3 CENT)
{
    shape_data<VERT> sphere;
    std::vector<VERT> ver;
    std::vector<GLuint> ind;
    VERT v;
//    glm::vec3 ret;

    for (float i = -PI / 2; i <= PI / 2; i += PERS)
    {
        for (float j = 0.0f; j <= PI * 2 + PI / 26.8935727626227514264201090554706752; j += PERS)
        {
            v.position = glm::vec3(cosf(j) * cosf(i) * RAD, sinf(i) * RAD, sinf(j) * cosf(i) * RAD) + CENT;
            v.color = rCol();
            v.normal = v.position;
            ver.push_back(v);
            v.position = glm::vec3(cosf(j) * cosf(i + PERS) * RAD, sinf(i + PERS) * RAD, sinf(j) * cosf(i + PERS) * RAD) + CENT;
            v.color = rCol();
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
    //std::copy(ver.begin(), ver.end(), stdext::checked_array_iterator<Vertex*>(sphere.verts, ver.size()));
    std::copy(ver.begin(), ver.end(), sphere.verts);

    sphere.indNum = ind.size();
    sphere.indices = new GLuint[ind.size()];
    //std::copy(ind.begin(), ind.end(), stdext::checked_array_iterator<GLuint*>(sphere.indices, ind.size()));
    std::copy(ind.begin(), ind.end(),sphere.indices);

    return sphere;
}

template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makePyramide()
{
    shape_data<VERT> pyramide;
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

    pyramide.verNum = NUM_OF(verts);
    pyramide.verts = new VERT[pyramide.verNum];
    memcpy(pyramide.verts, verts, sizeof(verts));
    pyramide.indNum = NUM_OF(indices);
    pyramide.indices = new unsigned int[pyramide.indNum];
    memcpy(pyramide.indices, indices, sizeof(indices));

    return pyramide;
}


template <typename VERT>
shape_data<VERT> shape_maker<VERT>::makeCube() {
    shape_data<VERT> cube;
    using glm::vec3;
//    vertex verts[] =
//    {
//        glm::vec3(-1.0f, +1.0f, +1.0f),  // 0
//        glm::vec3(+1.0f, +0.0f, +0.0f),	// Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
//        glm::vec3(+1.0f, +1.0f, +1.0f),  // 1
//        glm::vec3(+0.0f, +1.0f, +0.0f),	// Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
//        glm::vec3(+1.0f, +1.0f, -1.0f),  // 2
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
//        glm::vec3(-1.0f, +1.0f, -1.0f),  // 3
//        glm::vec3(+1.0f, +1.0f, +1.0f),  // Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal

//        glm::vec3(-1.0f, +1.0f, -1.0f),  // 4
//        glm::vec3(+1.0f, +0.0f, +1.0f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
//        glm::vec3(+1.0f, +1.0f, -1.0f),  // 5
//        glm::vec3(+0.0f, +0.5f, +0.2f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
//        glm::vec3(+1.0f, -1.0f, -1.0f),  // 6
//        glm::vec3(+0.8f, +0.6f, +0.4f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
//        glm::vec3(-1.0f, -1.0f, -1.0f),  // 7
//        glm::vec3(+0.3f, +1.0f, +0.5f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal

//        glm::vec3(+1.0f, +1.0f, -1.0f),  // 8
//        glm::vec3(+0.2f, +0.5f, +0.2f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(+1.0f, +1.0f, +1.0f),  // 9
//        glm::vec3(+0.9f, +0.3f, +0.7f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(+1.0f, -1.0f, +1.0f),  // 10
//        glm::vec3(+0.3f, +0.7f, +0.5f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(+1.0f, -1.0f, -1.0f),  // 11
//        glm::vec3(+0.5f, +0.7f, +0.5f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal

//        glm::vec3(-1.0f, +1.0f, +1.0f),  // 12
//        glm::vec3(+0.7f, +0.8f, +0.2f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(-1.0f, +1.0f, -1.0f),  // 13
//        glm::vec3(+0.5f, +0.7f, +0.3f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(-1.0f, -1.0f, -1.0f),  // 14
//        glm::vec3(+0.4f, +0.7f, +0.7f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(-1.0f, -1.0f, +1.0f),  // 15
//        glm::vec3(+0.2f, +0.5f, +1.0f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal

//        glm::vec3(+1.0f, +1.0f, +1.0f),  // 16
//        glm::vec3(+0.6f, +1.0f, +0.7f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
//        glm::vec3(-1.0f, +1.0f, +1.0f),  // 17
//        glm::vec3(+0.6f, +0.4f, +0.8f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
//        glm::vec3(-1.0f, -1.0f, +1.0f),  // 18
//        glm::vec3(+0.2f, +0.8f, +0.7f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
//        glm::vec3(+1.0f, -1.0f, +1.0f),  // 19
//        glm::vec3(+0.2f, +0.7f, +1.0f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal

//        glm::vec3(+1.0f, -1.0f, -1.0f),  // 20
//        glm::vec3(+0.8f, +0.3f, +0.7f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//        glm::vec3(-1.0f, -1.0f, -1.0f),  // 21
//        glm::vec3(+0.8f, +0.9f, +0.5f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//        glm::vec3(-1.0f, -1.0f, +1.0f),  // 22
//        glm::vec3(+0.5f, +0.8f, +0.5f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//        glm::vec3(+1.0f, -1.0f, +1.0f),  // 23
//        glm::vec3(+0.9f, +1.0f, +0.2f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//    };



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
    cube.verNum = NUM_OF(verts);
    cube.verts = new VERT[cube.verNum];
    memcpy(cube.verts, verts, sizeof(verts));
    cube.indNum = NUM_OF(indices);
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

//        glm::vec3(-1.0f,  1.0f, -1.0f),
//        glm::vec3(-1.0f, -1.0f, -1.0f),
//        glm::vec3( 1.0f, -1.0f, -1.0f),
//        glm::vec3( 1.0f, -1.0f, -1.0f),
//        glm::vec3( 1.0f,  1.0f, -1.0f),
//        glm::vec3(-1.0f,  1.0f, -1.0f),
//        glm::vec3(-1.0f, -1.0f,  1.0f),
//        glm::vec3(-1.0f, -1.0f, -1.0f),
//        glm::vec3(-1.0f,  1.0f, -1.0f),
//        glm::vec3(-1.0f,  1.0f, -1.0f),
//        glm::vec3(-1.0f,  1.0f,  1.0f),
//        glm::vec3(-1.0f, -1.0f,  1.0f),
//        glm::vec3( 1.0f, -1.0f, -1.0f),
//        glm::vec3( 1.0f, -1.0f,  1.0f),
//        glm::vec3( 1.0f,  1.0f,  1.0f),
//        glm::vec3( 1.0f,  1.0f,  1.0f),
//        glm::vec3( 1.0f,  1.0f, -1.0f),
//        glm::vec3( 1.0f, -1.0f, -1.0f),
//        glm::vec3(-1.0f, -1.0f,  1.0f),
//        glm::vec3(-1.0f,  1.0f,  1.0f),
//        glm::vec3( 1.0f,  1.0f,  1.0f),
//        glm::vec3( 1.0f,  1.0f,  1.0f),
//        glm::vec3( 1.0f, -1.0f,  1.0f),
//        glm::vec3(-1.0f, -1.0f,  1.0f),
//        glm::vec3(-1.0f,  1.0f, -1.0f),
//        glm::vec3( 1.0f,  1.0f, -1.0f),
//        glm::vec3( 1.0f,  1.0f,  1.0f),
//        glm::vec3( 1.0f,  1.0f,  1.0f),
//        glm::vec3(-1.0f,  1.0f,  1.0f),
//        glm::vec3(-1.0f,  1.0f, -1.0f),
//        glm::vec3(-1.0f, -1.0f, -1.0f),
//        glm::vec3(-1.0f, -1.0f,  1.0f),
//        glm::vec3( 1.0f, -1.0f, -1.0f),
//        glm::vec3( 1.0f, -1.0f, -1.0f),
//        glm::vec3(-1.0f, -1.0f,  1.0f),
//        glm::vec3( 1.0f, -1.0f,  1.0f)

    };


    unsigned int indices[] = {
       0,   1,  2,  0,  2,  3, // Top
        4,   5,  6,  4,  6,  7, // Front
        8,   9, 10,  8, 10, 11, // Right
        12, 13, 14, 12, 14, 15, // Left
        16, 17, 18, 16, 18, 19, // Back
        20, 22, 21, 20, 23, 22, // Bottom
    };

//    for(GLuint i = 0; i<NUM_OF(indices); ++i)
//        indices[i] = i;
    cube.verNum = NUM_OF(verts);
    cube.verts = new VERT[cube.verNum];
    memcpy(cube.verts, verts, sizeof(verts));
    cube.indNum = NUM_OF(indices);
    cube.indices = new unsigned int[cube.indNum];
    memcpy(cube.indices, indices, sizeof(indices));

    return cube;
}







////// ====================================================================
////// ======================================================================


//#include "__shape_maker__.hpp"

//#include <iterator>
//glm::vec3 randomColor()
//{
//    glm::vec3 ret;
//    ret.x = rand() / (float)RAND_MAX;
//    ret.y = rand() / (float)RAND_MAX;
//    ret.z = rand() / (float)RAND_MAX;
//    return ret;
//}

//shape_data shape_maker::makeGrid(GLfloat length, GLfloat width, GLfloat tessellation)
//{
//    shape_data plane;
//    std::vector<zaytuna::vertex> ver;
//    std::vector<GLuint> ind;
//    zaytuna::vertex v;


//    for (float i = -ceil(width / 2.0f); i <= floor(width / 2.0f); i += tessellation)
//    {
//        v.position = glm::vec3(-ceil(length / 2), 0.0f, i);
//        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(v.position.x, 1.0f, v.position.z);
//        ver.push_back(v);
//        v.position = glm::vec3(floor(length / 2), 0.0f, i);
//        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(v.position.x, 1.0f, v.position.z);
//        ver.push_back(v);
//    }
//    for (float i = -ceil(length / 2.0f); i <= floor(length / 2.0f); i += tessellation)
//    {
//        v.position = glm::vec3(i, 0.0f, -ceil(width / 2));
//        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(v.position.x, 1.0f, v.position.z);
//        ver.push_back(v);
//        v.position = glm::vec3(i, 0.0f, floor(width / 2));
//        v.color = glm::vec3(0.6f, 0.6f, 0.6f);
//        v.normal = glm::vec3(v.position.x, 1.0f, v.position.z);
//        ver.push_back(v);
//    }

//    for (size_t i{ 0 }; i < ver.size(); ++i)
//    {
//        ind.push_back(i);
//    }

//    plane.verNum = ver.size();
//    plane.verts = new zaytuna::vertex[ver.size()];
//    //std::copy(ver.begin(), ver.end(), stdext::checked_array_iterator<Vertex*>(plane.verts, ver.size()));
//    std::copy(ver.begin(), ver.end(), plane.verts);

//    plane.indNum = ind.size();
//    plane.indices = new GLuint[ind.size()];
//    //std::copy(ind.begin(), ind.end(), stdext::checked_array_iterator<GLuint*>(plane.indices, ind.size()));
//    std::copy(ind.begin(), ind.end(),plane.indices);

////    std::cout << ver.size() << std::endl;
////    std::cout << ind.size() << std::endl;
//    return plane;
//}

//shape_data shape_maker::makeCoord()
//{
//    shape_data coord;
//    coord.verNum = 6;
//    coord.indNum = 6;

//    coord.verts = new zaytuna::vertex[coord.verNum];
//    coord.indices = new GLuint[coord.indNum];
//    coord.verts[0].position = glm::vec3(0.0f, 0.001f, 0.0f); // X coord
//    coord.verts[0].color = glm::vec3(0.0f, 0.0f, 1.0f);
//    coord.verts[0].normal = glm::vec3(0.0f, 1.0f, 0.0f);

//    coord.verts[1].position = glm::vec3(10.0f, 0.001f, 0.0f);
//    coord.verts[1].color = glm::vec3(0.0f, 0.0f, 1.0f);
//    coord.verts[1].normal = glm::vec3(10.0f, 1.0f, 0.0f);


//    coord.indices = new GLuint[coord.indNum];
//    coord.verts[2].position = glm::vec3(0.0f, 0.001f, 0.0f);  // Z coord
//    coord.verts[2].color = glm::vec3(0.0f, 1.0f, 0.0f);
//    coord.verts[2].normal = glm::vec3(0.0f, 1.0f, 0.0f);

//    coord.verts[3].position = glm::vec3(0.0f, 0.001f, -10.0f);
//    coord.verts[3].color = glm::vec3(0.0f, 1.0f, 0.0f);
//    coord.verts[3].normal = glm::vec3(00.0f, 1.0f, -10.0f);


//    coord.indices = new GLuint[coord.indNum];
//    coord.verts[4].position = glm::vec3(0.0f, 0.001f, 0.0f); // Y coord
//    coord.verts[4].color = glm::vec3(1.0f, 0.0f, 0.0f);
//    coord.verts[4].normal = glm::vec3(1.0f, 0.0f, 0.0f);

//    coord.verts[5].position = glm::vec3(0.0f, 10.001f, 0.0f);
//    coord.verts[5].color = glm::vec3(1.0f, 0.0f, 0.0f);
//    coord.verts[5].normal = glm::vec3(1.0f, 10.0f, 0.0f);

//    for(int i =0 ; i< coord.indNum; ++i)
//        coord.indices[i] = i;

//    return coord;

//}

//shape_data shape_maker::makePlaneVerts(GLuint dim)
//{
//    shape_data ret;
//    ret.verNum = dim * dim;
//    int half = dim / 2;
//    ret.verts = new zaytuna::vertex[ret.verNum];
//    for (int i = 0; i < dim; i++)
//    {
//        for (int j = 0; j < dim; j++)
//        {
//            zaytuna::vertex& thisVert = ret.verts[i * dim + j];
//            thisVert.position.x = j - half;
//            thisVert.position.z = i - half;
//            thisVert.position.y = -0.004f;
//            thisVert.normal = glm::vec3(0.0f, 1.0f, 0.0f);
//            thisVert.color = glm::vec3(0.74f, 0.74f, 0.74f); // randomColor();
//        }
//    }
//    return ret;
//}

//shape_data shape_maker::makePlaneIndices(GLuint dim)
//{
//    shape_data ret;
//    ret.indNum = (dim - 1) * (dim - 1) * 2 * 3; // 2 triangles per square, 3 indices per triangle
//    ret.indices = new GLuint[ret.indNum];
//    int runner = 0;
//    for (int row = 0; row < dim - 1; row++)
//    {
//        for (int col = 0; col < dim - 1; col++)
//        {
//            ret.indices[runner++] = dim * row + col;
//            ret.indices[runner++] = dim * row + col + dim;
//            ret.indices[runner++] = dim * row + col + dim + 1;

//            ret.indices[runner++] = dim * row + col;
//            ret.indices[runner++] = dim * row + col + dim + 1;
//            ret.indices[runner++] = dim * row + col + 1;
//        }
//    }
//    assert(runner = ret.indNum);
//    return ret;
//}

//shape_data shape_maker::makePlane(GLuint dim)
//{
//    shape_data ret = makePlaneVerts(dim);
//    shape_data ret2 = makePlaneIndices(dim);
//    ret.indNum = ret2.indNum;
//    ret.indices = ret2.indices;
//    return ret;
//}

//shape_data shape_maker::makeSphere(GLfloat PERS, GLfloat RAD, glm::vec3 CENT)
//{
//    shape_data sphere;
//    std::vector<zaytuna::vertex> ver;
//    std::vector<GLuint> ind;
//    zaytuna::vertex v;
//    for (float i = -PI / 2; i <= PI / 2; i += PERS)
//    {
//        for (float j = 0.0f; j <= PI * 2 + PI / 26.8935727626227514264201090554706752; j += PERS)
//        {
//            v.position = glm::vec3(cosf(j) * cosf(i) * RAD, sinf(i) * RAD, sinf(j) * cosf(i) * RAD) + CENT;
//            v.color = randomColor();
//            v.normal = v.position;
//            ver.push_back(v);
//            v.position = glm::vec3(cosf(j) * cosf(i + PERS) * RAD, sinf(i + PERS) * RAD, sinf(j) * cosf(i + PERS) * RAD) + CENT;
//            v.color = randomColor();
//            v.normal = v.position;
//            ver.push_back(v);
//        }
//    }

//    for (size_t i{ 0 }; i < ver.size(); ++i)
//    {
//        ind.push_back(i);
//    }

//    sphere.verNum = ver.size();
//    sphere.verts = new zaytuna::vertex[ver.size()];
//    //std::copy(ver.begin(), ver.end(), stdext::checked_array_iterator<Vertex*>(sphere.verts, ver.size()));
//    std::copy(ver.begin(), ver.end(), sphere.verts);

//    sphere.indNum = ind.size();
//    sphere.indices = new GLuint[ind.size()];
//    //std::copy(ind.begin(), ind.end(), stdext::checked_array_iterator<GLuint*>(sphere.indices, ind.size()));
//    std::copy(ind.begin(), ind.end(),sphere.indices);

//    return sphere;
//}

//shape_data shape_maker::makePyramide()
//{
//    shape_data pyramide;
//    zaytuna::vertex verts[] = {
//        glm::vec3(+0.0f, +1.0f, +0.0f), // 0
//        glm::vec3(+0.0f, +0.5f, +0.2f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//        glm::vec3(+0.0f, +1.0f, +0.0f), // 1
//        glm::vec3(+0.2f, +0.5f, +0.2f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//        glm::vec3(+0.0f, +1.0f, +0.0f), // 2
//        glm::vec3(+0.8f, +0.6f, +0.4f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//        glm::vec3(+0.0f, +1.0f, +0.0f), // 3
//        glm::vec3(+0.3f, +1.0f, +0.5f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//        glm::vec3(-1.0f, -1.0f, +1.0f), // 4
//        glm::vec3(+0.9f, +0.3f, +0.7f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//        glm::vec3(+1.0f, -1.0f, +1.0f), // 5
//        glm::vec3(+0.3f, +0.7f, +0.5f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//        glm::vec3(+1.0f, -1.0f, -1.0f), // 6
//        glm::vec3(+0.5f, +0.7f, +0.5f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//        glm::vec3(-1.0f, -1.0f, -1.0f), // 7
//        glm::vec3(+0.7f, +0.8f, +0.2f), // color
//        glm::vec3(+0.0f, +1.0f, +0.0f), // normal

//    };
//    unsigned int indices[] = {
//        0, 4, 5, // first side
//        1, 5, 6, // second side
//        2, 6, 7, // third side
//        3, 7, 4, // fourth side
//        4, 5, 7, 5, 6, 7, // bottom
//    };

//    pyramide.verNum = NUM_OF(verts);
//    pyramide.verts = new zaytuna::vertex[pyramide.verNum];
//    memcpy(pyramide.verts, verts, sizeof(verts));
//    pyramide.indNum = NUM_OF(indices);
//    pyramide.indices = new unsigned int[pyramide.indNum];
//    memcpy(pyramide.indices, indices, sizeof(indices));

//    return pyramide;
//}


//shape_data shape_maker::makeCube() {
//    shape_data cube;
//    using glm::vec3;
////    vertex verts[] =
////    {
////        glm::vec3(-1.0f, +1.0f, +1.0f),  // 0
////        glm::vec3(+1.0f, +0.0f, +0.0f),	// Color
////        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
////        glm::vec3(+1.0f, +1.0f, +1.0f),  // 1
////        glm::vec3(+0.0f, +1.0f, +0.0f),	// Color
////        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
////        glm::vec3(+1.0f, +1.0f, -1.0f),  // 2
////        glm::vec3(+0.0f, +0.0f, +1.0f),  // Color
////        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
////        glm::vec3(-1.0f, +1.0f, -1.0f),  // 3
////        glm::vec3(+1.0f, +1.0f, +1.0f),  // Color
////        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal

////        glm::vec3(-1.0f, +1.0f, -1.0f),  // 4
////        glm::vec3(+1.0f, +0.0f, +1.0f),  // Color
////        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
////        glm::vec3(+1.0f, +1.0f, -1.0f),  // 5
////        glm::vec3(+0.0f, +0.5f, +0.2f),  // Color
////        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
////        glm::vec3(+1.0f, -1.0f, -1.0f),  // 6
////        glm::vec3(+0.8f, +0.6f, +0.4f),  // Color
////        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
////        glm::vec3(-1.0f, -1.0f, -1.0f),  // 7
////        glm::vec3(+0.3f, +1.0f, +0.5f),  // Color
////        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal

////        glm::vec3(+1.0f, +1.0f, -1.0f),  // 8
////        glm::vec3(+0.2f, +0.5f, +0.2f),  // Color
////        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
////        glm::vec3(+1.0f, +1.0f, +1.0f),  // 9
////        glm::vec3(+0.9f, +0.3f, +0.7f),  // Color
////        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
////        glm::vec3(+1.0f, -1.0f, +1.0f),  // 10
////        glm::vec3(+0.3f, +0.7f, +0.5f),  // Color
////        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
////        glm::vec3(+1.0f, -1.0f, -1.0f),  // 11
////        glm::vec3(+0.5f, +0.7f, +0.5f),  // Color
////        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal

////        glm::vec3(-1.0f, +1.0f, +1.0f),  // 12
////        glm::vec3(+0.7f, +0.8f, +0.2f),  // Color
////        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
////        glm::vec3(-1.0f, +1.0f, -1.0f),  // 13
////        glm::vec3(+0.5f, +0.7f, +0.3f),  // Color
////        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
////        glm::vec3(-1.0f, -1.0f, -1.0f),  // 14
////        glm::vec3(+0.4f, +0.7f, +0.7f),  // Color
////        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
////        glm::vec3(-1.0f, -1.0f, +1.0f),  // 15
////        glm::vec3(+0.2f, +0.5f, +1.0f),  // Color
////        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal

////        glm::vec3(+1.0f, +1.0f, +1.0f),  // 16
////        glm::vec3(+0.6f, +1.0f, +0.7f),  // Color
////        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
////        glm::vec3(-1.0f, +1.0f, +1.0f),  // 17
////        glm::vec3(+0.6f, +0.4f, +0.8f),  // Color
////        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
////        glm::vec3(-1.0f, -1.0f, +1.0f),  // 18
////        glm::vec3(+0.2f, +0.8f, +0.7f),  // Color
////        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
////        glm::vec3(+1.0f, -1.0f, +1.0f),  // 19
////        glm::vec3(+0.2f, +0.7f, +1.0f),  // Color
////        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal

////        glm::vec3(+1.0f, -1.0f, -1.0f),  // 20
////        glm::vec3(+0.8f, +0.3f, +0.7f),  // Color
////        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
////        glm::vec3(-1.0f, -1.0f, -1.0f),  // 21
////        glm::vec3(+0.8f, +0.9f, +0.5f),  // Color
////        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
////        glm::vec3(-1.0f, -1.0f, +1.0f),  // 22
////        glm::vec3(+0.5f, +0.8f, +0.5f),  // Color
////        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
////        glm::vec3(+1.0f, -1.0f, +1.0f),  // 23
////        glm::vec3(+0.9f, +1.0f, +0.2f),  // Color
////        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
////    };



//    zaytuna::vertex verts[] =
//    {
//        glm::vec3(-0.2f, 0.1f, 0.05f),  // 0
//        glm::vec3(+1.0f, +0.0f, +0.0f),	// Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
//        glm::vec3(0.2f, 0.1f, 0.05f),  // 1
//        glm::vec3(+0.0f, +1.0f, +0.0f),	// Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
//        glm::vec3(0.2f, 0.1f, -0.05f),  // 2
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal
//        glm::vec3(-0.2f, 0.1f, -0.05f),  // 3
//        glm::vec3(+1.0f, +1.0f, +1.0f),  // Color
//        glm::vec3(+0.0f, +1.0f, +0.0f),  // Normal

//        glm::vec3(-0.2f, 0.1f, -0.05f),  // 4
//        glm::vec3(+1.0f, +0.0f, +1.0f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
//        glm::vec3(0.2f, 0.1f, -0.05f),  // 5
//        glm::vec3(+0.0f, +0.5f, +0.2f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
//        glm::vec3(0.2f, 0.0f, -0.05f),  // 6
//        glm::vec3(+0.8f, +0.6f, +0.4f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal
//        glm::vec3(-0.2f, 0.0f, -0.05f),  // 7
//        glm::vec3(+0.3f, +1.0f, +0.5f),  // Color
//        glm::vec3(+0.0f, +0.0f, -1.0f),  // Normal

//        glm::vec3(0.2f, 0.1f, -0.05f),  // 8
//        glm::vec3(+0.8f, +0.5f, +0.2f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(0.2f, 0.1f, 0.05f),  // 9
//        glm::vec3(+0.9f, +0.3f, +0.5f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(0.2f, 0.0f, 0.05f),  // 10
//        glm::vec3(+0.9f, +0.7f, +0.5f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(0.2f, 0.0f, -0.05f),  // 11
//        glm::vec3(+0.9f, +0.7f, +0.5f),  // Color
//        glm::vec3(+1.0f, +0.0f, +0.0f),  // Normal

//        glm::vec3(-0.2f, 0.1f, 0.05f),  // 12
//        glm::vec3(+0.7f, +0.8f, +0.2f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(-0.2f, 0.1f, -0.05f),  // 13
//        glm::vec3(+0.5f, +0.7f, +0.3f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(-0.2f, 0.0f, -0.05f),  // 14
//        glm::vec3(+0.4f, +0.7f, +0.7f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal
//        glm::vec3(-0.2f, 0.0f, 0.05f),  // 15
//        glm::vec3(+0.2f, +0.5f, +1.0f),  // Color
//        glm::vec3(-1.0f, +0.0f, +0.0f),  // Normal

//        glm::vec3(0.2f, 0.1f, 0.05f),  // 16
//        glm::vec3(+0.6f, +1.0f, +0.7f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
//        glm::vec3( -0.2f, 0.1f, 0.05f),  // 17
//        glm::vec3(+0.6f, +0.4f, +0.8f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
//        glm::vec3(-0.2f, 0.0f, 0.05f),  // 18
//        glm::vec3(+0.2f, +0.8f, +0.7f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal
//        glm::vec3(0.2f, 0.0f, 0.05f),  // 19
//        glm::vec3(+0.2f, +0.7f, +1.0f),  // Color
//        glm::vec3(+0.0f, +0.0f, +1.0f),  // Normal

//        glm::vec3(0.2f, 0.0f, -0.05f),  // 20
//        glm::vec3(+0.8f, +0.3f, +0.7f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//        glm::vec3(-0.2f, 0.0f, -0.05f),  // 21
//        glm::vec3(+0.8f, +0.9f, +0.5f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//        glm::vec3(-0.2f, 0.0f, 0.05f),  // 22
//        glm::vec3(+0.5f, +0.8f, +0.5f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//        glm::vec3(0.2f, 0.0f, 0.05f),  // 23
//        glm::vec3(+0.9f, +1.0f, +0.2f),  // Color
//        glm::vec3(+0.0f, -1.0f, +0.0f),  // Normal
//    };

//    unsigned int indices[] = {
//        0,   1,  2,  0,  2,  3, // Top
//        4,   5,  6,  4,  6,  7, // Front
//        8,   9, 10,  8, 10, 11, // Right
//        12, 13, 14, 12, 14, 15, // Left
//        16, 17, 18, 16, 18, 19, // Back
//        20, 22, 21, 20, 23, 22, // Bottom
//    };
//    cube.verNum = NUM_OF(verts);
//    cube.verts = new zaytuna::vertex[cube.verNum];
//    memcpy(cube.verts, verts, sizeof(verts));
//    cube.indNum = NUM_OF(indices);
//    cube.indices = new unsigned int[cube.indNum];
//    memcpy(cube.indices, indices, sizeof(indices));

//    return cube;
//}

