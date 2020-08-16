

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





#include "zay_utility.hpp"

#ifndef BOOST_SPIRIT_X3_SEMANTIC_ACTION


BOOST_FUSION_ADAPT_STRUCT(glm::vec3, x,y,z)
BOOST_FUSION_ADAPT_STRUCT(glm::vec2, x,y)
BOOST_FUSION_ADAPT_STRUCT(zaytuna::OBJ, positions, texCoords, normals, faces)


namespace zaytuna {

namespace Parser {
    auto position = x3::rule<struct _position, glm::vec3> {"position"}
                = "v" >> x3::float_ >> x3::float_ >> x3::float_;
    auto texCoord = x3::rule<struct _texCoord, glm::vec2> {"texCoord"}
                = "vt" >> x3::float_ >> x3::float_;
    auto normal = x3::rule<struct _normal, glm::vec3> {"normal"}
                = "vn" >> x3::float_ >> x3::float_ >> x3::float_;
    auto vertex = x3::uint_ >> '/' >> x3::uint_ >> '/' >> x3::uint_;
    auto faces  = x3::rule<struct _faces, std::vector<uint32_t>> {"faces"}
                = *("f" >> *vertex >> (x3::eoi|+x3::eol));
    auto skipper = x3::blank | '#' >> *(x3::char_ - x3::eol) >> (x3::eol|x3::eoi);
    
    auto  OBJ = x3::rule<struct _OBJ, zaytuna::OBJ> {"OBJ"}
            = x3::skip(skipper) [ *x3::eol >> (
            +(position >> (x3::eoi|+x3::eol)) >> 
            *(texCoord >> (x3::eoi|+x3::eol)) >> 
            *(normal >> (x3::eoi|+x3::eol)) >> 
            faces
        )];
}

/////-------parse only one object per OBJ----------
shape_data<vertexL1_16> obj_parser::extractExternal
    (const std::string& _dir){

    std::ifstream f_stream(_dir, std::ios::in);
    if( !f_stream.is_open())
        throw std::runtime_error("could not open file <"+_dir+">\n");
    
    boost::spirit::istream_iterator _head(f_stream >> std::noskipws), _tail;
    OBJ primitives;
    if (x3::parse(_head, _tail, Parser::OBJ, primitives)){
        std::for_each(primitives.faces.begin(), primitives.faces.end(), 
                [](uint32_t& val) { val-=1; });
    } else{
        std::cerr << "failed to parse file <" << _dir << ">\n";
        exit(EXIT_FAILURE);
    }
    if (_head!=_tail){
        std::cerr << "WARNING: file did not parsed properly <"
                  << _dir << ">\n" << std::flush;
        std::cout << "Unparsed: " << std::distance(_head,_tail)
                  << " characters remained unparsed! object may not be rendered properly!\n" << std::flush;
    }
    
    shape_data<vertexL1_16> _object;
    _object.verNum = _object.indNum =  primitives.faces.size()/3;
    _object.verts = new vertexL1_16[_object.verNum];

    for(uint32_t i{0}, j{0}; i<_object.verNum; ++i, j+=3 )
        _object.verts[i] = {primitives.positions[primitives.faces[j]], 
                            primitives.normals[primitives.faces[j+2]],
                            primitives.texCoords[primitives.faces[j+1]]};

    _object.indices = new unsigned int[_object.indNum];
    std::iota(_object.indices, _object.indices+_object.indNum, 0);

    return _object;
}

#else
namespace zaytuna {


////--------parse multiple objects per OBJ-----------
shape_data<vertexL1_16> obj_parser::extractExternal
    (const std::string& _dir){

    std::ifstream f_stream(_dir, std::ios::in);
    if( !f_stream.is_open()){
        std::cerr << "could not open file <" << _dir << ">\n";
        exit(EXIT_FAILURE);
    }

    std::vector<float> positions, texCoords, normals;
    std::vector<uint32_t> faces;
    auto sequence_f = [](auto lemma, auto& container, const uint32_t repeats){
        auto capturer = [&container](auto& ctx){ container.emplace_back(x3::_attr(ctx)); };
        return lemma >> x3::repeat(repeats) [x3::float_[capturer]];
    };
    auto sequence_i = [](auto lemma, auto& container){
        auto capturer = [&container](auto& ctx){ container.emplace_back(x3::_attr(ctx)-1); };
        return lemma >> *(x3::uint_[capturer]%"/"); 
    };

    auto skipper = x3::blank | '#'
                   >> *(x3::char_ - x3::eol)
                   >> (x3::eol|x3::eoi);

    auto OBJ = x3::skip(skipper) [ *x3::eol >> *(
            +(sequence_f( "v", positions, 3)  >> (x3::eoi|+x3::eol)) >>
            *(sequence_f("vt", texCoords, 2)  >> (x3::eoi|+x3::eol)) >>
            *(sequence_f("vn",   normals, 3)  >> (x3::eoi|+x3::eol)) >>
            *(sequence_i("f", faces) >> (x3::eoi|+x3::eol))
        )];

   boost::spirit::istream_iterator _head(f_stream
                                   >> std::noskipws), _tail;
   shape_data<vertexL1_16> _object;

   if (x3::parse(_head, _tail, OBJ)) {

       _object.verNum = _object.indNum = faces.size()/3;
       _object.verts = new vertexL1_16[_object.verNum];
       glm::vec3 *pos_arr{reinterpret_cast<glm::vec3*>(positions.data())},
                 *norm_arr{reinterpret_cast<glm::vec3*>(normals.data())};
       glm::vec2 *tex_arr{reinterpret_cast<glm::vec2*>(texCoords.data())};

       for(uint32_t i{0}, j{0}; i<_object.verNum; ++i, j+=3 )
           _object.verts[i] = {pos_arr[faces[j]],
                               norm_arr[faces[j+2]],
                               tex_arr[faces[j+1]]
                              };

       _object.indices = new unsigned int[_object.indNum];
       std::iota(_object.indices, _object.indices+_object.indNum, 0);
   } else{
       std::cerr << "failed to parse file <" << _dir << ">\n";
       exit(EXIT_FAILURE);
   }
   if (_head!=_tail){
       std::cerr << "WARNING: file did not parsed properly <"
                 << _dir << ">\n" << std::flush;
       std::cout << "Unparsed: " << std::distance(_head,_tail)
                 << " characters remained unparsed! object may not be rendered properly!\n" << std::flush;
   }
   return _object;
}

#endif


void _load_tex(QImage& buff,
               const QString& _dir,
               const char* _format,
               bool hMir, bool vMir)
{
    if(!(buff.load(_dir, _format))){
        std::cout << "image couldn't be loaded <"
                  << _dir.toStdString() << ">!\n";
        exit(EXIT_FAILURE);
    }

    buff = QGLWidget::convertToGLFormat(buff.mirrored(hMir, vMir));
    if(buff.isNull()){
        std::cout << "error occurred while converting the image <"
                  <<_dir.toStdString() <<">!\n";
        exit(EXIT_FAILURE);
    }
}

const char *DebugGLerr(unsigned GL_enum)
{

    switch( GL_enum ){
        case 0:      return "GL Error Message: GL_NO_ERROR";
        case 0x0500: return "GL Error Message: GL_INVALID_ENUM";
        case 0x0501: return "GL Error Message: GL_INVALID_VALUE";
        case 0x0502: return "GL Error Message: GL_INVALID_OPERATION";
        case 0x0503: return "GL Error Message: GL_STACK_OVERFLOW";
        case 0x0504: return "GL Error Message: GL_STACK_UNDERFLOW";
        case 0x0505: return "GL Error Message: GL_OUT_OF_MEMORY";

        default:     return "GL Error Message: unknown error value";
    }
}



} // namespace  zaytuna













