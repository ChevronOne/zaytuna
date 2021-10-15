
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





#ifndef ZAY_COLL_TESTER_HPP
#define ZAY_COLL_TESTER_HPP


#include "zay_topics.hpp"

namespace zaytuna {


template<class vert_type>
class coll_tester{

    typedef glm::vec<ZAY_POINT_D, vert_type, glm::qualifier::packed_highp> coll_vert;
    typedef std::chrono::time_point<std::chrono::high_resolution_clock,
                    std::chrono::nanoseconds> nanSec;
    typedef std::chrono::duration<double,
                        std::ratio< 1, 1>> durSec;

    bool terminate_{0};
    bool is_terminated{0};
    std::mutex mLock;
    std::condition_variable condition;

    std::unique_ptr<std::thread> collision_tester{nullptr};
    std::string associated_obj{"unkown associated object!"};


    void _tester(std::string const * obj_name,
                bool* is_collided,
                double const * elapsed_t,
                std::string* collided_object,
                rect_collistion_object<vert_type> const * coll_rect,
                rect_collistion_pack<vert_type> const * _pack){ 
        std::unique_lock<std::mutex> locker(mLock);
        nanSec local_timer_t;

        double local_elapsed_t{std::numeric_limits<double>::max()};
        double glob_elapsed_t{0};

        do{
            local_timer_t = std::chrono::high_resolution_clock::now();

            for(uint32_t i{0}; i<_pack->static_objs.size(); ++i){

                if(axis_inter(coll_rect->points, _pack->static_objs[i].points, coll_rect->uniqueV[0]) |
                   axis_inter(coll_rect->points, _pack->static_objs[i].points, coll_rect->uniqueV[1]) |
                   axis_inter(coll_rect->points, _pack->static_objs[i].points, _pack->static_objs[i].uniqueV[0]) |
                   axis_inter(coll_rect->points, _pack->static_objs[i].points, _pack->static_objs[i].uniqueV[1]))
                    continue;

                *collided_object = _pack->static_objs[i].ID;
                ROS_WARN_STREAM("Collistion! " << *obj_name << " > " << *collided_object);

                *is_collided = 1;
                break;
            }

            if((_pack->dyn_objs.size()>1)& !(*is_collided) ){

                for(uint32_t i{0}; i<_pack->dyn_objs.size(); ++i){

                    if((_pack->dyn_objs[i]->ID == *obj_name) |
                        axis_inter(coll_rect->points, _pack->dyn_objs[i]->points, coll_rect->uniqueV[0]) |
                        axis_inter(coll_rect->points, _pack->dyn_objs[i]->points, coll_rect->uniqueV[1]) |
                        axis_inter(coll_rect->points, _pack->dyn_objs[i]->points, _pack->dyn_objs[i]->uniqueV[0])   |
                        axis_inter(coll_rect->points, _pack->dyn_objs[i]->points, _pack->dyn_objs[i]->uniqueV[1]))
                        continue;

                    *collided_object = _pack->dyn_objs[i]->ID;
                    ROS_WARN_STREAM("Collistion! " << *obj_name << " > " << *collided_object);

                    *is_collided = 1;
                    break;
                }
            }

            local_elapsed_t = durSec(std::chrono::high_resolution_clock::now() - local_timer_t).count();
            glob_elapsed_t = *elapsed_t;

            if(local_elapsed_t<glob_elapsed_t)
                std::this_thread::sleep_for(std::chrono::nanoseconds( int((glob_elapsed_t-local_elapsed_t)*1000000000) ));

        }while(!terminate_ & !(*is_collided));

        is_terminated = 1;
        locker.unlock();
        condition.notify_one();

    }

    bool axis_inter(const boost::array<coll_vert, ZAY_RECTANGLE_P>& collision_obj,
                    const boost::array<coll_vert, ZAY_RECTANGLE_P>& traget_obj,
                    const glm::dvec3 axis) const{

        std::multiset<projection_1d<vert_type>> order; 
    
        coll_vert proj;
        for(const coll_vert& vec:traget_obj){
            proj = glm::proj(vec, axis);
            order.insert(projection_1d<vert_type>(proj.x+proj.z, zaytuna::Collider_type::COUNTERPART));

        }

        for(const coll_vert& vec:collision_obj){
            proj = glm::proj(vec, axis);
            order.insert(projection_1d<vert_type>(proj.x+proj.z));

        }

        auto it{order.begin()};
        zaytuna::Collider_type _t = it->c_type;
        for(uint32_t i{1}; i<ZAY_RECTANGLE_P; ++i){
            ++it;

            if(_t != it->c_type)
                return 0;
        }

        return 1;
    }


public:
    coll_tester()=delete;
    coll_tester(std::string const * obj_name,
                bool* is_collided,
                double const * elapsed_t,
                std::string* collided_object,
                rect_collistion_object<vert_type> const * coll_rect,
                rect_collistion_pack<vert_type> const * coll_pack):associated_obj{*obj_name}{
        if(collision_tester == nullptr)
            collision_tester = std::make_unique<std::thread>
                (&coll_tester<vert_type>::_tester, this, obj_name, is_collided, elapsed_t, 
                collided_object, coll_rect, coll_pack);
    }

    void detach(){
        if(collision_tester == nullptr | !collision_tester->joinable()){
            ROS_ERROR_STREAM("Thread associated with <" << associated_obj << "> is not detachable!");
            return;
        }
        collision_tester->detach();
    }

    void join(){
        if(collision_tester == nullptr | !collision_tester->joinable()){
            ROS_ERROR_STREAM("Thread associated with <" << associated_obj << "> is not joinable!");
            return;
        }
        collision_tester->join();
    }

    void terminate(){
        terminate_ = 1;
    }

    virtual ~coll_tester(){
        terminate_=1;
        if(!is_terminated){
            std::unique_lock<std::mutex> locker(mLock);
            condition.wait(locker, [&] { return is_terminated; });
        }
    }

};




} // namespace zaytuna


#endif // ZAY_COLL_TESTER_HPP


