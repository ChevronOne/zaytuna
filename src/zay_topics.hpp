

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





#ifndef ZAY_TOPICS_HPP
#define ZAY_TOPICS_HPP


#include "zay_cam.hpp"


namespace zaytuna {


class vehicle_attributes;
enum class ZAY_PUB_TOPICS {GPS_PUB, TICKS_PUB, ORIENTATION_PUB, COLLISION_PUB, GEO_PUB, CAM_PUB};
enum class ZAY_SUB_TOPICS {SPEED_SUB, STEERING_SUB};

template<class precision_type, class _allocator>
class vehicle_topics final
{
    typedef glm::vec<ZAY_POINT_D, precision_type, glm::qualifier::packed_highp> zay_vert;
    ros::NodeHandlePtr n_handle{nullptr};
    

    zaytuna::vec3<_allocator> vehicle_pos;
    geometry_msgs::Vector3_<_allocator>& pos_ref{vehicle_pos};
    zaytuna::vec3<_allocator> vehicle_orientation;
    geometry_msgs::Vector3_<_allocator>& orientation_ref{vehicle_orientation};
    zaytuna::geo_pose<_allocator> vehicle_geometry;
    zaytuna::Pose_<_allocator>& vehicle_geometry_ref{vehicle_geometry};
    zaytuna::uint32<_allocator> current_ticks;
    std_msgs::UInt32_<_allocator>& current_ticks_ref{current_ticks};
    zaytuna::Bool<_allocator> is_collided;
    std_msgs::Bool_<_allocator>& is_collided_ref{is_collided};

    boost::array<ros::Publisher, ZAY_PUBLISHERS> zay_publishers;
    boost::array<ros::Subscriber, ZAY_SUBSCRIBERS> zay_subscribers;
    
    sensor_msgs::Image_<_allocator> local_cam_msg;
    QImage local_cam_img{QImage(ZAY_SCENE_WIDTH, 
                                ZAY_SCENE_HEIGHT, 
                                QImage::Format_RGB888)};

    const boost::array<const std::function<void (void)>, ZAY_PUBLISHERS> 
        advertisers = {
            [this](void) -> void { zay_publishers[(int)ZAY_PUB_TOPICS::GPS_PUB].publish(pos_ref);},
            [this](void) -> void { zay_publishers[(int)ZAY_PUB_TOPICS::GEO_PUB].publish(vehicle_geometry_ref);},
            [this](void) -> void { zay_publishers[(int)ZAY_PUB_TOPICS::TICKS_PUB].publish(current_ticks_ref);},
            [this](void) -> void { zay_publishers[(int)ZAY_PUB_TOPICS::ORIENTATION_PUB].publish(orientation_ref);},
            [this](void) -> void { zay_publishers[(int)ZAY_PUB_TOPICS::COLLISION_PUB].publish(is_collided_ref);},
            [this](void) -> void { zay_publishers[(int)ZAY_PUB_TOPICS::CAM_PUB].publish(local_cam_msg);}
        };


    std::string vehicle_id{"undefined_vehicle_name"};
    QGLFramebufferObject* localView_buffer{nullptr};
    precision_type REMOTE_SPEED{0.0}, REMOTE_STEERING{ZAY_STEERING_MARGIN_OF_ERROR};


    inline void grab_buffer(void);

    inline void advertise_current_state(const zay_vert&,
                                        const zay_vert&, 
                                        const zay_vert&);
    
    inline void shutdown(void);

    inline bool ok(void);

    template<class type_t>
    void set_remoteSpeed(const type_t&);

    template<class type_t>
    void set_remoteSteering(const type_t&);


    template<class T>
    typename std::enable_if<std::is_same<T, double>::value, void>::type
    speed_callback(const std_msgs::Float64&);

    template<class T>
    typename std::enable_if<std::is_same<T, float>::value, void>::type
    speed_callback(const std_msgs::Float32&);

    template<class T>
    typename std::enable_if<std::is_same<T, double>::value, void>::type
    steering_callback(const std_msgs::Float64&);

    template<class T>
    typename std::enable_if<std::is_same<T, float>::value, void>::type
    steering_callback(const std_msgs::Float32&);


    friend class vehicle_attributes;



public:
    vehicle_topics() = default;
    explicit vehicle_topics(const std::string& , QGLFramebufferObject *const);
    ~vehicle_topics();


};



} // namespace zaytuna



#include "zay_topics_def.hpp"




#endif // ZAY_TOPICS_HPP




