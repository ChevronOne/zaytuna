

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




#ifndef ZAY_TOPICS_DEF_HPP
#define ZAY_TOPICS_DEF_HPP




namespace zaytuna{


template<class precision_type, class _allocator>
inline vehicle_topics<precision_type, _allocator>::vehicle_topics(const std::string& _id, 
                            QGLFramebufferObject *const FBO_)
                :vehicle_id{_id}, localView_buffer{FBO_}{
    
    n_handle.reset(new ros::NodeHandle);

    zay_publishers[(int)ZAY_PUB_TOPICS::GPS_PUB] = n_handle->advertise<geometry_msgs::Vector3>
                                                    ("zaytuna/"+vehicle_id+"/gps/localization", 5);
    zay_publishers[(int)ZAY_PUB_TOPICS::GEO_PUB] = n_handle->advertise<geometry_msgs::Pose>
                                                    ("zaytuna/"+vehicle_id+"/geometry/pose", 5);
    zay_publishers[(int)ZAY_PUB_TOPICS::TICKS_PUB] = n_handle->advertise<std_msgs::UInt32>
                                                    ("zaytuna/"+vehicle_id+"/sensors/ticks", 10);
    zay_publishers[(int)ZAY_PUB_TOPICS::ORIENTATION_PUB] = n_handle->advertise<geometry_msgs::Vector3>
                                                        ("zaytuna/"+vehicle_id+"/compass/orientation", 5);
    zay_publishers[(int)ZAY_PUB_TOPICS::COLLISION_PUB] = n_handle->advertise<std_msgs::Bool>
                                                        ("zaytuna/"+vehicle_id+"/sensors/is_collided", 0);
    zay_publishers[(int)ZAY_PUB_TOPICS::CAM_PUB] = n_handle->advertise<sensor_msgs::Image>
                                                   ("zaytuna/"+vehicle_id+"/sensors/front_cam/image_raw", 0);
    


    zay_subscribers[(int)ZAY_SUB_TOPICS::SPEED_SUB] = n_handle->subscribe
            ("zaytuna/"+vehicle_id+"/controller/speed", 1,
             &vehicle_topics::speed_callback<precision_type>, this);
    zay_subscribers[(int)ZAY_SUB_TOPICS::STEERING_SUB] = n_handle->subscribe
            ("zaytuna/"+vehicle_id+"/controller/steering", 1,
             &vehicle_topics::steering_callback<precision_type>, this);


    local_cam_msg.header = std_msgs::Header();
    local_cam_msg.width = ZAY_SCENE_WIDTH;
    local_cam_msg.height = ZAY_SCENE_HEIGHT;
    local_cam_msg.encoding = "rgb8";
    local_cam_msg.step = ZAY_SCENE_WIDTH * ZAY_NUM_OF_CHANNELS;
    local_cam_msg.is_bigendian = 0;
    local_cam_msg.data.resize(ZAY_FRONT_IMG_SIZE);    

                
}


template<class precision_type, class _allocator>
inline void vehicle_topics<precision_type, _allocator>::grab_buffer(void){

    local_cam_img = localView_buffer->toImage().convertToFormat(QImage::Format_RGB888);

    memcpy((char*)local_cam_msg.data.data(), 
        local_cam_img.bits(), 
        ZAY_FRONT_IMG_SIZE);

    advertisers.back()();
    // local_cam_img.save((attribs.name+".jpg").c_str());

}
    


template<class precision_type, class _allocator>
inline void vehicle_topics<precision_type, _allocator>::advertise_current_state(const zay_vert& veh_position, 
                                                                                const zay_vert& veh_direction){

                                
    vehicle_geometry.update(veh_position, veh_direction);

    for(uint32_t i{0}; i<ZAY_PUBLISHERS-1; ++i)
        advertisers[i]();


}



template<class precision_type, class _allocator>
inline void vehicle_topics<precision_type, _allocator>::shutdown(){

    n_handle->shutdown();

}


template<class precision_type, class _allocator>
inline bool vehicle_topics<precision_type, _allocator>::ok(){

    return n_handle->ok();

}


template<class precision_type, class _allocator> 
template<class type_t>
inline void vehicle_topics<precision_type, _allocator>::set_remoteSpeed(const type_t& _val){
        
    precision_type val{_val.data};

    if(val>1.0){

        REMOTE_SPEED = -ZAY_SPEED_SCALAR;
        ROS_WARN_STREAM(vehicle_id << ": invalid value for speed received <"<<val<<">");

    }else if(val<-1.0){

        REMOTE_SPEED = ZAY_SPEED_SCALAR;
        ROS_WARN_STREAM(vehicle_id << ": invalid value for speed received <"<<val<<">");

    }else{

        REMOTE_SPEED = -(ZAY_SPEED_SCALAR*val);

    }

}




template<class precision_type, class _allocator> 
template<class type_t>
inline void vehicle_topics<precision_type, _allocator>::set_remoteSteering(const type_t& _val){

        
    precision_type val{_val.data};

    if(val==0.0){

        REMOTE_STEERING = ZAY_STEERING_MARGIN_OF_ERROR;

    }else if(val>1.0){

        REMOTE_STEERING = ZAY_MAX_TURN_ANGLE_RAD;
        ROS_WARN_STREAM(vehicle_id << ": invalid value for steering received <"<<val<<">");

    }else if(val<-1.0){

        REMOTE_STEERING = -ZAY_MAX_TURN_ANGLE_RAD;
        ROS_WARN_STREAM(vehicle_id << ": invalid value for steering received <"<<val<<">");

    }else{

        REMOTE_STEERING = ZAY_MAX_TURN_ANGLE_RAD*val;

    }

}


template<class precision_type, class _allocator>
inline vehicle_topics<precision_type, _allocator>::~vehicle_topics(){

    if(localView_buffer!=nullptr)
        delete localView_buffer;

}


template<class precision_type, class _allocator>
template<class T>
typename std::enable_if<std::is_same<T, double>::value, void>::type
inline vehicle_topics<precision_type, _allocator>::speed_callback(const std_msgs::Float64& _val){

    set_remoteSpeed<std_msgs::Float64>(_val);

}


template<class precision_type, class _allocator>
template<class T>
typename std::enable_if<std::is_same<T, float>::value, void>::type
inline vehicle_topics<precision_type, _allocator>::speed_callback(const std_msgs::Float32& _val){

    set_remoteSpeed<std_msgs::Float32>(_val);

}


template<class precision_type, class _allocator>
template<class T>
typename std::enable_if<std::is_same<T, double>::value, void>::type
inline vehicle_topics<precision_type, _allocator>::steering_callback(const std_msgs::Float64& _val){

    set_remoteSteering<std_msgs::Float64>(_val);

}


template<class precision_type, class _allocator>
template<class T>
typename std::enable_if<std::is_same<T, float>::value, void>::type
inline vehicle_topics<precision_type, _allocator>::steering_callback(const std_msgs::Float32& _val){

    set_remoteSteering<std_msgs::Float32>(_val);
    
}




} // namespace zaytuna




#endif // ZAY_TOPICS_DEF_HPP



