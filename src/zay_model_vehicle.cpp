

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





#include "zay_model_vehicle.hpp"
#include "zay_scene_widg.hpp"

namespace zaytuna {


////-----for--debugging-----
extern double GLOBAL_MOVEMENT_SPEED;
extern double GLOBAL_STEERING_WHEEL;



glm::dvec3 rotate(glm::dvec3 p, const glm::dvec3 c,
                  const double angle, const bool ccw);

glm::dvec3 intersect(const glm::dvec3 p1, const glm::dvec3 p2,
                     const glm::dvec3 p3, const glm::dvec3 p4);

zaytuna::vehicle_attributes::vehicle_attributes
            (USED_GL_VERSION* _widg,
             QGLFramebufferObject *const FBO_,
             const transform_attribs<GLdouble> attribs):
        _widg{_widg}, attribs{attribs},
        localView_buffer{FBO_}, elapsed_t{0.0},
        AMOUNT_OF_ROTATION(0.0), MOVEMENT_SPEED(0.0),
        REMOTE_SPEED{0.0}, STEERING_WHEEL(STEERING_MARGIN_OF_ERROR),
        REMOTE_STEERING(STEERING_MARGIN_OF_ERROR), accumulated_dist(0.0),
        traveled_dist(0.0), ticks_counter(0),
        radius_of_rotation(0.0),
        center_of_rotation(glm::dvec3(0.0,0.0,0.0)){

    frontCam.FIELD_OF_VIEW = 55.0; // should be adjusted!
    update_positional_attributes(attribs);

    transformationMats[1] = f_rightT;
    transformationMats[2] = f_leftT;
    transformationMats[3] = backT;
    transformationMats[4] = lidar;

    gps_pub = node_handle.advertise<geometry_msgs::Vector3>
            ("zaytuna/"+attribs.name+"/gps/localization",5);
    orientation_pub = node_handle.advertise<geometry_msgs::Vector3>
            ("zaytuna/"+attribs.name+"/compass/orientation",5);
    geo_pub = node_handle.advertise<geometry_msgs::Pose>
            ("zaytuna/"+attribs.name+"/geometry/pose",5);
    ticks_pub = node_handle.advertise<std_msgs::UInt32>
            ("zaytuna/"+attribs.name+"/sensors/ticks",10);

    cam_pub = node_handle.advertise<sensor_msgs::Image>
            ("zaytuna/"+attribs.name+"/sensors/front_cam",0);

    speed_sub = node_handle.subscribe
            ("zaytuna/"+attribs.name+"/controller/speed", 1,
             &vehicle_attributes::speed_callback, this);
    steering_sub = node_handle.subscribe
            ("zaytuna/"+attribs.name+"/controller/steering", 1,
             &vehicle_attributes::steering_callback, this);

    local_cam_msg.header = std_msgs::Header();
    local_cam_msg.width = WIDTH;
    local_cam_msg.height = HEIGHT;
    local_cam_msg.encoding = "rgb8";
    local_cam_msg.step = WIDTH * NUM_OF_CHANNELS;
    local_cam_msg.is_bigendian = 0;
    local_cam_msg.data.resize(FRONT_IMG_SIZE);
}
void vehicle_attributes::speed_callback
        (const std_msgs::Float64& _val){
    double val{_val.data};
    REMOTE_SPEED = -(val > 1.0? SPEED_SCALAR :
          (val<-1.0? -SPEED_SCALAR : SPEED_SCALAR*val));
}
void vehicle_attributes::steering_callback
        (const std_msgs::Float64& _val){
    double val{_val.data};
    REMOTE_STEERING = val == 0.0 ? STEERING_MARGIN_OF_ERROR :
          ((val>1.0? MAX_TURN_ANGLE:
          (val<-1.0? -MAX_TURN_ANGLE: MAX_TURN_ANGLE*val))*M_PI)/180.0;
}

void vehicle_attributes::update_positional_attributes
        (const transform_attribs<GLdouble> attribs){

    this->attribs = attribs;
    transformationMats[0] = attribs.transformMat();

    vehic_direction = glm::dvec3(1.0, 0.0, 0.0);
    back_ideal_tire = old_back_ideal_tire = glm::dvec3(0.0, 0.0, 0.0);
    front_ideal_tire = glm::dvec3(0.2862 , 0.0 , 0.0);

    rotationMat = glm::rotate(0.0, up_direction);

    vehicle_orientation = vehic_direction =
            glm::dmat3(attribs.rotationMat()) * vehic_direction;

    vehicle_pos = back_ideal_tire =
            old_back_ideal_tire = transformationMats[0]
            * glm::vec4(back_ideal_tire, 1.0);

    front_ideal_tire = transformationMats[0]
                        * glm::vec4(front_ideal_tire, 1.0);

    frontCam.camera_position = transformationMats[0] * camPos;
    frontCam.view_direction  = attribs.rotationMat()
                            * (frontCamViewDirection - camPos);
    frontCam.updateWorld_to_viewMat();

    tires_hRotation = glm::rotate(amount_of_hRotation,
                                  glm::dvec3(0.0, 0.0, 1.0));
    front_tires_vRotation = glm::rotate(STEERING_WHEEL,
                                        glm::dvec3(0.0, 1.0, 0.0));
    frontCam.updateProjection(WIDTH, HEIGHT);

    vehicle_geometry.update(back_ideal_tire, vehic_direction);
}


void vehicle_attributes::update_attribs()
{
    if(is_detached){
        MOVEMENT_SPEED = REMOTE_SPEED;
        STEERING_WHEEL = REMOTE_STEERING;
    }else{
        MOVEMENT_SPEED = GLOBAL_MOVEMENT_SPEED;
        STEERING_WHEEL = GLOBAL_STEERING_WHEEL;
    }

    if(MOVEMENT_SPEED != 0.0){

        update_rotation_att();
        accumulated_dist = traveled_dist + accumulated_dist;
        current_ticks = ticks_counter = static_cast<uint32_t>(accumulated_dist/meters_per_tick);
        accumulated_dist = fmod(accumulated_dist, meters_per_tick);
    } else{
        elapsed_t = std::chrono::duration<double,
        std::ratio< 1, 1>>
        (std::chrono::high_resolution_clock::now() - timer_t).count();
        timer_t = std::chrono::high_resolution_clock::now();
    }
    actuate();
}

void vehicle_attributes::update_rotation_att()
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

    vehicle_pos = back_ideal_tire =  rotationMat * glm::dvec4(back_ideal_tire, 1.0);
    front_ideal_tire =  rotationMat * glm::dvec4(front_ideal_tire, 1.0);

    traveled_dist = glm::radians(std::abs(AMOUNT_OF_ROTATION))
                    * radius_of_rotation;

    frontCam.camera_position = transformationMats[0] * camPos;

    vehicle_orientation = vehic_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                            up_direction))
                      * vehic_direction;
    frontCam.view_direction = glm::mat3(glm::rotate(glm::radians(AMOUNT_OF_ROTATION),
                                                    up_direction))
                              * frontCam.view_direction;
}

void vehicle_attributes::actuate()
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
                : amount_of_rotation_Lidar -= (lidar_spin_speed*elapsed_t)),
                  glm::dvec3(0.0, 1.0, 0.0));

    frontCam.updateWorld_to_viewMat();

    vehicle_geometry.update(back_ideal_tire, vehic_direction);
    geo_pub.publish(vehicle_geometry_ref);
    gps_pub.publish(pos_ref);
    orientation_pub.publish(orientation_ref);
    ticks_pub.publish(current_ticks_ref);
}

transform_attribs<GLdouble>
vehicle_attributes::current_state(void){
    GLdouble angle{glm::degrees
                (glm::angle(glm::dvec3(1.0, 0.0, 0.0),
                 vehic_direction))};
    attribs.angle = vehic_direction.z < 0.0 ? angle:-angle;
    attribs.translation_vec = back_ideal_tire;
    return attribs;
}

vehicle_attributes::~vehicle_attributes()
{
    if(localView_buffer!=nullptr)
        delete localView_buffer;
}

/////-----------removed OpenCV dependency-----------
// void vehicle_attributes::pubFront_img()
// {
//     // local_cam_img_reformed = local_cam_img.convertToFormat(QImage::Format_RGB888);
//     // cv::Mat mat(local_cam_img.height(), 
//     //             local_cam_img.width(), 
//     //             CV_8UC3, 
//     //             local_cam_img.bits(), 
//     //             local_cam_img.bytesPerLine());
//     // local_cam_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", mat).toImageMsg();
//     // cam_pub.publish(local_cam_msg);
//     //// local_cam_img.save((attribs.name+".jpg").c_str());
// }


//===================================================================
////-----------Old Approach For Ackerman Rotation----- 
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

void vehicle_attributes::update_cent()
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



} // namespace  zaytuna





