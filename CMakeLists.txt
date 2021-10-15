cmake_minimum_required(VERSION 3.0.2)
project(zaytuna)


set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-O2 -Wall)
# add_compile_options(-g2 -Wall)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()


find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui OpenGL
            # PATHS <path-to-Qt>
            # HINTS "<path-to-Qt>"
)


#-----------removed boost's dynamiclay linked dependency-----------
# find_package(Boost COMPONENTS thread chrono system date_time atomic REQUIRED)


#-----------removed OpenCV dependency-----------
# find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    std_msgs
    message_generation
    geometry_msgs
    
    # image_transport
    # cv_bridge
)

# set(CMAKE_CXX_FLAGS "-pthread")
# set(CMAKE_EXE_LINKER_FLAGS "-pthread")

add_message_files(
    FILES
    Localization.msg
    Pose.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
    roscpp 
    rospy 
    roslib 
    std_msgs

    # image_transport 
    # cv_bridge
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  
  # ${Boost_INCLUDE_DIR}
)


set(zaytuna_ui 
    src/zay_primary_win.ui
    src/zay_item_inputs_form.ui
    src/zay_obstacle_inputs_form.ui
  )

set(zaytuna_headers 
    src/zay_cam.hpp
    src/zay_commons.hpp
    src/zay_item.hpp
    src/zay_item_inputs_form.hpp
    src/zay_model_vehicle.hpp
    src/zay_obstacle_inputs_form.hpp
    src/zay_primary_win.hpp
    src/zay_scene_widg.hpp
    src/zay_shape_data.hpp
    src/zay_shape_maker.hpp
    src/zay_utilities.hpp
    src/zay_vertex.hpp
    src/zay_topics.hpp
    src/zay_topics_def.hpp
    src/zay_coll_tester.hpp
    src/zay_app.hpp
  )

set(zaytuna_sources 
    src/zay_cam.cpp
    src/zay_item.cpp
    src/zay_item_inputs_form.cpp
    src/zay_model_vehicle.cpp
    src/zay_obstacle_inputs_form.cpp
    src/zay_primary_win.cpp
    src/zay_scene_widg.cpp
    src/zay_shape_maker.inl
    src/zay_utilities.cpp
    src/zay_app.cpp
    src/zaytuna.cpp
  )


  
add_executable(zay_simu ${zaytuna_ui} ${zaytuna_headers} ${zaytuna_sources})

target_link_libraries(zay_simu 
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    ${catkin_LIBRARIES}

    # ${OpenCV_LIBS}
    # ${Boost_LIBRARIES}
    )

add_dependencies(zay_simu zaytuna_generate_messages_cpp)

    

