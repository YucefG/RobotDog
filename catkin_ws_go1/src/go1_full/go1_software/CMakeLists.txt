cmake_minimum_required(VERSION 2.8.3)
project(go1_software)
#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals -O3") 

list(APPEND CMAKE_PREFIX_PATH "/data/libtorch")

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

find_package(Torch REQUIRED)
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS})
if (NOT Torch_FOUND)
    message(FATAL_ERROR "libtorch not found, will not build.")
else ()
    message(STATUS "++++ found libtorch ")
endif ()

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ../unitree_legged_sdk/include
    ../unitree_ros_to_real/unitree_legged_real/include
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}
    src/body.cpp
    src/CurrentState.cpp
    src/LegController.cpp
    include/ControlFSMData.h
    src/PositionVelocityEstimator.cpp
    src/OrientationEstimator.cpp
    CPG/HopfPolar.cpp
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS} ${TORCH_LIBRARIES}
)

#add_executable(go1_servo src/exe/test_servo.cpp)
#target_link_libraries(go1_servo ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(go1_cs src/exe/test_current_state.cpp)
#target_link_libraries(go1_cs ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(go1_setup src/exe/test_setup.cpp)
#target_link_libraries(go1_setup ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(run_cpg src/exe/run_cpg.cpp)
target_link_libraries(run_cpg ${PROJECT_NAME} ${catkin_LIBRARIES} "${TORCH_LIBRARIES}")
set_property(TARGET run_cpg PROPERTY CXX_STANDARD 14)

add_executable(balance3leg src/exe/balance3leg.cpp)
target_link_libraries(balance3leg ${PROJECT_NAME} ${catkin_LIBRARIES} "${TORCH_LIBRARIES}")
set_property(TARGET balance3leg PROPERTY CXX_STANDARD 14)

################################################### [TODO] clean up below
################################################### add LCM related quantities here 
include(FindPkgConfig)
pkg_check_modules(LCM lcm REQUIRED)
include_directories(include ${LCM_INCLUDE_DIRS})

add_compile_options(-std=c++11)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

################################################### Read with ROS, send with LCM

include_directories(/data/$ENV{USER}/catkin_ws/src/go1_full/unitree_legged_sdk/include)
link_directories(/data/$ENV{USER}/catkin_ws/src/go1_full/unitree_legged_sdk/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

add_executable(example_pos_lcm src/exe/example_pos_lcm.cpp)
target_link_libraries(example_pos_lcm ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})
add_dependencies(example_pos_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(example_torque_lcm src/exe/example_torque.cpp)
target_link_libraries(example_torque_lcm ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})
add_dependencies(example_torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(example_walk_lcm src/exe/example_walk_lcm.cpp)
target_link_libraries(example_walk_lcm ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})
add_dependencies(example_walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(run_cpg_lcm src/exe/run_cpg_lcm.cpp)
target_link_libraries(run_cpg_lcm ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS} "${TORCH_LIBRARIES}")
add_dependencies(run_cpg_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_property(TARGET run_cpg_lcm PROPERTY CXX_STANDARD 14)

add_executable(camera_command_lcm src/exe/camera_command_lcm.cpp)
target_link_libraries(camera_command_lcm ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})
add_dependencies(camera_command_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_demo_2 src/exe/controller_demo_2.cpp)
target_link_libraries(controller_demo_2 ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})
add_dependencies(controller_demo_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_demo_3 src/exe/controller_demo_3.cpp)
target_link_libraries(controller_demo_3 ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})
add_dependencies(controller_demo_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_demo_4 src/exe/controller_demo_4.cpp)
target_link_libraries(controller_demo_4 ${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})
add_dependencies(controller_demo_4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})