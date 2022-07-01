cmake_minimum_required(VERSION 3.12.4) #command add_compile_definitions is added in 3.12.4
project(moveit_no_ros)
include(GNUInstallDirs)
## Compile as C++11
add_compile_options(-std=c++11)

OPTION(WITH_BULLET3 "Whether build library or executable program depend on bullet3" OFF)
IF(${WITH_BULLET3})
    MESSAGE(STATUS "Build with bullet3")
    find_package(Bullet REQUIRED)
    MESSAGE(STATUS "BULLET_INCLUDE_DIR ${BULLET_INCLUDE_DIR}")
    MESSAGE(STATUS "BULLET_LIBRARIES ${BULLET_LIBRARIES}")
    ADD_DEFINITIONS(-DWITH_BULLET3)
    find_library(bullet_robotics_lib BulletRobotics REQUIRED)
    MESSAGE(STATUS "lib BulletRobotics ${bullet_robotics_lib}")
ELSE()
    MESSAGE(STATUS "Do not build with bullet3")
ENDIF()

## Build in debug mode
SET(CMAKE_BUILD_TYPE "Debug")

## Set exported head file directory
SET(EXPORT_HEAD_DIR "exported_heads")

## Let tinyxml use stl string instead of tinystr
add_compile_definitions(TIXML_USE_STL)

SET(KINETIC_LIB_DIR ${PROJECT_SOURCE_DIR}/extern/lib)

## Set kinetic lib search directory
##IF(DEFINED ENV{KINETIC_LIB_DIR})
##    SET(KINETIC_LIB_DIR $ENV{KINETIC_LIB_DIR})
##ELSE()
##    SET(KINETIC_LIB_DIR "/opt/ros/kinetic/lib")
##ENDIF()
##MESSAGE(STATUS "ROS kinetic library path: ${KINETIC_LIB_DIR}")

SET(KINETIC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/extern/include)
## Set kinetic include search directory
#IF(DEFINED ENV{KINETIC_INCLUDE_DIR})
#    SET(KINETIC_INCLUDE_DIR $ENV{KINETIC_INCLUDE_DIR})
#ELSE()
#    SET(KINETIC_INCLUDE_DIR "/opt/ros/kinetic/include")
#ENDIF()
#MESSAGE(STATUS "ROS kinetic include path: ${KINETIC_INCLUDE_DIR}")


## Set Python Version for pybind11
SET(PYBIND11_PYTHON_VERSION "3.8")

## Set yaml-cpp shared-library on
SET(BUILD_SHARED_LIBS "ON")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install")

## Show install paths
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_BINDIR: ${CMAKE_INSTALL_BINDIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")

# we use yaml-cpp as submodule
#find_package(yaml-cpp REQUIRED)
#SET(YAML_CPP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/yaml-cpp/include)
#SET(YAML_CPP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/yaml-cpp/src)

SET(YAML_CPP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/extern/yaml-cpp/include)
SET(YAML_CPP_SRC_DIR ${PROJECT_SOURCE_DIR}/extern/yaml-cpp/src)

# SET(YAML_CPP_LIB ${CMAKE_CURRENT_SOURCE_DIR}/extern/yaml-cpp/build/libyaml-cpp.a)
MESSAGE(STATUS "YAML_CPP_INCLUDE_DIR ${YAML_CPP_INCLUDE_DIR}")
#MESSAGE(STATUS "YAML_CPP_LIB ${YAML_CPP_LIB}")
MESSAGE(STATUS "YAML_CPP_SRC_DIR ${YAML_CPP_SRC_DIR}")
find_package(Threads REQUIRED)
#message(YAML_CPP_LIBRARIES ${YAML_CPP_LIBRARIES})

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
MESSAGE(STATUS "include Eigen3: ${EIGEN3_INCLUDE_DIRS}")

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_library(rdf_loader_lib moveit_rdf_loader
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_rdf_loader: ${rdf_loader_lib}")

find_library(robot_model_lib moveit_robot_model
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_robot_model: ${robot_model_lib}")

find_library(kinematics_base_lib moveit_kinematics_base
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_kinematics_base: ${kinematics_base_lib}")

find_library(orocos_kdl_lib orocos-kdl # can also installed as third party library: https://orocos.org/kdl.html
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib orocos-kdl: ${orocos_kdl_lib}")

find_library(robot_state_lib moveit_robot_state
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_robot_state: ${robot_state_lib}")

find_library(kinematics_base_lib moveit_kinematics_base
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_kinematics_base: ${kinematics_base_lib}")

find_library(kdl_parser_lib kdl_parser
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib kdl_parser: ${kdl_parser_lib}")

find_library(kdl_conversions_lib kdl_conversions
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib kdl_conversions: ${kdl_conversions_lib}")

find_library(collision_detection_lib moveit_collision_detection
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_collision_detection: ${collision_detection_lib}")

find_library(planning_scene_lib moveit_planning_scene
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_planning_scene: ${planning_scene_lib}")

find_library(planning_interface_lib moveit_planning_interface
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_planning_interface: ${planning_interface_lib}")

find_library(ompl_interface_lib moveit_ompl_interface
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_ompl_interface: ${ompl_interface_lib}")

find_library(constraint_samplers_lib moveit_constraint_samplers
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_constraint_samplers: ${constraint_samplers_lib}")

find_library(robot_trajectory_lib moveit_robot_trajectory
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_robot_trajectory: ${robot_trajectory_lib}")

find_library(trajectory_processing_lib moveit_trajectory_processing
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_trajectory_processing: ${trajectory_processing_lib}")

find_library(lapack_lib lapack
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lapack: ${lapack_lib}")

find_library(blas_lib blas
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "blas: ${blas_lib}")

find_library(gfortran_lib gfortran
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "gfortran: ${gfortran_lib}")

find_package(Python ${PYBIND11_PYTHON_VERSION} REQUIRED COMPONENTS Interpreter Development)
MESSAGE(STATUS "Python_SITELIB: ${Python_SITELIB}")

# message(${kdl_solver_plugin})
# file(WRITE resources/plugins.yaml "kdl_kinematics_plugin/KDLKinematicsPlugin: ${kdl_solver_plugin}\n")

function(path_indent arg_path)
    execute_process(COMMAND sh path_indent.sh "${arg_path}" ";" "......" OUTPUT_VARIABLE shell_out WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    if (NOT ${shell_out} EQUAL "") # It may raise an error if argument of message is empty.
        message(${shell_out})
    endif (NOT ${shell_out} EQUAL "")
endfunction()

add_subdirectory(extern/pybind11)
add_subdirectory(src/yaml-cpp)
add_subdirectory(extern/kdl_kinematics_plugin)
add_subdirectory(extern/tobor_ikfast_plugin)
add_subdirectory(extern/franka_ikfast_plugin)
add_subdirectory(src/robot_model)
add_subdirectory(src/planner)
add_subdirectory(src/controller)
add_subdirectory(src/python)


# Include Directories used for executable demo
include_directories(
    ${KINETIC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    src/robot_model/include
    src/planner/include
    src/rokae/include
    extern
    ${PROJECT_SOURCE_DIR}/extern/include
)

# Compile demo executable
#add_executable(model_and_state_demo src/model_and_state_demo.cpp)
#target_link_libraries(model_and_state_demo
#        robot_model_noros)

#add_executable(planner_demo src/planner_demo.cpp)
#target_link_libraries(planner_demo
#        planner_noros
#        robot_model_noros
#        ${Boost_SYSTEM_LIBRARY})

#add_executable(rokae_client_demo src/rokae_client_demo.cpp)
#target_link_libraries(rokae_client_demo
#        robot_model_noros
#        rokae_client)

#add_executable(rokae_server_demo src/rokae_server_demo.cpp)
#target_link_libraries(rokae_server_demo
#        rokae_server)

add_executable(urdf_converter src/util/urdf_converter.cpp)


install(TARGETS  urdf_converter
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY resources
        DESTINATION ${CMAKE_INSTALL_BINDIR})
