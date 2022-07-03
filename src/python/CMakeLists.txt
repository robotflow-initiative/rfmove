include_directories(
        ${KINETIC_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/src/robot_model/include
        ${PROJECT_SOURCE_DIR}/src/planner/include
        ${PROJECT_SOURCE_DIR}/src/controller/include
        ${PROJECT_SOURCE_DIR}/src/util
        ${PROJECT_SOURCE_DIR}/extern/kdl_kinematics_plugin/include
        ${PROJECT_SOURCE_DIR}/extern/franka_ikfast_plugin/include
        ${PROJECT_SOURCE_DIR}/extern/tobor_ikfast_plugin/include
        ${PROJECT_SOURCE_DIR}/extern/ifast_inlcude
        include_python
        include
        include_python/rfmove_np
        ${EIGEN3_INCLUDE_DIR}
        ../../extern
        ${YAML_CPP_INCLUDE_DIR}
)

IF(${WITH_BULLET3})
    include_directories(
            ${BULLET_INCLUDE_DIR}
    )
ENDIF()


#TODO: Link robot model and planner as shared library
SET(ROBOT_MODEL_SRC
    ../robot_model/src/ExampleClass.cpp
    ../robot_model/src/JointLimitsLoader.cpp
    ../robot_model/src/KinematicsLoader.cpp
    ../robot_model/src/RobotModelLoader.cpp
)
SET(PLANNER_SRC
    ../planner/src/PlannerConfiguration.cpp
    ../planner/src/PlannerManager.cpp)


# Note
#   Dependency of pybind11 module must be declared as PRIVATE.
#   The target name must be same as module name defined in source file.
pybind11_add_module(moveit_noros
        ${ROBOT_MODEL_SRC}
        ${PLANNER_SRC}
        # src/EigenPy.cpp # head only
        ../util/path_util.cpp
        src/hardware_pybullet.cpp
        src/helper_pybullet.cpp
        RobotModelLoaderPy.cpp)
target_link_libraries(moveit_noros PRIVATE
        ${rdf_loader_lib}
        ${robot_model_lib}
        #${YAML_CPP_LIBRARIES}
        yaml-cpp
        ${kinematics_base_lib}
        ${collision_detection_lib}
        ${planning_scene_lib}
        ${planning_interface_lib}
        ${ompl_interface_lib}
        ${constraint_samplers_lib}
        ${robot_trajectory_lib}
        ${trajectory_processing_lib}
        kdl_kinematics_noros
        franka_ik_kinematics_noros
        tobor_left_kinematics_noros
        controller_noros
        ${lapack_lib}
        ${blas_lib}
        ${gfortran_lib})        

IF(${WITH_BULLET3})
    MESSAGE("Build moveit_noros with bullet robotics from ${bullet_robotics_lib}")
    target_link_libraries(moveit_noros PRIVATE
        moveit_bullet3
        #${bullet_robotics_lib} # Already linked through moveit_bullet3
    )
    #target_compile_options(moveit_noros PRIVATE -fPIC)
ENDIF()

# Install to python site-packages directory
install(TARGETS moveit_noros
        LIBRARY DESTINATION ${Python_SITELIB})

# Install to install directory within repo
install(TARGETS moveit_noros
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

#install(DIRECTORY include_python/
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#        FILES_MATCHING PATTERN "*.h")

