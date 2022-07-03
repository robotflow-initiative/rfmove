include_directories(
        include
        ${KINETIC_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/src/util
        ${PROJECT_SOURCE_DIR}/extern/kdl_kinematics_plugin/include
        ${PROJECT_SOURCE_DIR}/extern/franka_ikfast_plugin/include
        ${PROJECT_SOURCE_DIR}/extern/tobor_ikfast_plugin/include
        ${PROJECT_SOURCE_DIR}/extern/ifast_inlcude
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
)

add_library(robot_model_noros SHARED
        src/RobotModelLoader.cpp
        src/ExampleClass.cpp
        src/KinematicsLoader.cpp
        src/JointLimitsLoader.cpp
        ../util/path_util.cpp)

target_link_libraries(robot_model_noros
        ${rdf_loader_lib}
        ${robot_model_lib}
        #${YAML_CPP_LIBRARIES}
        ${kinematics_base_lib}
        ${collision_detection_lib}
        ${planning_scene_lib}
        yaml-cpp
        kdl_kinematics_noros
        franka_ik_kinematics_noros
        tobor_left_kinematics_noros
        ${lapack_lib}
        ${blas_lib}
        ${gfortran_lib})

install(TARGETS robot_model_noros
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
        
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h")