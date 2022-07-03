include_directories(
    include
    ${KINETIC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(kdl_kinematics_noros SHARED src/kdl_kinematics_plugin.cpp
         src/chainiksolver_pos_nr_jl_mimic.cpp
  src/chainiksolver_vel_pinv_mimic.cpp)
# set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(kdl_kinematics_noros
        ${rdf_loader_lib}
        ${orocos_kdl_lib}
        ${robot_state_lib}
        ${robot_model_lib}
        ${kinematics_base_lib}
        ${kdl_parser_lib}
        ${kdl_conversions_lib})

#install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS kdl_kinematics_noros
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "kdl_kinematics_plugin.h")
