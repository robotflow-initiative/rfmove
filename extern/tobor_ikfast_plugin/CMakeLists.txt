include_directories(
    include
    ${KINETIC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/extern/ifast_inlcude
)


add_library(tobor_left_kinematics_noros SHARED tobor_left_arm_group_ikfast_moveit_plugin.cpp 
                                               tobor_left_arm_group_ikfast_solver.cpp)
                                          

# set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(tobor_left_kinematics_noros
        ${rdf_loader_lib}
        ${orocos_kdl_lib}
        ${robot_state_lib}
        ${robot_model_lib}
        ${kinematics_base_lib})

#install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS tobor_left_kinematics_noros
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
