include_directories(
        include
        ${KINETIC_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/src/util
        ${PROJECT_SOURCE_DIR}/extern/franka_ikfast_plugin/include
        ${YAML_CPP_INCLUDE_DIR}
)

add_library(planner_noros SHARED
        src/PlannerManager.cpp
        src/PlannerConfiguration.cpp
        ../util/path_util.cpp)

target_link_libraries(planner_noros
        ${planning_interface_lib}
        ${ompl_interface_lib}
        #${YAML_CPP_LIBRARIES}
        yaml-cpp
        ${robot_model_lib}
        ${constraint_samplers_lib})

install(TARGETS planner_noros
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h*")