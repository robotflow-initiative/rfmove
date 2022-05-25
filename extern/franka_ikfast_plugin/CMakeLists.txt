include_directories(
    include
    ${KINETIC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "franka_panda_arm_ikfast_moveit_plugin.cpp")

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "franka_panda_arm_ikfast_solver.cpp")

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "ikfast.h")