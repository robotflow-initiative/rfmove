include_directories(
        include
        ${KINETIC_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(controller_noros SHARED
        src/trajectory.cpp)

target_link_libraries(controller_noros
        ${robot_trajectory_lib})

install(TARGETS controller_noros
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h")