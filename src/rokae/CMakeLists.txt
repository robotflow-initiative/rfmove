include_directories(
        include
        ../../extern
        ${KINETIC_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(rokae_client SHARED
        src/rokae_client.cpp)
target_link_libraries(rokae_client ${robot_trajectory_lib} Threads::Threads)

add_library(rokae_server SHARED
        src/rokae_server.cpp)
target_link_libraries(rokae_server Threads::Threads)

pybind11_add_module(rokae_cli
        src/rokae_client.cpp
        python/RokaeClientPy.cpp)

target_link_libraries(rokae_cli PRIVATE
        ${robot_trajectory_lib}
        Threads::Threads)

install(TARGETS rokae_client rokae_cli rokae_server
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h")
