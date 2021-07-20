include_directories(
        include
        ${KINETIC_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ../../extern/pybind11/include
)

IF(${WITH_BULLET3})
    include_directories(
        ${BULLET_INCLUDE_DIR}
    )
    #add_compile_definitions(-fPIC) #bullet_robotics need fPIC to build into shared object
ENDIF()

add_library(controller_noros SHARED
        src/trajectory.cpp
        src/trajectory_controller.cpp
        src/hardware.cpp)

target_link_libraries(controller_noros
        ${robot_trajectory_lib}
        ${trajectory_processing_lib})

add_executable(dummy_controller
        dummy_hardware.cpp)

target_link_libraries(dummy_controller
        controller_noros)

IF(${WITH_BULLET3})
    MESSAGE("Build moveit_bullet3 with bullet robotics from ${bullet_robotics_lib}")
    add_library(moveit_bullet3
        src/hardware_bullet3.cpp)
    target_link_libraries(moveit_bullet3 PUBLIC
        ${bullet_robotics_lib})
    #set_property(TARGET moveit_bullet3 PROPERTY POSITION_INDEPENDENT_CODE ON)
ENDIF()

install(TARGETS controller_noros
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(TARGETS dummy_controller
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

IF(${WITH_BULLET3})
    install(TARGETS moveit_bullet3
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF()

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h")