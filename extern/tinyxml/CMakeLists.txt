cmake_minimum_required(VERSION 3.5)
project(tinyxml)
include(GNUInstallDirs)

MESSAGE(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "CMAKE_INSTALL_BINDIR: ${CMAKE_INSTALL_BINDIR}")
MESSAGE(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
MESSAGE(STATUS "CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")

# use stl so that we do not need to install tinystr.h
# SET(CMAKE_CXX_FLAGS "-DTIXML_USE_STL")
# ADD_DEFINITIONS(-DTIXML_USE_STL)


add_library(${PROJECT_NAME} SHARED
        tinyxml.cpp
        tinyxmlerror.cpp
        tinyxmlparser.cpp
        tinyxmlerror.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION 2.6.2)
target_compile_definitions(${PROJECT_NAME} PRIVATE TIXML_USE_STL)

#target_include_directories(tinyxml
#        PUBLIC tinyxml.h)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "tinyxml.h")

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(FILES tinyxml.h
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
