include(CMakeDependentOption)
include(CheckCXXCompilerFlag)

set(contrib-pattern "${YAML_CPP_SRC_DIR}/contrib/*.cpp")
set(src-pattern "${YAML_CPP_SRC_DIR}/*.cpp")
if (CMAKE_VERSION VERSION_GREATER 3.12)
    list(INSERT contrib-pattern 0 CONFIGURE_DEPENDS)
    list(INSERT src-pattern 0 CONFIGURE_DEPENDS)
endif()

file(GLOB yaml-cpp-contrib-sources ${contrib-pattern})
file(GLOB yaml-cpp-sources ${src-pattern})

add_library(yaml-cpp SHARED)
add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)

target_include_directories(yaml-cpp
        PUBLIC
        ${YAML_CPP_INCLUDE_DIR}
        PRIVATE
        ${YAML_CPP_SRC_DIR})

target_sources(yaml-cpp
        PRIVATE
        ${yaml-cpp-contrib-sources}
        ${yaml-cpp-sources})

install(TARGETS yaml-cpp
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
