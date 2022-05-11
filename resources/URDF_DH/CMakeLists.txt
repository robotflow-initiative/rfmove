cmake_minimum_required(VERSION 3.1)
project(miniKF)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "  -O3 -fopenmp -pthread")
set(CMAKE_CXX_FLAGS "-fPIC -lpthread")

SET(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install) 
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/install/lib)


#Eigen
include_directories("usr/include/eigen3")
#local include
include_directories("include")

#Opencv
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(TPARTY_LIBS ${OpenCV_LIBS})

add_library(dh_valid src/csv_loader.cpp src/opt_utils.cpp)

# 编译库文件
target_link_libraries(dh_valid ${TPARTY_LIBS})

# 编译可执行文件
add_executable(valid_eigen src/valid_eigen.cpp)
target_link_libraries(valid_eigen ${TPARTY_LIBS} dh_valid)

file(GLOB_RECURSE HEADERS include/*.h)

# 安装头文件
install(FILES ${HEADERS} DESTINATION include)

# 安装库文件
install(TARGETS dh_valid
        LIBRARY DESTINATION lib)

file(GLOB_RECURSE CONFIG script/*)
install(FILES ${CONFIG} DESTINATION script)

#install data and model
file(GLOB_RECURSE CONFIG data/*)
install(FILES ${CONFIG} DESTINATION data)

install(TARGETS valid_eigen
        RUNTIME DESTINATION bin)
