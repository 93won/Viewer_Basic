cmake_minimum_required(VERSION 3.8)
project(graphOptimizer)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# # Dependencies

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(PCL REQUIRED)

# pcl

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)



# #${Pangolin_LIBRARIES} GL GLU GLEW glut
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        pthread
        ${GLOG_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_subdirectory(src)
add_subdirectory(app)
