cmake_minimum_required(VERSION 2.8.3)
project(sbpl_geometry_utils)

find_package(catkin REQUIRED COMPONENTS geometry_msgs)

# inspect for eigen include directories
find_package(Eigen3)
if (${EIGEN3_FOUND})
    set(EIGEN_INCLUDES ${EIGEN3_INCLUDE_DIR})
else()
    find_package(Eigen REQUIRED)
    set(EIGEN_INCLUDES ${Eigen_INCLUDE_DIRS})
endif()

catkin_package(
    INCLUDE_DIRS include src
    LIBRARIES sbpl_geometry_utils
    CATKIN_DEPENDS geometry_msgs
    DEPENDS eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDES})

set(CMAKE_BUILD_TYPE Release)

# inspect for c++11 flags
if (CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GNUCXX_VERSION)
    if (GNUCXX_VERSION VERSION_LESS 4.7)
        set(C11_FLAGS "-std=c++0x")
    else()
        set(C11_FLAGS "-std=c++11")
    endif()
else()
    message("Compiler is not g++. You're on your own for C++11 features")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C11_FLAGS}")

add_library(
    sbpl_geometry_utils
    src/measure_similarity.cpp
    src/bounding_spheres.cpp
    src/voxelize.cpp
    src/interpolate.cpp
    src/rasterize.cpp
    src/mesh_utils.cpp)
target_link_libraries(sbpl_geometry_utils ${catkin_LIBRARIES})

install(TARGETS sbpl_geometry_utils
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/sbpl_geometry_utils DESTINATION include FILES_MATCHING PATTERN "*.h")
