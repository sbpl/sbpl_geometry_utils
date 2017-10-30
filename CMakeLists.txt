cmake_minimum_required(VERSION 2.8.12)
project(sbpl_geometry_utils)

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs)

# inspect for eigen include directories
find_package(Eigen)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sbpl_geometry_utils
    CATKIN_DEPENDS geometry_msgs
    DEPENDS Eigen)

set(CMAKE_BUILD_TYPE Release)

# inspect for c++11 flags
if (CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GNUCXX_VERSION)
    if (GNUCXX_VERSION VERSION_LESS 4.7)
        set(C11_FLAGS "-std=c++0x")
    else()
        set(C11_FLAGS "-std=c++11")
    endif()
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(C11_FLAGS "-std=c++11")
else()
    message("You're on your own for C++11 features")
endif()

add_compile_options(${C11_FLAGS})

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    sbpl_geometry_utils
    src/measure_similarity.cpp
    src/bounding_spheres.cpp
    src/voxelize.cpp
    src/interpolate.cpp
    src/rasterize.cpp
    src/mesh_utils.cpp)
target_link_libraries(sbpl_geometry_utils ${catkin_LIBRARIES})

install(
    TARGETS sbpl_geometry_utils
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
    DIRECTORY include/sbpl_geometry_utils
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")
