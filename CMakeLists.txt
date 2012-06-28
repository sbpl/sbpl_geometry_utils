cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

include_directories(src)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

macro(build_test test_name test_source)
    rosbuild_add_executable(${test_name} src/test/${test_source})
    target_link_libraries(${test_name} sbpl_geometry_utils)
endmacro()

rosbuild_add_library(${PROJECT_NAME}
                     src/Voxelizer.cpp
                     src/SphereEncloser.cpp
                     src/Interpolator.cpp
                     src/PathSimilarityMeasurer.cpp
                     src/Triangle.cpp
                     src/Sphere.cpp)

build_test(voxelize_sphere_list_test VoxelizeSphereListTest.cpp)
build_test(measure_test MeasureTest.cpp)
build_test(simple_measure_test SimpleMeasureTest.cpp)
build_test(dtw_tests DTWTest.cpp)

