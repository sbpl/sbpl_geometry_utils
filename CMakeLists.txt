cmake_minimum_required(VERSION 2.8.12)
project(sbpl_geometry_utils)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3)
find_package(catkin REQUIRED COMPONENTS geometry_msgs)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES sbpl_geometry_utils
    CATKIN_DEPENDS geometry_msgs
    )

add_library(
    sbpl_geometry_utils
    src/measure_similarity.cpp
    src/bounding_spheres.cpp
    src/voxelize.cpp
    src/interpolate.cpp
    src/rasterize.cpp
    src/mesh_utils.cpp)
target_compile_options(sbpl_geometry_utils PUBLIC -std=c++11)
target_include_directories(sbpl_geometry_utils PUBLIC include)
target_include_directories(sbpl_geometry_utils SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(sbpl_geometry_utils SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(sbpl_geometry_utils PUBLIC ${catkin_LIBRARIES})

install(
    TARGETS sbpl_geometry_utils
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
    DIRECTORY include/sbpl_geometry_utils/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
