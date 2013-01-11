cmake_minimum_required(VERSION 2.8.3)
project(sbpl_geometry_utils)

find_package(catkin REQUIRED COMPONENTS geometry_msgs)
find_package(Eigen REQUIRED)

catkin_package(INCLUDE_DIRS include src
               LIBRARIES sbpl_geometry_utils
               CATKIN_DEPENDS geometry_msgs
               DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(sbpl_geometry_utils
            src/Interpolator.cpp
            src/PathSimilarityMeasurer.cpp
            src/Sphere.cpp
            src/SphereEncloser.cpp
            src/Triangle.cpp
            src/Voxelizer.cpp)
target_link_libraries(sbpl_geometry_utils ${catkin_LIBRARIES})

install(TARGETS sbpl_geometry_utils
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/sbpl_geometry_utils DESTINATION include FILES_MATCHING PATTERN "*.h")
