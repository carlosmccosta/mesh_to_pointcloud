cmake_minimum_required(VERSION 2.8.3)
project(mesh_to_pointcloud)



#######################################################################################################################
## packages
#######################################################################################################################


find_package(catkin REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS osgDB)



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        mesh_to_pointcloud
    DEPENDS
        PCL
        OpenSceneGraph
)



#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(mesh_to_pointcloud
    src/mesh_to_pointcloud.cpp
)



#==================
# executables
#==================

add_executable(mesh_to_pcd
    src/mesh_to_pcd.cpp
)



#==================
# library link
#==================

target_link_libraries(mesh_to_pointcloud
    ${PCL_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(mesh_to_pcd
    mesh_to_pointcloud
    ${catkin_LIBRARIES}
)
