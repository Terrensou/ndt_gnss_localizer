cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)

set(CMAKE_CXX_STANDARD 11)
find_package(Eigen3 REQUIRED)
find_package(libxml2 QUIET)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
        std_srvs
        geodesy
        )

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter nodes/points_downsampler.cpp)

add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

add_executable(map_loader nodes/map_loader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_gnss_loader nodes/map_gnss_loader.cpp include/gnssTools.hpp)
target_link_libraries(map_gnss_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ndt_localizer_node nodes/ndt.cpp)
target_link_libraries(ndt_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ndt_gnss_localizer_node nodes/gnss_ndt.cpp include/gnssTools.hpp)
target_link_libraries(ndt_gnss_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(odometry_to_path_node nodes/odometry_to_path.cpp include/gnssTools.hpp)
target_link_libraries(odometry_to_path_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})