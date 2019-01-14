cmake_minimum_required(VERSION 2.8.3)

project(tuw_realsense_d435)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        geometry_msgs
        tf
        tf_conversions
        cmake_modules
        )

find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
        #        CATKIN_DEPENDS
        #        roscpp
        #        sensor_msgs
        #        geometry_msgs
        #        tf
        #        tf_conversions
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)


add_executable(realsense_transforms_node
        src/realsense_transforms_node.cpp)

target_link_libraries(realsense_transforms_node
        ${catkin_LIBRARIES}
        ${TinyXML_LIBRARIES}
        ${Boost_LIBRARIES}
        )

# catkin_install_python(PROGRAMS scripts/tuw_extrinsic_camera
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

