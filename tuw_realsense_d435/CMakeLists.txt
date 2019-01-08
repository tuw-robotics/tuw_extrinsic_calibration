cmake_minimum_required(VERSION 2.8.3)

project(tuw_extrinsic_camera)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        geometry_msgs
        tuw_measurement_utils
        )

find_package(OpenCV REQUIRED)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
else ()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif ()

catkin_package(
        INCLUDE_DIRS ${rqt_image_view_INCLUDE_DIRECTORIES}
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        sensor_msgs
        geometry_msgs
        image_geometry
        )

catkin_python_setup()


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# catkin_install_python(PROGRAMS scripts/tuw_extrinsic_camera
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
