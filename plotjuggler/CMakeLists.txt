cmake_minimum_required(VERSION 3.7)

project(log2plotjuggler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#-------  Include Qt dependencies -------
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
    Concurrent
    Xml
    Svg
    OpenGL)
set(QT_LIBRARIES
    Qt5::Core
    Qt5::Widgets
    Qt5::Xml
    Qt5::Svg )

add_definitions( ${QT_DEFINITIONS} -DQT_PLUGIN )

find_package(yaml-cpp)
include_directories(${yaml-cpp-INCLUDE_DIRS})

find_package(Eigen3)

if(Eigen3_FOUND)
    add_definitions(-DWITH_EIGEN)
endif()

set( PJ_LIBRARIES ${QT_LIBRARIES} yaml-cpp ${EIGEN3_INCLUDE_DIRS})

#--------------------------------------------------------
#-------------- Build with CATKIN (ROS1) ----------------
if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)

    set(COMPILING_WITH_CATKIN 1)
    message(STATUS "COMPILING_WITH_CATKIN")
    add_definitions(-DCOMPILED_WITH_CATKIN)

    find_package(catkin REQUIRED COMPONENTS plotjuggler )
    catkin_package(
        CATKIN_DEPENDS plotjuggler
        )
    include_directories(${catkin_INCLUDE_DIRS})
    list(APPEND ${PJ_LIBRARIES} ${catkin_LIBRARIES} )
    set(PJ_PLUGIN_INSTALL_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION} )

    #--------------------------------------------------------
    #-------------- Build with AMENT (ROS2) -----------------
elseif( DEFINED ENV{AMENT_PREFIX_PATH})

    set(COMPILING_WITH_AMENT 1)
    message(STATUS "COMPILING_WITH_CATKIN")
    add_definitions(-DCOMPILED_WITH_AMENT)

    find_package(ament_cmake REQUIRED)
    find_package(plotjuggler REQUIRED)
    set(PJ_PLUGIN_INSTALL_DIRECTORY lib/${PROJECT_NAME} )

    #--------------------------------------------------------
    #------------- Build without any ROS support ------------
else()

    find_package(plotjuggler REQUIRED)
    message(STATUS "PlotJuggler FOUND")
    message(STATUS "plotjuggler_INCLUDE_DIR: ${plotjuggler_INCLUDE_DIR}")
    message(STATUS "plotjuggler_LIBRARIES: ${plotjuggler_LIBRARIES}")

    include_directories(${plotjuggler_INCLUDE_DIR})
    list(APPEND ${PJ_LIBRARIES} ${plotjuggler_LIBRARIES} )
    set(PJ_PLUGIN_INSTALL_DIRECTORY bin )

endif()

#------- Create the libraries -------

add_library(log2plotjuggler SHARED src/log2plot_yaml.cpp)

target_link_libraries(log2plotjuggler ${PJ_LIBRARIES})

if (COMPILING_WITH_AMENT)
    ament_target_dependencies(log2plotjuggler plotjuggler)
endif()
#------- Install the libraries -------

install(
    TARGETS
        log2plotjuggler
    DESTINATION
        ${CMAKE_INSTALL_PREFIX})

