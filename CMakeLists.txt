cmake_minimum_required(VERSION 2.8.3)
project(log2plot)

add_compile_options(-std=c++11)

option(BUILD_EXAMPLES "BUILD_EXAMPLES" ON)

find_package(catkin COMPONENTS REQUIRED
    roscpp
    )
    
# build as a ROS package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES log2plot
    CATKIN_DEPENDS roscpp
    )

include_directories(include ${catkin_INCLUDE_DIRS})

# main library
add_library(log2plot SHARED include/log2plot/container.h
    include/log2plot/logger.h src/logger.cpp
    include/log2plot/log_publisher.h include/log2plot/container_publisher.h
    )
target_link_libraries(log2plot ${catkin_LIBRARIES})

# get compile directory for default script path
add_definitions(-DLOG2PLOT_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/plot")

if(BUILD_EXAMPLES)

    # default file path
    add_definitions(-DLOG2PLOT_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/examples/")
    
    # build std containers examples
    add_executable(std_containers examples/src/std_containers.cpp)
    target_link_libraries(std_containers log2plot)

    # with publisher
    add_executable(std_publisher examples/src/std_publisher.cpp)
    target_link_libraries(std_publisher log2plot)

    # build Eigen example
    find_package(Eigen3)
    if(EIGEN3_FOUND)
        include_directories(${EIGEN3_INCLUDE_DIR})
        add_executable(eigen_containers examples/src/eigen_containers.cpp)
        target_link_libraries(eigen_containers log2plot)
    endif()

    # build ViSP example
    find_package(VISP)
    if(VISP_FOUND)
        include_directories(${VISP_INCLUDE_DIR})
        add_executable(visp_containers examples/src/visp_containers.cpp)
        target_link_libraries(visp_containers log2plot ${VISP_LIBRARIES})
    endif()
endif()


#############
## Install ##
#############

install(PROGRAMS
   src/plot
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 # only install actual library
 install(TARGETS log2plot
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
