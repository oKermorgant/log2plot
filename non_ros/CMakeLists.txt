cmake_minimum_required(VERSION 2.8.3)
project(log2plot)

add_compile_options(-std=c++11)

option(BUILD_EXAMPLES "BUILD_EXAMPLES" ON)
    
include_directories(../include)

# main library
add_library(log2plot SHARED ../include/log2plot/container.h
    ../include/log2plot/logger.h ../src/logger.cpp
    )
target_link_libraries(log2plot)

# get compile directory for default script path
add_definitions(-DLOG2PLOT_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../src/plot")

if(BUILD_EXAMPLES)

    # default file path
    add_definitions(-DLOG2PLOT_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../examples/")
    
    # build std containers examples
    add_executable(std_containers ../examples/src/std_containers.cpp)
    target_link_libraries(std_containers log2plot)

    # build Eigen example
    find_package(Eigen3)
    if(EIGEN3_FOUND)
        include_directories(${EIGEN3_INCLUDE_DIRS})
        add_executable(eigen_containers ../examples/src/eigen_containers.cpp)
        target_link_libraries(eigen_containers log2plot)
    endif()

    # build ViSP example
    find_package(VISP)
    if(VISP_FOUND)
        include_directories(${VISP_INCLUDE_DIRS})
        add_executable(visp_containers ../examples/src/visp_containers.cpp)
        target_link_libraries(visp_containers log2plot ${VISP_LIBRARIES})
    endif()
endif()


#############
## Install ##
#############

install(TARGETS log2plot EXPORT log2plotConfig
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  
install(DIRECTORY ../include/ DESTINATION include
    PATTERN "*publisher*" EXCLUDE # do not install ROS-related files
)

set(ConfigPackageLocation lib/cmake/log2plot)

set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/log2plotConfig.cmake.in
  "${PROJECT_BINARY_DIR}/log2plotConfig.cmake" @ONLY)
   
 install (FILES
     ${CMAKE_CURRENT_BINARY_DIR}/log2plotConfig.cmake
     DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/log2plot")

