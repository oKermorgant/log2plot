cmake_minimum_required(VERSION 2.8.3)
project(log2plot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

option(BUILD_EXAMPLES "BUILD_EXAMPLES" ON)
option(USE_MATPLOTLIBCPP "USE_MATPLOTLIBCPP" ON)
option(USE_CATKIN "USE_CATKIN" ON)
option(CATKIN_ENABLE_TESTING "CATKIN_ENABLE_TESTING" OFF)

# main files
set(LOG2PLOT_HEADERS
  include/log2plot/generic_container.h
  include/log2plot/container.h
  include/log2plot/logger.h)
set(LOG2PLOT_SRC src/logger.cpp src/generic_container.cpp)
set(INCLUDE_DEPENDS include)
set(LIB_DEPENDS "")

# check MATPLOTLIBCPP
if(USE_MATPLOTLIBCPP)
    find_file(MATPLOTLIBCPP matplotlibcpp.h
                HINTS matplotlib-cpp)
    if(MATPLOTLIBCPP)
        message("-- Found Matplotlib-cpp: ${MATPLOTLIBCPP}")
    else()
        message("-- Could not find Matplotlib-cpp")
        set(USE_MATPLOTLIBCPP OFF)
    endif()
endif()

if(USE_MATPLOTLIBCPP)
  set(LOG2PLOT_HEADERS ${LOG2PLOT_HEADERS}
    include/log2plot/container_plotter.h
    include/log2plot/log_plotter.h
    include/log2plot/plot.h
    ${MATPLOTLIBCPP})
  set(LOG2PLOT_SRC ${LOG2PLOT_SRC} src/plot.cpp)

  find_package(PythonLibs 2.7)
  get_filename_component(MATPLOTLIBCPP_INCLUDE ${MATPLOTLIBCPP} DIRECTORY)
  set(LIB_DEPENDS ${LIB_DEPENDS}  ${PYTHON_LIBRARIES})
  set(INCLUDE_DEPENDS ${INCLUDE_DEPENDS}  ${PYTHON_INCLUDE_DIRS} ${MATPLOTLIBCPP_INCLUDE})
endif()

# check CATKIN
if(USE_CATKIN)
find_package(catkin COMPONENTS roscpp QUIET)
    if(${catkin_FOUND})
        # build as a ROS package
        catkin_package(
            INCLUDE_DIRS ${INCLUDE_DEPENDS} 
            LIBRARIES log2plot
            CATKIN_DEPENDS roscpp
            )
        # add log_publisher
        set(LOG2PLOT_HEADERS ${LOG2PLOT_HEADERS}
          include/log2plot/container_publisher.h
          include/log2plot/log_publisher.h)
        # update depends
        set(LIB_DEPENDS ${LIB_DEPENDS} ${catkin_LIBRARIES})
        set(INCLUDE_DEPENDS ${INCLUDE_DEPENDS} ${catkin_INCLUDE_DIRS})
    else()
        set(USE_CATKIN OFF)
        message("-- Could not find CATKIN")
   endif()
endif()


# Compile with available deps
add_library(log2plot SHARED ${LOG2PLOT_HEADERS} ${LOG2PLOT_SRC})
target_include_directories(log2plot PRIVATE ${INCLUDE_DEPENDS})
target_link_libraries(log2plot ${LIB_DEPENDS})


##############
## Examples ##
##############

# get compile directory for default script path
add_definitions(-DLOG2PLOT_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/plot")
if(BUILD_EXAMPLES)

    # default output datafile path
    add_definitions(-DLOG2PLOT_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/examples/")
    
    # build std containers examples
    add_executable(std_containers examples/src/std_containers.cpp)
    target_include_directories(std_containers PRIVATE ${INCLUDE_DEPENDS})
    target_link_libraries(std_containers log2plot)
    
    # std container with publisher
    if(USE_CATKIN)
        add_executable(std_publisher examples/src/std_publisher.cpp)
        target_include_directories(std_publisher
          PRIVATE ${INCLUDE_DEPENDS})
        target_link_libraries(std_publisher log2plot)
    endif()

    # build Eigen example
    find_package(Eigen3 QUIET)
    if(EIGEN3_FOUND)
        add_executable(eigen_containers examples/src/eigen_containers.cpp)
        target_include_directories(eigen_containers
          PRIVATE ${EIGEN3_INCLUDE_DIRS} ${INCLUDE_DEPENDS})
        target_link_libraries(eigen_containers log2plot)
    endif()

    # build ViSP example
    find_package(VISP QUIET)
    if(VISP_FOUND)
        add_executable(visp_containers examples/src/visp_containers.cpp)
        target_include_directories(visp_containers
          PRIVATE ${VISP_INCLUDE_DIRS} ${INCLUDE_DEPENDS})
        target_link_libraries(visp_containers log2plot ${VISP_LIBRARIES})
    endif()

    # animated graph
    if(USE_MATPLOTLIBCPP)
        add_executable(animation examples/src/animation.cpp)
        target_include_directories(animation PRIVATE ${INCLUDE_DEPENDS})
        target_link_libraries(animation log2plot)
    endif()
endif()


#############
## Install ##
#############

if(USE_CATKIN)
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

    install(FILES ${LOG2PLOT_HEADERS} 
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  else()
  
    install(TARGETS log2plot EXPORT log2plotConfig
        ARCHIVE  DESTINATION lib
        LIBRARY  DESTINATION lib
        RUNTIME  DESTINATION bin)  
    install(FILES ${LOG2PLOT_HEADERS} DESTINATION include/${PROJECT_NAME})

    set(ConfigPackageLocation lib/cmake/log2plot)

    set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/log2plotConfig.cmake.in
    "${PROJECT_BINARY_DIR}/log2plotConfig.cmake" @ONLY)
    
    install (FILES
        ${CMAKE_CURRENT_BINARY_DIR}/log2plotConfig.cmake
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/log2plot")
endif()
