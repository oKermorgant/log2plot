cmake_minimum_required(VERSION 2.8.3)
project(log2plot)

set(CMAKE_BUILD_TYPE Release)

option(BUILD_EXAMPLES "BUILD_EXAMPLES" ON)
option(ENABLE_DYNAMIC_PLOT "ENABLE_DYNAMIC_PLOT" ON)
option(BUILD_PARSER "BUILD_PARSER" ON)

# main files
set(LOG2PLOT_HEADERS
  include/log2plot/generic_container.h
  include/log2plot/container.h
  include/log2plot/logger.h
  include/log2plot/defines.h
)
set(LOG2PLOT_SRC src/logger.cpp src/generic_container.cpp)

set(INCLUDE_DEPENDS include)
set(LIB_DEPENDS "")

# check dynamic plots
if(ENABLE_DYNAMIC_PLOT)
    find_package(PythonLibs 3)
    if(PythonLibs_FOUND)
      set(LOG2PLOT_HEADERS ${LOG2PLOT_HEADERS}
        include/log2plot/log_plotter.h
        include/log2plot/plot/container_plotter.h
        include/log2plot/plot/container_plotter2d.h
        include/log2plot/plot/container_plotter3d.h
        include/log2plot/plot/figure_utils.h
        include/log2plot/plot/figure2d.h
        include/log2plot/plot/figure3d.h)
      set(LOG2PLOT_SRC ${LOG2PLOT_SRC}
        src/figure_utils.cpp
        src/figure2d.cpp
        src/figure3d.cpp
        src/container_plotter.cpp)

      set(LIB_DEPENDS ${LIB_DEPENDS}  ${PYTHON_LIBRARIES})
      set(INCLUDE_DEPENDS ${INCLUDE_DEPENDS}  ${PYTHON_INCLUDE_DIRS})
    else()
      set(ENABLE_DYNAMIC_PLOT OFF)
    endif()
endif()

find_package(VISP QUIET COMPONENTS core)

# check YAML for configuration file parser
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

if(YamlCpp_FOUND AND BUILD_PARSER)

  if(VISP_FOUND)
    file(WRITE include/log2plot/defines.h "#define LOG2PLOT_WITH_VISP")
      set(LOG2PLOT_SRC ${LOG2PLOT_SRC} src/config_manager_visp.cpp)
    set(LIB_DEPENDS ${LIB_DEPENDS} ${VISP_LIBRARIES})
    set(INCLUDE_DEPENDS ${INCLUDE_DEPENDS} ${VISP_INCLUDE_DIRS})
  else()
    file(WRITE include/log2plot/defines.h "")
  endif()

  set(LOG2PLOT_HEADERS ${LOG2PLOT_HEADERS} 
  include/log2plot/dir_tools.h
  include/log2plot/config_manager.h
  include/log2plot/config_manager_detail.h
  )  
  set(LOG2PLOT_SRC ${LOG2PLOT_SRC} src/config_manager.cpp)
  set(LIB_DEPENDS ${LIB_DEPENDS} yaml-cpp)
  set(INCLUDE_DEPENDS ${INCLUDE_DEPENDS} ${YamlCpp_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_STANDARD 17)
# Compile with available deps
add_library(log2plot SHARED ${LOG2PLOT_HEADERS} ${LOG2PLOT_SRC})
target_include_directories(log2plot PRIVATE ${INCLUDE_DEPENDS})
target_link_libraries(log2plot ${LIB_DEPENDS})


##############
## Examples ##
##############

# get compile directory for default script path
add_definitions(-DLOG2PLOT_SCRIPT_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/src/plot")
add_definitions(-DLOG2PLOT_SCRIPT_INSTALL="${CMAKE_INSTALL_PREFIX}/bin/log2plot")

if(BUILD_EXAMPLES)

    # default output datafile path
    add_definitions(-DLOG2PLOT_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/examples/")
    
    # build std containers examples
    add_executable(std_containers examples/src/std_containers.cpp)
    target_include_directories(std_containers PRIVATE ${INCLUDE_DEPENDS})
    target_link_libraries(std_containers log2plot)
    
    # build Eigen example
    find_package(Eigen3 QUIET)
    if(EIGEN3_FOUND)
        add_executable(eigen_containers examples/src/eigen_containers.cpp)
        target_include_directories(eigen_containers
          PRIVATE ${EIGEN3_INCLUDE_DIRS} ${INCLUDE_DEPENDS})
        target_link_libraries(eigen_containers log2plot)
    endif()

    # build ViSP example
    if(VISP_FOUND)
        add_executable(visp_containers examples/src/visp_containers.cpp)
        target_include_directories(visp_containers
          PRIVATE ${VISP_INCLUDE_DIRS} ${INCLUDE_DEPENDS})
        target_link_libraries(visp_containers log2plot ${VISP_LIBRARIES})
    endif()

    # animated graph
    if(ENABLE_DYNAMIC_PLOT)
        add_executable(animation examples/src/animation.cpp)
        target_include_directories(animation PRIVATE ${INCLUDE_DEPENDS})
        target_link_libraries(animation log2plot)
    endif()

    if(BUILD_PARSER)
      add_executable(parser examples/parser/parser.cpp)
      target_include_directories(parser
        PRIVATE ${INCLUDE_DEPENDS})
      target_link_libraries(parser log2plot)
    endif()

endif()



#############
## Install ##
#############

install(TARGETS log2plot EXPORT log2plotConfig
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  
install(DIRECTORY include/log2plot DESTINATION include)
install(PROGRAMS src/plot DESTINATION bin RENAME ${PROJECT_NAME})

set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})

set(INCLUDE_DEPENDS ${CMAKE_INSTALL_PREFIX}/include ${INCLUDE_DEPENDS})
set(LIB_DEPENDS ${CMAKE_INSTALL_PREFIX}/lib/liblog2plot.so ${LIB_DEPENDS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/log2plotConfig.cmake.in
"${PROJECT_BINARY_DIR}/log2plotConfig.cmake")

install (FILES
    ${CMAKE_CURRENT_BINARY_DIR}/log2plotConfig.cmake
    DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/log2plot")

