This module contains a library to be used from C++ that generates YAML files for data logs.

A Python script is then used to plot the files with various options.

## Compilation and installation

The module can be compiled as a ROS package or standalone. When compiled as a ROS package, numeric logged data can also be published to provide online analysis through rqt_plot or other tools.

### As a ROS package
Just clone the repository in your ROS workspace and build it with catkin:  `catkin build log2plot`

### As a standalone library

Go to the `non_ros` folder and perform a classical CMake build:

* `mkdir build`
* `cd build`
* `cmake ..`
* `make` or `make install`

The library can then be found through CMake find_package.

## Use from C++ code

Examples can be found in the `examples` folder. The main class is `log2plot::Logger` and should be instanciated with the desired data file path and prefix:  `log2plot::Logger logger(fileprefix)`. If no fileprefix is given then the files will be created in the `/tmp` folder. Shipped examples use the `examples` path at compile time. 

The logged variables have be containers of some sort, as long as the following member functions are available:
* operator[] to get the value at a given index
* size() to get the length of the logged container

Three types of data may be logged:
* Iteration-based data will use the index as the X-axis for the plots.
  * `logger.save(v, name, legend, ylabel)`
  * `legend` should be a YAML-style list and may be using Latex: `"[v_x, \\omega_z]"`
* Time-based data has to be given a time and will use it as the X-axis.
  * `logger.setTime(t, "s");` where `t` is a `double`
  * `logger.saveTimed(v, name, legend, ylabel)`
  * `legend` should be a YAML-style list and may be using Latex: `"[v_x, \\omega_z]"`
* 3D pose data has to be given a 6-components pose vector (as in translation + angle-axis representation). 
  * `logger.save3Dpose(v, name, trajectory_name, invert_pose)`
  * `trajectory_name` should be a single string
  * `invert_pose` (default false) allows to log a pose whom inverse will be actually plotted. This can be useful typically when working with a world-to-camera pose but we still want to display the camera-to-world pose afterwards.

This will log data into the file: `fileprefix + name + .yaml`

### Common options

Log is actually done when calling `logger.update();`, typically from inside a loop. Two parameters can be changed:
* Subsampling to log only once every n updates: `logger.setSubSampling(n)` (default 1)
* Buffer size before writing to the file: `logger.setBuffer(b)` (default 10)
* The plot can be done directly from C++ if needed: `logget.plot(script_path)`, where `script_path` is the path to the Python script. The default value is the path at library compile time.

Options should be given before calling the first `update()`.

### Iteration or time-based options

The following commands will be applied to the last added variable:
* Units: `logger.setUnits("[unit1, unit2, unit2]");` will save the units for the 3 first components
* Line styles: `logger.setLineType(["b, g, r--]");`, line styles have to be defined in Matplotlib styles (color + line style)

### 3D pose options

The following commands will be applied to the last added variable:
* Display an object following the logged pose: `logger.showMovingObject(nodes, graph, desired_pose)`
  * The object is displayed in wireframe from nodes (3D positions of points) and graph (segments linking 2 nodes)
  * Nodes should be a `vector<vector<double> >` of dimension 3xn where n in the number of points
  * Graph should be a string indicating the links between the node indices: `"[[0,1],[1,2],[2,0]]"` for a triangle
  * Desired pose is a 6-dimensional pose given as a std::vector. The argument can thus be passed as `{0,1,2,3,4,5}`
  * Built-in function allow to show a moving camera and a moving box
* Display a fixed object: `logger.showFixedObject(nodes, graph, color)`
  * With the same syntax for nodes and graph, will display this wireframe object in the given Matplotlib color.
* Change built-in colors: `logger.setLineType("[b, g, r, k--]")` allows to define the color and line style of the 3D elements:
  * First element is the trajectory line style (solid blue here)
  * Second element is the moving object (solid green)
  * Third one is for the initial and final poses (solid red)
  * Fourth one is for the desided pose (dashed black)
  
## Python syntax

The Python module used to plot the files is in the `src` folder and requires `matplotlib`, `YAML`, and `argparse`. It may be useful to re-plot a file with different options. The script can be called from the command line or with the `rosrun` syntax if compiled with ROS:
* `python plot <file.yaml>`
* `rosrun log2plot plot <file.yaml>`

Many options are available from the command line, call `plot -h` to have a list. Several files can be plotted at the same time, in this case if they have the same y-label their y-axis will be at the same scale. By default they will be plotted in different subplots, but can be plotted in the same plot with the `-g` option. 

Videos can be created using the `-v <subsampling>` option. ffmpeg or avconv will be used to create a mp4 file showing the plot evolution. 

## Examples

In the `examples` folder are shipped 4 use cases:
* `std_container` uses std::vectors and shows iteration-based, time-based and 3D pose plots
* `std_publisher` shows how to publish data to ROS topics when logging
* `visp_containers` uses containers from the ViSP library (vpColVector and vpPoseVector) and logs an inverted 3D pose
* `eigen_containers` uses containers from the Eigen library (Eigen::Vector3d)
