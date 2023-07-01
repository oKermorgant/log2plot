This module contains a library to be used from C++ that generates YAML files for data logs.

A Python script is then used to plot the files with various options.

## Compilation and installation

As of version 2, the module is ROS-agnostic and is a classical CMake package:

* `mkdir build`
* `cd build`
* `cmake ..`
* `make` or `sudo make install`

The library can then be found through CMake find_package.

The plotting script will be placed at `/usr/local/bin/log2plot` and can be used just typing `log2plot <yaml data file>`.

### Dynamic plot option

The logger can also plot 2D or 3D graphs at runtime, by using -DENABLE_DYNAMIC_PLOT=ON with Cmake. This requires Python3 and Matplotlib.

In this case a new logger, namely `LogPlotter` can be used instead of `Logger`. This class requires a target rate and will try to update the plot within this rate, despite the possibly slow Python interpreter.

## Use from C++ code

Examples can be found in the `examples` folder. The main class is `log2plot::Logger` and should be instanciated with the desired data file path and prefix:  `log2plot::Logger logger(fileprefix)`. If no fileprefix is given then the files will be created in the `/tmp` folder. Shipped examples use the `examples` path at compile time.

The logged variables have to be containers of some sort, as long as the following member functions are available:
* `operator[]` to get the value at a given index
* `size()` to get the length of the logged container

Besides these two points, all kind of data can be saved, but of course they will not be plottable if not numerical.

Four types of data may be logged:
* Iteration-based data will use the index as the X-axis for the plots.
  * `logger.save(v, name, legend, ylabel)`
  * `legend` should be a YAML-style list and may be using Latex: `"[v_x, \\omega_z]"`
* Time-based data has to be given a time and will use it as the X-axis.
  * `logger.setTime(t, "s");` where `t` is a `double`
  * `logger.saveTimed(v, name, legend, ylabel)`
  * `legend` should be a YAML-style list and may be using Latex: `"[v_x, \\omega_z]"`
* XY-based data are defined as {x1, y1, x2, y2, ...}
  * `logger.saveXY(v, legend, x-label, y-label)`
  * `legend` should be a YAML-style list with half the dimension of v`
* 3D pose data has to be given a 6-components pose vector (as in translation + angle-axis representation).
  * `logger.save3Dpose(v, name, trajectory_name, invert_pose)`
  * `trajectory_name` should be a single string
  * `invert_pose` (default false) allows to log a pose whom inverse will be actually plotted. This can be useful typically when working with a world-to-camera pose but we still want to display the camera-to-world pose afterwards.
* Timed-XY-based data are defined as {x1, y1, x2, y2, ...}
  * `logger.saveTimedXY(v, legend, x-label, y-label)`
  * `legend` should be a YAML-style list with one element
  * This data type is used to visualize a changing XY curve. Only the video option would be relevant in this case

This will log data into the file: `fileprefix + name + .yaml`

### General options

Log is actually done when calling `logger.update();`, typically from inside a loop. Two parameters can be changed:
* Subsampling to log only once every n updates: `logger.setSubSampling(n)` (default 1)
* Buffer size before writing to the file: `logger.setBuffer(b)` (default 10)
* The plot can be done directly from C++ if needed: `logget.plot(script_path)`, where `script_path` is the path to the Python script. The default value is the path at library compile time.

Options should be given before calling the first `update()`.

### Iteration or time-based options

The following commands will be applied to the last added variable:
* Units: `logger.setUnits("[unit1, unit2, unit2]");` will save the units for the 3 first components
* Line styles: `logger.setLineType(["b, g, r--]");`, line styles have to be defined in Matplotlib styles (color + line style)
* particular time steps: `logger.setSteps({});`, will display dashed vertical lines at those instant
* Steps can also be added while recording with `logger.writeStep();`

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
  * if the `logger.displayObjectAs(log2plot::Surface)` method is called then the point cloud will be displayed as a meshed surface
* Change built-in colors: `logger.setLineType("[b, g, r, k--]")` allows to define the color and line style of the 3D elements:
  * First element is the trajectory line style (solid blue here)
  * Second element is the moving object (solid green)
  * Third one is for the initial and final poses (solid red)
  * Fourth one is for the desided pose (dashed black)


### Not a Number

If some (double) logged data is non defined or irrelevant at some point, it is possible to keep logging but write Not a Number so that it will not be plotted. The syntax is:
* `v[0] = log2plot::nan;` to erase only one component.
* `log2plot::setNaN(v, 0, 2);` to erase components 0 and 1 from the v vector or array.

## Reload existing log file

The `log2plot/loader.h` header defines a `log2plot::Log` class that takes a file path in the constructor.

The resulting `Log` will get data from the file, allowing to re-process or re-populate it if needed

## Use from Python code

A basic Python wrapper (only for the `Logger` class) is proposed and installed if `PYTHON_BINDINGS` is set to `True` (default).
It relies on [`cppyy`](https://cppyy.readthedocs.io/en/latest/), install it with: `pip3 install cppyy`.

Please look at the `from_python.py` example. Note that the logged variables have to be underlying C++ objects. The way to do it is:
 - Initialize the logger: `logger = log2plot.Logger(<base path>)`
 - initialize object: `v = log2plot.Vec(5)` for a vector of dimension 5
 - save it through your logger : `logger.save(v, path and legends)`
 - when logging your data e.g. `my_array`, copy it to `v`:
    - `log2plot.copy(my_array, v)`
    - `logger.update()`

## Plotting script syntax

The Python script used to plot the files is in the `src` folder and requires `matplotlib`, `YAML`, and `argparse`. It may be useful to re-plot a file with different options. The script can be called from the command line:
* `python3 path/to/log2plot/src/plot <file.yaml>` (if not installed)
* `log2plot <file.yaml>` (if installed)

Many (probably too many) options are available from the command line, call `plot -h` to have a list. Several files can be plotted at the same time, in this case if they have the same y-label their y-axis will be at the same scale. By default they will be plotted in different subplots, but can be plotted in the same plot with the `-g` option.

Videos can be created using the `-v <subsampling>` option. ffmpeg or avconv will be used to create a mp4 file showing the plot evolution. Similarly, passing `--gif` uses pillow or imagemagick to create an animated gif.

## Examples

In the `examples` folder are shipped 4 use cases:
* `std_container` uses std::vectors and shows iteration-based, time-based and 3D pose plots. It also shows how to use Not a Number for iterations where some logged values are not defined.
* `visp_containers` uses containers from the ViSP library (vpColVector and vpPoseVector) and logs an inverted 3D pose
* `eigen_containers` uses containers from the Eigen library (Eigen::Vector3d)
* `animation` shows how to perform a plot during runtime
* `timed_xy` is an example of time-varying XY trajectory, only useful if saved as a video
* `static_3d` shows a 3D plot with only static 3D objects, some of which are displayed with a mesh
* `from_python.py` shows how to use the logger from Python

## Configuration file parser

If the option `BUILD_PARSER` is set to True (default) then a `log2plot::ConfigManager` class is also available. It allows easy loading of a configuration file written in Yaml through the templated `read` method.

The configuration manager can also generate dynamically suitable names for experimental files through the following methods:
* `setDirName(std::string s)`
* `addNameElement(std::string str)`
* `addNameElement(std::string pref, T val)`
* `addConditionalNameElement(std::string strTrue, bool condition, std::string strFalse)`
* `fullName()`: outputs the resulting file name from all above information
* `saveConfig()`: saves under `fullName() + _config.yaml`

See the corresponding example in `examples/parser`

## PlotJuggler plugin

YAML files that comply with the `log2plot` representation can be loaded into [PlotJuggler](https://github.com/facontidavide/PlotJuggler) through the plugin available in `plotjuggler` folder.
See the corresponding readme to compile it.
