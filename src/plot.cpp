#include <log2plot/plot.h>

namespace log2plot
{

Axis::Axis()
{
  const auto empty = matplotlibcpp::detail::_interpreter::get().s_python_empty_tuple;
  fig = PyObject_CallObject(matplotlibcpp::detail::_interpreter::get().s_python_function_figure,
                            empty);
  gca = PyObject_Call(PyObject_GetAttrString(fig, "gca"), empty, nullptr);
  axis_fct = PyObject_GetAttrString(gca, "axis");
}

void Axis::addLine(const std::string& name, const std::string& format)
{
  lines.push_back(std::unique_ptr<Line>(new Line(name, format, gca)));
  if(name != "$$")
    PyObject_Call(PyObject_GetAttrString(gca, "legend"),
                  matplotlibcpp::detail::_interpreter::get().s_python_empty_tuple, nullptr);
}

void Axis::updateLine(uint idx, double x, double y)
{
  if(idx == 0)
  {
    xmin = std::min(xmin, x);
    xmax = std::max(xmax, x);
  }
  ymin = std::min(ymin, y);
  ymax = std::max(ymax, y);

  lines[idx]->update(x, y);
}

void Axis::draw()
{
  auto axis_args = PyTuple_New(1);
  auto axis_values = PyTuple_New(4);
  PyTuple_SetItem(axis_values, 0, PyFloat_FromDouble(xmin));
  PyTuple_SetItem(axis_values, 1, PyFloat_FromDouble(xmax>xmin?xmax:(xmax+1)));
  PyTuple_SetItem(axis_values, 2, PyFloat_FromDouble(ymin));
  PyTuple_SetItem(axis_values, 3, PyFloat_FromDouble(ymax>ymin?ymax:(ymax+1)));

  PyTuple_SetItem(axis_args, 0, axis_values);
  auto res = PyObject_Call(axis_fct, axis_args, nullptr);
  Py_DECREF(res);
  Py_DECREF(axis_args);
  Py_DECREF(axis_values);
}


DynPyArray::DynPyArray(): array(PyList_New(0)), app_fct(PyObject_GetAttrString(array,"append"))
{}


void DynPyArray::append(double v)
{
  auto app_args = PyTuple_New(1);
  PyTuple_SetItem(app_args, 0, PyFloat_FromDouble(v));
  PyObject* res = PyObject_CallObject(app_fct, app_args);
  if(res)
    Py_DECREF(res);
  Py_DECREF(app_args);
}


// for 2D plot
Line::Line(const std::string& name, const std::string& format, PyObject* gca)
{
  PyObject* kwargs = PyDict_New();
  if(name != "")
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

  PyObject* pystring = PyString_FromString(format.c_str());

  PyObject* plot_args = PyTuple_New(3);
  PyTuple_SetItem(plot_args, 0, x.array);
  PyTuple_SetItem(plot_args, 1, y.array);
  PyTuple_SetItem(plot_args, 2, pystring);

  PyObject* res = PyObject_Call(PyObject_GetAttrString(gca, "plot"), plot_args, kwargs);

  Py_DECREF(kwargs);
  Py_DECREF(plot_args);

  if(res)
  {
    line= PyList_GetItem(res, 0);

    if(line)
      set_data_fct = PyObject_GetAttrString(line,"set_data");
    else
      Py_DECREF(line);
    Py_DECREF(res);
  }
}


void Line::update(double _x, double _y)
{
  x.append(_x);
  y.append(_y);

  if(set_data_fct)
  {
    PyObject* plot_args = PyTuple_New(2);
    PyTuple_SetItem(plot_args, 0, x.array);
    PyTuple_SetItem(plot_args, 1, y.array);

    PyObject* res = PyObject_CallObject(set_data_fct, plot_args);
    if (res) Py_DECREF(res);
  }
}

// for 3D plot
/*
Line::Line(const std::string& name, const std::string& format, long fig_nb)
{
  // init 3D plot
  if (!mpl_toolkitsmod) {
    matplotlibcpp::detail::_interpreter::get();

    PyObject* mpl_toolkits = PyString_FromString("mpl_toolkits");
    PyObject* axis3d = PyString_FromString("mpl_toolkits.mplot3d");
    if (!mpl_toolkits || !axis3d) { throw std::runtime_error("couldnt create string"); }

    mpl_toolkitsmod = PyImport_Import(mpl_toolkits);
    Py_DECREF(mpl_toolkits);
    if (!mpl_toolkitsmod) { throw std::runtime_error("Error loading module mpl_toolkits!"); }

    axis3dmod = PyImport_Import(axis3d);
    Py_DECREF(axis3d);
    if (!axis3dmod) { throw std::runtime_error("Error loading module mpl_toolkits.mplot3d!"); }
  }

  PyObject* kwargs = PyDict_New();
  if(name != "")
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

  PyObject* pystring = PyString_FromString(format.c_str());

  PyObject* plot_args = PyTuple_New(4);
  PyTuple_SetItem(plot_args, 0, x.array);
  PyTuple_SetItem(plot_args, 1, y.array);
  PyTuple_SetItem(plot_args, 2, z.array);
  PyTuple_SetItem(plot_args, 3, pystring);

  PyObject* res = PyObject_Call(matplotlibcpp::detail::_interpreter::get().s_python_function_plot, plot_args, kwargs);

  Py_DECREF(kwargs);
  Py_DECREF(plot_args);

  if(res)
  {
    line= PyList_GetItem(res, 0);

    if(line)
      set_data_fct = PyObject_GetAttrString(line,"set_data");
    else
      Py_DECREF(line);
    Py_DECREF(res);
  }
}

void Line::update(double _x, double _y, double _z)
{
  x.append(_x);
  y.append(_y);
  z.append(_z);
}*/

}
