#include <Python.h>
#include <log2plot/plot/figure_utils.h>
#include <iostream>

namespace log2plot {

namespace py
{

void call(const char* c)
{
 // std::cout << "Calling " << c << std::endl;
  PyRun_SimpleString(c);
}
void call(std::string s)
{
  call(s.c_str());
}
void call(const std::ostringstream& ss)
{
  call(ss.str().c_str());
}

std::string join(double v1, double v2)
{
  return std::to_string(v1) + ", " + std::to_string(v2);
}
}

int Figure::ax_count = 0;
int Figure::fig_count = 0;
Figure::Figure() :  ax_tag("ax_" + std::to_string(ax_count++))
{
  fig_count++;
  // load Python interpreter for first figure
  if(fig_count == 1)
  {
    Py_Initialize();
    py::call("import pylab as pl");
    py::call("from mpl_toolkits.mplot3d import axis3d");
    py::call("from numpy import nan");
    py::call("pl.switch_backend('tkagg')");
    py::call("pl.ion()");
  }
}

Figure::~Figure()
{
  py::call("pl.close(" + ax_tag + ".figure)");
  fig_count--;
  if(fig_count == 0)
    Py_Finalize();
}

}
