#ifndef LOG2PLOT_H
#define LOG2PLOT_H

#include "matplotlibcpp.h"
#include <memory>

namespace log2plot
{

void decRefs(std::vector<PyObject*> refs)
{
  for(auto &ref: refs)
  {
    if(ref && ref->ob_refcnt)
      Py_DECREF(ref);
  }
}

struct DynPyArray
{
  DynPyArray();
  ~DynPyArray() {decRefs({array, app_fct});}
  void append(double v);
  PyObject* array;
  PyObject* app_fct;
};

class Line
{
public:
    // for 2D plot
    Line(const std::string& name, const std::string& format, PyObject* gca);
    void update(double _x, double _y);

    // for 3D plot -- maybe later
    /*Line(const std::string& name, const std::string& format, long fig_nb);
    void update(double _x, double _y, double _z);*/

    ~Line() {decRefs({line, set_data_fct});}

private:
    PyObject* line = nullptr;
    PyObject* set_data_fct = nullptr;
    DynPyArray x, y;
};

struct Axis
{
  Axis();
  ~Axis() {decRefs({axis_fct, gca, fig});}

  void addLine(const std::string& name, const std::string& format);
  void updateLine(uint idx, double x, double y);
  void draw();

  double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
  PyObject* gca;
  PyObject* fig;
  PyObject* axis_fct;
  std::vector<std::unique_ptr<Line>> lines;
};

}

#endif // LOG2PLOT_H
