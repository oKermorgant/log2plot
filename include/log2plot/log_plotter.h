#ifndef LOG2PLOT_LOG_PLOTTER_H
#define LOG2PLOT_LOG_PLOTTER_H

#include <log2plot/defines.h>
#include <log2plot/logger.h>
#include "plot/container_plotter2d.h"
#include "plot/container_plotter3d.h"
#include <assert.h>

namespace log2plot
{

class LogPlotter : public Logger
{
  ContainerPlotter *last_plot;

public:
  LogPlotter(std::string _file_path = "", unsigned int _buffer = 10, unsigned int _subsampling = 1)
    : Logger(_file_path, _buffer, _subsampling)
  { }

  void setLineType(const std::string lineType)
  {
    last->writeInfo("lineType", lineType);
  }

  void update(const bool &flush = false)
  {
    Logger::update(flush);
    py::call("pl.pause(0.0001)");
  }

  void showFixedObject(const std::vector<std::vector<double>> &M, const std::string &graph, const std::string &color = "")
  {
    Logger::showFixedObject<std::vector<double>>(M, graph, color);
    last_plot->showFixedObject(M, graph, color);
  }
  void showMovingObject(const std::vector<std::vector<double>> &M, const std::string &graph, const std::vector<double> &desired_pose = {})
  {
    assert(desired_pose.size() %3 == 0);
    Logger::showMovingObject(M, graph, desired_pose);
    last_plot->showMovingObject(M, graph, desired_pose);
  }

  // Save iteration-based vector
  template<class T>
  inline void save(T &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
  {
    // add this to logged variables
    logged_vars.push_back(std::make_unique<ContainerPlotter2D<T>>(LogType::ITERATION, v, buildLegend(legend, v.size())));
    last_plot = static_cast<ContainerPlotter*>(logged_vars.back().get());
    // set labels
    py::call("pl.xlabel('iterations')");
    py::call("pl.ylabel('" + ylabel + "')");
    py::call("pl.tight_layout()");
    // and write initial info
    writeInitialInfo(name, buildLegend(legend, v.size()), "iterations", ylabel, keep_file);
  }

  // Save time-based vector
  template<class T>
  inline void saveTimed(T &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
  {
    // add this to logged variables
    logged_vars.push_back(std::make_unique<ContainerPlotter2D<T>>(LogType::TIME, v, buildLegend(legend, v.size())));
    last_plot = static_cast<ContainerPlotter*>(logged_vars.back().get());
    // set labels
    py::call("pl.xlabel('time [" + time_unit + "]')");
    py::call("pl.ylabel('" + ylabel + "')");
    py::call("pl.tight_layout()");
    // and write initial info
    writeInitialInfo(name, buildLegend(legend, v.size()), "time [" + time_unit + "]", ylabel, keep_file);
  }

  // Save XY vector
  template<class T>
  inline void saveXY(T &v, const std::string &name, const std::string &legend,
                     const std::string &xlabel, const std::string &ylabel, bool keep_file = true)
  {
    // add this to logged variables
    logged_vars.push_back(std::make_unique<ContainerPlotter2D<T>>(LogType::XY, v, buildLegend(legend, v.size()/2)));
    last_plot = static_cast<ContainerPlotter*>(logged_vars.back().get());
    // set labels
    py::call("pl.xlabel('" + xlabel + "')");
    py::call("pl.ylabel('" + ylabel + "')");
    py::call("pl.tight_layout()");
    // and write initial info
    writeInitialInfo(name, buildLegend(legend, v.size()/2), xlabel, ylabel, keep_file);
  }

  // Save 3D pose or position
  template<class T>
  inline void save3Dpose(T &v, const std::string &name, const std::string &legend,
                         bool invert = false, bool keep_file = true)
  {
    assert(v.size() == 3 || v.size() == 6);
    // add this to logged variables
    logged_vars.push_back(std::make_unique<ContainerPlotter3D<T>>(LogType::POSE, v, legend, invert));
    last_plot = static_cast<ContainerPlotter*>(logged_vars.back().get());
    // and write initial info
    writeInitialInfo(name, "["+legend+"]", "", "", keep_file);
    if(invert)
      last->writeInfo("invertPose", "True");
  }

  void plot(bool verbose = false, bool display = true)
  {
    Logger::plot(verbose, display);
    py::call("try:\n  pl.pause(0)\nexcept:\n  pass");
  }
};
}

#endif // LOG2PLOT_LOG_PLOTTER_H
