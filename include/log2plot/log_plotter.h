#ifndef LOG2PLOT_LOG_PLOTTER_H
#define LOG2PLOT_LOG_PLOTTER_H

#include <log2plot/logger.h>
#include <log2plot/plot/container_plotter2d.h>
#include <log2plot/plot/container_plotter3d.h>
#include <assert.h>
#include <Python.h>

namespace log2plot
{

class LogPlotter : public Logger
{
  ContainerPlotter *last_plot = nullptr;

public:
  LogPlotter(const std::string &_file_path = "", float dt = 0.1, unsigned int _buffer = 10, unsigned int _subsampling = 1)
    : Logger(_file_path, _buffer, _subsampling)
  {
    Figure::setTargetSampling(dt);
  }

  void setLineType(const std::string &lineType)
  {
    last->writeInfo("lineType", lineType);
  }

  void update(const bool &flush = false)
  {
    const bool first_update_plot(first_update);
    Logger::update(flush);

    if(last_plot)
    {
      if(first_update_plot)
        last_plot->run("pl.pause(0.0001)");
      last_plot->waitExecution();
    }
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
    last_plot->run("pl.xlabel('iterations')");
    last_plot->run("pl.ylabel('" + ylabel + "')");
    last_plot->run("pl.tight_layout()");
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
    last_plot->run("pl.xlabel('time [" + time_unit + "]')");
    last_plot->run("pl.ylabel('" + ylabel + "')");
    last_plot->run("pl.tight_layout()");
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
    last_plot->run("pl.xlabel('" + xlabel + "')");
    last_plot->run("pl.ylabel('" + ylabel + "')");
    last_plot->run("pl.tight_layout()");
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
};
}

#endif // LOG2PLOT_LOG_PLOTTER_H
