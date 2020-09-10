#ifndef LOG2PLOT_CONTAINER_PLOTTER_H
#define LOG2PLOT_CONTAINER_PLOTTER_H

#include <log2plot/container.h>
#include <sstream>

namespace log2plot
{

class ContainerPlotter : public GenericContainer
{
public:
  ContainerPlotter(LogType _type, std::string _legend);

  // write additionnal info
  void writeInfo(const std::string &_label, const std::string &_info);

  virtual void run(const std::string &line) = 0;
  virtual void showFixedObject(const std::vector<std::vector<double>> &M, const std::string &graph, const std::string &color = "") = 0;
  virtual void showMovingObject(const std::vector<std::vector<double>> &, const std::string &, const std::vector<double> & = {})
  {}

  virtual void waitExecution() = 0;


protected:
  std::vector<std::string> legend, line_type;

  static std::vector<std::string> decompose(std::string list);
  static std::vector<std::pair<size_t, size_t>> decomposeGraph(const std::string &graph);

  std::string format(size_t idx) const;
  std::string label(size_t idx) const;
};

}

#endif // LOG2PLOT_CONTAINER_PLOTTER_H
