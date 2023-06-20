#ifndef LOG2PLOT_LOADER_H
#define LOG2PLOT_LOADER_H

#include <vector>
#include <string>
#include <log2plot/log_types.h>

namespace log2plot
{

namespace Loader
{
template <class Node, class Dst>
inline bool load(const Node &node, Dst &dst)
{
  if(!node)
    return false;
  dst = node.template as<Dst>();
  return true;
}
}

// load and store basic content of a data file
struct Log
{
  struct FixedObject
  {
    std::vector<std::vector<double>> points;
    std::vector<std::vector<uint>> graph;
    std::string legend;
    explicit FixedObject() {}
  };

  std::vector<std::string> files;
  std::vector<std::string> legend;
  LogType type;
  bool invert_pose;
  std::vector<std::vector<double>> data;
  std::vector<FixedObject> fixed_objects;

  explicit Log(const std::string &path);

};

}


#endif // LOADER_H
