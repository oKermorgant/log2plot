#ifndef LOG2PLOT_CONTAINER_PLOTTER3D_H
#define LOG2PLOT_CONTAINER_PLOTTER3D_H
#include <log2plot/container.h>
#include "figure3d.h"
#include "container_plotter.h"

namespace log2plot
{

template<class T> class ContainerPlotter3D : public ContainerPlotter
{
public:
  ContainerPlotter3D(LogType _type, T &original, std::string _legend, bool invert) :
    ContainerPlotter(_type, _legend), content(&original), fig(invert)
  { }

  void init()
  {
    yaml_stream << "data:" << std::endl;
    fig.init("$" + label(0) + "$", format(0), format(1), format(2), format(3));
  }

  // log update + refresh plot
  void update(double *t)
  {
    yaml_stream << "    - [";

    // write content
    unsigned int i;
    for(i=0;i<content->size()-1;++i)
      yaml_stream << (*content)[i] << ", ";
    yaml_stream << (*content)[i] << "]\n";

    fig.refresh(*content);
  }

  void showFixedObject(const std::vector<Vectord> &M, const std::string &graph_s, const std::string &color = "")
  {
    const auto graph = decomposeGraph(graph_s);
    for(const auto &[p1, p2]: graph)
      fig.plotEdge(M[p1], M[p2], color);
  }

  void showMovingObject(const std::vector<Vectord> &M, const std::string &graph_s, const Vectord &desired_pose)
  {
    fig.addObject(M, decomposeGraph(graph_s), desired_pose);
  }

protected:

  T* content;
  Figure3D fig;
};

}
#endif // CONTAINER_PLOTTER3D_H
