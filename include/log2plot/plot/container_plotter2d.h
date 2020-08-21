#ifndef LOG2PLOT_CONTAINERPLOTTER2D_H
#define LOG2PLOT_CONTAINERPLOTTER2D_H

#include <log2plot/container.h>
#include "figure2d.h"
#include "container_plotter.h"
#include <sstream>

namespace log2plot
{

template<class T> class ContainerPlotter2D : public ContainerPlotter
{
public:
  ContainerPlotter2D(LogType _type, T &original, std::string _legend) :
    ContainerPlotter(_type, _legend), content(&original), fig(_type == LogType::XY)
  { }

  void init()
  {
    yaml_stream << "data:" << std::endl;

    if(log_type == LogType::XY)
    {
      for(size_t i = 0; i < content->size()/2; ++i)
        fig.addLine("$" + label(i) + "$", format(i));
    }
    else
    {
      for(size_t i = 0; i < content->size(); ++i)
       fig.addLine("$" + label(i) + "$", format(i));
    }
  }

  // log update + refresh plot
  void update(double *t)
  {
    yaml_stream << "    - [";

    if(log_type == LogType::XY)
    {
      // write content
      for(uint i=0;i<content->size()/2;++i)
      {
        const double x = (*content)[2*i];
        const double y = (*content)[2*i+1];
        fig.updateLine(i, x, y);
        yaml_stream << x << ", " << y << ((i == content->size()/2-1) ? "]\n" : ", ");
      }
    }
    else
    {
      // write time if needed
      if(log_type == LogType::TIME)
      {
        yaml_stream << *t << ", ";
        x_coord = *t;
      }
      else
        x_coord++;

      // write & display content      
      for(uint i=0;i<content->size();++i)
      {
        const double v = (*content)[i];
        fig.updateLine(i, x_coord, v);
        yaml_stream << v << ((i == content->size()-1) ? "]\n" : ", ");
      }
    }
    fig.refresh();
  }

  void showFixedObject(const std::vector<Vectord> &M, const std::string &graph_s, const std::string &color = "")
  {
    const auto graph = decomposeGraph(graph_s);
    // build x and y vectors from coords in M and graph
    for(const auto &[p1, p2]: graph)
      fig.plotEdge(M[p1],M[p2], color);
  }

protected:

  T* content;

  Figure2D fig;
  double x_coord = 0;
};

}

#endif // LOG2PLOTCONTAINERPLOTTER2D_H
