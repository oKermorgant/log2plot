#ifndef LOG2PLOTCONTAINERPLOTTER_H
#define LOG2PLOTCONTAINERPLOTTER_H

#include <log2plot/container.h>
#include <log2plot/plot.h>
#include <sstream>

namespace log2plot
{


template<class T> class ContainerPlotter : public GenericContainer
{
public:
  // constructor also takes nodehandle and variable name, used for the published topic
  ContainerPlotter(T &original, std::string _legend) : actual(&original), legend(decompose(_legend))
  { }

  // write additionnal info
  inline void writeInfo(const std::string &_label, const std::string &_info)
  {
    GenericContainer::writeInfo(_label, _info);

    // hijack line type
    if(_label == "lineType")
      line_type = decompose(_info);
  }

  void init()
  {
    ofs_ << "data:" << std::endl;

    // potentially several 2D-plots
    if(log_type == LogType::XY)
    {
      for(size_t i = 0; i < actual->size()/2; ++i)
        axis.addLine("$" + label(i) + "$", format(i));
    }
    else
    {
      for(size_t i = 0; i < actual->size(); ++i)
        axis.addLine("$" + label(i) + "$", format(i));
    }
  }

  // log update + refresh plot
  void update(double *t)
  {
    ofs_ << "    - [";

    if(log_type == LogType::XY)
    {
      // write content
      for(uint i=0;i<actual->size()/2;++i)
      {
        const double x = (*actual)[2*i];
        const double y = (*actual)[2*i+1];
        axis.updateLine(i, x, y);
        ofs_ << x << ", " << y << ((i == actual->size()/2-1) ? "]\n" : ", ");
      }
    }
    else
    {
      // write time if needed
      if(log_type == LogType::TIME)
      {
        ofs_ << *t << ", ";
        x_coord = *t;
      }
      else
        x_coord++;

      // write content
      for(uint i=0;i<actual->size();++i)
      {
        const double v = (*actual)[i];
        axis.updateLine(i, x_coord, v);
        ofs_ << v << ((i == actual->size()-1) ? "]\n" : ", ");
      }
    }

    axis.draw();
  }

protected:

  std::string format(size_t idx) const
  {
    if(idx < line_type.size())
      return line_type[idx];
    return "";
  }
  std::string label(size_t idx) const
  {
    if(idx < legend.size())
      return legend[idx];
    return "";
  }

  static std::vector<std::string> decompose(std::string list)
  {
    std::vector<std::string> tokens;
    if(list.size())
    {
      if(list.substr(0, 1) == "[")  // has brackets
        list = list.substr(1, list.size()-2);

      std::string token;
      std::istringstream tokenStream(list);
      while (std::getline(tokenStream, token, ','))
      {
        while(token.substr(0, 1) == " ")
          token = token.substr(1, token.npos);
        while(token.substr(token.size()-1, 1) == " ")
          token = token.substr(0, token.size()-2);
        tokens.push_back(token);
      }
    }

    return tokens;
  }

  T* actual;
  std::vector<std::string> legend, line_type;
  Axis axis;
  double x_coord = 0;
};

}

#endif // LOG2PLOTCONTAINERPLOTTER_H