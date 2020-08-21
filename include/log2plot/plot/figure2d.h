#ifndef LOG2PLOT_FIGURE2D_H
#define LOG2PLOT_FIGURE2D_H

#include "figure_utils.h"
#include <vector>

namespace log2plot {

using Vectord = std::vector<double>;

class Figure2D : public Figure
{
  struct Line2D
  {
    Line2D(const std::string &ax_tag, bool xy, const std::string& format, const std::string& label = "");
    static void createEdge(const std::string &ax_tag,
                const std::string &format,
                const Vectord &p1, const Vectord &p2);
    void append(double y);
    void append(double x, double y);
    void draw()
    {
      py::call(set_data_cmd);
    }
    const std::string tag;
    const std::string set_data_cmd;
    static int line_count;
  };

public:
  Figure2D(bool _xy);

  void addLine(const std::string& label, const std::string& format)
  {
    lines.emplace_back(ax_tag, xy, format, label);
  }
  void refresh();
  void updateLine(uint idx, double x, double y);

  // display fixed plot
  void plotEdge(const Vectord &p1, const Vectord &p2, const std::string &format);

private:
  const bool xy;
  void updateLimits(double x, double y);
  std::vector<Line2D> lines;
};


}

#endif // LOG2PLOT_FIGURE2D_H
