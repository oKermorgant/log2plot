#include <log2plot/plot/figure2d.h>

namespace log2plot {

int Figure2D::Line2D::line_count = 0;

Figure2D::Line2D::Line2D(const std::string &ax_tag, bool xy, const std::string& format, const std::string& label)
  : tag(ax_tag + "_" + std::to_string(line_count++)),
    set_data_cmd(tag + ".set_data(" + (xy?tag:ax_tag) + "_x, " + tag + "_y)")
{
  // init xy variables
  if(xy)
    py::call(tag + "_x = []");
  py::call(tag + "_y = []");
  std::string plot_cmd = tag + ", = " + ax_tag + ".plot([], []";
  if(format != "")
    plot_cmd += ", '" + format + "'";
  if(label != "")
    plot_cmd += ", label = '" + label + "'";
  py::call(plot_cmd + ")");
}

void Figure2D::Line2D::append(double x, double y)
{
  py::call(tag + "_x.append(" + std::to_string(x) + ")");
  py::call(tag + "_y.append(" + std::to_string(y) + ")");
}

void Figure2D::Line2D::append(double y)
{
  py::call(tag + "_y.append(" + std::to_string(y) + ")");
}

void Figure2D::Line2D::createEdge(const std::string &ax_tag, const std::string &format, const Vectord &p1, const Vectord &p2)
{
  std::ostringstream plot_cmd(ax_tag);
  plot_cmd << ".plot(["
           << p1[0] << ", " << p2[0] << "], ["
           << p1[1] << ", " << p2[1] << "]";
  if(format != "")
    plot_cmd << ", '" << format << "'";
  plot_cmd << ")";
  py::call(plot_cmd);
}

Figure2D::Figure2D(bool _xy)
  : xy(_xy)
{
  // create new 3D figure
  py::call("pl.figure(figsize=(7.5,5))");
  py::call(ax_tag + " = pl.gca()");

  // single variable for x-data if not xy plot
  if(!xy)
  {
    py::call(ax_tag + "_x = []");

    // add a 0-line
    py::call(ax_tag + "_00, = pl.plot([], [], 'k-')");
  }
}

void Figure2D::updateLimits(double x, double y)
{
  if(xl_prev.max < xl_prev.min)
  {
    xl.set(x);
    yl.set(y);
  }
  else
  {
    xl.update(x);
    yl.update(y);
  }
}

void Figure2D::plotEdge(const Vectord &p1, const Vectord &p2, const std::string &format)
{
  updateLimits(p1[0], p1[1]);
  updateLimits(p2[0], p2[1]);
  Line2D::createEdge(ax_tag, format, p1, p2);
}

void Figure2D::refresh()
{
  if(xl_prev.different(xl))
  {
    const double dx(std::max(0.05*(xl.max-xl.min), 0.01));
    py::call(ax_tag + ".set_xlim(" + py::join(xl.min-dx, xl.max+dx) + ")");
    xl_prev.update(xl);
  }
  if(yl_prev.different(yl))
  {
    const double dy(std::max(0.05*(yl.max-yl.min), 0.01));
    py::call(ax_tag + ".set_ylim(" + py::join(yl.min-dy, yl.max+dy) + ")");
    yl_prev.update(yl);
    py::call(ax_tag + ".legend(loc = 'upper right')");
  }
}

void Figure2D::updateLine(uint idx, double x, double y)
{
  if(xy)
  {
    lines[idx].append(x, y);
  }
  else
  {
    if(idx == 0)
    {
      py::call(ax_tag + "_x.append(" + std::to_string(x) + ")");
      py::call(ax_tag + "_00.set_data([" + py::join(0, x) + "], [0,0])");
    }
    lines[idx].append(y);
  }
  lines[idx].draw();
  updateLimits(x, y);
}

}
