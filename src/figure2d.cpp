#include <log2plot/plot/figure2d.h>
#include <iostream>
namespace log2plot {

int Figure2D::Line2D::line_count = 0;

Figure2D::Line2D::Line2D(const std::string &ax_tag,
                         size_t id,
                         bool xy, const std::string& format, const std::string& label)
  : callerID(id), tag(ax_tag + "_" + std::to_string(line_count++)),
    set_data_cmd(tag + ".set_data(" + (xy?tag:ax_tag) + "_x, " + tag + "_y)")
{
  // init xy variables
  if(xy)
    run(tag + "_x = []");
  run(tag + "_y = []");
  std::string plot_cmd = tag + ", = " + ax_tag + ".plot([], []";
  if(format != "")
    plot_cmd += ", '" + format + "'";
  if(label != "")
    plot_cmd += ", label = '" + label + "'";
  run(plot_cmd + ")");
}

void Figure2D::Line2D::append(double x, double y)
{
  run(tag + "_x.append(" + std::to_string(x) + ")");
  run(tag + "_y.append(" + std::to_string(y) + ")");
}

void Figure2D::Line2D::append(double y)
{
  run(tag + "_y.append(" + std::to_string(y) + ")");
}

void Figure2D::Line2D::createEdge(const std::string &ax_tag,
                                  const std::string &format, const Vectord &p1, const Vectord &p2)
{
  std::ostringstream plot_cmd;;
  plot_cmd << ax_tag << ".plot(["
           << p1[0] << ", " << p2[0] << "], ["
           << p1[1] << ", " << p2[1] << "]";
  if(format != "")
    plot_cmd << ", '" << format << "'";
  plot_cmd << ")";
  interp->run(plot_cmd.str());
}

Figure2D::Figure2D(bool _xy) : xy(_xy)
{
  // create new 3D figure
  run("pl.figure(figsize=(7.5,5))");
  run(ax_tag + " = pl.gca()");

  // single variable for x-data if not xy plot
  if(!xy)
  {
    run(ax_tag + "_x = []");

    // add a 0-line
    run(ax_tag + "_00, = pl.plot([], [], 'k-')");
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
  bool redraw_fig(false);
  if(xl_prev.different(xl))
  {
    const double dx(std::max(0.05*(xl.max-xl.min), 0.01));
    run(ax_tag + ".set_xlim(" + py::join(xl.min-dx, xl.max+dx) + ")", false);
    xl_prev.update(xl);
    redraw_fig = true;
  }
  if(yl_prev.different(yl))
  {
    const double dy(std::max(0.05*(yl.max-yl.min), 0.01));
    run(ax_tag + ".set_ylim(" + py::join(yl.min-dy, yl.max+dy) + ")", false);
    yl_prev.update(yl);
    run(ax_tag + ".legend(loc = 'upper right')", false);
    redraw_fig = true;
  }
  if(redraw_fig)
    run(ax_tag + ".figure.canvas.draw()", false);    
  else
  {
    for(const auto &line: lines)
      run(ax_tag + ".draw_artist(" + line.tag + ")",false);
  }
  run(ax_tag + ".figure.canvas.flush_events()", false);
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
      run(ax_tag + "_x.append(" + std::to_string(x) + ")");
      run(ax_tag + "_00.set_data([" + py::join(0, x) + "], [0,0])");
    }
    lines[idx].append(y);
  }
  lines[idx].draw();
  updateLimits(x, y);
}
}
