#include <log2plot/plot/figure3d.h>

#include <iostream>
#include <sstream>

namespace log2plot {

int Figure3D::Line3D::line_count = 0;

void Figure3D::Line3D::setData(const std::string &x, const std::string &y, const std::string &z,
                               bool required)
{
  run(tag + ".set_data_3d(" + x + ", " + y + ", " + z + ")", required);
}

void Figure3D::Line3D::init(const std::string &ax_tag,
                            const std::string &format,
                            const std::string &label)
{
  std::string plot_cmd = tag + ", = " + ax_tag + ".plot([], [], []";
  if(format != "")
    plot_cmd += ", '" + format + "'";
  if(label != "")
  {
    plot_cmd += ", label = '" + label + "'";
    // init xyz variables
    run(tag + "_x = []");
    run(tag + "_y = []");
    run(tag + "_z = []");
  }
  run(plot_cmd + ")");
}

void Figure3D::Line3D::append(double x, double y, double z)
{
  run(tag + "_x.append(" + std::to_string(x) + ")");
  run(tag + "_y.append(" + std::to_string(y) + ")");
  run(tag + "_z.append(" + std::to_string(z) + ")");
}

void Figure3D::Edge3D::createEdge(const std::string &ax_tag, const std::string &format, const Vectord &p1, const Vectord &p2)
{
  std::ostringstream plot_cmd;
  plot_cmd << ax_tag << ".plot(["
           << p1[0] << ", " << p2[0] << "], ["
           << p1[1] << ", " << p2[1] << "], ["
           << p1[2] << ", " << p2[2] << "]";
  if(format != "")
    plot_cmd << ", '" << format << "'";
  plot_cmd << ")";
  interp->run(plot_cmd.str());
}

void Figure3D::Edge3D::update(const Translation &t, const Rotation &R, double scale)
{
  const auto p1 = t + R.rotate(t1)*scale;
  const auto p2 = t + R.rotate(t2)*scale;

  setData("[" + py::join(p1[0], p2[0]) + "]",
      "[" + py::join(p1[1], p2[1]) + "]",
      "[" + py::join(p1[2], p2[2]) + "]", false);
}

void Figure3D::updateLimits(double x, double y, double z)
{
  if(xl.min > xl.max)
  {
    xl.set(x);
    yl.set(y);
    zl.set(z);
  }
  else
  {
    xl.update(x);
    yl.update(y);
    zl.update(z);
  }
}

Figure3D::Figure3D(bool _invert) : invert(_invert), traj(ax_tag,newCallerID())
{
  // create new 3D figure
  run("F = pl.figure(figsize=(7,7))");
  run(ax_tag + " = F.add_subplot(111, projection='3d')");
  run("pl.tight_layout()");
}

void Figure3D::init(const std::string &label, const std::string &format,
                    const std::string &moving_fmt,
                    const std::string &initial_fmt,
                    const std::string &desired_fmt)
{
  // main line
  traj.init(ax_tag, format, label);

  // moving edges
  for(auto &edge: moving)
    edge.init(ax_tag, moving_fmt);
  // scaled edges
  for(auto &edge: scaled)
    edge.init(ax_tag, desired_fmt);

  if(moving.size())
    initial_format = initial_fmt;

  run("pl.legend(loc = 'upper right')");
}

void Figure3D::addObject(const std::vector<Vectord> &M,
                         const std::vector<std::pair<size_t, size_t>> &graph,
                         const Vectord &desired_pose)
{
  auto objID(newCallerID());

  std::vector<Translation> nodes;
  for(const auto &p: M)
    nodes.emplace_back(p[0], p[1], p[2]);

  for(const auto &[p1, p2]: graph)
    moving.emplace_back(ax_tag, objID, nodes[p1], nodes[p2]);

  if(desired_pose.size())
  {
    const auto [t, R] = decompose(desired_pose); {}
    updateLimits(t);

    objID = newCallerID();

    for(const auto &[p1, p2]: graph)
      scaled.emplace_back(ax_tag, objID, nodes[p1], nodes[p2], t, R);
  }
}

void Figure3D::addInitialObject(const Translation &t, const Rotation &R)
{
  if(moving.size() == 0 || initial_format == "done")
    return;

  const auto objID(newCallerID());
  for(const auto &edge: moving)
  {
    scaled.emplace_back(ax_tag, objID, edge.t1, edge.t2, t, R);
    scaled.back().init(ax_tag, initial_format);
  }
  initial_format = "done";
}

void Figure3D::plotEdge(const Vectord &p1, const Vectord &p2, const std::string &format)
{
  updateLimits(p1);
  updateLimits(p2);
  Edge3D::createEdge(ax_tag, format, p1, p2);
}

double Figure3D::resizeBox()
{
  const double half_range(.5*std::max(xl.max-xl.min, std::max(yl.max-yl.min,
                                                              std::max(zl.max-zl.min, 0.01))));

  const double x0(.5*(xl.max+xl.min)),
      y0(.5*(yl.max+yl.min)), z0(.5*(zl.max+zl.min));
  run(ax_tag + ".set_xlim3d(" + py::join(x0-half_range, x0+half_range) + ")", false);
  run(ax_tag + ".set_ylim3d(" + py::join(y0-half_range, y0+half_range) + ")", false);
  run(ax_tag + ".set_zlim3d(" + py::join(z0-half_range, z0+half_range) + ")", false);

  xl_prev.update(xl);
  yl_prev.update(yl);
  zl_prev.update(zl);

  return 0.04*half_range;
}
}


