#ifndef LOG2PLOT_FIGURE3D_H
#define LOG2PLOT_FIGURE3D_H

#include <log2plot/plot/figure_utils.h>
#include <vector>
#include <array>
#include <math.h>

namespace log2plot {

using Vectord = std::vector<double>;

class Figure3D : public Figure
{
  // some helpers to avoid dependencies
  struct Translation : public std::array<double, 3>
  {
    Translation(double x, double y, double z)
      : std::array<double, 3>{x, y, z} {}
    Translation operator+(const Translation &t) const
    {
      return {data()[0]+t[0], data()[1]+t[1], data()[2]+t[2]};
    }
    Translation operator*(double scale) const
    {
      return {scale*data()[0], scale*data()[1], scale*data()[2]};
    }
  };

  struct Rotation : public std::array<double, 9>
  {
    double & at(int row, int col)
    {
      return data()[row+3*col];
    }
    double at(int row, int col) const
    {
      return data()[row+3*col];
    }
    Rotation(double x = 0, double y = 0, double z = 0)
    {
      buildFrom(x, y, z);
    }
    void buildFrom(double x, double y, double z)
    {
      const double theta = sqrt(x*x+y*y+z*z);
      if(theta > 1e-3)
      {
        x /= theta;
        y /= theta;
        z /= theta;
      }
      const double c(cos(theta)), s(sin(theta)), cm(1-c);
      at(0,0) = x*x*cm + c;
      at(0,1) = x*y*cm - z*s;
      at(0,2) = x*z*cm + y*s;
      at(1,0) = x*y*cm + z*s;
      at(1,1) = y*y*cm + c;
      at(1,2) = y*z*cm - x*s;
      at(2,0) = x*z*cm - y*s;
      at(2,1) = y*z*cm + x*s;
      at(2,2) = z*z*cm + c;
    }
    Rotation t() const
    {
      Rotation transpose;
      for(int i = 0; i < 3; ++i)
      {
        for(int j = 0; j < 3; ++j)
          transpose.at(i, j) = at(j,i);
      }
      return transpose;
    }
    Translation rotate(const Translation &t) const
    {
      return {at(0,0)*t[0] + at(0,1)*t[1] + at(0,2)*t[2],
            at(1,0)*t[0] + at(1,1)*t[1] + at(1,2)*t[2],
            at(2,0)*t[0] + at(2,1)*t[1] + at(2,2)*t[2]};
    }
  };

  struct Line3D
  {
    Line3D(const std::string &ax_tag,size_t id)
      : callerID(id), tag(ax_tag + "_" + std::to_string(line_count++)) {}

    void init(const std::string &ax_tag,
              const std::string &format,
              const std::string &label = "");
    void append(double x, double y, double z);
    void setData(const std::string &x, const std::string &y, const std::string &z,
                 bool required = true);
    template <class Position>
    void append(const Position &p)
    {
      append(p[0], p[1], p[2]);
    }
    void draw()
    {
      setData(tag + "_x", tag + "_y", tag + "_z", false);
    }

    void run(const std::ostringstream& ss, bool required = true)
    {
      interp->run(ss.str(), required ? 0 : callerID);
    }
    void run(const std::string &line, bool required = true)
    {
      interp->run(line, required ? 0 : callerID);
    }

    const size_t callerID;
    const std::string tag;
    static int line_count;
  };

  struct Edge3D : public Line3D
  {
    const Translation t1, t2;
    Edge3D(const std::string &ax_tag, size_t id, Translation _t1, Translation _t2)
      : Line3D(ax_tag, id), t1(_t1), t2(_t2)
    {}
    void update(const Translation &t, const Rotation &R, double scale = 1);
    static void createEdge(const std::string &ax_tag,
                           const std::string &format,
                           const Vectord &p1, const Vectord &p2);
  };

  struct FixedEdge3D : public Edge3D
  {
    const Translation t;
    const Rotation R;
    FixedEdge3D(const std::string &tag, size_t id, Translation _t1, Translation _t2,
                const Translation &_t, const Rotation &_R)
      : Edge3D(tag, id, _t1, _t2), t(_t), R(_R) {}
    void scale(double scale)
    {
      update(t, R, scale);
    }
  };

public:
  Figure3D(bool _invert);
  ~Figure3D() = default;

  void init(const std::string& name,
            const std::string& format,
            const std::string &moving_fmt,
            const std::string &initial_fmt,
            const std::string &desired_fmt);

  template <class Pose>
  void refresh(const Pose &pose)
  {
    const auto [t, R] = decompose(pose); {}
    traj.append(t);
    traj.draw();

    updateLimits(t);

    const bool limits_change(xl_prev.different(xl) ||
                             yl_prev.different(yl) ||
                             zl_prev.different(zl));

    if(limits_change)
      scale = resizeBox();

    // move and scale attached objects
    for(auto &edge: moving)
      edge.update(t, R, scale);

    addInitialObject(t, R);

    // scale fixed objects
    if(limits_change)
    {
      for(auto &edge: scaled)
        edge.scale(scale);
    }
    run(ax_tag + ".figure.canvas.draw()", false);
    run(ax_tag + ".figure.canvas.flush_events()", false);
  }

  void addObject(const std::vector<Vectord> &M,
                 const std::vector<std::pair<size_t, size_t>> &graph,
                 const Vectord &desired_pose);

  void plotEdge(const Vectord &p1, const Vectord &p2, const std::string &format);

private:
  template <class Position>
  void updateLimits(const Position &p)
  {
    updateLimits(p[0], p[1], p[2]);
  }

  void updateLimits(double x, double y, double z);
  double resizeBox();

  void addInitialObject(const Translation &t, const Rotation &R);

  std::string initial_format;
  const bool invert;
  Limits zl, zl_prev;
  double scale = 0;

  template <class Pose>
  std::pair<Translation, Rotation>
  decompose(const Pose &pose)
  {
    Translation t{pose[0], pose[1], pose[2]};
    Rotation R;
    if(pose.size() == 6)
      R.buildFrom(pose[3], pose[4], pose[5]);
    if(invert)
    {
      R = R.t();
      t = R.rotate(t);
      t[0] *= -1;
      t[1] *= -1;
      t[2] *= -1;
    }
    return {t, R};
  }

  Line3D traj;
  std::vector<Edge3D> moving;
  std::vector<FixedEdge3D> scaled;

};

}

#endif // FIGURE3D_H
