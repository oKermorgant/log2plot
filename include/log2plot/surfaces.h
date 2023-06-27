#ifndef LOG2PLOT_SURFACES_H
#define LOG2PLOT_SURFACES_H

#include <string>
#include <vector>

namespace log2plot
{

struct Surface
{
  double color_alpha;
  std::string point_color;
  explicit Surface(double color_alpha, const std::string &point_color)
    : color_alpha{color_alpha}, point_color{point_color} {}
  virtual std::string infos() const = 0;
protected:
  inline virtual std::string infos_full(const std::vector<double> &params) const
  {
    std::string out{'[' + std::to_string(color_alpha)
          + ",'" + point_color + "',"};

    for(auto p: params)
      out += std::to_string(p) + ',';
    out.back() = ']';

    return out;
  }
};

struct ConvexHull : public Surface
{
  explicit ConvexHull(double color_alpha = 1., const std::string &point_color = "")
    : Surface(color_alpha, point_color) {}
  inline std::string infos() const override
  {
    return infos_full({-1});
  }
};

struct AlphaShape : public Surface
{
  double alpha;
  double edge_min;
  explicit AlphaShape(double alpha, double color_alpha = 1., const std::string &point_color = "",
                      double edge_min = -1)
    : Surface(color_alpha, point_color), alpha{alpha}, edge_min{edge_min}
  {}
  inline std::string infos() const override
  {
    return infos_full({alpha, edge_min});
  }
};

}

#endif // LOG2PLOT_SURFACES_H
