#ifndef LOG2PLOT_SHAPE_H
#define LOG2PLOT_SHAPE_H

#include <string>
#include <vector>
#include <array>
#include <log2plot/yaml.h>

namespace log2plot
{

using Graph = std::vector<std::vector<size_t>>;
using Infos = std::vector<Yaml>;

/// tries to reconstruct a smooth surface from the point cloud
/// works fine for non-closed shapes
struct Surface
{
  double color_alpha;
  explicit  inline Surface(double color_alpha) : color_alpha{color_alpha} {}
  virtual inline Infos infos() const {return yaml("reconstructed");}
protected:
  Infos yaml(const std::string &type,
             const std::vector<double> &params = {}) const;
};

/// default surface when showing a point cloud
struct PointCloud : public Surface
{
  explicit inline PointCloud() : Surface{0} {}
  inline Infos infos() const override {return {};}
};

/// reconstruct the convex hull of the given point cloud
struct ConvexHull : public Surface
{
  explicit  inline ConvexHull(double color_alpha = 1.) : Surface(color_alpha) {}
  inline Infos infos() const override {return yaml("convex_hull");}
};

struct AlphaShape : public Surface
{
  double alpha, clean, decimate;
  /// builds an alpha shape
  /// alpha: alpha-parameter
  /// clean [0-1]: threshold to merge vertices, expressed in percentage of bounding box
  /// decimate [0-1]: proportion of edges to simplify
  explicit inline  AlphaShape(double alpha, double color_alpha = 1., double clean = 0.01, double decimate = -1)
    : Surface(color_alpha), alpha{alpha}, clean{clean}, decimate{decimate} {}
  inline Infos infos() const override {return yaml("alpha_shape", {alpha, decimate, clean});}
};

/// use the object graph to build corresponding faces
struct Faces : public Surface
{
  double decimate;
  explicit  inline Faces(double color_alpha = 1., double decimate = -1) : Surface(color_alpha), decimate{decimate} {}
  inline Infos infos() const override {return yaml("faces", {decimate});}
};

enum class Fully{Connected, Disconnected};

struct Shape
{
  std::vector<std::vector<double>> nodes;
  Graph graph;
  std::string color;
  std::string legend;
  std::vector<double> pose;

  template <class Point = std::vector<double>>
  inline explicit Shape(const std::vector<Point> &nodes, const std::string &color = "", const std::string &legend = "", Fully fully = Fully::Disconnected)
    : Shape(nodes, {}, color, legend)
  {
    if(fully == Fully::Connected)
    {
      const auto n{nodes.size()};
      graph.reserve((n*(n-1)/2));
      for(size_t i = 0; i < n-1; ++i)
      {
        for(size_t j = i+1; j < n; ++j)
          graph.push_back({i,j});
      }
    }
  }

  template <class Point = std::vector<double>>
  inline explicit Shape(const std::vector<Point> &nodes, const Graph &graph,
                        const std::string &color = "", const std::string &legend = "")
    : graph{graph}, color{color.empty() ? "''" : color}, legend{legend}
  {
    if(nodes.empty())
      return;

    this->nodes.reserve(nodes.size());
    const auto dim{nodes[0].size()};
    for(const auto &node: nodes)
    {
      auto &copy{this->nodes.emplace_back(dim)};
      for(uint i = 0; i < dim; ++i)
        copy[i] = node[i];
    }
  }
  Infos infos() const;

  template <class Pose = std::array<double, 6>>
  inline Shape transform(const Pose &wMo, const std::string &color = "", const std::string &legend = "") const
  {
    Shape other{*this};
    if(!color.empty())
      other.color = color;
    if(!legend.empty())
      other.legend = legend;
    other.pose.reserve(wMo.size());
    for(uint i = 0; i < wMo.size(); ++i)
      other.pose.push_back(wMo[i]);
    return other;
  }

  inline Shape scale(double ratio) const
  {
    Shape scaled{*this};
    scaled.scale_pc = std::min(1., std::max(0., ratio));
    return scaled;
  }

protected:
  double scale_pc = -1;
};

  struct Camera : public Shape
  {
    explicit Camera(const std::string &color = "", const std::string &legend = "", double scale = 0.1);
  };

  struct Box : public Shape
  {
    explicit Box(double xm, double ym, double zm,
                 double xM, double yM, double zM,
                 const std::string &color = "", const std::string &legend = "");
  };
  struct Frame : public Shape
  {
    explicit Frame(double scale = 0.1, const std::vector<double> &pose = {});
  };
}


#endif // LOG2PLOT_SHAPE_H
