#ifndef LOG2PLOT_SURFACES_H
#define LOG2PLOT_SURFACES_H

#include <string>
#include <vector>

namespace log2plot
{

/// tries to reconstruct a smooth surface from the point cloud
/// works fine for non-closed shapes
struct Surface
{
  double color_alpha;
  explicit  inline Surface(double color_alpha) : color_alpha{color_alpha} {}
  virtual inline std::string infos() const {return yaml("reconstructed");}
protected:
  virtual std::string yaml(const std::string &type,
                            const std::vector<double> &params = {}) const;
};

/// reconstruct the convex hull of the given point cloud
struct ConvexHull : public Surface
{
  explicit  inline ConvexHull(double color_alpha = 1.) : Surface(color_alpha) {}
  inline std::string infos() const override {return yaml("convex_hull");}
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
  inline std::string infos() const override {return yaml("alpha_shape", {alpha, decimate, clean});}
};

/// use the object graph to build corresponding faces
struct Faces : public Surface
{
  double decimate;
  explicit  inline Faces(double color_alpha = 1., double decimate = -1) : Surface(color_alpha), decimate{decimate} {}
  inline std::string infos() const override {return yaml("faces", {decimate});}
};


}

#endif // LOG2PLOT_SURFACES_H
