#ifndef LOG2PLOT_SURFACES_H
#define LOG2PLOT_SURFACES_H

#include <string>
#include <vector>

namespace log2plot
{

struct Surface
{
  double color_alpha;
  explicit Surface(double color_alpha);
  virtual std::string infos() const;
protected:
  virtual std::string yaml(const std::string &type,
                            const std::vector<double> &params = {}) const;
};

struct ConvexHull : public Surface
{
  explicit ConvexHull(double color_alpha = 1.);
  std::string infos() const override;
};

struct AlphaShape : public Surface
{
  double alpha, clean, decimate;
  /// builds an alpha shape
  /// alpha: alpha-parameter
  /// clean [0-1]: threshold to merge vertices, expressed in percentage of bounding box
  /// decimate [0-1]: proportion of edges to simplify
  explicit AlphaShape(double alpha, double color_alpha = 1., double clean = 0.01, double decimate = -1);
  virtual std::string infos() const override;
};

}

#endif // LOG2PLOT_SURFACES_H
