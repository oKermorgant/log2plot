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
  virtual std::string infos() const = 0;
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
  double alpha;
  explicit AlphaShape(double alpha, double color_alpha = 1.);
  virtual std::string infos() const override;
};

struct Reconstructed : public AlphaShape
{
  explicit Reconstructed(double alpha, double color_alpha = 1.);
  inline std::string infos() const override;
};

}

#endif // LOG2PLOT_SURFACES_H
