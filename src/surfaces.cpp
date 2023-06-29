#include <log2plot/surfaces.h>

using namespace log2plot;

Surface::Surface(double color_alpha) : color_alpha{color_alpha} {}
std::string Surface::yaml(const std::string &type, const std::vector<double> &params) const
{
  const std::string offset{"        "};

  auto out{'\n'+offset + "type: " + type + '\n'};
  out += offset + "alpha: " + std::to_string(color_alpha);

  if(params.empty())
    return out;

  out += '\n' + offset + "params: [";
  for(auto p: params)
    out += std::to_string(p) + ',';
  out.back() = ']';

  return out;
}

std::string Surface::infos() const
{
  return yaml("reconstructed", {});
}

ConvexHull::ConvexHull(double color_alpha) : Surface(color_alpha) {}
std::string ConvexHull::infos() const
{
  return yaml("convex_hull");
}

AlphaShape::AlphaShape(double alpha, double color_alpha, double decimate)
  : Surface(color_alpha), alpha{alpha}, decimate{decimate} {}

std::string AlphaShape::infos() const
{
  return yaml("alpha_shape", {alpha, decimate});
}
