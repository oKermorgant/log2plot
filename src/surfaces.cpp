#include <log2plot/surfaces.h>

using namespace log2plot;

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
