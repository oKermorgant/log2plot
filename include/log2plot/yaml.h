#ifndef LOG2PLOT_YAML_H
#define LOG2PLOT_YAML_H

#include <string>
#include <vector>
#include <sstream>
#include <log2plot/defines.h>

namespace log2plot
{

struct Yaml
{
  std::string key;
  std::string value;
  size_t offset{};
  explicit Yaml(const std::string &key, const std::string &value, size_t offset = 0)
    : key{key}, value{value}, offset{offset} {}

  /// Builds YAML-proof single-lined matrix
  template<class Coord>
  explicit Yaml(const std::string &key, const std::vector<Coord> &M, size_t offset = 0)
    : key{key}, offset{offset}
  {
    if(M.empty())
      return;
    std::stringstream ss;
    ss << '[';
    for(uint i=0;i<M.size();++i)
    {
      ss << '[';
      uint j{};
      for(j=0;j<M[i].size()-1;++j)
        ss << M[i][j] << ',';
      ss << M[i][j] << "],";
    }
    value = ss.str();
    value.back() = ']';
  }

  /// Builds YAML-proof single-lined matrix
  explicit Yaml(const std::string &key, const std::vector<double> &M, size_t offset)
    : key{key}, offset{offset}
  {
    if(M.empty())
      return;
    std::stringstream ss;
    ss << '[';
    for(uint i=0;i<M.size();++i)
        ss << M[i] << ',';
    value = ss.str();
    value.back() = ']';
  }
};

#ifdef LOG2PLOT_WITH_DEPRECATED
/// legend for a number of points (x_i,y_i)
std::string legend2DPoint(const unsigned int &n=4);

/// legend for a fully connected graph of n points
std::string legendFullyConnected(const uint n);

/// legend for a fully disconnected graph of n points
inline std::string legendFullyDisconnected(const uint n) {return {};}
#endif
}

inline std::ostream & operator<<(std::ostream &oss, const log2plot::Yaml &yaml)
{
  if(!yaml.key.empty() && !yaml.value.empty())
    oss << std::string(4*yaml.offset, ' ') << yaml.key << ": " << yaml.value << '\n';
  return oss;
}

#endif // LOG2PLOT_YAML_H
