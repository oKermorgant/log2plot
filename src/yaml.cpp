#include <log2plot/yaml.h>

#ifdef LOG2PLOT_WITH_DEPRECATED
namespace log2plot
{

// **** Legends helpers ****

std::string legend2DPoint(const unsigned int &n)
{
  std::stringstream ss;
  ss << "[";
  unsigned int i;
  for(i=0;i<n-1;++i)
    ss << "x_" << i+1 << ", y_" << i+1 << ", ";
  ss << "x_" << i+1 << ", y_" << i+1 << "]";
  return ss.str();
}

// legend for a fully connected graph of n points
std::string legendFullyConnected(const uint n)
{
  std::vector<std::vector<uint>> M;
  for(uint i =  0; i < n-1; ++i)
  {
    for(uint j = i+1; j < n; ++j)
      M.push_back({i,j});
  }
  return Yaml("", M, 0).value;
}

}
#endif
