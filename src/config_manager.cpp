
#include <log2plot/config_manager.h>

namespace log2plot
{

void ConfigManager::addConditionalNameElement(std::string strTrue, bool condition, std::string strFalse)
{
  if(condition)
    addNameElement(strTrue);
  else
    addNameElement(strFalse);
}

void ConfigManager::addNameElement(std::string str)
{
  if(str.size())
  {
    if(base_name.size())
      base_name += "_";
    base_name += str;
  }
}


bool ConfigManager::has(const YAML::Node &node, vsit begin, vsit end) const
{
  if(begin == end)
    return false;
  const auto & tag = *begin;

  if (node[tag].IsNull())
    return false;

  if(std::next(begin) == end)
    return !node[tag].IsNull();

  return has(node[tag], std::next(begin), end);
}

double ConfigManager::str2double(const std::string &s)
{
  if(s.size() > 1)
  {
    // look for fractions of pi
    for(const auto denum: {2, 3, 4, 6, 1})
    {
      for(const auto sign: {1, -1})
      {
        // build string to be searched
        std::stringstream ss;
        if(sign == -1)
          ss << "-";
        ss << "PI";
        if(denum != 1)
          ss << "/" << denum;
        if(s == ss.str())
          return sign * M_PI/denum;
      }
    }
  }

  return atof(s.c_str());
}

#ifdef WITH_VISP
void ConfigManager::readViSPArray(std::vector<std::string> keys, vpArray2D<double> &M,
                                  const uint cols, const uint rows) const
{
  std::vector<std::string> content;
  // try to read as 1-dim
  try
  {
    read(keys, content);
  }
  catch(...)  {}

  // or read as 2-dim
  if(content.size() == 0)
  {
    std::vector<std::vector<std::string>> content_serial;
    read(keys, content_serial);
    for(const auto &row: content_serial)
      std::copy(row.begin(), row.end(), std::back_inserter(content));
  }

  // resize M to correct dimension
  if(cols != 0 && rows != 0)
    M.resize(rows, cols);

  if(content.size() != M.getRows() * M.getCols())
    throw(vpException::dimensionError);

  // and copy
  uint idx = 0;
  for(uint row = 0; row< M.getRows(); ++row)
  {
    for(uint col = 0; col < M.getCols(); ++col)
      M[row][col] = str2double(content[idx++]);
  }
}
#endif

}


