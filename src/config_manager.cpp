#include <log2plot/config_manager.h>
#include <math.h>

namespace log2plot
{

void ConfigManager::updateFrom(int argc, char **argv, bool warn)
{
  for(int i = 0; i < (argc-1)/2; i++)
  {
    const auto tags = toTags(argv[2*i+1]);
    if(has(tags))
      finalNode(tags, config) = std::string(argv[2*i+2]);
    else if(warn)
      std::cout << "log2plot: Passed argument '" << argv[2*i+1] << "' is not a configuration tag" << std::endl;
  }
}

std::vector<std::string> ConfigManager::toTags(const std::string &tag) const
{
  std::vector<std::string> tags;
  std::string out;
  std::istringstream iss(tag);
  while(std::getline(iss, out, ':'))
    tags.push_back(out);

  return tags;
}

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

// print tag error
std::string ConfigManager::tagPathMessage(TagList tags, std::string msg) const
{
  std::string ret(msg + "\n  - file: " + config_file + "\n  -  tag: ");

  for(size_t idx = 0; idx < tags.size()-1; ++idx)
    ret += tags[idx] + ":";
  return ret + tags.back();
}

YAML::Node ConfigManager::finalNode(TagList tags, const YAML::Node &node, size_t idx) const
{
  const auto& tag = tags[idx];
  const auto &child = node[tag];
  if(!child)
    throw std::runtime_error(tagPathMessage(tags, "tag does not exist"));

  if(idx == tags.size()-1)
    return child;

  return finalNode(tags, child, idx+1);

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
        for(const auto pi: {"PI", "pi"})
        {
          // build string to be searched
          std::stringstream ss;
          if(sign == -1)
            ss << "-";
          ss << pi;
          if(denum != 1)
            ss << "/" << denum;
          if(s == ss.str())
            return sign * M_PI/denum;
        }
      }
    }
  }
  return atof(s.c_str());
}

}


