#ifndef LOG2PLOT_FILESYSTEM_H
#define LOG2PLOT_FILESYSTEM_H

#include <filesystem>

namespace log2plot
{

constexpr auto sep{std::filesystem::path::preferred_separator};

inline std::string expand_home_dir(const std::string &path)
{
  const auto index{path.find("~")};
  if(index == std::string::npos)
    return path;

  const auto home{std::getenv("HOME")};
  if(home == nullptr)
    return path;

  auto expanded_path = path;
  expanded_path.replace(index, 1, home);
  return expanded_path;
}

}



#endif // LOG2PLOT_FILESYSTEM_H
