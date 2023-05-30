#ifndef LOG2PLOT_CONFIGMANAGER_H
#define LOG2PLOT_CONFIGMANAGER_H

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <log2plot/dir_tools.h>
#include <log2plot/defines.h>
#ifdef LOG2PLOT_WITH_VISP
#include <visp/vpHomogeneousMatrix.h>
#endif

namespace log2plot
{

class ConfigManager
{
  using TagList = const std::vector<std::string> &;

public:
  explicit ConfigManager(std::string filename) : config_file(filename)
  {
    try {
      config = YAML::LoadFile(filename);
    } catch (...) {
      throw std::runtime_error("log2plot::ConfigManager: cannot load file \n"
                               "  - file: " + config_file);
    }
  }

  void updateFrom(int argc, char**argv, bool warn = false);

  template <typename T>
  void forceParameter(const std::string &tag, T value)
  {
    const auto tags = toTags(tag);
    if(has(tags))
      finalNode(tags, config) = value;
  }

  // deal with output file names
  void setDirName(const std::string &path)
  {
    if(!dirTools::exists(path))
      dirTools::makePath(path);
    base_dir = path;
    if(base_dir.back() != dirTools::sep)
      base_dir += dirTools::sep;
  }

  template <class Numeric>
  void addNameElement(const std::string &pref, Numeric val)
  {
    std::stringstream ss;
    ss.precision(3);
    ss << pref << val;
    addNameElement(ss.str());
  }

  void addConditionalNameElement(const std::string &strTrue, bool condition, const std::string &strFalse = "");

  void addNameElement(const std::string &str);

  inline auto fullName() const
  {
    return base_dir + base_name;
  }
  inline auto baseName() const
  {
    return base_name;
  }
  inline auto dirName() const
  {
    return base_dir;
  }

  void saveConfig()
  {
    std::ofstream  dst(fullName() + "_config.yaml",   std::ios::binary);
    dst << YAML::Dump(config);
    dst.close();
  }

  // content of configuration file

  // test if tags exist
  bool has(TagList tags) const
  {
    try {
      finalNode(tags);
      return true;
    } catch (...) {}
    return false;

  }
  bool has(std::initializer_list<std::string> tags) const
  {
    return has(TagList(tags));
  }
  bool has(const std::string &tag) const
  {
    return has(toTags(tag));
  }

  std::string tagPathMessage(TagList tags,
                             std::string msg="") const;

  // some specializations
  void read(TagList tags, bool & v) const
  {
    const auto vstr = read<std::string>(tags);
    v = (vstr != "0" &&
        vstr != "false" &&
        vstr != "False" &&
        vstr != "0.");
  }

  void read(TagList tags, double & v) const
  {
    v = str2double(read<std::string>(tags));
  }

  void read(TagList tags, std::vector<double> & v) const
  {
    const auto vstr = read<std::vector<std::string>>(tags);
    v.clear();
    v.reserve(vstr.size());
    std::transform(vstr.begin(), vstr.end(),
                   std::back_inserter(v), ConfigManager::str2double);
  }

  // main reading function from vector of tags
  // prevent access from ViSP class if needed
  template <typename T>
#ifdef LOG2PLOT_WITH_VISP
  typename std::enable_if_t<!std::is_base_of<vpArray2D<double>, T>::value, void>
#else
  void
#endif
  read(TagList tags, T&val) const
  {
    try {
      val = finalNode(tags).as<T>();
    } catch (...) {
      throw std::runtime_error(tagPathMessage(tags, "cannot be cast"));
    }
  }
  template <typename T>
  T read(TagList tags) const
  {
    T val;
    read(tags, val);
    return val;
  }

  // read from simple string
  template <typename T>
  void read(const std::string &tag, T&val) const
  {
    read(toTags(tag), val);
  }
  template <typename T>
  T read(const std::string &tag) const
  {
    return read<T>(toTags(tag));
  }

  // read from brace-initializer list
  template <typename T>
  void read(std::initializer_list<std::string> tags, T&val) const
  {
    read(TagList(tags), val);
  }
  template <typename T>
  T read(std::initializer_list<std::string> tags) const
  {
    return read<T>(std::vector<std::string>(tags));
  }

#ifdef LOG2PLOT_WITH_VISP
  // can also read vpArray with passed dimensions
  void read(TagList tags, vpArray2D<double> &M, uint rows, uint cols) const;
  inline void read(TagList tags, vpArray2D<double> &M) const
  {
    read(tags, M, M.getRows(), M.getCols());
  }
  // special cases for those two, may be written in angle-axis
  void read(TagList tags, vpHomogeneousMatrix &M) const;
  void read(TagList tags, vpRotationMatrix &M) const;
#endif

private:
  YAML::Node config;
  std::string config_file;
  std::string base_name;
  std::string base_dir;

  YAML::Node finalNode(TagList tags) const
  {
    return finalNode(tags, config);
  }

  YAML::Node finalNode(TagList tags, const YAML::Node &node,
                       size_t idx = 0) const;

  std::vector<std::string> toTags(const std::string &tag) const;

  static double str2double(const std::string &s);
};

}

#endif // LOG2PLOT_CONFIGMANAGER_H
