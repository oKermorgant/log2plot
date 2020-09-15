#ifndef LOG2PLOT_CONFIGMANAGER_H
#define LOG2PLOT_CONFIGMANAGER_H

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <fstream>
#include <iostream>
#include <log2plot/config_manager_detail.h>
#include <log2plot/dir_tools.h>

namespace log2plot
{

class ConfigManager
{
  using TagList = const std::vector<std::string> &;

public:
  ConfigManager(std::string filename) : config_file(filename)
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
  void forceParameter(std::string tag, T value)
  {
    const auto tags = toTags(tag);
    if(has(tags))
      finalNode(tags, config) = value;
  }

  // deal with output file names
  void setDirName(std::string path)
  {
    if(!dirTools::exists(path))
      dirTools::makePath(path);
    base_dir = path + dirTools::sep;
  }
  template <class Numeric>
  void addNameElement(std::string pref, Numeric val)
  {
    std::stringstream ss;
    ss.precision(3);
    ss << pref << val;
    addNameElement(ss.str());
  }

  void addConditionalNameElement(std::string strTrue, bool condition, std::string strFalse = "");

  void addNameElement(std::string str);

  std::string fullName()
  {
    return base_dir + base_name;
  }

  void saveConfig(bool actuallySave = true)
  {
    if(actuallySave)
    {
     /* std::ifstream  src(config_file, std::ios::binary);
      std::ofstream  dst(fullName() + "_config.yaml",   std::ios::binary);
      std::cout << "Saving config to " << fullName() + "_config.yaml\n";
      dst << src.rdbuf();*/
      std::ofstream  dst(fullName() + "_config.yaml",   std::ios::binary);
      dst << YAML::Dump(config);
      dst.close();
    }
  }

  // content of configuration file

  // test if tags exist
  bool has(std::vector<std::string> tags) const
  {
    try {
      finalNode(tags);
      return true;
    } catch (...) {}
    return false;

  }
  bool has(std::initializer_list<std::string> tags) const
  {
    return has(std::vector<std::string>(tags));
  }
  bool has(std::string tag) const
  {
    return has(toTags(tag));
  }

  std::string tagPathMessage(TagList tags,
                             std::string msg="") const;

  // some specializations
  void read(std::vector<std::string> tags, bool & v) const
  {
    const auto vstr = read<std::string>(tags);
    v = (vstr != "0" &&
        vstr != "false" &&
        vstr != "False" &&
        vstr != "0.");
  }

  void read(std::vector<std::string> tags, double & v) const
  {
    v = str2double(read<std::string>(tags));
  }

  void read(std::vector<std::string> tags, std::vector<double> & v) const
  {
    const auto vstr = read<std::vector<std::string>>(tags);
    v.clear();
    for(const auto &val: vstr)
      v.push_back(str2double(val));
  }

  // main reading function from vector of tags
  // prevent access from ViSP class if needed
  template <typename T>
#ifdef LOG2PLOT_WITH_VISP
  typename std::enable_if_t<!detail::vpArrayDerived<T>::value, void>
#else
  void
#endif
  read(std::vector<std::string> tags, T&val) const
  {
    try {
      val = finalNode(tags).as<T>();
    } catch (...) {
      throw std::runtime_error(tagPathMessage(tags, "cannot be cast"));
    }
  }
  template <typename T>
  T read(std::vector<std::string> tags) const
  {
    T val;
    read(tags, val);
    return val;
  }

  // read from simple string
  template <typename T>
  void read(std::string tag, T&val) const
  {
    read(toTags(tag), val);
  }
  template <typename T>
  T read(std::string tag) const
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
  void read(std::vector<std::string> tags, vpArray2D<double> &M,
            uint rows = 0, uint cols = 0) const;
  // special cases for those two
  void read(std::vector<std::string> tags, vpHomogeneousMatrix &M) const;
  void read(std::vector<std::string> tags, vpRotationMatrix &M) const;
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

  std::vector<std::string> toTags(std::string tag) const;

  static double str2double(std::string s);
};

}

#endif // LOG2PLOT_CONFIGMANAGER_H
