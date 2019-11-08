#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <fstream>
#include <iostream>
#include <log2plot/config_manager_detail.h>

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

  // deal with output file names
  void setDirName(std::string s) {base_dir = s + "/";}
  template <class Numeric>
  void addNameElement(std::string pref, Numeric val)
  {
    std::stringstream ss;
    ss.precision(3);
    ss << pref << val;
    addNameElement(ss.str());
  }

  void addConditionalNameElement(std::string strTrue, bool condition, std::string strFalse);

  void addNameElement(std::string str);

  std::string fullName()
  {
    return base_dir + base_name;
  }

  void saveConfig(bool actuallySave = true)
  {
    if(actuallySave)
    {
      std::ifstream  src(config_file, std::ios::binary);
      std::ofstream  dst(fullName() + "_config.yaml",   std::ios::binary);
      std::cout << "Saving config to " << fullName() + "_config.yaml\n";
      dst << src.rdbuf();
    }
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
    return has(std::vector<std::string>(tags));
  }
  bool has(std::string tag) const
  {
    return has({tag});
  }

  std::string tagPathMessage(TagList tags,
                             std::string msg="") const;

  // some specializations to read doubles
  void read(TagList tags, double & v) const
  {
    v = str2double(read<std::string>(tags));
  }

  void read(TagList tags, std::vector<double> & v) const
  {
    const auto vstr = read<std::vector<std::string>>(tags);
    v.clear();
    for(const auto &val: vstr)
      v.push_back(str2double(val));
  }

  // main reading function from vector of tags
  // prevent access from ViSP class if needed
  template <typename T>
#ifdef WITH_VISP
  typename std::enable_if<!detail::vpArrayDerived<T>::value, void>::type
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
  void read(std::string tag, T&val) const
  {
    read({tag}, val);
  }
  template <typename T>
  T read(std::string tag) const
  {
    return read<T>({tag});
  }

  // read from brace-initializer list
  template <typename T>
  void read(std::initializer_list<std::string> tags, T&val) const
  {
    read(std::vector<std::string>(tags), val);
  }
  template <typename T>
  T read(std::initializer_list<std::string> tags) const
  {
    return read<T>(std::vector<std::string>(tags));
  }

#ifdef WITH_VISP
  // can also read vpArray with passed dimensions
  void read(TagList tags, vpArray2D<double> &M,
            uint rows = 0, uint cols = 0) const;
  // special cases for those two
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

  static double str2double(std::string s);
};

}

#endif // CONFIGMANAGER_H
