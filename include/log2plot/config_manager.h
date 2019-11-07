#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <fstream>
#include <iostream>
#include <log2plot/defines.h>

#ifdef WITH_VISP
#include <visp/vpHomogeneousMatrix.h>
#endif

namespace log2plot
{

class ConfigManager
{
public:
  ConfigManager(std::string filename) : config_file(filename)
  {
    config = YAML::LoadFile(filename);
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


  // content of configuration file

  // test if keys exist
  bool has(std::vector<std::string> keys) const
  {
    return has(config, keys.begin(), keys.end());
  }
  bool has(std::initializer_list<std::string> keys) const
  {
    return has(std::vector<std::string>(keys));
  }
  bool has(std::string key) const
  {
    return has({key});
  }


  // main reading function
  // Dummy is to allow partial specialization
  template <class T, class Dummy=int>
  void read(std::vector<std::string> keys, T&val) const
  {
    val = read<T>(config, keys.begin(), keys.end());
  }

  template <class Dummy=int>
  void read(std::vector<std::string> keys, double & v) const
  {
    v = str2double(read<std::string>(keys));
  }

  template <class Dummy=int>
  void read(std::vector<std::string> keys, std::vector<double> & v) const
  {
    const auto vstr = read<std::vector<std::string>>(keys);
    v.clear();
    for(const auto &val: vstr)
      v.push_back(str2double(val));
  }

#ifdef WITH_VISP
  // ViSP core parsing is compiled
  void readViSPArray(std::vector<std::string> keys, vpArray2D<double> & M,
                     const uint cols = 0,
                     const uint rows = 0) const;

  template <class Dummy=int>
  void read(std::vector<std::string> keys, vpColVector & v,
            const uint rows = 0) const
  {
    readViSPArray(keys, v, 1, rows);
  }

  template <class Dummy=int>
  void read(std::vector<std::string> keys, vpTranslationVector& v) const
  {
    readViSPArray(keys, v);
  }

  template <class Dummy=int>
  void read(std::vector<std::string> keys, vpMatrix & M,
            const uint cols = 0,
            const uint rows = 0) const
  {
    readViSPArray(keys, M, cols, rows);
  }

  template <class Dummy=int>
  void read(std::vector<std::string> keys, vpColVector & V) const
  {
    readViSPArray(keys, V);
  }

  template <class Dummy=int>
  void read(std::vector<std::string> keys, vpHomogeneousMatrix &M) const
  {
    // try to read as 6-dim vector
    try
    {
      vpPoseVector p;
      readViSPArray(keys, p);
      M.buildFrom(p);
    } catch (...)
    {
      // read as 4x4 matrix
      readViSPArray(keys, M);
    }
  }

  template <class Dummy=int>
  void read(std::vector<std::string> keys, vpRotationMatrix &R) const
  {
    // try to read as 3-dim vector
    try
    {
      vpThetaUVector tu;
      readViSPArray(keys, tu);
      R.buildFrom(tu);
    } catch (...)
    {
      // read as 3x3 matrix
      readViSPArray(keys, R);
    }
  }

#endif // WITH_VISP


  template <class T>
  T read(std::vector<std::string> keys) const
  {
    T val;
    read<T>(keys, val);
    return val;
  }

  // read from simple string
  template <class T>
  void read(std::string key, T&val) const
  {
    read<T>({key}, val);
  }
  template <class T>
  T read(std::string key) const
  {
    return read<T>({key});
  }

  // read from brace-initializer list
  template <class T>
  void read(std::initializer_list<std::string> keys, T&val) const
  {
    read<T>(std::vector<std::string>(keys), val);
  }
  template <class T>
  T read(std::initializer_list<std::string> keys) const
  {
    return read<T>(std::vector<std::string>(keys));
  }

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

private:
  YAML::Node config;
  std::string config_file;
  std::string base_name;
  std::string base_dir;

  using vsit = std::vector<std::string>::iterator;

  // content of configuration file
  template <class T>
  T read(const YAML::Node &node, vsit begin, vsit end) const
  {
    if(begin == end)
      return {};

    const auto & tag = *begin;
    if(std::next(begin) == end)
      return node[tag].as<T>();

    return read<T>(node[tag], std::next(begin), end);
  }

  bool has(const YAML::Node &node, vsit begin, vsit end) const;

  static double str2double(const std::string &s);
};

}

#endif // CONFIGMANAGER_H
