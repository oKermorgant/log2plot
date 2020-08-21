#ifndef DIR_TOOLS_H
#define DIR_TOOLS_H

#include <iostream>
#include <string>
#include <sys/stat.h>
#include <errno.h>    // errno, ENOENT, EEXIST
#if defined(_WIN32)
#include <direct.h>   // _mkdir
#endif

namespace log2plot
{

namespace {
template <typename T = void>
struct dirToolsTmp
{
#if defined(_WIN32)
  static constexpr char sep = '\\';
#else
  static constexpr char sep = '/';
#endif

  static bool fileExists(const std::string &path)
  {
    struct stat info;
    if (stat(path.c_str(), &info) != 0)
    {
      return false;
    }
    return true;
  }

  static bool exists(const std::string& path)
  {
#if defined(_WIN32)
    struct _stat info;
    if (_stat(path.c_str(), &info) != 0)
    {
      return false;
    }
    return (info.st_mode & _S_IFDIR) != 0;
#else
    struct stat info;
    if (stat(path.c_str(), &info) != 0)
    {
      return false;
    }
    return (info.st_mode & S_IFDIR) != 0;
#endif
  }

  static bool makePath(const std::string& path)
  {
#if defined(_WIN32)
    int ret = _mkdir(path.c_str());
#else
    mode_t mode = 0755;
    int ret = mkdir(path.c_str(), mode);
#endif
    if (ret == 0)
      return true;

    switch (errno)
    {
    case ENOENT:
      // parent didn't exist, try to create it
    {
      int pos = path.find_last_of(sep);
      if (pos == std::string::npos)
        return false;
      if (!makePath( path.substr(0, pos)))
        return false;
    }
      // now, try to create again
#if defined(_WIN32)
      return 0 == _mkdir(path.c_str());
#else
      return 0 == mkdir(path.c_str(), mode);
#endif
    case EEXIST:
      // done!
      return exists(path);

    default:
      return false;
    }
  }
};

template <typename T>
constexpr char dirToolsTmp<T>::sep;
}
using dirTools = dirToolsTmp<void>;
}
#endif // DIR_TOOLS_H
