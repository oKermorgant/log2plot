#ifndef LOG2PLOT_LOG_TYPES_H
#define LOG2PLOT_LOG_TYPES_H

#include <string>

namespace log2plot
{
enum class LogType
{
  ITERATION,
  TIME,
  XY,
  POSE,
  TIMED_XY,
  METAFILE,
  NONE
};

inline std::string getType(LogType type)
{
  switch(type)
  {
    case(LogType::XY):
      return "XY";
    case(LogType::ITERATION):
      return "iteration-based";
    case(LogType::TIME):
      return "time-based";
    case(LogType::TIMED_XY):
      return "timed-XY";
    case(LogType::POSE):
      return "3D pose";
    case(LogType::METAFILE):
      return "metafile";
    case(LogType::NONE):
      return {};
  }
  return {};
}

inline LogType getType(const std::string &s)
{
  // a little margin here, do we all agree it is without consequences? for now at least
  for(auto i = 0; i < 10; ++i)
  {
    try
    {
      const auto type{static_cast<LogType>(i)};
      const auto log{getType(type)};
      if(s == log)
        return type;
    }
    catch (...)
    {
      break;
    }
  }
  return LogType::NONE;
}

}
#endif
