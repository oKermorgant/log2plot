#ifndef LOG2PLOT_FIGURE_H
#define LOG2PLOT_FIGURE_H

#include <string>
#include <sstream>

namespace log2plot
{

namespace py
{
void call(const char* c);
void call(std::string s);
void call(const std::ostringstream& ss);
std::string join(double v1, double v2);
}

class Figure
{
public:
  Figure();
  virtual ~Figure();

protected:
  struct Limits
  {
    double min, max;
    bool different(const Limits &prev) const
    {
      return max != prev.max || min != prev.min;
    }
    void set(double val)
    {
      min = max = val;
    }
    void update(double val)
    {
      min = std::min(min, val);
      max = std::max(max, val);
    }
    void update(const Limits &src)
    {
      max = src.max;
      min = src.min;
    }
  };

  Limits xl, xl_prev, yl, yl_prev;
  const std::string ax_tag;
  static int ax_count;
  static int fig_count;
};
}

#endif
