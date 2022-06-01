#ifndef LOG2PLOT_FIGURE_H
#define LOG2PLOT_FIGURE_H

#include <string>
#include <sstream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <Python.h>

namespace log2plot
{
namespace py
{
std::string join(double v1, double v2);

struct Command
{
  const std::string line;
  size_t callerID;
};

struct Caller
{
  size_t id;
  float ms;
  // rough estimate that a caller category takes
  Caller(size_t _id, float _ms=10) : id(_id),ms(_ms) {}
};

class Interp
{
public:
  Interp();
  ~Interp();

  size_t newCallerID()
  {
    callers_timing.insert(callers_timing.begin()+1, {allowed_callers++});
    return allowed_callers-1;
  }

  void setTargetDuration(float _dt)
  {
    dt = 1000*_dt;
  }

  void run(const std::string &line, size_t callerID = 0);
  void waitExecution();

protected:

  void loop();

  bool shouldProcess(size_t callerID, std::vector<Caller>::const_iterator last) const;

  float dt = 50;  // ms
  std::vector<Caller> callers_timing;
  size_t allowed_callers = 1;

  std::thread thread;
  std::mutex mtx;
  std::condition_variable cv;
  std::deque<Command> commands;
};
}

class Figure
{
public:

  Figure();
  virtual ~Figure();

  static void bringUpPython()
  {
    if(!interp)
      interp = std::make_unique<py::Interp>();
  }

  static void setTargetSampling(float _dt)
  {
    bringUpPython();
    interp->setTargetDuration(_dt);
  }

  static size_t newCallerID()
  {
    return interp->newCallerID();
  }

  void run(const std::ostringstream& ss, bool required = true);
  void run(const std::string &line, bool required = true);
  void waitExecution()
  {
    run("pl.draw();");
    // run("pl.pause(0.0001)");
    interp->waitExecution();
  }

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

  static std::unique_ptr<py::Interp> interp;
  static int ax_count;
  const size_t callerID;
  const std::string ax_tag;
  Limits xl, xl_prev, yl, yl_prev;

};
}

#endif
