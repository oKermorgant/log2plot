#include <Python.h>
#include <log2plot/plot/figure_utils.h>
#include <iostream>
#include <algorithm>
#include <numeric>
namespace log2plot {

namespace py
{

using namespace std;

Interp::Interp() : callers_timing(1, {0, 500})
{
  thread = std::thread([this](){loop();});
}

Interp::~Interp()
{
  run("try:\n  pl.pause(0)\nexcept:\n  pass", true);
  run("stop", true);
  thread.join();
}

bool Interp::shouldProcess(size_t callerID, std::vector<Caller>::const_iterator last) const
{
  return std::find_if(callers_timing.begin(), last,
                      [&](auto &c){return c.id == callerID;})
      != last;
}

void Interp::loop()
{
  Py_Initialize();
  while(true)
  {
    std::string line;
    {
      std::unique_lock<std::mutex> lock(mtx);
      cv.wait(lock, [this](){return commands.size() != 0;});

      auto last(callers_timing.begin()+allowed_callers);
      line = "";
      while(line == "" && commands.size())
      {
        const auto &cmd = commands.front();
        if(shouldProcess(cmd.callerID, last))
          line = cmd.line;
        commands.pop_front();
      }
    }
    cv.notify_one();
    if(line == "stop")
      break;
    PyRun_SimpleString(line.c_str());
  }
  Py_Finalize();
}

void Interp::run(const std::string &line, size_t callerID)
{
  {
    std::lock_guard<std::mutex> lock(mtx);
    commands.push_back({line, callerID});
  }
  cv.notify_all();
}

void Interp::waitExecution()
{
  auto start = chrono::system_clock::now();

  // get what callers we are waiting for
  float expected(0);
  for(size_t c = 0; c < allowed_callers; c++)
    expected += callers_timing[c].ms;

  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this](){return commands.size() == 0;});
  }

  // compare expected wait duration against actual one
  const float actual(chrono::duration_cast<chrono::milliseconds>(std::chrono::system_clock::now() - start).count());
  const float ratio(actual/expected);

  // update estimation of these callers duration
  for(size_t c = 0; c < allowed_callers; c++)
    callers_timing[c].ms = std::max(1.f, callers_timing[c].ms*ratio);

  // go to first not called
  if(allowed_callers != callers_timing.size())
    std::rotate(callers_timing.begin()+1, callers_timing.begin()+allowed_callers, callers_timing.end());

  // find how many can be called next time
  expected = callers_timing[0].ms;
  allowed_callers = 1;
  do
  {
    expected += callers_timing[allowed_callers++].ms;
  } while(allowed_callers < callers_timing.size() && expected < dt);
  if(allowed_callers > 2 && expected > dt)
    allowed_callers--;
  cv.notify_one();
}


std::string join(double v1, double v2)
{
  return std::to_string(v1) + ", " + std::to_string(v2);
}
}

int Figure::ax_count = 0;
std::unique_ptr<py::Interp> Figure::interp;

Figure::Figure() : callerID(newCallerID()), ax_tag("ax_" + std::to_string(ax_count++))
{
  if(ax_count == 1)
  {
    bringUpPython();
    run("import pylab as pl");
    run("from mpl_toolkits.mplot3d import axis3d");
    run("from numpy import nan");
    run("pl.switch_backend('tkagg')");
    // run("pl.ion()");
  }
}

Figure::~Figure()
{
  run("pl.close(" + ax_tag + ".figure)");
}

void Figure::run(const std::ostringstream& ss, bool required)
{
  run(ss.str(), required ? 0 : callerID);
}

void Figure::run(const std::string &line, bool required)
{
  interp->run(line, required ? 0 : callerID);
}

}
