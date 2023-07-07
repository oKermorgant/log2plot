#include <sstream>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <log2plot/logger.h>


namespace log2plot
{

using std::stringstream;
using std::string;
using std::vector;
using std::cout;
using std::endl;

template <typename... Args> inline void UNUSED(Args&&...) {}

// Generic variable
void Logger::writeInitialInfo(const std::string &name, const std::string &legend, const std::string &xlabel, const std::string &ylabel, const bool &keep_file)
{
  nb_fixed_objects = 0;
  last = logged_vars.back().get();

  // corresponding file name
  if(keep_file && file_path !="")
    last->setFile(file_path + name + ".yaml");
  else
  {
    char ctpl[] = "XXXXXX";
    UNUSED(mkstemp(ctpl));	// randomizing name
    last->setFile("/tmp/log2plot-" + name + "." + string(ctpl));
  }

  // add time unit if needed
  if(last->isTimed())
    last->writeInfo("time unit", time_unit);

  // additionnal info
  if(legend != "")
    last->writeInfo("legend", legend);
  if(xlabel != "")
    last->writeInfo("xlabel", xlabel);
  if(ylabel != "")
    last->writeInfo("ylabel", ylabel);
}

std::string Logger::buildLegend(const std::string &legend, const unsigned int len)
{
  if(legend.substr(0,1) == "[")
    return legend;
  std::stringstream ss;
  ss << "[";
  for(unsigned int i=1;i<len;++i)
    ss << "'" << legend << "{" << i << "}', ";
  ss << "'" << legend << "{" << len << "}']";
  return ss.str();
}

// **** Functions to specify metadata for the last registered variable ****
#ifdef LOG2PLOT_WITH_DEPRECATED
void Logger::showMovingCamera(const std::vector<double> &desired_pose, const double &x, const double &y, const double &z)
{
  // init 5x3 matrix
  vector< vector<double> > M(5, vector<double>(3,0));
  M[1][0] = x;    M[1][1] = -y;    M[1][2] = z;
  M[2][0] = -x;   M[2][1] = -y;    M[2][2] = z;
  M[3][0] = -x;   M[3][1] = y;     M[3][2] = z;
  M[4][0] = x;    M[4][1] = y;     M[4][2] = z;
  showMovingObject(M, "[[0,1],[0,2],[0,3],[0,4],[1,2],[2,3],[3,4],[4,1]]", desired_pose);
}

// 3D plot: custom object with a (nx3) matrix
void Logger::showMovingObject(const std::vector<std::vector<double> > &M, const std::string &graph, const std::vector<double> &desired_pose)
{
  assert(desired_pose.size() %3 == 0);
  last->writeInfo("movingObject", "");
  last->writeInfo(Yaml("nodes", M, 1));
  last->writeInfo("    graph", graph);
  // write desired pose if any
  if(desired_pose.size())
    last->writeInfo(Yaml("desiredPose", {desired_pose}, 1));
}

// 3D plot: fixed 2D-rectangle on Z=0
void Logger::showFixedRectangle(const double &xm, const double &ym, const double &xM, const double &yM, const std::string &color, const std::string &legend)
{
  vector< vector<double> > M(4,vector<double>(3,0));
  M[0][0] = xM;   M[0][1] = yM;
  M[1][0] = xM;   M[1][1] = ym;
  M[2][0] = xm;   M[2][1] = ym;
  M[3][0] = xm;   M[3][1] = yM;
  showFixedObject(M, "[[0,1],[1,2],[2,3],[3,0]]", color, legend);
}
#endif


// Updates all saved vectors
void Logger::update(const bool &flush)
{
  // begin the data field
  if(first_update)
  {
    for(auto &c: logged_vars)
      c->init();
    first_update = false;
  }

  // check subsampling
  iter_count++;
  if(subsamp == 1 || !(iter_count % subsamp) || flush)
  {
    // buffer full, call update + flush
    if(++buff_count == buff || flush)
    {
      for(auto &c: logged_vars)
      {
        c->update(time);
        c->flush();
      }
      // reset buffer counter
      buff_count = 0;
    }
    else
    {
      for(auto &c: logged_vars)
        c->update(time);
    }
  }
}


// ***** Plotting functions

// calls python to plot this / these files
void Logger::plotFiles(const std::string &script_path, const vector<std::string> &files, bool verbose, bool display)
{
  string cmdline = "python3 " + script_path;
  string all_files;
  for(const auto &file: files)
  {
    cmdline += " '" + file + "'";
    all_files += file + " ";
  }

  if(!display)
    cmdline += " --nodisplay &";
  else
    cmdline += " &";

  if(verbose)
    cout << "log2plot: plotting "<< all_files << endl;
  UNUSED(system(cmdline.c_str()));
}

void Logger::plot(bool verbose, bool display)
{
  // find source vs install paths of script (lets us easily test devel plot script)
  if(std::filesystem::exists(LOG2PLOT_SCRIPT_SOURCE))
    plot(LOG2PLOT_SCRIPT_SOURCE, verbose, display);
  else
    plot(LOG2PLOT_SCRIPT_INSTALL, verbose, display);
}

// Plot a file
void Logger::plot(const std::string &script_path, bool verbose, bool display)
{
  vector<uint> plotted_group;
  vector<string> data_files;

  // first plot groups
  for(const auto &group: grouped_vars)
  {
    // build list of files
    for(const auto &idx: group)
    {
      plotted_group.push_back(idx);
      data_files.push_back(logged_vars[idx]->close(steps, steps_timed));
    }
    plotFiles(script_path, data_files, verbose, display);
  }

  // plot non-grouped files
  for(uint idx = 0; idx < logged_vars.size(); ++idx)
  {
    if(std::find(plotted_group.begin(),
                 plotted_group.end(),
                 idx) == plotted_group.end())
    {
      plotFiles(script_path,
      {logged_vars[idx]->close(steps, steps_timed)},
                verbose, display);
    }
  }
}

// **** end plotting functions

}
