#include <sstream>
#include <fstream>
#include <log2plot/logger.h>
#include <algorithm>


namespace log2plot
{

using std::stringstream;
using std::string;
using std::vector;
using std::cout;
using std::endl;

template <typename... Args> inline void UNUSED(Args&&...) {}

// Generic variable
void Logger::writeInitialInfo(const LogType &log_type, const std::string &name, const std::string &legend, const std::string &xlabel, const std::string &ylabel, const bool &keep_file)
{
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

  // set log type in container + in file
  last->setType(log_type);
  switch(log_type)
  {
  case(LogType::XY):
    last->writeInfo("dataType", "XY");
    break;
  case(LogType::ITERATION):
    last->writeInfo("dataType", "iteration-based");
    break;
  case(LogType::TIME):
    last->writeInfo("dataType", "time-based");
    last->writeInfo("time unit", time_unit);
    break;
  default:
    last->writeInfo("dataType", "3D pose");
  }

  // additionnal info
  if(legend != "")
    last->writeInfo("legend", legend);
  if(xlabel != "")
    last->writeInfo("xlabel", xlabel);
  if(ylabel != "")
    last->writeInfo("ylabel", ylabel);
}

std::string Logger::buildLegend(const std::string legend, const unsigned int len)
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

// 3D plot: show moving camera
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

// 3D plot: show moving box
void Logger::showMovingBox(const double &x, const double &y, const double &z, const std::vector<double> &desired_pose)
{
  UNUSED(desired_pose);
  showMovingObject(buildBoxNodes(-x/2,-y/2,-z/2,x,y,z),
                   "[[0,1],[1,2],[2,3],[3,0],[0,4],[1,5],[2,6],[3,7],[4,5],[5,6],[6,7],[7,4]]");
}


// 3D plot: custom object with a (nx3) matrix
void Logger::showMovingObject(const std::vector<std::vector<double> > &M, const std::string &graph, const std::vector<double> &desired_pose)
{
  last->writeInfo("movingObject", "");
  last->writeInfo("    nodes", toYAMLVector(M));
  last->writeInfo("    graph", graph);
  // write desired pose if any
  if(desired_pose.size())
    last->writeInfo("    desiredPose", toYAMLVector(std::vector<std::vector<double> >(1, desired_pose)));
}

// 3D plot: fixed 3D-rectangle
void Logger::showFixedBox(const double &xm, const double &ym, const double &zm, const double &xM, const double &yM, const double &zM, const std::string &color)
{
  showFixedObject(buildBoxNodes(xm, ym, zm, xM, yM, zM), "[[0,1],[1,2],[2,3],[3,0],[0,4],[1,5],[2,6],[3,7],[4,5],[5,6],[6,7],[7,4]]", color);
}

// 3D plot: fixed 2D-rectangle on Z=0
void Logger::showFixedRectangle(const double &xm, const double &ym, const double &xM, const double &yM, const std::string &color)
{
  vector< vector<double> > M(4,vector<double>(3,0));
  M[0][0] = xM;   M[0][1] = yM;
  M[1][0] = xM;   M[1][1] = ym;
  M[2][0] = xm;   M[2][1] = ym;
  M[3][0] = xm;   M[3][1] = yM;
  showFixedObject(M, "[[0,1],[1,2],[2,3],[3,0]]", color);
}

// 3D plot: fixed object (related to object frame)
void Logger::showFixedObject(const std::vector<std::vector<double> > &M, const std::string &graph, const std::string &color)
{
  std::stringstream ss;
  ss << "fixedObject" << ++nb_fixed_objects;
  last->writeInfo(ss.str(), "");
  last->writeInfo("    nodes", toYAMLVector(M));
  last->writeInfo("    graph", graph);
  if(color!="")
    last->writeInfo("    color", color);
}



// 3D plot: build 3D box nodes
vector< vector<double> > Logger::buildBoxNodes(const double &xm, const double &ym, const double &zm, const double &xM, const double &yM, const double &zM)
{
  vector< vector<double> > M(8,vector<double>(3,0));
  M[0][0] = xm;    M[0][1] = ym;   M[0][2] = zm;
  M[1][0] = xM;     M[1][1] = ym;   M[1][2] = zm;
  M[2][0] = xM;     M[2][1] = yM;    M[2][2] = zm;
  M[3][0] = xm;    M[3][1] = yM;    M[3][2] = zm;
  M[4][0] = xm;    M[4][1] = ym;   M[4][2] = zM;
  M[5][0] = xM;     M[5][1] = ym;   M[5][2] = zM;
  M[6][0] = xM;     M[6][1] = yM;    M[6][2] = zM;
  M[7][0] = xm;    M[7][1] = yM;    M[7][2] = zM;
  return M;
}


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
void Logger::plotFiles(const std::string &script_path, const std::string &files, bool verbose, bool display)
{
  string cmdline = "python " + script_path + " " + files;
  if(!display)
    cmdline += " --nodisplay &";
  else
    cmdline += " &";

  if(verbose)
    cout << "executing "<< cmdline << endl;
  UNUSED(system(cmdline.c_str()));
}

void Logger::plot(bool verbose, bool display)
{
  plot(LOG2PLOT_SCRIPT_PATH, verbose, display);
}

// Plot a file
void Logger::plot(const std::string &script_path, bool verbose, bool display)
{
  std::vector<uint> plotted_group;
  // first plot groups
  for(const auto &group: grouped_vars)
  {
    // build list of files
    std::stringstream ss;
    for(const auto &idx: group)
    {
      plotted_group.push_back(idx);
      ss << logged_vars[idx]->close(steps, steps_timed) << " ";
    }
    plotFiles(script_path, ss.str(), verbose, display);
  }

  // plot non-grouped files
  for(uint idx = 0; idx < logged_vars.size(); ++idx)
  {
    if(std::find(plotted_group.begin(),
                 plotted_group.end(),
                 idx) == plotted_group.end())
    {
      plotFiles(script_path,
                logged_vars[idx]->close(steps, steps_timed),
                verbose, display);
    }
  }
}

// **** end plotting functions

// **** Related to legends ****

string legend2DPoint(const unsigned int &n)
{
  stringstream ss;
  ss << "[";
  unsigned int i;
  for(i=0;i<n-1;++i)
    ss << "x_" << i+1 << ", y_" << i+1 << ", ";
  ss << "x_" << i+1 << ", y_" << i+1 << "]";
  return ss.str();
}

// legend for a fully connected graph of n points
std::string legendFullyConnected(const uint n)
{
  std::vector<std::vector<uint>> M;
  for(uint i =  0; i < n-1; ++i)
  {
    for(uint j = i+1; j < n; ++j)
      M.push_back({i,j});
  }
  return toYAMLVector(M);
}

// legend for a fully disconnected graph of n points
std::string legendFullyDisconnected(const uint n)
{
  std::vector<std::vector<uint>> M;
  for(uint i = 0; i < n; ++i)
    M.push_back({i,i});
  return toYAMLVector(M);
}
}
