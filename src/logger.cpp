#include <sstream>
#include <fstream>
#include <log2plot/logger.h>


namespace log2plot
{

using std::stringstream;
using std::string;
using std::vector;
using std::cout;
using std::endl;

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
        mkstemp(ctpl) == 0;	// randomizing name
        last->setFile("/tmp/log2plot-" + name + "." + string(ctpl));
    }

    // set log type in container + in file
    last->setType(log_type);
    switch(log_type)
    {
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
    for(unsigned int i=0;i<len-1;++i)
        ss << legend << i << ", ";
    ss << legend << len-1 << "]";
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
    last->writeInfo("fixedObject", "");
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
    if(++iter_count == subsamp || flush)
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
        // reset counter for subsample
        iter_count = 0;
    }
}


// writes a matrix into a single line YAML format
string Logger::toYAMLVector(const std::vector<std::vector<double> > &M)
{
    stringstream ss;
    unsigned int i,j;
    ss << "[";
    for(i=0;i<M.size();++i)
    {
        ss << "[";
        for(j=0;j<M[i].size()-1;++j)
            ss << M[i][j] << ", ";
        ss << M[i][j] << "]";
        if(i != M.size()-1)
            ss << ",";
    }
    ss << "]";
    return ss.str();
}


// ***** Plotting functions

// Plot a file
void Logger::plot(std::string script_path, bool verbose)
{
    if(script_path == "")
        script_path = LOG2PLOT_SCRIPT_PATH;

    for(auto &g: logged_vars)
    {
        // close the corresponding file and call Python to plot it
        string cmdline = "python " + script_path + " " + g->close() + " &";
        if(verbose)
            cout << "executing "<< cmdline << endl;
        system(cmdline.c_str()) == 0;
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


}
