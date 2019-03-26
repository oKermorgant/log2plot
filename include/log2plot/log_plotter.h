#ifndef LOG_PLOTTER_H
#define LOG_PLOTTER_H

#include <log2plot/logger.h>
#include <log2plot/container_plotter.h>

namespace log2plot
{

class  LogPlotter : public Logger
{

public:
    LogPlotter(std::string _file_path = "", unsigned int _buffer = 10, unsigned int _subsampling = 1)
      : Logger(_file_path, _buffer, _subsampling) {matplotlibcpp::ion();}

    void setLineType(const std::string lineType)
    {
      last->writeInfo("lineType", lineType);
    }

    // Save iteration-based vector
    template<class T>
    inline void save(T &v, const std::string name, const std::string legend, const std::string ylabel, const bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPlotter<T>>
                              (new ContainerPlotter<T>(v, buildLegend(legend, v.size()))));
        // and write initial info
        writeInitialInfo(log2plot::ITERATION, name, buildLegend(legend, v.size()), "iterations", ylabel, keep_file);
    }

    // Save time-based vector
    template<class T>
    inline void saveTimed(T &v, const std::string name, const std::string legend, const std::string ylabel, const bool keep_file = true)
    {

        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPlotter<T> >
                              (new ContainerPlotter<T>(v, buildLegend(legend, v.size()))));
        // and write initial info
        writeInitialInfo(log2plot::TIME, name, buildLegend(legend, v.size()), "time [" + time_unit + "]", ylabel, keep_file);
    }

    // Save 3D pose or position
    template<class T>
    inline void save3Dpose(T &v, const std::string name, const std::string legend, const bool &invert = false, const bool keep_file = true)
    {
        // add this to logged variables - for now we do not plot 3D curves
        logged_vars.push_back(std::unique_ptr<Container<T> >(new Container<T>(v)));
        // and write initial info
        writeInitialInfo(log2plot::POSE, name, "["+legend+"]", "", "", keep_file);
        if(invert)
            last->writeInfo("invertPose", "True");
    }
};

}


#endif // LOG_PLOTTER_H
