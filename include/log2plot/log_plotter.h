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
    inline void save(T &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPlotter<T>>
                              (new ContainerPlotter<T>(v, buildLegend(legend, v.size()))));
        // and write initial info
        writeInitialInfo(log2plot::ITERATION, name, buildLegend(legend, v.size()), "iterations", ylabel, keep_file);
    }

    // Save time-based vector
    template<class T>
    inline void saveTimed(T &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
    {

        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPlotter<T> >
                              (new ContainerPlotter<T>(v, buildLegend(legend, v.size()))));
        // and write initial info
        writeInitialInfo(log2plot::TIME, name, buildLegend(legend, v.size()), "time [" + time_unit + "]", ylabel, keep_file);
    }

    // Save XY vector
    template<class T>
    inline void saveXY(T &v, const std::string &name, const std::string &legend, const std::string &xlabel, const std::string &ylabel, bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPlotter<T> >
                              (new ContainerPlotter<T>(v, buildLegend(legend, v.size()/2))));
        // and write initial info
        writeInitialInfo(log2plot::XY, name, buildLegend(legend, v.size()/2), xlabel, ylabel, keep_file);
    }
};

}


#endif // LOG_PLOTTER_H
