#ifndef LOG2PLOT_LOGPUBLISHER_H_
#define LOG2PLOT_LOGPUBLISHER_H_

#include <log2plot/logger.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <log2plot/container_publisher.h>
#include <ros/package.h>

namespace log2plot
{

class  LogPublisher : public Logger
{
protected:
    // ROS dependent
    ros::NodeHandle nh;

public:
    LogPublisher(std::string _file_path = "", unsigned int _buffer = 10, unsigned int _subsampling = 1)
      : Logger(_file_path, _buffer, _subsampling), nh("log2plot")
    {}

    // Save iteration-based vector
    template<class T>
    inline void save(T &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPublisher<T> >(new ContainerPublisher<T>(v, nh, name)));
        // and write initial info
        writeInitialInfo(log2plot::ITERATION, name, buildLegend(legend, v.size()), "iterations", ylabel, keep_file);
    }

    // Save time-based vector
    template<class T>
    inline void saveTimed(T &v, const std::string &name, const std::string &legend, const std::string &ylabel, bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPublisher<T> >(new ContainerPublisher<T>(v, nh, name)));
        // and write initial info
        writeInitialInfo(log2plot::TIME, name, buildLegend(legend, v.size()), "time [" + time_unit + "]", ylabel, keep_file);
    }

    // Save XY vector
    template<class T>
    inline void saveXY(T &v, const std::string &name, const std::string &legend, const std::string &xlabel, const std::string &ylabel, bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPublisher<T> >(new ContainerPublisher<T>(v, nh, name)));
        // and write initial info
        writeInitialInfo(log2plot::XY, name, buildLegend(legend, v.size()/2), xlabel, ylabel, keep_file);
    }

    // Save 3D pose or position
    template<class T>
    inline void save3Dpose(T &v, const std::string &name, const std::string &legend, bool invert = false, bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::unique_ptr<ContainerPublisher<T> >(new ContainerPublisher<T>(v, nh, name)));
        // and write initial info
        writeInitialInfo(log2plot::POSE, name, "["+legend+"]", "", "", keep_file);
        if(invert)
            last->writeInfo("invertPose", "True");
    }

};

}

#endif /* LOGPUBLISHER_H_ */
