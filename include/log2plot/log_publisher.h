#ifndef LOG2PLOTLOGPUBLISHER_H_
#define LOG2PLOTLOGPUBLISHER_H_

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
    {
        nh = ros::NodeHandle("log2plot");
        Logger(_file_path, _buffer, _subsampling);
    }

    // Save iteration-based vector
    template<class T>
    inline void save(T &v, const std::string name, const std::string legend, const std::string ylabel, const bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::shared_ptr<GenericContainer>(new ContainerPublisher<T>(v, nh, name)));
        // and write initial info
        writeInitialInfo(log2plot::ITERATION, name, buildLegend(legend, v.size()), "iterations", ylabel, keep_file);
    }

    // Save time-based vector
    template<class T>
    inline void saveTimed(T &v, const std::string name, const std::string legend, const std::string ylabel, const bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::shared_ptr<GenericContainer>(new ContainerPublisher<T>(v, nh, name)));
        // and write initial info
        writeInitialInfo(log2plot::TIME, name, buildLegend(legend, v.size()), "time [" + time_unit + "]", ylabel, keep_file);
    }

    // Save 3D pose or position
    template<class T>
    inline void save3Dpose(T &v, const std::string name, const std::string legend, const bool keep_file = true)
    {
        // add this to logged variables
        logged_vars.push_back(std::shared_ptr<GenericContainer>(new ContainerPublisher<T>(v, nh, name)));
        // and write initial info
        writeInitialInfo(log2plot::POSE, name, "["+legend+"]", "", "", keep_file);
    }

};

}

#endif /* LOGPUBLISHER_H_ */
