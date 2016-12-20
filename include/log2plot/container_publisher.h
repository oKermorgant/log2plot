#ifndef LOG2PLOTCONTAINERPUBLISHER_H
#define LOG2PLOTCONTAINERPUBLISHER_H

#include <std_msgs/Float32MultiArray.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <log2plot/container.h>

namespace log2plot
{

template<class T> class ContainerPublisher : public GenericContainer
{
public:
    // constructor also takes nodehandle and variable ame, used for the published topic
    ContainerPublisher(T &original, ros::NodeHandle &nh, std::string name)
    {
        actual = &original;
        // build publisher
        pub = nh.advertise<std_msgs::Float32MultiArray>(name, 10);
    }

    void init()
    {
        // init in file
        ofs_ << "data:" << std::endl;

        // also resize message to be published
        msg.data.resize(actual->size());
    }


    // log update + publish message
    void update(double *t)
    {
        ofs_ << "    - [";
        // write time if needed
        if(log_type == LogType::TIME)
            ofs_ << *t << ", ";

        // write content
        unsigned int i;
        for(i=0;i<actual->size()-1;++i)
        {
            ofs_ << actual->operator[](i) << ", ";
            msg.data[i] = actual->operator[](i);
        }

        ofs_ << actual->operator[](i) << "]\n";
        msg.data[i] = actual->operator[](i);

        pub.publish(msg);
    }

protected:
    T* actual;
    ros::Publisher pub;
    std_msgs::Float32MultiArray msg;
};


}


#endif // LOG2PLOTCONTAINERPUBLISHER_H
