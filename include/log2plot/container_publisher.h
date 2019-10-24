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
  // constructor also takes nodehandle and variable name, used for the published topic
  ContainerPublisher(T &original, ros::NodeHandle &nh, std::string name)
    : actual(&original), pub(nh.advertise<std_msgs::Float32MultiArray>(name, 10))
  {}

  void init()
  {
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
    for(uint i=0;i<actual->size();++i)
    {
      const double v = (*actual)[i];
      msg.data[i] = v;
      ofs_ << v << ((i == actual->size()-1) ? "]\n" : ", ");
    }
    pub.publish(msg);
  }

protected:
  T* actual;
  ros::Publisher pub;
  std_msgs::Float32MultiArray msg;
};


}


#endif // LOG2PLOTCONTAINERPUBLISHER_H
