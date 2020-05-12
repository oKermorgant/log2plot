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
    : content(&original), pub(nh.advertise<std_msgs::Float32MultiArray>(name, 10))
  {}

  void init()
  {
    yaml_stream << "data:" << std::endl;

    // also resize message to be published
    msg.data.resize(content->size());
  }


  // log update + publish message
  void update(double *t)
  {
    yaml_stream << "    - [";
    // write time if needed
    if(log_type == LogType::TIME)
      yaml_stream << *t << ", ";

    // write content
    for(uint i=0;i<content->size();++i)
    {
      const double v = (*content)[i];
      msg.data[i] = v;
      yaml_stream << v << ((i == content->size()-1) ? "]\n" : ", ");
    }
    pub.publish(msg);
  }

protected:
  T* content;
  ros::Publisher pub;
  std_msgs::Float32MultiArray msg;
};


}


#endif // LOG2PLOTCONTAINERPUBLISHER_H
