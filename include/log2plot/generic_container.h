#ifndef LOG2PLOT_GENERIC_CONTAINER_H
#define LOG2PLOT_GENERIC_CONTAINER_H

#include <fstream>
#include <vector>
#include <memory>
#include <log2plot/log_types.h>
#include <log2plot/yaml.h>

namespace log2plot
{

class GenericContainer
{
public:
    GenericContainer(LogType _type) : log_type(_type) {}

    inline LogType type() const {return log_type;}
    inline bool isTimed() const {return log_type == LogType::TIME || log_type == LogType::TIMED_XY;}

    // opens file stream and write plot type
    void setFile(const std::string &file);
    // begin actual data and flush header to file
    virtual void init() = 0;
    // append the stream with current data from actual container
    virtual void update(double *t) = 0;

    // flush buffer into file
    inline void flush() {yaml_stream.flush();}
    // close file, add potential steps
    std::string close(const std::vector<double> &steps,
                      const std::vector<double> &steps_timed);

    void setSteps(const std::vector<double> &steps);

    // write additionnal info
    virtual void writeInfo(const std::string &key, const std::string &value = "");

    inline void writeInfo(const Yaml &yaml)
    {
      yaml_stream << yaml;
    }
    inline void writeInfo(const std::vector<Yaml> &yaml)
    {
      for(auto &info: yaml)
      yaml_stream << info;
    }


protected:
    std::ofstream yaml_stream;
    const LogType log_type;
    std::string filename;
    bool has_steps = false;
};
}

#endif // GENERIC_CONTAINER_H
