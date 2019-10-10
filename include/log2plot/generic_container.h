#ifndef GENERIC_CONTAINER_H
#define GENERIC_CONTAINER_H

#include <fstream>
#include <vector>
#include <memory>


namespace log2plot
{
enum LogType
{
    ITERATION,
    TIME,
    XY,
    POSE
};

class GenericContainer
{
public:
    GenericContainer() {}

    void setType(LogType _type) {log_type = _type;}
    LogType type() const {return log_type;}

    // opens file stream
    inline void setFile(std::string s) {filename = s; ofs_.open(s, std::ios_base::trunc);}
    // begin actual data and flush header to file
    virtual void init() = 0;
    // append the stream with current data from actual container
    virtual void update(double *t) = 0;

    // flush buffer into file
    inline void flush() {ofs_.flush();}
    // close file, add potential steps
    std::string close(const std::vector<double> &steps,
                      const std::vector<double> &steps_timed);

    void setSteps(const std::vector<double> &steps);

    // write additionnal info
    virtual void writeInfo(const std::string &_label, const std::string &_info);

protected:
    // stream to given file
    std::ofstream ofs_;
    LogType log_type;
    std::string filename;
    bool has_steps = false;
};
}
#endif // GENERIC_CONTAINER_H
