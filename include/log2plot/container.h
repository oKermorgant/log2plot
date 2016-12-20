#ifndef LOG2PLOTCONTAINER_H
#define LOG2PLOTCONTAINER_H

#include <fstream>
#include <vector>
#include <memory>

namespace log2plot
{

enum LogType
{
    ITERATION,
    TIME,
    POSE
};

class GenericContainer
{
public:
    GenericContainer() {}
    // final flush and close file
    ~GenericContainer() {ofs_.close();}

    void setType(LogType _type) {log_type = _type;}

    // opens file stream
    inline void setFile(std::string s) {filename = s; ofs_.open(s, std::ios_base::trunc);}
    // begin actual data and flush header to file
    virtual void init() {ofs_ << "data:" << std::endl;}
    // append the stream with current data from actual container
    virtual void update(double *t) = 0;

    // flush buffer into file
    inline void flush() {ofs_.flush();}
    // close file
    inline std::string close() {ofs_.close();return filename;}

    // write additionnal info
    inline void writeInfo(const std::string &_label, const std::string &_info)
    {ofs_ << _label << ": " << _info << '\n';}

protected:
    // stream to given file
    std::ofstream ofs_;
    LogType log_type;
    std::string filename;
};


template<class T> class Container : public GenericContainer
{
public:
    Container(T &original) {actual=&original;}

    void update(double *t)
    {
        ofs_ << "    - [";
        // write time if needed
        if(log_type == LogType::TIME)
            ofs_ << *t << ", ";

        // write content
        unsigned int i;
        for(i=0;i<actual->size()-1;++i)
            ofs_ << actual->operator[](i) << ", ";
        ofs_ << actual->operator[](i) << "]\n";
    }

protected:
    T* actual;
};

}

#endif // CONTAINER_H
