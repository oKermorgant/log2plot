#ifndef LOG2PLOT_CONTAINER_H
#define LOG2PLOT_CONTAINER_H

#include <log2plot/generic_container.h>

namespace log2plot
{

template<class T> class Container : public GenericContainer
{
public:
    Container(LogType _type, T &original) : GenericContainer(_type), content(&original) {}

    void init() {yaml_stream << "data:" << std::endl;}

    void update(double *t)
    {
        yaml_stream << "    - [";
        // write time if needed
        if(log_type == LogType::TIME)
            yaml_stream << *t << ", ";

        // write content
        unsigned int i;
        for(i=0;i<content->size()-1;++i)
            yaml_stream << (*content)[i] << ", ";
        yaml_stream << (*content)[i] << "]\n";
    }

protected:
    T* content;
};

}

#endif // CONTAINER_H
