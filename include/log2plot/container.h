#ifndef LOG2PLOTCONTAINER_H
#define LOG2PLOTCONTAINER_H

#include <log2plot/generic_container.h>

namespace log2plot
{

template<class T> class Container : public GenericContainer
{
public:
    Container(T &original) : actual(&original) {}

    void init() {ofs_ << "data:" << std::endl;}

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
