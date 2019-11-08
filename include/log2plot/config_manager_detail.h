#ifndef CONFIG_MANAGER_DETAIL_H
#define CONFIG_MANAGER_DETAIL_H

#include <log2plot/defines.h>
#ifdef WITH_VISP
#include <visp/vpHomogeneousMatrix.h>
#endif

namespace log2plot
{
namespace detail {

#ifndef WITH_VISP
template <typename T>
class vpArrayDerived
{
public:
  enum {value = true};
};
#else

  // have to use SFINAE to prevent all derived of vpArray from using default method
  // just assume a vpArray2D has ::loadYAML
  template <typename T>
  class vpArrayDerived
  {
  private:
    typedef char YesType[1];
    typedef char NoType[2];

    template <typename C> static YesType& test(decltype(&C::loadYAML));
    template <typename C> static NoType& test(...);

  public:
    enum { value = sizeof(test<T>(nullptr)) == sizeof(YesType) };
  };

#endif // WITH_VISP

} // detail
} // log2plot


#endif // CONFIG_MANAGER_DETAIL_H
