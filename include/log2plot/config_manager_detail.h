#ifndef CONFIG_MANAGER_DETAIL_H
#define CONFIG_MANAGER_DETAIL_H

#include <log2plot/defines.h>
#ifdef WITH_VISP
#include <visp/vpHomogeneousMatrix.h>
#endif

namespace log2plot
{
namespace detail {

#ifdef WITH_VISP

// have to use SFINAE to prevent all derived of vpArray from using default method
// just assume a vpArray2D has .loadYAML
template <typename T>
struct vpArrayDerived
{
  template <typename C> static constexpr decltype(std::declval<C>().loadYAML, bool()) test(int /* unused */)
  {return true;}
  template <typename C> static constexpr bool test(...) {return false;}
  static constexpr bool value = test<T>(int());
};

#endif // WITH_VISP

} // detail
} // log2plot


#endif // CONFIG_MANAGER_DETAIL_H
