#ifndef GM2_COREUTILS_H
#define GM2_COREUTILS_H


// stl
#include <cstddef>
#include <utility>






/////////////
// Algorithms

namespace gm::utils
{

  namespace detail
  {

    // Static type
    template <typename Value_T>
    constexpr auto getValueHelper(size_t /*idx*/, const Value_T& val)
    {
      return val;
    }

    template <typename Container_T, typename Value_T, size_t... i>
    constexpr auto initStaticContainer(std::index_sequence<i...>,
                                       const Value_T& val)
    {
      return Container_T{{getValueHelper<Value_T>(i, val)...}};
    }

    template <typename ContainerIn_T, typename ContainerOut_T, size_t... i,
              size_t... j, typename Value_T>
    constexpr auto
    extendStaticContainer(const ContainerIn_T& C, std::index_sequence<i...>,
                          std::index_sequence<j...>, const Value_T& val)
    {
      return ContainerOut_T{{C[i]..., getValueHelper<Value_T>(j, val)...}};
    }

  }   // namespace detail


  // Init static container with value
  template <typename Container_T, size_t size, typename Value_T>
  constexpr auto initStaticContainer(const Value_T& val)
  {
    return detail::initStaticContainer<Container_T>(
      std::make_index_sequence<size>{}, val);
  }


  // Extend static container by initializer list expansion
  template <typename ContainerIn_T, typename ContainerOut_T, size_t size,
            size_t ext_size, typename Value_T>
  constexpr auto extendStaticContainer(const ContainerIn_T& C,
                                       const Value_T&       val)
  {
    return detail::extendStaticContainer<ContainerIn_T, ContainerOut_T>(
      C, std::make_index_sequence<size>{}, std::make_index_sequence<ext_size>{},
      val);
  }


  // Visitor pattern overload helpers
  template <typename... Ts>
  struct overloaded : Ts... {
    using Ts::operator()...;
  };

  template <typename... Ts>
  overloaded(Ts...)->overloaded<Ts...>;


}   // namespace gm::utils


#endif   // GM2_COREUTILS_H
