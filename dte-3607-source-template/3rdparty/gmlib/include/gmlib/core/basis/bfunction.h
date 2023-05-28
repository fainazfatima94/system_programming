#ifndef GM2_BASIS_BFUNCTION_H
#define GM2_BASIS_BFUNCTION_H

// stl
#include <tuple>

namespace gm::basis::bfunction
{

  template <typename... dxN_B_Ts>
  struct BFunction {

    template <typename Unit_T, size_t xN_T = 0>
    constexpr Unit_T operator()(const Unit_T& s) const
    {
      static_assert(std::tuple_size<Tuple_dxN_Bs>::value > xN_T,
                    "BFunction derivative index out of bounce.");
      return std::get<xN_T>(dxN_Bs)(s);
    }

    //    template <typename Unit_T, size_t xN_T = 0>
    //    constexpr Unit_T evaluate()(const Unit_T& s) const
    //    {
    //      return operator()<Unit_T, xN_T>(s);
    //    }

  private:
    using Tuple_dxN_Bs = std::tuple<dxN_B_Ts...>;
    static constexpr Tuple_dxN_Bs dxN_Bs{};
  };


}   // namespace gm::basis::bfunction

#endif   // GM2_BASIS_BFUNCTION_H
