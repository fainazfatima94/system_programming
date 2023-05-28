#ifndef GM2_BASIS_BFUNCTION_LINEAR_H
#define GM2_BASIS_BFUNCTION_LINEAR_H


#include "../bfunction.h"

// stl
#include <tuple>

namespace gm::basis::bfunction
{

  namespace linear
  {
    template <typename = void>
    struct BFunctionKernel {
      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& s) const
      {
        return s;
      }
    };

    template <typename = void>
    struct BFunctionD1Kernel
    {
      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& s) const
      {
        return Unit_T(1);
      }
    };

    template <typename = void>
    struct BFunctionD2Kernel
    {
      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& s) const
      {
        return Unit_T(0);
      }
    };

  }   // namespace linear

  using LinearBFunction
    = BFunction<linear::BFunctionKernel<>, linear::BFunctionD1Kernel<>,
                linear::BFunctionD2Kernel<>>;

}   // namespace gm::basis::bfunction

#endif // GM2_BASIS_BFUNCTION_LINEAR_H
