#ifndef GM2_BASIS_BFUNCTION_BFBS_H
#define GM2_BASIS_BFUNCTION_BFBS_H


#include "../bfunction.h"

// stl
#include <tuple>

namespace gm::basis::bfunction {

  namespace bfbs
  {
    template <typename = void>
    struct BFunctionKernel {

      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& s) const
      {
        const auto s2 = s * s;
        const auto s3 = s * s2;
        return 3 * s2 - 2 * s3;
      }
    };

    template <typename = void>
    struct BFunctionD1Kernel {

      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& s) const
      {
        const auto s2 = s * s;
        return 6 * s - 6 * s2;
      }
    };

  }   // namespace bfbs


  using BFBSBFunction
    = BFunction<bfbs::BFunctionKernel<>, bfbs::BFunctionD1Kernel<>>;

}   // namespace gm::basis::bfunction

#endif // GM2_BASIS_BFUNCTION_BFBS_H
