#ifndef GM2_BASIS_BFUNCTION_LOGISTIC_EXPO_RATIONAL_H
#define GM2_BASIS_BFUNCTION_LOGISTIC_EXPO_RATIONAL_H


#include "../bfunction.h"

// stl
#include <tuple>

namespace gm::basis::bfunction {

  namespace ler {


    template <typename Unit_T>
    struct LerBFunctionEpsilon {
      static_assert(sizeof(Unit_T) == 0,
                    "No implementation for LerBFunctionEpsilon of "
                    "unspecialized Unit typename");
    };

    template <>
    struct LerBFunctionEpsilon<float> {
      static constexpr float value {1e-2f};
    };

    template <>
    struct LerBFunctionEpsilon<double> {
      static constexpr double value {1e-3};
    };


    template <typename = void>
    struct BFunctionKernel {

      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& s) const
      {
        constexpr auto epsilon = LerBFunctionEpsilon<Unit_T>::value;
        if (s < epsilon) return 0.0;
        if (1.0 - s < epsilon) return 1.0;
        return 1.0 / (1.0 + std::exp(1.0 / s - 1.0 / (1.0 - s)));
      }
    };

    template <typename = void>
    struct BFunctionD1Kernel {

      template <typename Unit_T>
      constexpr auto operator()(const Unit_T& s) const
      {
        constexpr auto epsilon = LerBFunctionEpsilon<Unit_T>::value;
        if (s < epsilon) return 0.0;
        if (1.0 - s < epsilon) return 0.0;
        return (2 * std::exp(-((1.0) / ((s - 1.0) * s)))
                * (std::pow(s, 2.0) - s + 0.5))
               / (std::pow(std::exp(-(1.0 / (s - 1.0))) + std::exp(1.0 / s),
                           2.0)
                  * std::pow(s - 1.0, 2.0) * std::pow(s, 2.0));
      }
    };

  }   // namespace ler

  using LerBFunction
    = BFunction<ler::BFunctionKernel<>, ler::BFunctionD1Kernel<>>;

}   // namespace gm::basis::bfunction

#endif // GM2_BASIS_BFUNCTION_LOGISTIC_EXPO_RATIONAL_H
