#ifndef GM2_BASIS_BFUNCTION_FABIUS_H
#define GM2_BASIS_BFUNCTION_FABIUS_H

#include "../bfunction.h"



namespace gm::basis::bfunction
{


  /**!
   * cite:
   *   "Blending functions based on
   *    trigonometric and polynomial approsimations
   *    of the Fabius function",
   *   Hans Olofsen,
   *   NiK, 2019
   */

  namespace fabius
  {

    namespace eq5
    {

      template <typename = void>
      struct BFunctionKernel {

        template <typename Unit_T>
        constexpr auto operator()(const Unit_T& s) const
        {
          return 0.5 - 0.5 * std::cos(M_PI * s);
        }
      };

      template <typename = void>
      struct BFunctionD1Kernel {

        template <typename Unit_T>
        constexpr auto operator()(const Unit_T& s) const
        {
          return M_PI * 0.5 * std::sin(M_PI * s);
        }
      };

    }   // namespace eq5

    namespace eq8
    {

      template <typename = void>
      struct BFunctionKernel {

        template <typename Unit_T>
        constexpr auto operator()(const Unit_T& s) const
        {
          return 0.5
                 - ((9. / 16. * std::cos(M_PI * s))
                    - (1. / 16. * std::cos(3. * M_PI * s)));
        }
      };

    }   // namespace eq8

    namespace eq21
    {

      template <typename = void>
      struct BFunctionKernel {

        template <typename Unit_T>
        constexpr auto operator()(const Unit_T& s) const
        {
          return s - (std::sin(2 * M_PI * s) / (2 * M_PI));
        }
      };

      template <typename = void>
      struct BFunctionD1Kernel {

        template <typename Unit_T>
        constexpr auto operator()(const Unit_T& s) const
        {
          return 1 - s * std::cos(M_PI * std::pow(s, 2));
        }
      };

    }   // namespace eq21

  }   // namespace fabius

  using FabiusEq5BFunction = BFunction<fabius::eq5::BFunctionKernel<>,
                                       fabius::eq5::BFunctionD1Kernel<>>;

  using FabiusEq8BFunction = BFunction<fabius::eq8::BFunctionKernel<>>;

  using FabiusEq21BFunction = BFunction<fabius::eq21::BFunctionKernel<>,
                                        fabius::eq21::BFunctionD1Kernel<>>;

  using FabiusBFunction = FabiusEq21BFunction;


}   // namespace gm::basis::bfunction

#endif   // GM2_BASIS_BFUNCTION_FABIUS_H
