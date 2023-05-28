#ifndef GM2_DIVIDEDDIFFERENCE_FINITEDIFFERENCE_H
#define GM2_DIVIDEDDIFFERENCE_FINITEDIFFERENCE_H

#include "../gm2_blaze.h"

namespace gm::divideddifference::fd
{
  namespace kernel
  {

    template <typename = void>
    struct ForwardDifferenceKernel {

      template <typename Point_T, typename Vector_T, typename Fn_T>
      constexpr auto operator()(Point_T&& t, Vector_T&& dt, Fn_T fn) const
      {
        return std::pair(
          fn(std::forward<Point_T>(t) + std::forward<Vector_T>(dt)),
          fn(std::forward<Point_T>(t)));
      }
    };

    template <typename = void>
    struct BackwardDifferenceKernel {

      template <typename Point_T, typename Vector_T, typename Fn_T>
      constexpr auto operator()(Point_T&& t, Vector_T&& dt, Fn_T fn) const
      {
        return std::pair(
          fn(std::forward<Point_T>(t)),
          fn(std::forward<Point_T>(t) - std::forward<Vector_T>(dt)));
      }
    };

    template <typename = void>
    struct CentralDifferenceKernel {

      template <typename Point_T, typename Vector_T, typename Fn_T>
      constexpr auto operator()(Point_T&& t, Vector_T&& dt, Fn_T fn) const
      {
        using EType = typename std::decay_t<Vector_T>::ElementType;
        return std::pair(fn(std::forward<Point_T>(t)
                            + EType(0.5) * std::forward<Vector_T>(dt)),
                         fn(std::forward<Point_T>(t)
                            - EType(0.5) * std::forward<Vector_T>(dt)));
      }
    };

  }   // namespace kernel



  template <typename FDKernelOperator_T>
  struct FiniteDifference {

    template <typename Point_T, typename Vector_T, typename Fn_T>
    auto operator()(Point_T&& t, Vector_T&& dt, Fn_T&& fn) const
    {
      const auto [a, b]
        = op(std::forward<Point_T>(t), std::forward<Vector_T>(dt),
             std::forward<Fn_T>(fn));
      return blaze::evaluate((a - b) / blaze::l2Norm(dt));
    }

  private:
    static constexpr FDKernelOperator_T op{};
  };



  using ForwardDifference = FiniteDifference<kernel::ForwardDifferenceKernel<>>;
  using BackwardDifference
    = FiniteDifference<kernel::BackwardDifferenceKernel<>>;
  using CentralDifference = FiniteDifference<kernel::CentralDifferenceKernel<>>;


}   // namespace gm::divideddifference::fd

#endif   // GM2_DIVIDEDDIFFERENCE_FINITEDIFFERENCE_H
