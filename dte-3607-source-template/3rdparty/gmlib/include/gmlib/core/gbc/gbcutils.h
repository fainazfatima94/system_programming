#ifndef GM2_GBC_GBCUTILS_H
#define GM2_GBC_GBCUTILS_H

#include "../gm2_blaze.h"

namespace gm::gbc
{
//  namespace detail {

//    template <typename Unit_T>
//    inline auto rotateLambda(const DVectorT<Unit_T>& Lambda, int i)
//    {
//      const auto N = Lambda.size();
//      const int  j = (int(N) + i) % int(N);

//      DVectorT<Unit_T> r_Lambda(N);
//      std::rotate_copy(std::begin(Lambda), std::begin(Lambda) + j,
//                       std::end(Lambda), std::begin(r_Lambda));
//      return r_Lambda;
//    }

//  }   // namespace detail


  template <typename Unit_T>
  auto generateLeftRotatedLambdaSet(const DVectorT<Unit_T>& l)
  {
    using Lambda  = DVectorT<Unit_T>;
    using Lambdas = DVectorT<Lambda>;

    auto const N = int(l.size());
    auto       L = Lambdas(size_t(N), Lambda(size_t(N)));
    for (auto L_i = 0; L_i < N; ++L_i)
      std::rotate_copy(std::begin(l), std::begin(l) + L_i, std::end(l),
                       std::begin(L[size_t(L_i)]));
    return L;
  }

  template <typename Unit_T>
  auto generateRightRotatedLambdaSet(const DVectorT<Unit_T>& l)
  {
    using Lambda  = DVectorT<Unit_T>;
    using Lambdas = DVectorT<Lambda>;

    const auto N = l.size();
    auto       L = Lambdas(N, Lambda(N));
    for (auto L_i = 0UL; L_i < N; ++L_i)
      std::rotate_copy(std::begin(l), std::end(l) - L_i, std::end(l),
                       std::begin(L[L_i]));
    return L;
  }

}   // namespace gm::gbc

#endif // GM2_GBC_GBCUTILS_H
