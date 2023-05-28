#ifndef DTE3607_PHYSENGINE_UTILS_TO_STRING_H
#define DTE3607_PHYSENGINE_UTILS_TO_STRING_H

// GMlib
#include <gmlib/core/gm2_blaze.h>

namespace dte3607::physengine::utils
{

  template <typename ValueType_T, size_t Dim_T>
  std::string toString(gm::VectorT<ValueType_T, Dim_T> const& vec)
  {
    static_assert(Dim_T > 0, "Dim_T must be greater than zero");

    std::string s("[");
    for (auto i = 0ul; i < Dim_T - 1; ++i) s += std::to_string(vec[i]) + ", ";

    s += std::to_string(vec[Dim_T - 1]) + "]";

    return s;
  }

}   // namespace dte3607::physengine::utils

#endif   // DTE3607_PHYSENGINE_UTILS_TO_STRING_H
