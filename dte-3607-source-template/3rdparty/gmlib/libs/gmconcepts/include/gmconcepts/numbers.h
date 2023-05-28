#ifndef GMC_NUMBERS_H
#define GMC_NUMBERS_H

#include "platform.h"

namespace gmc::numbers
{

  template <typename Value_T>
  concept R = std::floating_point<Value_T>;

  template <typename Value_T>
  concept N = std::signed_integral<Value_T>;

  template <typename Value_T>
  concept Z = std::integral<Value_T>;

}   // namespace gmc::numbers

#endif   // GMC_NUMBERS_H
