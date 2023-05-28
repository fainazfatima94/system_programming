#ifndef DTE3607_PHYSENGINE_UTILS_TYPE_CONVERSION_H
#define DTE3607_PHYSENGINE_UTILS_TYPE_CONVERSION_H

#include "../bits/types.h"

namespace dte3607::physengine::utils
{

  types::DtRep        toDtScalar(types::Duration const& timestep);
  inline types::DtRep toDtScalar(types::Duration const& timestep)
  {
    return std::chrono::duration_cast<types::Dt>(timestep).count();
  }

  types::Duration        toDuration(types::Dt const& dt);
  inline types::Duration toDuration(types::Dt const& dt)
  {
    return std::chrono::duration_cast<types::Duration>(dt);
  }

}   // namespace dte3607::physengine::utils

#endif   // DTE3607_PHYSENGINE_UTILS_TYPE_CONVERSION_H
