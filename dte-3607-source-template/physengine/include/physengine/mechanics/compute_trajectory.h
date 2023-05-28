#ifndef DTE3607_PHYSENGINE_MECHANICS_COMPUTE_TRAJECTORY_H
#define DTE3607_PHYSENGINE_MECHANICS_COMPUTE_TRAJECTORY_H


#include "../bits/types.h"
#include "../utils/type_conversion.h"
namespace dte3607::physengine::mechanics
{

  inline std::pair<types::Vector3, types::Vector3> computeLinearTrajectory(
    [[maybe_unused]] types::Vector3 const& velocity,
    [[maybe_unused]] types::Vector3 const& external_forces,
    [[maybe_unused]] types::Duration       timestep)
  {
    auto const F = external_forces;
    auto const dt = utils::toDtScalar(timestep);

    auto const v = velocity;
    auto const a = F * dt;
    auto const ds = v * dt + .5 * a * dt;
      return {ds, a};
  }


}   // namespace dte3607::physengine::mechanics

#endif // DTE3607_PHYSENGINE_MECHANICS_COMPUTE_TRAJECTORY_H
