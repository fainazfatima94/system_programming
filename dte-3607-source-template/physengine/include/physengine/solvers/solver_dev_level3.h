#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H

#include "../bits/types.h"
#include "../bits/concepts.h"

namespace dte3607::physengine::solver_dev::level3
{

  template <concepts::SolverFixtureLevel3 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
  }

}   // namespace dte3607::physengine::solver_dev::level3


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H
