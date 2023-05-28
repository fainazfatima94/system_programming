#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL4_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL4_H

#include "../bits/types.h"
#include "../bits/concepts.h"

namespace dte3607::physengine::solver_dev::level4
{

  template <concepts::SolverFixtureLevel4 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
  }

}   // namespace dte3607::physengine::solver_dev::level4


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL4_H
