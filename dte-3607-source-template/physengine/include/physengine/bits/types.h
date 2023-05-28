#ifndef DTE3607_PHYSENGINE_TYPES_H
#define DTE3607_PHYSENGINE_TYPES_H

// gmlib
#include <gmlib/core/gm2_blaze.h>
#include <gmlib/core/spaceobject.h>

// stl
#include <chrono>
#include <cstdint>

namespace dte3607::physengine::types
{
  // Point and Vector and Frame
  using ValueType  = double;
  using Point3     = gm::VectorT<ValueType, 3>;
  using Point3H    = gm::VectorT<ValueType, 4>;
  using Vector3    = gm::VectorT<ValueType, 3>;
  using Vector3H   = gm::VectorT<ValueType, 4>;
  using Frame3     = gm::SqMatrixT<ValueType, 3>;
  using Frame3H    = gm::SqMatrixT<ValueType, 4>;
  using ControlNet = gm::DMatrixT<Point3>;


  // Clock
  using HighResolutionClock = std::chrono::high_resolution_clock;
  using HighResolutionTP    = std::chrono::time_point<HighResolutionClock>;

  // Integer time types
  using Seconds      = std::chrono::duration<intmax_t>;
  using MilliSeconds = std::chrono::duration<intmax_t, std::milli>;
  using MicroSeconds = std::chrono::duration<intmax_t, std::micro>;
  using NanoSeconds  = std::chrono::duration<intmax_t, std::nano>;

  // Floating point time types
  using SecondsD      = std::chrono::duration<ValueType>;
  using MilliSecondsD = std::chrono::duration<ValueType, std::milli>;
  using MicroSecondsD = std::chrono::duration<ValueType, std::micro>;
  using NanoSecondsD  = std::chrono::duration<ValueType, std::nano>;

  using Duration = NanoSeconds;
  using Dt       = SecondsD;
  using DtRep    = Dt::rep;

  // Spatial objects
  using ProjectiveSpaceObject
    = gm::SpaceObject<gm::spaces::ProjectiveSpace<ValueType, 3>>;

  // Rigid body
  enum class RBMode { NonFixed, Fixed };
  enum class RBState { Free, Resting, Sliding, Rolling };


}   // namespace dte3607::physengine::types

#endif   // DTE3607_PHYSENGINE_TYPES_H
