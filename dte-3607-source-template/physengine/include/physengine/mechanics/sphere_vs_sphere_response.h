#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H

#include "../bits/types.h"

// stl
#include <utility>

namespace dte3607::physengine::mechanics
{

  inline
    std::pair<types::Vector3, types::Vector3>
    computeImpactResponseSphereSphere(
      [[maybe_unused]] types::Point3 const&  s1_p, //p0
      [[maybe_unused]] types::Vector3 const& s1_v, //v0
      [[maybe_unused]] types::ValueType      s1_mass, //m0
      [[maybe_unused]] types::Point3 const&  s2_p,    //p1
      [[maybe_unused]] types::Vector3 const& s2_v,    //v1
      [[maybe_unused]] types::ValueType      s2_mass) //m1
  {
      auto const d = (s2_p - s1_p) / blaze::length(s2_p - s1_p);
      auto const v0d = blaze::inner (s1_v, d);
      auto const v1d = blaze::inner(s2_v, d);

      auto const v0n = s1_v - v0d * d;
      auto const v1n = s2_v - v1d * d;

      auto const new_v0d = ((s1_mass - s2_mass)/(s1_mass + s2_mass)) * v0d + ((2*s2_mass)/(s1_mass+s2_mass))*v1d;
      auto const new_v1d = ((s2_mass - s1_mass)/(s1_mass + s2_mass))*v1d + ((2*s1_mass)/(s1_mass+s2_mass))*v0d;

      auto const new_v0 = new_v0d*d + v0n;
      auto const new_v1 = new_v1d*d + v1n;

      return {new_v0, new_v1};
  }


}   // namespace dte3607::physengine::mechanics


#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
