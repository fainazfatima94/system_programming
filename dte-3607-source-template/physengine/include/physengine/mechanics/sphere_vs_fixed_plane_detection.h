#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "compute_trajectory.h"

// stl
#include <optional>

namespace  dte3607::physengine::mechanics {
namespace implementation {
inline std::optional<types::HighResolutionTP> computeIntersectionSpFp(
        types::HighResolutionTP const&        sphere_tc,
        types::Point3 const&                  sp_p,
        types::ValueType                      sp_r,
        types::Vector3 const&                 sp_ds,
        types::Point3 const&                  fp_q,
        types::Vector3 const&                 fp_n,
        types::HighResolutionTP const&        t_0,
        types::Duration                       timestep){
    auto const epsilon = 1e-5;
    auto const d = (fp_q + sp_r * fp_n) - sp_p;

    auto const ds_scalar = utils::toDtScalar (timestep - (sphere_tc - t_0))/
            utils::toDtScalar (timestep);

    //Example inner product between ds and planes normal
    auto const ds_n = blaze::inner (sp_ds*ds_scalar, fp_n);
    auto const d_n = blaze::inner (d, fp_n);

    //If sphere is parallel to the plane, no collision
    if(std::abs(ds_n) < epsilon)
        return std::nullopt;

    //If sphere is touching the plane, no collision
    if(std::abs(d_n) < epsilon)
        return std::nullopt;

    auto const x = d_n / ds_n;

    //No collision if x is out of bound
    if(x<=0 or x>1)
        return std::nullopt;

    //The duration scaled using x
    auto const dt_scaled = (timestep * ds_scalar) * x;

    //Return scalar timepoint
    auto const tp = types::HighResolutionTP(sphere_tc + std::chrono::duration_cast<types::Duration>(dt_scaled));
    return tp;
   }
}

inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
        types::HighResolutionTP const&             sphere_tc,
        types::Point3 const&                       sphere_p,
        types::ValueType                           sphere_r,
        types::Vector3 const&                      sphere_v,
        types::Point3 const&                       fplane_q,
        types::Vector3 const&                      fplane_n,
        types::Vector3 const&                      external_forces,
        types::HighResolutionTP const&             t_0,
        types::Duration                            timestep){
    auto const [ds, a] = computeLinearTrajectory (sphere_v, external_forces, timestep - (sphere_tc - t_0));

    return implementation::computeIntersectionSpFp (sphere_tc, sphere_p, sphere_r, ds, fplane_q, fplane_n, t_0, timestep);
}

} //namespace dte3607::physengine::mechanics

#endif //DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H









