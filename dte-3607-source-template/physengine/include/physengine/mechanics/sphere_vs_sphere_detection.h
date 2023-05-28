#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H

#include "compute_trajectory.h"
#include "../bits/types.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

inline std::optional<types::HighResolutionTP> detectCollisionSphereSphere(
        [[maybe_unused]] types::HighResolutionTP const& s1_tc,
[[maybe_unused]] types::Point3 const&           s1_p,
[[maybe_unused]] types::ValueType               s1_r,
[[maybe_unused]] types::Vector3 const&          s1_v,
[[maybe_unused]] types::HighResolutionTP const& s2_tc,
[[maybe_unused]] types::Point3 const&           s2_p,
[[maybe_unused]] types::ValueType               s2_r,
[[maybe_unused]] types::Vector3 const&          s2_v,
[[maybe_unused]] types::Vector3 const&          external_forces,
[[maybe_unused]] types::HighResolutionTP const& t_0,
[[maybe_unused]] types::Duration                timestep)
{
    auto const Q = s2_p - s1_p;
    auto const ds1 = computeLinearTrajectory (s1_v, external_forces, timestep - (s1_tc - t_0)).first;
    auto const ds2 = computeLinearTrajectory (s2_v, external_forces, timestep - (s2_tc - t_0)).first;
    auto const R = ds2 - ds1;
    auto const r = s1_r + s2_r;
    auto const r2 = std::pow (r, 2);
    auto const epsilon = types::ValueType(10e-5);

    auto const RR = blaze::inner (R, R);
    auto const QR = blaze::inner (Q, R);
    auto const QQ = blaze::inner (Q, Q);

    // Check if speres are touching each other
    if(std::abs(QQ - r2) < epsilon)
        return std::nullopt;

    // Check if the speres are moving in parallel
    if(RR < epsilon)
        return std::nullopt;

    // term inside square root
    auto const sqrt_term = std::abs(std::pow(QR, 2) - RR * (QQ - r2));

    // check if the spheres are moving in orthogonal directions
    if(sqrt_term < epsilon)
        return std::nullopt;

    // Compute x, it should be safe now...
    auto const x = (- QR - std::sqrt(sqrt_term)) / RR;

    // Check bounds: 0<x<=1
    if(x <= 0 or x > 1)
        return std::nullopt;

    // Still survived? Return scalar time point analoguous to the SpFp det. met.
    auto const dt_scaled = timestep * x;
    auto const tp = types::HighResolutionTP(
                t_0 + std::chrono::duration_cast<types::Duration>(dt_scaled));

    return tp;
}


}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
