#include <physengine/mechanics/sphere_vs_fixed_plane_response.h>

#include <physengine/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file


/////////////////
/// \brief TEST
///
///

using namespace dte3607::physengine;

TEST(Mechanics_Sp_vs_FPl_Response, BasicTest)
{

  // Sphere velocity
  types::Vector3 const sphere_v{10.0, 0, 0};

  // The plane's normal
  types::Vector3 const fplane_n{-1, 0, 0};

  auto const response
    = mechanics::computeImpactResponseSphereFixedPlane(sphere_v, fplane_n);

  auto gold_res        = types::Vector3{-10.0, 0, 0};
  auto diff_res        = gold_res - response;
  auto length_res_diff = blaze::evaluate(blaze::length(diff_res));

  EXPECT_NEAR(length_res_diff, 0, 1e-5);
}

TEST(Mechanics_Sp_vs_FPl_Response, HorizontalPlane_AngleTest)
{

  // Sphere velocity
  types::Vector3 const sphere_v{5.0, 0.0, -5.0};

  // The plane's normal
  types::Vector3 const fplane_n{0, 0, 1};

  auto const response
    = mechanics::computeImpactResponseSphereFixedPlane(sphere_v, fplane_n);

  auto gold_res        = types::Vector3{5.0, 0.0, 5.0};
  auto diff_res        = gold_res - response;
  auto length_res_diff = blaze::evaluate(blaze::length(diff_res));

  EXPECT_NEAR(length_res_diff, 0, 1e-5);
}

TEST(Mechanics_Sp_vs_FPl_Response, HorizontalPlane_Tilted_Test)
{
  // Sphere velocity
  types::Vector3 const sphere_v{5.0, 0.0, -5.0};

  // The plane's normal
  types::Vector3 const fplane_n{-0.70710678, 0, 0.70710678};

  auto const response
    = mechanics::computeImpactResponseSphereFixedPlane(sphere_v, fplane_n);

  auto gold_res        = types::Vector3{-5.0, 0.0, 5.0};
  auto diff_res        = gold_res - response;
  auto length_res_diff = blaze::evaluate(blaze::length(diff_res));

  EXPECT_NEAR(length_res_diff, 0, 1e-5);
}
