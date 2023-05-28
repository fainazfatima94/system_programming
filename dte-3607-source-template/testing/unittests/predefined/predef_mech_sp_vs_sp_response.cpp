#include <physengine/mechanics/sphere_vs_sphere_response.h>

#include <physengine/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

/////////////////
/// \brief TEST
///
///

using namespace dte3607::physengine;

TEST(Mechanics_Sp_vs_Sp_Response, BasicTest)
{
  // Sphere
  types::Vector3 const sphere_p{-0.5, 0, 0};
  types::Vector3 const sphere_v{1.0, 0, 0};
  types::ValueType const sphere_m{1.0};

  // Other
  types::Vector3 const other_p{0.5, 0, 0};
  types::Vector3 const other_v{-1.0, 0, 0};
  types::ValueType const other_m{1.0};

  auto const response = mechanics::computeImpactResponseSphereSphere(
    sphere_p, sphere_v, sphere_m, other_p, other_v, other_m);

  auto const gold_sphere = types::Vector3{-1.0, 0, 0};
  auto const gold_other  = types::Vector3{1.0, 0, 0};

  auto const diff_sphere = response.first - gold_sphere;
  auto const diff_other  = response.second - gold_other;

  auto const length_diff_sphere = blaze::evaluate(blaze::length(diff_sphere));
  auto const length_diff_other  = blaze::evaluate(blaze::length(diff_other));

  EXPECT_NEAR(length_diff_sphere, 0, 1e-5);
  EXPECT_NEAR(length_diff_other, 0, 1e-5);
}

TEST(Mechanics_Sp_vs_Sp_Response, DifferentMassTest)
{
  // Sphere
  types::Vector3 const sphere_p{-0.5, 0, 0};
  types::Vector3 const sphere_v{1.0, 0, 0};
  types::ValueType const sphere_m{1.0};

  // Other
  types::Vector3 const other_p{0.5, 0, 0};
  types::Vector3 const other_v{-1.0, 0, 0};
  types::ValueType const other_m{4.0};

  auto const response = mechanics::computeImpactResponseSphereSphere(
    sphere_p, sphere_v, sphere_m, other_p, other_v, other_m);

  auto const gold_sphere = types::Vector3{-2.2, 0, 0};
  auto const gold_other  = types::Vector3{-0.2, 0, 0};

  auto const diff_sphere = response.first - gold_sphere;
  auto const diff_other  = response.second - gold_other;

  auto const length_diff_sphere = blaze::evaluate(blaze::length(diff_sphere));
  auto const length_diff_other  = blaze::evaluate(blaze::length(diff_other));

  EXPECT_NEAR(length_diff_sphere, 0, 1e-5);
  EXPECT_NEAR(length_diff_other, 0, 1e-5);
}
