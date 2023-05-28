#include <physengine/mechanics/compute_trajectory.h>

// gtest
#include <gtest/gtest.h>   // googletest header file


/////////////////
/// \brief TEST
///
///  Create tests from gold data tables


TEST(Mechanics_ComputeLinearTrajectory, TrajectoryOverZeroTime)
{
  using namespace dte3607::physengine;
  using Vector3 = types::Vector3;

  auto const v  = Vector3{1, 0, 0};
  auto const ds
    = mechanics::computeLinearTrajectory(v, Vector3(0), types::Seconds(0));

  auto gold_null_vec = Vector3{0, 0, 0};
  EXPECT_EQ(ds.first, gold_null_vec);
  EXPECT_EQ(ds.second, gold_null_vec);
}

TEST(Mechanics_ComputeLinearTrajectory, TrajectoryOver16msTime)
{
  using namespace dte3607::physengine;
  using Vector3 = types::Vector3;

  auto const v  = Vector3{1, 0, 0};
  auto const F  = Vector3{0, 0, -9.81};
  auto const ds
    = mechanics::computeLinearTrajectory(v, F, types::MilliSeconds(16));

  auto gold_a_16ms_vec  = Vector3{0, 0, -0.15696};
  auto gold_ds_16ms_vec = Vector3{0.016, 0., -0.00125568};
  auto diff_a           = gold_a_16ms_vec - ds.second;
  auto diff_ds          = gold_ds_16ms_vec - ds.first;
  auto length_a_diff    = blaze::evaluate(blaze::length(diff_a));
  auto length_ds_diff   = blaze::evaluate(blaze::length(diff_ds));

  EXPECT_NEAR(length_a_diff, 0, 1e-5);
  EXPECT_NEAR(length_ds_diff, 0, 1e-5);
}

TEST(Mechanics_ComputeLinearTrajectory, TrajectoryOver200msTime)
{
  using namespace dte3607::physengine;
  using Vector3 = types::Vector3;

  auto const v  = Vector3{5, 1, 0};
  auto const F  = Vector3{0, 0, -9.81};
  auto const ds
    = mechanics::computeLinearTrajectory(v, F, types::MilliSeconds(200));

  auto gold_a_200ms_vec  = Vector3{0, 0, -1.962};
  auto gold_ds_200ms_vec = Vector3{1., 0.2, -0.1962};
  auto diff_a            = gold_a_200ms_vec - ds.second;
  auto diff_ds           = gold_ds_200ms_vec - ds.first;
  auto length_a_diff     = blaze::evaluate(blaze::length(diff_a));
  auto length_ds_diff    = blaze::evaluate(blaze::length(diff_ds));

  EXPECT_NEAR(length_a_diff, 0, 1e-5);
  EXPECT_NEAR(length_ds_diff, 0, 1e-5);
}
