// gtest
#include <gtest/gtest.h>   // googletest header file

// gmlib2
#include <core/spaces/vectorspace.h>
#include <core/spaces/affinespace.h>
#include <core/spaces/projectivespace.h>
using namespace gmlib2::spaces;


TEST(Core_Space_Traits, D3R3SpaceInfo_isSpaceInfo)
{
  EXPECT_TRUE(
    gmlib2::traits::Is_SpaceInfo<gmlib2::spaces::D3R3SpaceInfo<>>::value);
}




TEST(Core_Space_Traits, VectorSpace_isVectorSpace)
{
  using D3R3_space_info = gmlib2::spaces::D3R3SpaceInfo<>;

  EXPECT_TRUE(
    gmlib2::traits::Is_VectorSpace<
      gmlib2::spaces::vectorspace::VectorSpace<D3R3_space_info>>::value);
}


TEST(Core_Space_Traits, AffineSpace_isAffineSpace)
{
  using D3R3_space_info = gmlib2::spaces::D3R3SpaceInfo<>;

  EXPECT_TRUE(
    gmlib2::traits::Is_VectorSpace<
      gmlib2::spaces::affinespace::AffineSpace<D3R3_space_info>>::value);
  EXPECT_TRUE(
    gmlib2::traits::Is_AffineSpace<
      gmlib2::spaces::affinespace::AffineSpace<D3R3_space_info>>::value);
}

TEST(Core_Space_Traits, ProjectiveSpace_isProjectiveSpace)
{
  using D3R3_space_info = gmlib2::spaces::D3R3SpaceInfo<>;
  EXPECT_TRUE(
    gmlib2::traits::Is_VectorSpace<gmlib2::spaces::projectivespace::
                                     ProjectiveSpace<D3R3_space_info>>::value);
  EXPECT_TRUE(
    gmlib2::traits::Is_AffineSpace<gmlib2::spaces::projectivespace::
                                     ProjectiveSpace<D3R3_space_info>>::value);
  EXPECT_TRUE(
    gmlib2::traits::Is_ProjectiveSpace<
      gmlib2::spaces::projectivespace::ProjectiveSpace<D3R3_space_info>>::
      value);
}
