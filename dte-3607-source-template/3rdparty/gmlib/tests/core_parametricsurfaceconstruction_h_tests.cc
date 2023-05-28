// gtest
#include <gtest/gtest.h>   // googletest header file

// gmlib2
#include <parametric/polygonsurface_constructions/polygon.h>







namespace unittest_detail
{
}   // namespace unittest_detail










TEST(Core_ParametricSurfaceConstruction, Stack_Compile_Test)
{

  namespace gm2  = gmlib2;
  namespace gm2p = gm2::parametric;

  using EmbedSpaceInfo = gm2::spaces::D3R3SpaceInfo<>;
  using SO             = gm2::ProjectiveSpaceObject<EmbedSpaceInfo>;
  using Polygon        = gm2p::Polygon<SO>;


  // Space object
  [[maybe_unused]] SO so;

  // Polygon surfaces construction
  [[maybe_unused]] Polygon polygon4{
    gm2::polygonutils::generateRegularPolygon2DXZ(4)};
  [[maybe_unused]] Polygon polygon5{
    gm2::polygonutils::generateRegularPolygon2DXZ(5)};
  [[maybe_unused]] Polygon polygon6{
    gm2::polygonutils::generateRegularPolygon2DXZ(6)};

  EXPECT_EQ(polygon4.sides(), 4UL);
  EXPECT_EQ(polygon5.sides(), 5UL);
  EXPECT_EQ(polygon6.sides(), 6UL);
}
