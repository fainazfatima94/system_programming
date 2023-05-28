// gtest
#include <gtest/gtest.h>   // googletest header file

// gmlib2
#include <core/space.h>
#include <core/spaces/vectorspace.h>
#include <core/spaces/projectivespace.h>
using namespace gmlib2::spaces;


namespace unittest_detail
{

  template <size_t M, size_t N>
  struct CustomMNSpaceInfo {
    using Unit                      = double;
    static constexpr auto VectorDim = M;
    static constexpr auto FrameDim  = N;
  };



  template <size_t ROWS, size_t COLS, typename Frame>
  void blazeZeroFrameChecks(const Frame& frame)
  {

    EXPECT_EQ(ROWS, frame.rows());
    EXPECT_EQ(COLS, frame.columns());

    for (size_t ri = 0; ri < ROWS; ++ri)
      for (size_t ci = 0; ci < COLS; ++ci) EXPECT_DOUBLE_EQ(0.0, frame(ri, ci));
  }

  template <size_t ROWS, size_t COLS, typename Frame>
  void blazeIdentityFrameChecks(const Frame& frame)
  {

    ASSERT_EQ(ROWS, frame.rows());
    ASSERT_EQ(COLS, frame.columns());

    for (size_t ri = 0; ri < ROWS; ++ri) {
      for (size_t ci = 0; ci < COLS; ++ci) {
        if (ri == ci)
          EXPECT_DOUBLE_EQ(1.0, frame(ri, ci));
        else
          EXPECT_DOUBLE_EQ(0.0, frame(ri, ci));
      }
    }
  }

  template <size_t ROWS, size_t COLS, typename Frame>
  void blazeIdentityASFrameHChecks(const Frame& frame)
  {

    ASSERT_EQ(ROWS + 1, frame.rows());
    ASSERT_EQ(COLS + 1, frame.columns());

    for (size_t ci = 0; ci < COLS; ++ci) {
      for (size_t ri = 0; ri < ROWS; ++ri) {
        if (ri == ci)
          EXPECT_DOUBLE_EQ(1.0, frame(ri, ci));
        else
          EXPECT_DOUBLE_EQ(0.0, frame(ri, ci));
      }
      EXPECT_DOUBLE_EQ(0.0, frame(ROWS, ci));
    }
    EXPECT_DOUBLE_EQ(1.0, frame(ROWS, COLS));
  }


}   // namespace unittest_detail




TEST(Core_Space, VectorSpace_NoInitFrame_D3R3)
{

  using Space = vectorspace::VectorSpace<D3R3SpaceInfo<>>;
  Space::Frame frame;

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeZeroFrameChecks<3, 3>(frame);
}

TEST(Core_Space, VectorSpace_IdentityFrame_D3R3)
{


  using Space = vectorspace::VectorSpace<D3R3SpaceInfo<>>;
  auto frame  = vectorspace::identityFrame<Space>();

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeIdentityFrameChecks<3, 3>(frame);
}

TEST(Core_Space, VectorSpace_IdentityFrame_MN_5_2)
{


  using Space
    = vectorspace::VectorSpace<unittest_detail::CustomMNSpaceInfo<5, 2>>;
  auto frame = vectorspace::identityFrame<Space>();

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeIdentityFrameChecks<5, 2>(frame);
}

TEST(Core_Space, VectorSpace_IdentityFrame_MN_2_5)
{
  using Space
    = vectorspace::VectorSpace<unittest_detail::CustomMNSpaceInfo<2, 5>>;
  auto frame = vectorspace::identityFrame<Space>();

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeIdentityFrameChecks<2, 5>(frame);
}


TEST(Core_Space, ProjectiveSpace_NoInitFrame_D3R3)
{
  using ProjSpace = projectivespace::ProjectiveSpace<D3R3SpaceInfo<>>;
  ProjSpace::ASFrameH frame;

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeZeroFrameChecks<3 + 1, 3 + 1>(frame);
}

TEST(Core_Space, ProjectiveSpace_IdentityFrame_D3R3)
{
  using ProjSpace = projectivespace::ProjectiveSpace<D3R3SpaceInfo<>>;
  auto frame      = projectivespace::identityFrame<ProjSpace>();

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeIdentityASFrameHChecks<3, 3>(frame);
}

TEST(Core_Space, ProjectiveSpace_IdentityFrame_MN_5_2)
{
  using ProjSpace = projectivespace::ProjectiveSpace<
    unittest_detail::CustomMNSpaceInfo<5, 2>>;
  auto frame = projectivespace::identityFrame<ProjSpace>();

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeIdentityASFrameHChecks<5, 2>(frame);
}

TEST(Core_Space, ProjectiveSpace_IdentityFrame_MN_2_5)
{
  using ProjSpace = projectivespace::ProjectiveSpace<
    unittest_detail::CustomMNSpaceInfo<2, 5>>;
  auto frame = projectivespace::identityFrame<ProjSpace>();

  //  std::cout << "frame: " << std::endl << frame << std::endl;
  unittest_detail::blazeIdentityASFrameHChecks<2, 5>(frame);
}
