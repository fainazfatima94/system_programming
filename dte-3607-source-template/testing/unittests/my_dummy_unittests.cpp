// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>



namespace dte3607::testing {

  struct GoldDummyTestF : ::testing::Test {

    using ::testing::Test::Test;
    ~GoldDummyTestF() override {}

    void SetUp() final {}
    void TearDown() final {}
  };

}   // namespace dte3607::testing


// Qualify predefined fixtures
using namespace dte3607::testing;


// Dummy tests
TEST_F(GoldDummyTestF, fail_dummy)
{
  auto const gold = true;
  EXPECT_EQ(false, gold);
}

TEST_F(GoldDummyTestF, pass_dummy)
{
  auto const gold = true;
  EXPECT_EQ(true, gold);
}
