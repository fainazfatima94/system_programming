#include <physengine/solvers/solver_dev_level4.h>
#include <physengine/bits/fixtures.h>
#include <physengine/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>


// Safely enable namespaces
using namespace dte3607::physengine;
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///


struct SolverDevLevel4_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel4;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // ...
  }
  void TearDown() final { m_scenario.release(); }
};

TEST_F(SolverDevLevel4_Fixture001, Test001)
{
  solver_dev::level4::solve(*m_scenario, 10s);
  GTEST_FAIL();
}
