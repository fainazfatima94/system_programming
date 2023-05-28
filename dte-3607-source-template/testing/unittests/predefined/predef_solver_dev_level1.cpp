#include <physengine/solvers/solver_dev_level0.h>
#include <physengine/solvers/solver_dev_level1.h>
#include <physengine/bits/fixtures.h>
#include <physengine/bits/types_oop.h>
#include <physengine/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>
#include <memory>


// Safely enable namespaces
using namespace dte3607::physengine;
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///




struct SolverDevLevel0_Fixture001 : ::testing::Test {

  using TestFixture = types_ext::FixtureOOP;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make sphere: radius, velocity, translation
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
    m_scenario->createSphere(2.0, {100, 0, 0}, {0, 10, 0});
    m_scenario->createSphere(3.0, {100, 0, 0}, {0, 0, 10});
  }
  void TearDown() final { m_scenario.release(); }
};






TEST_F(SolverDevLevel0_Fixture001, Test001)
{
  solver_dev::level0::solve(*m_scenario, 1s);

  auto no_rbs = m_scenario->noRigidBodies();
  for( auto rid = 0; rid < no_rbs; ++rid )
  {
    // Ask for global frame position of object nr. i
    auto const pos = m_scenario->globalFramePosition(rid);
    EXPECT_NEAR(pos[0], 100, 1e-7);
  }
}




struct SolverDevLevel1_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel1;
  std::unique_ptr<TestFixture> m_scenario;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();

    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make sphere: radius, velocity, translation
    m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
    m_scenario->createSphere(2.0, {100, 0, 0}, {0, 10, 0});
    m_scenario->createSphere(3.0, {100, 0, 0}, {0, 0, 10});
  }
  void TearDown() final { m_scenario.release(); }
};



TEST_F(SolverDevLevel1_Fixture001, Test001)
{
  solver_dev::level1::solve(*m_scenario, 1s);

  auto no_rbs = m_scenario->noRigidBodies();
  for( auto rid = 0; rid < no_rbs; ++rid )
  {
    // Ask for global frame position of object nr. i
    auto const pos = m_scenario->globalFramePosition(rid);
    EXPECT_NEAR(pos[0], 100, 1e-7);
  }
}



