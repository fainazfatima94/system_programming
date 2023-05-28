#include <physengine/solvers/solver_dev_level3.h>
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


struct SolverDevLevel3_Fixture001 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();


    // Set external forces
    m_scenario->setGravity({0, -9.82, 0});


    // make plane
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0},
                                    m_scenario->rbPlaneMaxFrictionCoef());

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {0, 0, 0}, {0, 1.001, 0}, types::RBState::Free,
      m_scenario->rbSphereMaxFrictionCoef());
  }
  void TearDown() final { m_scenario.release(); }
};

TEST_F(SolverDevLevel3_Fixture001, Test001)
{
  solver_dev::level3::solve(*m_scenario, 10s);

  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Resting);
}



struct SolverDevLevel3_Fixture002 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();


    // Set external forces
    m_scenario->setGravity({10, -9.82, 0});

    // make plane
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0},
                                    m_scenario->rbPlaneMaxFrictionCoef());

    // make sphere
    m_the_s_rid = m_scenario->createSphere(
      1.0, {0, 0, 0}, {0, 1.001, 0}, types::RBState::Free,
      m_scenario->rbSphereMaxFrictionCoef());
  }
  void TearDown() final { m_scenario.release(); }
};



TEST_F(SolverDevLevel3_Fixture002, Test001)
{
  solver_dev::level3::solve(*m_scenario, 10s);
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Sliding);
}





struct SolverDevLevel3_Fixture003 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();


    // Set external forces
    m_scenario->setGravity({0, -9.82, 0});

    // make plane
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});

    // make sphere
    m_the_s_rid = m_scenario->createSphere(1.0, {0, 0, 0}, {0, 1.000, 0},
                                           types::RBState::Resting);

    // make sphere 2
    m_scenario->createSphere(0.2, {0, -5, 0}, {-.5, 2.000, 0},
                             types::RBState::Free,
                             m_scenario->rbSphereMaxFrictionCoef(), 10.);
  }
  void TearDown() final { m_scenario.release(); }
};



TEST_F(SolverDevLevel3_Fixture003, Test001)
{
  solver_dev::level3::solve(*m_scenario, 10s);
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Free);
}





struct SolverDevLevel3_Fixture004 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();


    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0});

    // make sphere
    m_the_s_rid = m_scenario->createSphere(1.0, {0, 0, 0}, {0, 1.000, 0},
                                           types::RBState::Resting);

    // make sphere 2
    m_scenario->createSphere(0.2, {5, 0, 0}, {-1.5, .5, 0},
                             types::RBState::Free,
                             m_scenario->rbSphereMaxFrictionCoef(), 10.);
  }
  void TearDown() final { m_scenario.release(); }
};



TEST_F(SolverDevLevel3_Fixture004, Test001)
{
  solver_dev::level3::solve(*m_scenario, 2s);
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Free);
}





struct SolverDevLevel3_Fixture005 : ::testing::Test {

  using TestFixture = fixtures::FixtureLevel3;
  std::unique_ptr<TestFixture> m_scenario;
  size_t                       m_the_s_rid;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_scenario = std::make_unique<TestFixture>();


    // Set external forces
    m_scenario->setGravity({0, 0, 0});

    // make plane
    m_scenario->createFixedInfPlane({0, 1, 0}, {0, 0, 0},
                                    m_scenario->rbPlaneMaxFrictionCoef());

    // make sphere
    m_the_s_rid = m_scenario->createSphere(1.0, {10, 0, 0}, {-5, 1, 0},
                                           types::RBState::Sliding);

    // make sphere 2
    m_scenario->createSphere(0.2, {0, 0, 0}, {-5, 1, 0},
                             types::RBState::Resting,
                             m_scenario->rbSphereMaxFrictionCoef(), 10000.);
  }
  void TearDown() final { m_scenario.release(); }
};



TEST_F(SolverDevLevel3_Fixture005, Test001)
{
  solver_dev::level3::solve(*m_scenario, 2s);
  EXPECT_EQ(m_scenario->rbState(m_the_s_rid), types::RBState::Free);
}
